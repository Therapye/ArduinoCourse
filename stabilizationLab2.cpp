#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h" // бібліотека для роботи з DMP
#include "Wire.h"                       // бібліотека для I2C
#include <Servo.h>


const int PIN_SERVO_PITCH = 5; // нахил вперед назад
const int PIN_SERVO_YAW = 10;  //  поворот вліво вправо
const int PIN_SERVO_ROLL = 4;  // крен, боковий нахил
const int INTERRUPT_PIN = 2;   // переривання від MPU

const float SMOOTHING_ALPHA = 0.15; // коеф згладжування, 0.15 означає, що берем 15% від нового значення, і що залишилось від старого

class LowPassFilter // створюємо клас фільтру 
{
private:
    float alpha;
    double currentVal;

public:
    LowPassFilter(float smoothingAlpha, double initialValue)
    {
        alpha = smoothingAlpha;
        currentVal = initialValue;
    }

    // currentVal - значення зараз, targetVal відповідно ціль
    float smooth(float targetVal)
    {
        // і чим менше це число, тим плавніше рух, але більше затримка
        currentVal = (targetVal * alpha) + (currentVal * (1.0 - alpha));
        return (float)currentVal;
    }
};



class ServoBase // клас для роботи з серво (початкове положення, захисні межі і так далі)
{
private:
    Servo servo;
    int pin;
    LowPassFilter filter;

public:
    ServoBase(int servoPin, float smoothingAlpha)
        : pin(servoPin), filter(smoothingAlpha, 90.0) {} // Початкове положення 90 градусів для серво

    void attach()
    {
        servo.attach(pin);
        servo.write(90); // переводимо в 90
    }

    void update(float targetAngle)
    {
        // захист від виходу за межі 0-180
        float constrainedAngle = constrain(targetAngle, 0, 180);

        // згладжування рухів
        // застосовуємо фільтр щоб прибрати тремтіння сервоприводів
        float smoothedAngle = filter.smooth(constrainedAngle);

        // приводимо float до int бо серво приймає цілі градуси
        servo.write((int)smoothedAngle);
    }
};



class Controller // в цьому класі виконується робота з DMP і MPU 
{
private:
    MPU6050 mpu;
    uint8_t devStatus;      // статус повернення операцій, 0  успіх, !0 помилка
    uint16_t packetSize;    // очікуваний розмір пакету даних DMP
    uint16_t fifoCount;     // кількість байтів наявних зараз у буфері FIFO
    uint8_t fifoBuffer[64]; // масив для зберігання пакету даних

    Quaternion q;          // w, x, y, z
    VectorFloat gravity;   // x, y, z вектор гравітації, викор для корекції кутів
    float ypr[3];          // yaw, pitch, roll
    bool dmpReady = false; // flag для визначення чи ініціалізовано DMP

public:
    void init(int interruptPin)
    {
        Wire.begin();
        Wire.setClock(400000);
        Wire.setWireTimeout(3000, true); // скидаємо шину якщо зависла на 3 мс

        Serial.begin(9600);

        mpu.initialize();
        pinMode(interruptPin, INPUT);

        // перевірка зв'язку з чіпом
        Serial.println(mpu.testConnection() ? F("MPU6050 OK") : F("MPU6050 Error"));

        // пишемо наші офсети
        mpu.setXAccelOffset(26817);
        mpu.setYAccelOffset(18809);
        mpu.setZAccelOffset(-16468);
        mpu.setXGyroOffset(-1);
        mpu.setYGyroOffset(4);
        mpu.setZGyroOffset(1);

        // запуск DMP
        devStatus = mpu.dmpInitialize();

        if (devStatus == 0)
        {
            // успішний запуск
            mpu.setDMPEnabled(true);
            dmpReady = true;
            packetSize = mpu.dmpGetFIFOPacketSize(); // Отримуємо розмір пакету
            Serial.println(F("Ready"));
        }
        else
        {
            // помилка ініціалізації
            Serial.print(F("Error DMP "));
            Serial.println(devStatus);
            while (1)
                ; // зупиняємо програму вічним циклом
        }
    }

    bool getData(float &yawDeg, float &pitchDeg, float &rollDeg)
    {
        // якщо DMP не запустився нічого не робимо
        if (!dmpReady)
            return false;

        // отримуємо дані
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        {

            // матем. обробка
            mpu.dmpGetQuaternion(&q, fifoBuffer);      // отримуємо кватерніон з буфера
            mpu.dmpGetGravity(&gravity, &q);           // виділяємо вектор гравітації
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // рахуємо кути Ейлера без впливу гравітації

            // Переведення з радіан  у градуси
            yawDeg = ypr[0] * 180 / M_PI;
            pitchDeg = ypr[1] * 180 / M_PI;
            rollDeg = ypr[2] * 180 / M_PI;

            return true;
        }
        return false;
    }

    void checkBusTimeout()
    {
        // скидаємо flag таймауту I2C щоб шина не зависла
        if (Wire.getWireTimeoutFlag())
        {
            Wire.clearWireTimeoutFlag();
        }
    }
};


// ініціалізуємо об'єкти
Controller controller;
ServoBase axisPitch(PIN_SERVO_PITCH, SMOOTHING_ALPHA);
ServoBase axisYaw(PIN_SERVO_YAW, SMOOTHING_ALPHA);
ServoBase axisRoll(PIN_SERVO_ROLL, SMOOTHING_ALPHA);


void setup()
{
    controller.init(INTERRUPT_PIN);

    // підключаємо серво
    axisPitch.attach();
    axisYaw.attach();
    axisRoll.attach();
}

void loop()
{
    float yawDeg, pitchDeg, rollDeg;

    if (controller.getData(yawDeg, pitchDeg, rollDeg))
    {

        /* логіка стабілізації

         щодо знаків +/-, то це визначає напрямок компенсації
        якщо датчик нахилився вперед +, серво має крутити назад -, або навпаки
        залежно від механіки важелів
        */
        float targetPitch = 90.0 + pitchDeg;
        float targetRoll = 90.0 - rollDeg;
        float targetYaw = 90.0 - yawDeg;

        // Оновлення осей (включає constrain, smooth та write)
        axisPitch.update(targetPitch);
        axisRoll.update(targetRoll);
        axisYaw.update(targetYaw);
    }

    controller.checkBusTimeout();
}
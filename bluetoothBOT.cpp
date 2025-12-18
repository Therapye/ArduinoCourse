#include <AFMotor_R4.h> 

//створюємо клас перевірки заряду наших акумуляторів
class BatteryManager {
private:
    int pin;    //пін до дільника напруги
    unsigned long lowBatTimer;      // таймер для фільтрації сигналу
    bool actualLowBatState;      // Поточний стан, true = розряджений

public:
    BatteryManager(int batteryPin) {
        pin = batteryPin;       
        lowBatTimer = 0;   // Скидаємо таймер
        actualLowBatState = false;    // будемо рахувати спочатку, що батарея заряджена
        pinMode(pin, INPUT);
    }

    // Метод перевірки стану батареї 
    bool isLowBattery() {

        double voltage = analogRead(pin) * (5.0 / 1023.0) * 2; 
        
        if (voltage < 7.0) // перевіряємо поріг і потім відштовхуємсь від нього
        {
            // перша фіксація розряду акумуляторів
            if (lowBatTimer == 0) {
                lowBatTimer = millis(); // засікаємо поточний час
            }
            if (millis() - lowBatTimer > 1000) 
            {
                actualLowBatState = true; // Якщо просадка триває довше 1 c значить ставимо true
            }
        } 
        else 
        {
            // Якщо напруга все-таки відновилась скидаємо таймер і ставимо false змінній
            lowBatTimer = 0;         
            actualLowBatState = false; 
        }

        return actualLowBatState; // повертаємо результат
    }
};

//створюємо клас вдтворення звуку з horn
class SoundManager {
private:
    int pin;   // Пін horn'a
    unsigned long buzzerTimer;    // таймер початку звуку
    unsigned long buzzerDuration; // тривалість звуку
    bool buzzerActive;   // перевірка чи зараз звук активний

public:
    SoundManager(int buzzerPin) {
        pin = buzzerPin;      
        buzzerTimer = 0;       
        buzzerDuration = 0;     
        buzzerActive = false;   // звук вимкнений
        pinMode(pin, OUTPUT);
    }

    // стартова мелодія
    void playStartupMelody() {
        tone(pin, 400, 200);     
        delay(250);                 
        tone(pin, 700, 300);         
        delay(350);              
        noTone(pin);       
    }

    // метод для оновлення стану
    void update() {
        if (buzzerActive) 
        { //якщо звук активний перевіряємо час, якщо пройшов, то вимикаємо
            if (millis() - buzzerTimer >= buzzerDuration) 
            {
                noTone(pin);
                buzzerActive = false;
            }
        }
    }

    // метод сигналу перешкоди 
    void playObstacleTone() {
        if (!buzzerActive) {   //перевіряємо, чи не грає звук
            tone(pin, 200, 120);
            buzzerTimer = millis(); // засікаємо час
            buzzerDuration = 300;  // тривалість
            buzzerActive = true;
        }
    }

    // Метод для гудка
    void honk() {
        if (!buzzerActive) {  
            tone(pin, 1000, 300);    
            buzzerTimer = millis();  
            buzzerDuration = 250;    
            buzzerActive = true;   
        }
    }
};

// клас для керування світлодіодами
class LightManager {
private:
    int headPin;   //передні фари
    int backPin;         //задні габарити
    bool brakeLightsOn;  //стан стоп сигналів
    bool headLightsOn;   //стан передніх фар
    
    // змінні для блимання
    unsigned long prevMillis;         
    const unsigned long interval = 300; 
    bool blinkState;

public:
    LightManager(int hPin, int bPin) {
        headPin = hPin;    
        backPin = bPin;  
        brakeLightsOn = true;  // за замовчуванням стоїмо, задні габарити світяться
        headLightsOn = false; //передні вимкнені
        prevMillis = 0;  // таймер блимання скидаємо
        blinkState = false;
        
        pinMode(headPin, OUTPUT); 
        pinMode(backPin, OUTPUT);
    }

    //сетери для зміни станів світлодіодів
    void setBrakeLights(bool state) { brakeLightsOn = state; }
    void setHeadLights(bool state) { headLightsOn = state; }

    //метод для блимання
    void blink() {
        unsigned long currentMillis = millis();
        if (currentMillis - prevMillis >= interval) //якщо пройшов інтервал оновлюємо час, інвертуємо стан і так виходить блимання
        { 
            prevMillis = currentMillis; 
            blinkState = !blinkState; 
            digitalWrite(headPin, blinkState);
            digitalWrite(backPin, blinkState);
        }
    }

    //головний метод управління
    void update(bool isLowBat) {
        if (isLowBat) 
        {
            blink(); // Якщо батарея сівша блимаємо
        } 
        else // якщо все окей з акумулятором йде звичайна логіка
        {
            
            if (brakeLightsOn) digitalWrite(backPin, HIGH); // задні габарити
            else digitalWrite(backPin, LOW);
            
            if (headLightsOn) digitalWrite(headPin, HIGH); // передні фари
            else digitalWrite(headPin, LOW);
        }
    }
};

// клас для IR датчиків
class SensorManager {
private:
    int frontPin; // передній датчик
    int backPin;  // задній датчик

public:
    SensorManager(int fPin, int bPin) {
        frontPin = fPin; 
        backPin = bPin; 
        pinMode(frontPin, INPUT);
        pinMode(backPin, INPUT); 
    }

    // перевірка перешкоди спереду 
    bool isBlockedFront() {
        return digitalRead(frontPin) == LOW; // коли low то це перешкода
    }
    // ззаду
    bool isBlockedBack() {
        return digitalRead(backPin) == LOW;
    }
};

// клас для сервоприводів
class MotorManager {
private:
    // Об'єкти серво
    AF_DCMotor mFR; 
    AF_DCMotor mFL;
    AF_DCMotor mBL;
    AF_DCMotor mBR;

    AF_DCMotor* motors[4]; 
    
    const int MAX_SPEED[4] = {235, 235, 250, 250}; // масив макс. швидкостей

public:
    MotorManager() : mFR(4), mFL(1), mBL(2), mBR(3) { // вказуємо вірний пін на кожен серво
        motors[0] = &mFR;
        motors[1] = &mFL;
        motors[2] = &mBL;
        motors[3] = &mBR;
    }

    // початкове налаштування
    void init(int startSpeed) {
        mFL.setSpeed(startSpeed);
        mFR.setSpeed(startSpeed);
        mBL.setSpeed(startSpeed);
        mBR.setSpeed(startSpeed);
        stopAll();
    }

    //зупинка всіх моторів
    void stopAll() {
        mFL.run(RELEASE);
        mFR.run(RELEASE);
        mBL.run(RELEASE);
        mBR.run(RELEASE);
    }

    // вперед
    void moveForward() {
        for (int i = 0; i < 4; i++) 
        {
            motors[i]->setSpeed(MAX_SPEED[i]);
        }
        mFL.run(FORWARD);
        mFR.run(FORWARD);
        mBL.run(FORWARD);
        mBR.run(FORWARD);
    }

    //назад
    void moveBack() {
        for (int i = 0; i < 4; i++) 
        {
            motors[i]->setSpeed(MAX_SPEED[i]);
        }
        mFL.run(BACKWARD);
        mFR.run(BACKWARD);
        mBL.run(BACKWARD);
        mBR.run(BACKWARD);
    }

    // праворуч
    void turnRight() {
        for (int i = 0; i < 4; i++) 
        {
            motors[i]->setSpeed(MAX_SPEED[i]);
        }
        mFL.run(FORWARD);
        mFR.run(BACKWARD);
        mBL.run(FORWARD);
        mBR.run(BACKWARD);
    }

    // ліворуч
    void turnLeft() {
        for (int i = 0; i < 4; i++) 
        {
            motors[i]->setSpeed(MAX_SPEED[i]);
        }
        mFL.run(BACKWARD);
        mFR.run(FORWARD);
        mBL.run(BACKWARD);
        mBR.run(FORWARD);
    }

    // вперед-вправо
    void moveForwardRight() {
        motors[0]->setSpeed(MAX_SPEED[0] / 4); // Зменшуємо швидкість діагональних
        motors[3]->setSpeed(MAX_SPEED[3] / 4);
        motors[1]->setSpeed(MAX_SPEED[1]);
        motors[2]->setSpeed(MAX_SPEED[2]);

        mFL.run(FORWARD);
        mFR.run(FORWARD);
        mBL.run(FORWARD);
        mBR.run(FORWARD);
    }

    // вперед-ліво
    void moveForwardLeft() {
        motors[1]->setSpeed(MAX_SPEED[1] / 4);
        motors[2]->setSpeed(MAX_SPEED[2] / 4);
        motors[0]->setSpeed(MAX_SPEED[0]);
        motors[3]->setSpeed(MAX_SPEED[3]);

        mFL.run(FORWARD);
        mFR.run(FORWARD);
        mBL.run(FORWARD);
        mBR.run(FORWARD);
    }

    // назад вправо
    void moveBackwardRight() {
        motors[0]->setSpeed(MAX_SPEED[0] / 4);
        motors[3]->setSpeed(MAX_SPEED[3] / 4);
        motors[1]->setSpeed(MAX_SPEED[1]);
        motors[2]->setSpeed(MAX_SPEED[2]);

        mFL.run(BACKWARD);
        mFR.run(BACKWARD);
        mBL.run(BACKWARD);
        mBR.run(BACKWARD);
    }

    // назад вліво
    void moveBackwardLeft() {
        motors[1]->setSpeed(MAX_SPEED[1] / 4);
        motors[2]->setSpeed(MAX_SPEED[2] / 4);
        motors[0]->setSpeed(MAX_SPEED[0]);
        motors[3]->setSpeed(MAX_SPEED[3]);

        mFL.run(BACKWARD);
        mFR.run(BACKWARD);
        mBL.run(BACKWARD);
        mBR.run(BACKWARD);
    }
};

// створюємо тепер об'єкти наших класів прописаних вище
MotorManager  robotMotors;              // сервоприводи
SensorManager robotSensors(A3, A2);     // IR давачі Front A3, Back A2 
BatteryManager robotBattery(A1);        // акумулятор
LightManager  robotLights(A5, A4);      // світлодіоди A5 - передні фари, A4 - задні габарити 
SoundManager  robotSound(2);            // звук

int valSpeed = 150; // початкова швидкість

// допоміжна функція на реакцію від перешкоди
void handleObstacle() {
    robotMotors.stopAll();
    robotLights.setBrakeLights(true);
    robotSound.playObstacleTone();
}


void setup() {
    Serial.begin(9600); 
    robotMotors.init(valSpeed);
    robotSound.playStartupMelody();
}


void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();

        switch (command) {
            case 'F':   // Move forward
                //перевіряємо перешкоду спереду
                if (robotSensors.isBlockedFront()) 
                {           
                    handleObstacle(); 
                    break; 
                }
                robotMotors.moveForward();        // Їдемо
                robotLights.setBrakeLights(false); // Вимикаємо стопи
                break;

            case 'B':   // Move backward
                robotMotors.moveBack();
                if (robotSensors.isBlockedBack())  //перевіряємо перешкоду ззаду
                { 
                    handleObstacle(); 
                    break; 
                }
                robotLights.setBrakeLights(true);
                break;

            case 'R':   // Turn right
                robotMotors.turnRight();
                robotLights.setBrakeLights(false);
                break;

            case 'L':   // Turn left
                robotMotors.turnLeft();
                robotLights.setBrakeLights(false);
                break;

            case 'G':   // Forward left
                if (robotSensors.isBlockedFront()) 
                { 
                    handleObstacle(); break; 
                }
                robotMotors.moveForwardLeft();
                robotLights.setBrakeLights(false);
                break;

            case 'H':   // Forward right
                if (robotSensors.isBlockedFront()) 
                {
                     handleObstacle(); break; 
                }
                robotMotors.moveForwardRight();
                robotLights.setBrakeLights(false);
                break;

            case 'I':   // Backward left
                if (robotSensors.isBlockedBack()) 
                { 
                    handleObstacle(); break; 
                }
                robotMotors.moveBackwardLeft();
                robotLights.setBrakeLights(true);
                break;

            case 'J':   // Backward right
                if (robotSensors.isBlockedBack()) 
                { 
                    handleObstacle(); break; 
                }
                robotMotors.moveBackwardRight();
                robotLights.setBrakeLights(true);
                break;

            case 'S':   // Stop all motors
                robotMotors.stopAll();
                robotLights.setBrakeLights(true);
                break;

            case 'Y':   // Honk horn
                robotSound.honk();
                break;

            case 'U':   // Turn headlight ON
                robotLights.setHeadLights(true);
                break;

            case 'u':   // Turn headlight OFF
                robotLights.setHeadLights(false);
                break;
        }
    }
    // оновлюємо стан систем у кожному проході циклу
    
    //horn
    robotSound.update();
    
    //стан батареї
    bool isLow = robotBattery.isLowBattery();
    
    //передаємо статус батареї, бо це впливає на блимання
    robotLights.update(isLow);
}
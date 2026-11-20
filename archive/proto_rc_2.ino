#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 mpu6050;

// --- Pins ---
const int BUTTON_PIN = 2;   
const int STATUS_LED = 13;     
const int MOTOR_LED = 3;      

// --- Sensitivity ---
const int SETPOINT = 5000;    
float Kp = 1.2, Ki = 0.1, Kd = 0.2;               

// --- Variables ---
float error = 0, lastError = 0, integral = 0, pidOutput = 0;
bool tcsEnabled = true;     
bool lastButtonState = HIGH;

// --- Timing ---
unsigned long nextToggleTime = 0; 
unsigned long lastStatusFlash = 0;
bool motorLedState = false;
bool statusLedState = false;
unsigned long currentInterval = 5000; 

void setup() {
    Wire.begin();
    Wire.setClock(400000); 
    Serial.begin(115200);
    
    // Force wait for Serial Monitor
    while (!Serial) { delay(10); }

    mpu6050.initialize();
    mpu6050.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

    if (mpu6050.testConnection()) {
        Serial.println("SYSTEM STATUS: ACTIVE");
    } else {
        Serial.println("MPU6050 CONNECTION FAILED");
        while(1);
    }

    pinMode(STATUS_LED, OUTPUT);
    pinMode(MOTOR_LED, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP); 
    
    tcsEnabled = true; 
    digitalWrite(STATUS_LED, LOW); // LED OFF = System Active
    nextToggleTime = micros() + 5000;
}

void loop() {
    // 1. MOTOR LED TOGGLE (Scope Precision)
    if (micros() >= nextToggleTime) {
        motorLedState = !motorLedState;
        digitalWrite(MOTOR_LED, motorLedState);
        nextToggleTime += currentInterval; 
    }

    checkButton();

    if (tcsEnabled) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        int16_t currentYaw = abs(gz);

        if (currentYaw > SETPOINT) {
            // PID Calculation
            error = currentYaw - SETPOINT;
            integral += error;
            pidOutput = (Kp * error) + (Ki * integral) + (Kd * (error - lastError));
            lastError = error;

            // Frequency Adjustment (100Hz down to 50Hz)
            currentInterval = 5000 + constrain(pidOutput, 0, 5000);

            // --- STATUS LED FLASH ---
            if (millis() - lastStatusFlash > 50) { // Fast 50ms blink
                lastStatusFlash = millis();
                statusLedState = !statusLedState;
                digitalWrite(STATUS_LED, statusLedState);
            }

            // --- SERIAL OUTPUT ---
            Serial.print("TRACTION ALERT! | Yaw Rate: ");
            Serial.print(gz);
            Serial.print(" | Lat Accel: ");
            Serial.println(ay);
            
        } else {
            // Normal Driving
            error = 0; integral = 0; lastError = 0;
            currentInterval = 5000; 
            digitalWrite(STATUS_LED, LOW); // Keep OFF when active but no slip
        }
    } else {
        // System Deactivated
        digitalWrite(STATUS_LED, HIGH); // Solid ON
        currentInterval = 5000;
    }
}

void checkButton() {
    bool currentButtonState = digitalRead(BUTTON_PIN);
    if (lastButtonState == HIGH && currentButtonState == LOW) {
        tcsEnabled = !tcsEnabled;
        if (tcsEnabled) Serial.println("SYSTEM STATUS: ACTIVE");
        else Serial.println("SYSTEM STATUS: DEACTIVATED");
        delay(200); 
    }
    lastButtonState = currentButtonState;
}

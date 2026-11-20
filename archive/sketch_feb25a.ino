#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <SD.h>

// --- PIN CONFIGURATION ---
#define THROTTLE_IN_PIN  2   
#define ESC_OUT_PIN      6   
#define BUTTON_PIN       5   
#define SD_CHIP_SELECT   10

// --- TUNING ---
#define YAW_THRESHOLD    35.0  
float Kp = 1.8, Ki = 0.05, Kd = 0.1;

MPU6050 mpu;
Servo esc;

volatile long throttlePulseStart = 0;
volatile int throttleIn = 1500; 
bool isArmed = false;
bool mpuOnline = false;
float integral, lastError;
unsigned long lastTime;

void readThrottle() {
  if (digitalRead(THROTTLE_IN_PIN) == HIGH) throttlePulseStart = micros();
  else {
    int pulse = micros() - throttlePulseStart;
    if (pulse > 900 && pulse < 2100) throttleIn = pulse;
  }
}

void setup() {
  // Start Serial but DO NOT wait for it
  Serial.begin(115200);
  
  Wire.begin();
  Wire.setClock(400000);
  
  mpu.initialize();
  if (mpu.testConnection()) {
    mpuOnline = true;
    if (Serial) Serial.println("TCS: Sensor Online");
  }

  pinMode(THROTTLE_IN_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_IN_PIN), readThrottle, CHANGE);

  esc.attach(ESC_OUT_PIN);
  esc.writeMicroseconds(1500); 

  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  // 1. Arming Logic
  static bool lastBtn = HIGH;
  bool currentBtn = digitalRead(BUTTON_PIN);
  if (currentBtn == LOW && lastBtn == HIGH) {
    isArmed = !isArmed;
    integral = 0;
    if (Serial) Serial.println(isArmed ? "SYSTEM ARMED" : "SYSTEM DISARMED");
    delay(200);
  }
  lastBtn = currentBtn;

  // 2. Sensor Processing
  int16_t gz = mpu.getRotationZ();
  float yawRate = abs(gz / 131.0);
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.01;

  int throttleOut = throttleIn; 

  // 3. TCS Logic (Only if Armed, Sensor is working, and Moving Forward)
  if (isArmed && mpuOnline && throttleIn > 1530 && yawRate > YAW_THRESHOLD) {
    float error = yawRate - YAW_THRESHOLD;
    integral += error * dt;
    float derivative = (error - lastError) / dt;
    
    int reduction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    throttleOut = throttleIn - reduction;
    if (throttleOut < 1500) throttleOut = 1500; 
    
    lastError = error;

    // Smart Serial: Only prints if USB is connected
    if (Serial && (currentTime % 50 == 0)) {
      Serial.print("TCS ACTIVE - Reduction: "); 
      Serial.println(reduction);
    }
  } else {
    integral = 0;
    lastError = 0;
  }

  // 4. Signal Output
  esc.writeMicroseconds(throttleOut);
  lastTime = currentTime;

  // 5. Periodic Health Check (Silent)
  if (currentTime % 1000 == 0) {
    if (!mpu.testConnection()) {
      mpuOnline = false;
      if (Serial) Serial.println("WARNING: MPU6050 DISCONNECTED");
    }
  }

  delay(5);
}

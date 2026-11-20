#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include "MPU6050.h"

// ==========================================
//              USER CONFIGURATION
// ==========================================
#define YAW_SENSITIVITY    45.0   
#define DEMO_THROTTLE      2000   
#define SD_CHIP_SELECT     10     
#define BUTTON_PIN         5      
#define LOG_FILENAME       "/tcs_demo.csv"
#define WARNING_TIMEOUT    10000  

const int LED_R = LED_RED;
const int LED_G = LED_GREEN;
const int LED_B = LED_BLUE;

float Kp = 1.8; float Ki = 0.05; float Kd = 0.1;
// ==========================================

MPU6050 mpu;
bool sdInitialized = false;
bool isArmed = false;
bool lastButtonState = HIGH;

float error, lastError, integral;
unsigned long lastTime, lastLogTime, lastSDCheck = 0;
unsigned long sdMissingTime = 0; 

void allOff() {
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_R, HIGH);
  digitalWrite(LED_G, HIGH);
  digitalWrite(LED_B, HIGH);
}

void setRGB(bool r, bool g, bool b) {
  allOff(); 
  if(r) digitalWrite(LED_R, LOW);
  if(g) digitalWrite(LED_G, LOW);
  if(b) digitalWrite(LED_B, LOW);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  
  allOff();

  while (!Serial) {
    setRGB(0, 0, 1);
    delay(200);
    allOff();
    delay(200);
  }

  Serial.println("--- TCS INITIALIZING ---");
  Wire.begin(); 
  Wire.setClock(400000); 
  mpu.initialize();

  if(mpu.testConnection()) Serial.println("DEBUG: MPU6050 Connection Successful");
  else Serial.println("DEBUG: MPU6050 Connection FAILED");

  tryMountSD();
  if (!sdInitialized) sdMissingTime = millis(); 
  
  lastTime = millis();
}

void loop() {
  // 1. SD BACKGROUND CHECK
  if (millis() - lastSDCheck > 2000) {
    checkSDStatus(); 
    lastSDCheck = millis();
  }

  // 2. BUTTON LOGIC (Includes Flush Notification)
  bool currentButtonState = digitalRead(BUTTON_PIN);
  if (currentButtonState == LOW && lastButtonState == HIGH) {
    delay(50);
    isArmed = !isArmed;
    allOff();
    integral = 0; 
    
    if (!isArmed) {
      Serial.println("--- SYSTEM DEACTIVATED ---");
      if (sdInitialized) {
        Serial.println("DEBUG: SD Data Synced. Safe to remove card.");
      }
    } else {
      Serial.println("--- SYSTEM ARMED ---");
      Serial.println("YawRate:0,Throttle:2000,Correction:0,Threshold:45");
    }
  }
  lastButtonState = currentButtonState;

  // --- SENSOR DATA ---
  int16_t gz = mpu.getRotationZ();
  float yawRate = abs(gz / 131.0);
  
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.01;

  float pidAdj = 0;
  int out = DEMO_THROTTLE;

  // --- LOGIC & LED STATES ---
  if (!sdInitialized && (millis() - sdMissingTime < WARNING_TIMEOUT)) {
    float pulse = (sin(millis() / 800.0) * 100) + 155; 
    analogWrite(LED_R, 255 - (int)pulse);         
    analogWrite(LED_G, 255 - ((int)pulse / 10)); 
    digitalWrite(LED_B, HIGH);                    
  } 
  else if (!isArmed) {
    setRGB(1, 0, 0);
  } 
  else {
    if (yawRate > YAW_SENSITIVITY) {
      setRGB(1, 1, 0);
      error = yawRate - YAW_SENSITIVITY; 
      integral += error * dt;
      float derivative = (error - lastError) / dt;
      pidAdj = (Kp * error) + (Ki * integral) + (Kd * derivative);
      out = DEMO_THROTTLE - (int)pidAdj;
      if (out < 1000) out = 1000;

      if (sdInitialized && (millis() - lastLogTime > 100)) {
        logData(yawRate, out);
        lastLogTime = millis();
      }
      lastError = error;
    } 
    else {
      allOff();
      integral = 0;
      lastError = 0;
    }
    lastTime = currentTime;

    Serial.print("YawRate:");    Serial.print(yawRate);      Serial.print(",");
    Serial.print("Throttle:");   Serial.print(out);          Serial.print(",");
    Serial.print("Correction:"); Serial.print(abs(pidAdj));   Serial.print(",");
    Serial.print("Threshold:");  Serial.println(YAW_SENSITIVITY);
  }

  delay(10);
}

void checkSDStatus() {
  if (sdInitialized) {
    if (!SD.exists("/")) { 
      Serial.println("DEBUG: SD Card Not Detected/Ejected.");
      sdInitialized = false;
      sdMissingTime = millis(); 
      SD.end(); 
    }
  } else {
    tryMountSD(); 
  }
}

void tryMountSD() {
  if (SD.begin(SD_CHIP_SELECT)) {
    sdInitialized = true;
    allOff();
    Serial.println("DEBUG: SD Card Mounted.");
    File file = SD.open(LOG_FILENAME, FILE_APPEND);
    if (file && file.size() == 0) file.println("Time,Yaw,Throttle");
    file.close();
  }
}

void logData(float yaw, int throttle) {
  File dataFile = SD.open(LOG_FILENAME, FILE_APPEND);
  if (dataFile) {
    dataFile.printf("%lu,%.2f,%d\n", millis(), yaw, throttle);
    dataFile.close();
  } else {
    sdInitialized = false;
    sdMissingTime = millis(); 
  }
}

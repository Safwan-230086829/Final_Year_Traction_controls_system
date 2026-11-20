// ============================================================================
//  PROJECT: Traction Control System For RC car applications
//  VERSION: 12.0 (RC Toggle + Fail-Safe Active)
//
//  DISSERTATION LOGIC:
//  - Uses PWM Pulse Width Decoding on D5 instead of a physical switch.
//  - FAIL-SAFE: If RC signal is lost (0us), TC defaults to ENABLED (Safety).
//  - CH3 PWM > 1500: TC ENABLED.
//  - CH3 PWM < 1500: TC DISABLED (Blue LED Warning).
// ============================================================================

#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SPI.h>
#include <SD.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// --- PIN DEFINITIONS ---
#define RX_PIN     D2
#define CH3_PIN    D5    // RC Receiver Channel 3 Input
#define ESC_PIN    D3
#define SD_CS      D10
#define HALL_FL    D7
#define HALL_FR    D6
#define HALL_RL    D9
#define HALL_RR    D8

// --- SYSTEM CONSTANTS ---
#define MAGNETS          2
#define GEAR_RATIO       1.0f
#define WHEEL_CIRC_M     0.2765f
#define SLIP_THRESHOLD   25.0f
#define IMU_DEADZONE     0.20f
#define MAX_PITCH_DEG    45.0f
#define MAX_PWM_CHANGE   15

// --- GLOBAL STATE VARIABLES ---
bool tcMasterEnable = true;
volatile bool requestNewFile = false;

// --- PID CONTROLLER CLASS ---
class PIDController {
  public:
    float kp, ki, kd;
    float integral = 0, lastError = 0;
    float pTerm = 0, iTerm = 0, dTerm = 0;
    bool firstRun = true;

    PIDController(float p, float i, float d) : kp(p), ki(i), kd(d) {}
    void reset() { integral = 0; lastError = 0; pTerm = 0; iTerm = 0; dTerm = 0; firstRun = true; }

    float compute(float input, float dt) {
      float error = 0.30f - input;
      if (error < 0) error = 0;
      pTerm = kp * error;
      integral = constrain(integral + (error * dt), 0.0f, 1.0f);
      iTerm = ki * integral;
      float delta = (firstRun) ? 0 : (input - lastError) / dt;
      dTerm = kd * delta;
      lastError = input;
      firstRun = false;
      return constrain(pTerm + iTerm - dTerm, 0.0f, 0.8f);
    }
};

PIDController tcPID(0.40f, 0.10f, 0.02f);

struct LogEntry {
  unsigned long ms;
  uint32_t loopUs;
  int16_t rxIn, escOut, ch3In;
  float ax, gz, pitch;
  float wFL, wFR, wRL, wRR;
  float gKmh, slip;
  bool tcActive, flipTrigger;
};

QueueHandle_t logQueue;
Servo esc;
MPU6050 mpu;
float axOffset = 0;

// --- INTERRUPTS ---
volatile unsigned long wheelDelta[4] = {0,0,0,0};
volatile unsigned long lastPulseTime[4] = {0,0,0,0};
volatile long rxPulseWidth = 1500;
volatile long ch3PulseWidth = 0; // Initialize at 0 to detect signal loss
volatile unsigned long rxRiseTime = 0;
volatile unsigned long ch3RiseTime = 0;

void IRAM_ATTR handlePulse(int i) {
  unsigned long now = micros();
  unsigned long diff = now - lastPulseTime[i];
  if (diff > 1000) { wheelDelta[i] = diff; lastPulseTime[i] = now; }
}
void IRAM_ATTR countFL() { handlePulse(0); }
void IRAM_ATTR countFR() { handlePulse(1); }
void IRAM_ATTR countRL() { handlePulse(2); }
void IRAM_ATTR countRR() { handlePulse(3); }

void IRAM_ATTR rxISR() {
  if (digitalRead(RX_PIN)) rxRiseTime = micros();
  else {
    unsigned long w = micros() - rxRiseTime;
    if (w >= 800 && w <= 2200) rxPulseWidth = (long)w;
  }
}

void IRAM_ATTR ch3ISR() {
  if (digitalRead(CH3_PIN)) ch3RiseTime = micros();
  else {
    unsigned long w = micros() - ch3RiseTime;
    if (w >= 800 && w <= 2200) ch3PulseWidth = (long)w;
  }
}

void setInterventionLED(bool active) {
  if (active) {
    analogWrite(LED_RED, 0);
    analogWrite(LED_GREEN, 180);
  } else {
    analogWrite(LED_RED, 255);
    analogWrite(LED_GREEN, 255);
  }
}

// --- LOGGING TASK (CORE 0) ---
void loggingTask(void* p) {
  LogEntry e;
  String buffer = "";
  bool sdOk = SD.begin(SD_CS);
  String fileName = "";
  int fileCount = 0;

  auto createNewFile = [&]() {
    String prefix = tcMasterEnable ? "/TractionON_" : "/TractionOFF_";
    while (SD.exists(prefix + String(fileCount) + ".csv")) fileCount++;
    fileName = prefix + String(fileCount) + ".csv";
    File f = SD.open(fileName, FILE_WRITE);
    if (f) {
      f.println("ms,loopUs,Rx,ESC,CH3,AccX,YawZ,Pitch,FL_Kmh,FR_Kmh,RL_Kmh,RR_Kmh,G_Kmh,Slip,TC_Active,Flip");
      f.close();
    }
  };

  if (sdOk) createNewFile();

  while (true) {
    if (requestNewFile && sdOk) { createNewFile(); requestNewFile = false; }
    if (xQueueReceive(logQueue, &e, pdMS_TO_TICKS(10))) {
      char line[220];
      snprintf(line, sizeof(line), "%lu,%u,%d,%d,%d,%.3f,%.2f,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.1f,%d,%d\n",
                e.ms, e.loopUs, e.rxIn, e.escOut, e.ch3In, e.ax, e.gz, e.pitch, e.wFL, e.wFR, e.wRL, e.wRR, e.gKmh, e.slip, e.tcActive, e.flipTrigger);
      buffer += line;
      if (sdOk && buffer.length() > 512) {
        File f = SD.open(fileName, FILE_APPEND);
        if (f) { f.print(buffer); f.close(); buffer = ""; }
      }
    }
    vTaskDelay(1);
  }
}

// --- CONTROL TASK (CORE 1) ---
void controlTask(void* p) {
  unsigned long lastLoop = millis();
  float groundKmh = 0;
  int lastEscOut = 1500;

  while (true) {
    unsigned long startMicros = micros();
    unsigned long now = millis();

    // FAIL-SAFE TOGGLE LOGIC
    noInterrupts(); long c3In = ch3PulseWidth; interrupts();

    // Logic: If signal is 0 (No receiver) OR signal is High (>1500), enable TC.
    // If signal is specifically Low (1000-1500), disable TC.
    bool ch3State = (c3In < 800 || c3In > 1500);

    if (ch3State != tcMasterEnable) {
        tcMasterEnable = ch3State;
        requestNewFile = true;
    }
    digitalWrite(LED_BLUE, tcMasterEnable ? HIGH : LOW);



    if (now - lastLoop >= 10) {
      float dt = (now - lastLoop) / 1000.0f;
      lastLoop = now;

      float wheelSpeeds[4];
      unsigned long nowU = micros();
      for(int i=0; i<4; i++) {
        noInterrupts(); unsigned long d = wheelDelta[i]; unsigned long lp = lastPulseTime[i]; interrupts();
        float rpm = (nowU - lp > 100000) ? 0 : (60000000.0f / d) / MAGNETS;
        wheelSpeeds[i] = (rpm / GEAR_RATIO / 60.0f) * WHEEL_CIRC_M * 3.6f;
      }
      float avgWheelKmh = (wheelSpeeds[0] + wheelSpeeds[1] + wheelSpeeds[2] + wheelSpeeds[3]) / 4.0f;

      int16_t rax, ray, raz, rgx, rgy, rgz;
      mpu.getMotion6(&rax, &ray, &raz, &rgx, &rgy, &rgz);
      float axG = ((rax / 16384.0f) - axOffset) * -1.0f;
      float yawRate = rgz / 131.0f;
      float pitchAngle = atan2((float)rax, (float)raz) * 180.0f / M_PI;

      noInterrupts(); long rIn = rxPulseWidth; interrupts();

      float fAcc = (abs(axG) < IMU_DEADZONE) ? 0 : axG;
      if (rIn < 1520) groundKmh = 0;
      else groundKmh += (fAcc * 9.81f * dt) * 3.6f;
      groundKmh = constrain(groundKmh, 0.0f, avgWheelKmh);
      float slip = (avgWheelKmh > 5.0f) ? ((avgWheelKmh - groundKmh) / avgWheelKmh) * 100.0f : 0;

      bool flipTriggered = (abs(pitchAngle) > MAX_PITCH_DEG);
      bool isActivelyCorrecting = (tcMasterEnable && rIn > 1550 && (slip > SLIP_THRESHOLD || abs(yawRate) > 100.0f));

      setInterventionLED(isActivelyCorrecting);

      int targetFOut = (int)rIn;
      if (flipTriggered) {
        targetFOut = 1500;
      } else if (isActivelyCorrecting) {
        float reduction = tcPID.compute(axG, dt);
        targetFOut = 1500 + (int)((rIn - 1500) * (1.0f - reduction));
      } else {
        tcPID.reset();
      }

      int diff = targetFOut - lastEscOut;
      if (diff > MAX_PWM_CHANGE) targetFOut = lastEscOut + MAX_PWM_CHANGE;
      if (diff < -MAX_PWM_CHANGE) targetFOut = lastEscOut - MAX_PWM_CHANGE;
      lastEscOut = targetFOut;
      esc.writeMicroseconds(targetFOut);

      LogEntry le = {now, (uint32_t)(micros() - startMicros), (int16_t)rIn, (int16_t)targetFOut, (int16_t)c3In,
                     axG, yawRate, pitchAngle, wheelSpeeds[0], wheelSpeeds[1], wheelSpeeds[2], wheelSpeeds[3],
                     groundKmh, slip, isActivelyCorrecting, flipTriggered};
      xQueueSend(logQueue, &le, 0);
    }
    vTaskDelay(1);
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  setCpuFrequencyMhz(160);
  //seting up interuprts
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_GREEN, HIGH);
  digitalWrite(LED_BLUE, HIGH);

  pinMode(CH3_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CH3_PIN), ch3ISR, CHANGE);
  //serial starting
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();

  pinMode(HALL_FL, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(HALL_FL), countFL, FALLING);
  pinMode(HALL_FR, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(HALL_FR), countFR, FALLING);
  pinMode(HALL_RL, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(HALL_RL), countRL, FALLING);
  pinMode(HALL_RR, INPUT_PULLUP); attachInterrupt(digitalPinToInterrupt(HALL_RR), countRR, FALLING);
  pinMode(RX_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(RX_PIN), rxISR, CHANGE);
  //setting  up the IMU
  float sumX = 0;
  for(int i=0; i<100; i++) { sumX += (mpu.getAccelerationX() / 16384.0f); delay(5); }
  axOffset = sumX / 100.0f;
  //triggering ESC to disable Safety switch if ESC does not an arming signal
  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(1500);

  //starting RTOS tasks which are pined to Core 1 (Control Task) which is used for the system task while Core 0(Logging and Background task)
  logQueue = xQueueCreate(64, sizeof(LogEntry));
  xTaskCreatePinnedToCore(loggingTask, "Log", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(controlTask, "Ctrl", 8192, NULL, 5, NULL, 1);
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }

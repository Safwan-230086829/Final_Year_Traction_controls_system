# Arduino Traction Control System Archive

This directory contains archived Arduino sketches for the ESP32 RC car traction control system.

## Contents

- `proto_rc_2.ino` — an early traction-control prototype using an MPU6050, button toggle, motor/status LEDs, and PID-based yaw intervention.
- `proto_rc_3_SD.ino` — an MPU6050 yaw-control prototype with RGB status indicators, button arming, PID correction, serial plotting, and SD-card data logging.
- `sketch_feb25a.ino` — an arming and PID-based yaw traction-control prototype with MPU6050 sensor monitoring, PWM throttle output, and optional serial status reporting.

## Usage

Open an individual sketch in the Arduino IDE, select the appropriate ESP32 board, install the required libraries, and verify the pin configuration before uploading.

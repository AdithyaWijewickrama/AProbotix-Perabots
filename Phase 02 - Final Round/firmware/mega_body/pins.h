// GuardianBot v3 - Arduino Mega 2560 pin map (the "body": all real-time I/O)
// Keep this file in sync with docs/wiring_and_pinmap.md
#pragma once

// ---- L298N (motor supply = 3S LiPo straight, 11.1-12.6 V; N20 12 V motors) ----
#define PIN_ENA        5    // left  PWM  (Timer3, ~3.9 kHz)
#define PIN_IN1       22
#define PIN_IN2       23
#define PIN_ENB        6    // right PWM  (Timer4, ~3.9 kHz)
#define PIN_IN3       24
#define PIN_IN4       25

// ---- N20 hall encoders (x4 quadrature, all four on external interrupts) ----
#define PIN_ENC_L_A    2    // INT4
#define PIN_ENC_L_B    3    // INT5
#define PIN_ENC_R_A   18    // INT3
#define PIN_ENC_R_B   19    // INT2

// ---- MPU6050 on the hardware I2C bus ----
//  SDA 20, SCL 21 (module has its own 3.3 V regulator and pull-ups)

// ---- link to the ESP32-S3 brain: Serial2 through the 4-ch level shifter ----
//  TX2 16 -> shifter HV1 -> LV1 -> ESP32-S3 GPIO18 (RX)
//  RX2 17 <- shifter HV2 <- LV2 <- ESP32-S3 GPIO17 (TX)
#define LINK_BAUD 250000UL   // 0 % baud error on a 16 MHz Mega

// ---- Sharp GP2Y0A21YK0F (10-80 cm) - analog, 10 uF across each sensor's 5 V/GND ----
#define PIN_IR_FRONT  A0
#define PIN_IR_FL     A1
#define PIN_IR_FR     A2
#define PIN_IR_LEFT   A3
#define PIN_IR_RIGHT  A4
#define PIN_VBAT      A5    // 20k / 10k divider from the switched battery (12.6 V -> 4.2 V)

// ---- 5-ch TCRT5000 line array (analog outputs, gs0 = right-most) ----
#define PIN_GS0       A8
#define PIN_GS1       A9
#define PIN_GS2       A10
#define PIN_GS3       A11
#define PIN_GS4       A12

// ---- 4 x HC-SR04: trig on plain outputs, echo on PCINT0 pins (non-blocking) ----
#define PIN_US_F_TRIG 30
#define PIN_US_L_TRIG 31
#define PIN_US_R_TRIG 32
#define PIN_US_B_TRIG 33
#define PIN_US_F_ECHO 10    // PB4 / PCINT4
#define PIN_US_L_ECHO 11    // PB5 / PCINT5
#define PIN_US_R_ECHO 12    // PB6 / PCINT6
#define PIN_US_B_ECHO 13    // PB7 / PCINT7

// ---- 2 x TCS3200 (GY-31). S0 -> 5 V and S1 -> GND on BOTH boards (20 % output scale).
//      S2/S3 are shared, OUT pins are counted with pin-change interrupts. ----
#define PIN_TCS_S2    34
#define PIN_TCS_S3    35
#define PIN_TCS_FLOOR_OUT 50  // PB3 / PCINT3
#define PIN_TCS_FRONT_OUT 51  // PB2 / PCINT2
#define PIN_TCS_FLOOR_LED 38  // module LED pin (HIGH = on)
#define PIN_TCS_FRONT_LED 39

// ---- gripper servos (Kpower M0170, powered from the 6 V buck) ----
#define PIN_SERVO_L    7
#define PIN_SERVO_R    8

// ---- user interface ----
#define PIN_BTN_START 46    // to GND, internal pull-up
#define PIN_BTN_CAL   47
#define PIN_LED_BLUE  48    // clan LEDs (330 R)
#define PIN_LED_GREEN 49
#define PIN_BUZZER     9    // active buzzer via NPN

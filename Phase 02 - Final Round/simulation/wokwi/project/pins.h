// GuardianBot v4.1 - Arduino Mega 2560 pin map (the "body": all real-time I/O)
// The Mega sits upside-down on the PCB mid deck (hardware/pcb_v4). Keep in sync with docs/GuardianBot_v4.1_build_guide.md
#pragma once
#include <stdint.h>

// ---- Pololu DRV8833 carrier (VIN = 2S LiPo, 7.4-8.4 V). PWM on the IN pins at 31 kHz.
#define PIN_AIN1       5    // left motor  (Timer3)
#define PIN_AIN2       6    //             (Timer4)
#define PIN_BIN1       7    // right motor (Timer4)
#define PIN_BIN2       8    //             (Timer4)
#define PIN_DRV_SLEEP  4    // nSLEEP: HIGH = driver on (LOW while the brain is silent)

// ---- N20 hall encoders (x4 quadrature, all four on external interrupts)
#define PIN_ENC_L_A    2    // INT4
#define PIN_ENC_L_B    3    // INT5
#define PIN_ENC_R_A   18    // INT3
#define PIN_ENC_R_B   19    // INT2

// ---- I2C bus (SDA 20, SCL 21): L3G4200D gyro 0x69, 8 x VL53L0X (re-addressed 0x30..0x37), OLED 0x3C

// ---- VL53L0X XSHUT lines. Open-drain use only: pinMode INPUT = sensor on (module pull-up),
//      OUTPUT LOW = sensor off. Never drive HIGH: the XSHUT pin is not 5 V tolerant.
//      order: front, fl, fr, lf, lr, rf, rr, back
static const uint8_t PIN_XSHUT[8] = {30, 31, 32, 33, 34, 35, 36, 37};

// ---- Pololu QTR-8RC on PORTA (D22..D29 = PA0..PA7; sensor 1 = right-most) + emitter control
#define PIN_QTR_LEDON 38

// ---- link to the ESP32-S3 brain: Serial2. TX2 16 -> 1k8/3k3 divider -> S3 GPIO41 (RX),
//      RX2 17 <- 2N7000 shifter <- S3 GPIO42 (TX). Both on the PCB (socket J18, silkscreen "ESP32CAM" from v4.0).
#define LINK_BAUD 250000UL

// ---- 2 x HC-SR04: trig on plain outputs, echo on PCINT0 pins (non-blocking)
#define PIN_US_F_TRIG 39
#define PIN_US_B_TRIG 40
#define PIN_US_F_ECHO 10    // PB4 / PCINT4
#define PIN_US_B_ECHO 11    // PB5 / PCINT5

// ---- 2 x TCS3200 (GY-31). S0 -> 5 V, S1 -> GND on both boards (20 % output scale). S2/S3 shared.
#define PIN_TCS_S2    41
#define PIN_TCS_S3    42
#define PIN_TCS_FLOOR_OUT 50  // PB3 / PCINT3
#define PIN_TCS_FRONT_OUT 51  // PB2 / PCINT2
#define PIN_TCS_FLOOR_LED 43
#define PIN_TCS_FRONT_LED 44

// ---- gripper servos (Kpower M0170 on the 5.5 V servo rail)
#define PIN_SERVO_L   45
#define PIN_SERVO_R   46

// ---- 1x4 membrane keypad (common -> GND, internal pull-ups)
//      K1 START   K2 start at the maze entrance   K3 Section 3 restart   K4 CAL (white balance)
static const uint8_t PIN_KEY[4] = {47, 48, 49, 53};

// ---- clan LEDs, buzzer, battery
#define PIN_LED_BLUE  A1
#define PIN_LED_GREEN A2
#define PIN_BUZZER     9    // active buzzer via NPN
#define PIN_VBAT      A0    // 20k / 10k divider from the switched battery (8.4 V -> 2.8 V)

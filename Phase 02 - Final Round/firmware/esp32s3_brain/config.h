// GuardianBot v3 - ESP32-S3 brain configuration
// Geometry matches hardware/cad/guardianbot_geometry.py and the Webots model.
#pragma once
#include <math.h>

// ---------------------------------------------------------------- link to the Mega
#define LINK_RX_PIN 18          // <- Mega TX2 (via level shifter)
#define LINK_TX_PIN 17          // -> Mega RX2 (via level shifter)
#define LINK_BAUD   250000UL
#ifndef STEP_S
#define STEP_S      0.020f      // one telemetry frame = one control step (50 Hz)
#endif

// ---------------------------------------------------------------- ESP32-CAM link and optics
#define CAM_RX_PIN 15           // <- ESP32-CAM U0TXD (GPIO1)
#define CAM_TX_PIN 16           // -> ESP32-CAM U0RXD (GPIO3)
#define CAM_BAUD   921600UL
#define CAM_W 48
#define CAM_H 32
static const float CAM_X = 0.062f, CAM_Z = 0.140f, CAM_PITCH = 0.50f, CAM_FOV = 0.90f;
static const uint8_t CAM_LED = 0;          // ESP32-CAM flash LED PWM during the run (0 = off)

// ---------------------------------------------------------------- drive
static const float R_WHEEL    = 0.0215f;   // 43 mm wheels
static const float HALF_TRACK = 0.062f;    // 124 mm track
static const float ENC_CPR    = 2800.0f;   // ticks per wheel revolution (must match mega_body)
static const float V_MAX      = 0.30f;     // m/s cruising (N20 300 rpm tops out ~0.6 m/s)
static const float ACC        = 1.6f;      // m/s^2 used for the stopping profile
static const float WHEEL_ACC  = 2.5f;      // m/s^2 wheel slew limit (no wheel spin on MDF/vinyl)
static const float W_RUN      = 0.15f;     // rad/s max steering while cruising in the maze
static const float WHEEL_VMAX = 0.40f;     // m/s per-wheel cap
static const float TURN_WMAX  = 4.0f;      // rad/s spin-in-place
static const float TURN_ALPHA = 25.0f;     // rad/s^2 spin deceleration profile

// ---------------------------------------------------------------- sensors (robot frame, faces)
// index: 0 us_front 1 us_left 2 us_right 3 us_back 4 ir_front 5 ir_fl 6 ir_fr 7 ir_left 8 ir_right
#define N_SENS 9
struct SensDef { float x, y, a, maxr; bool sonar; };
static const SensDef SENS[N_SENS] = {
  {0.075f, 0.0f, 0.0f, 2.50f, true},
  {0.0f, 0.075f, (float)M_PI_2, 2.50f, true},
  {0.0f, -0.075f, -(float)M_PI_2, 2.50f, true},
  {-0.075f, 0.0f, (float)M_PI, 2.50f, true},
  {0.040f, 0.0f, 0.0f, 0.80f, false},
  {0.0389f, 0.0389f, (float)M_PI_4, 0.80f, false},
  {0.0389f, -0.0389f, -(float)M_PI_4, 0.80f, false},
  {-0.004f, 0.0165f, (float)M_PI_2, 0.80f, false},
  {-0.004f, -0.0165f, -(float)M_PI_2, 0.80f, false},
};
enum { US_FRONT = 0, US_LEFT, US_RIGHT, US_BACK, IR_FRONT, IR_FL, IR_FR, IR_LEFT, IR_RIGHT };
static const float IR_MIN = 0.101f;        // GP2Y0A21 blind zone (reads 100 mm)
static const float IR_MAP_MAX = 0.50f;
static const float US_MAP_MAX = 0.40f;
static const float US_STOP = 0.035f;       // front bumper distance (HC-SR04 is reliable from ~3 cm)
static const float TCS_FLOOR_X = 0.012f, TCS_FLOOR_Y = 0.030f;

#define N_GS 5
static const float LINE_X = 0.045f;        // line array ahead of the axle
static const float GS_PITCH = 0.012f;
#ifndef GS_INVERT
#define GS_INVERT true          // TCRT5000 AO rises on black: invert so white = high
#endif
#ifndef TCS_WHITE_DEFAULT
#define TCS_WHITE_DEFAULT 300.0f  // counts per 6 ms window on white paper (CAL button measures it)
#endif

// ---------------------------------------------------------------- gripper (servo degrees)
static const uint8_t SERVO_L_OPEN = 30, SERVO_L_CLOSED = 90;    // tune on the bench (guardian test)
static const uint8_t SERVO_R_OPEN = 150, SERVO_R_CLOSED = 90;
static const float POCKET_OFF = 0.033f;    // held piece centre behind the axle

// ---------------------------------------------------------------- arena (world frame, metres)
static const float START_X = 2.315f, START_Y = 0.125f;
static const float CELL = 0.25f, MX0 = 0.095f, MY0 = 1.33f;
#define COLS 9
#define ROWS 4
static const int END_C = 8, END_R = 3;

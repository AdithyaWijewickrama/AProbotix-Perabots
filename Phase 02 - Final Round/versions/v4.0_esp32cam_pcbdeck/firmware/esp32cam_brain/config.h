// GuardianBot v4.0 - ESP32-CAM brain configuration
// Geometry matches hardware/cad/guardianbot_geometry.py and the Webots model.
#pragma once
#include <math.h>
#include <stdint.h>

// ---------------------------------------------------------------- link to the Mega (UART2)
#define LINK_RX_PIN 15          // <- Mega TX2 (1k8/3k3 divider on the PCB)
#define LINK_TX_PIN 14          // -> Mega RX2 (2N7000 shifter on the PCB)
#define LINK_BAUD   250000UL
#ifndef STEP_S
#define STEP_S      0.020f      // one telemetry frame = one control step (50 Hz)
#endif

// ---------------------------------------------------------------- camera (OV2640 on the ESP32-CAM)
#define CAM_W 48
#define CAM_H 32
static const float CAM_X = 0.062f, CAM_Z = 0.140f, CAM_PITCH = 0.50f, CAM_FOV = 0.90f;
static const float LENS_HFOV = 0.94f;      // OV2640 stock lens, horizontal: the frame is cropped to CAM_FOV
static const uint8_t CAM_LED = 0;          // flash LED PWM during the run (0 = off, ~20 in a dark hall)

// ---------------------------------------------------------------- drive (N20 6 V ~300 rpm on 2S via DRV8833)
static const float R_WHEEL    = 0.0215f;   // 43 mm wheels
static const float HALF_TRACK = 0.062f;    // 124 mm track
static const float ENC_CPR    = 2800.0f;   // ticks per wheel revolution (must match mega_body)
static const float V_MAX      = 0.30f;     // m/s cruising (motors top out ~0.6 m/s at the 80 % PWM cap)
static const float ACC        = 1.6f;      // m/s^2 used for the stopping profile
static const float WHEEL_ACC  = 2.5f;      // m/s^2 wheel slew limit
static const float W_RUN      = 0.15f;     // rad/s max steering while cruising in the maze
static const float WHEEL_VMAX = 0.40f;     // m/s per-wheel cap
static const float TURN_WMAX  = 4.0f;      // rad/s spin-in-place
static const float TURN_ALPHA = 25.0f;     // rad/s^2 spin deceleration profile

// ---------------------------------------------------------------- range sensors (robot frame, faces)
// index: 0 us_front 1 us_back | 2 tof_front 3 tof_fl 4 tof_fr 5 tof_lf 6 tof_lr 7 tof_rf 8 tof_rr 9 tof_back
#define N_SENS 10
struct SensDef { float x, y, a, maxr; bool sonar; };
static const SensDef SENS[N_SENS] = {
  {0.075f, 0.0f, 0.0f, 2.50f, true},
  {-0.075f, 0.0f, (float)M_PI, 2.50f, true},
  {0.068f, 0.0f, 0.0f, 1.20f, false},
  {0.050f, 0.046f, (float)M_PI_4, 1.20f, false},
  {0.050f, -0.046f, -(float)M_PI_4, 1.20f, false},
  {0.030f, 0.064f, (float)M_PI_2, 1.20f, false},
  {-0.036f, 0.064f, (float)M_PI_2, 1.20f, false},
  {0.030f, -0.064f, -(float)M_PI_2, 1.20f, false},
  {-0.036f, -0.064f, -(float)M_PI_2, 1.20f, false},
  {-0.066f, 0.030f, (float)M_PI, 1.20f, false},
};
enum { US_FRONT = 0, US_BACK, TOF_FRONT, TOF_FL, TOF_FR, TOF_LF, TOF_LR, TOF_RF, TOF_RR, TOF_BACK };
static const float TOF_MIN = 0.031f;       // VL53L0X reads 30 mm for anything nearer
static const float TOF_MAP_MAX = 0.80f;    // map only the first 80 cm (noise grows with range)
static const float US_MAP_MAX = 0.40f;
static const float US_STOP = 0.035f;       // front bumper distances
static const float TOF_STOP = 0.036f;
static const float SIDE_PAIR_DX = 0.066f;  // tof_lf / tof_lr are 66 mm apart (wall-angle fix)
static const float TCS_FLOOR_X = 0.012f, TCS_FLOOR_Y = 0.030f;

// ---------------------------------------------------------------- line array (Pololu QTR-8RC)
#define N_GS 8
static const float LINE_X = 0.045f;        // line array ahead of the axle
static const float GS_PITCH = 0.009525f;
static const float QTR_TIMEOUT_US = 2500.0f;   // decay time: white short, black long -> GSV = 2500 - t
#ifndef TCS_WHITE_DEFAULT
#define TCS_WHITE_DEFAULT 300.0f  // counts per 6 ms window on white paper (K4 measures it)
#endif

// ---------------------------------------------------------------- gripper (servo degrees)
static const uint8_t SERVO_L_OPEN = 30, SERVO_L_CLOSED = 90;    // tune on the bench
static const uint8_t SERVO_R_OPEN = 150, SERVO_R_CLOSED = 90;
static const float POCKET_OFF = 0.033f;    // held piece centre behind the axle

// ---------------------------------------------------------------- arena (world frame, metres)
static const float START_X = 2.315f, START_Y = 0.125f;
static const float CELL = 0.25f, MX0 = 0.095f, MY0 = 1.33f;
#define COLS 9
#define ROWS 4
static const int END_C = 8, END_R = 3;

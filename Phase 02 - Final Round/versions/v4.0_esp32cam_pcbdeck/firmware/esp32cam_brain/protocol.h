// GuardianBot v4.0 - Mega <-> ESP32-CAM link protocol (identical copy in esp32cam_brain/)
// Frame: 0xA5 0x5A <type> <len> <payload...> <sum8>   (sum8 = 8-bit sum of type, len and payload)
// Both CPUs are little-endian, payloads are packed structs.
#pragma once
#include <stdint.h>

#define PKT_TELEMETRY 0x54   // 'T'  Mega -> ESP, every 20 ms (50 Hz)
#define PKT_COMMAND   0x43   // 'C'  ESP -> Mega, every control cycle (motors stop after 250 ms of silence)
#define PKT_TEXT      0x58   // 'X'  ESP -> Mega, 2 x 21 characters for the OLED (sent when it changes)

// keys
#define KEY_START   0x01     // K1
#define KEY_MAZE    0x02     // K2
#define KEY_S3      0x04     // K3
#define KEY_CAL     0x08     // K4
// flags
#define FLAG_LOWBAT 0x01
#define FLAG_GYRO_OK 0x02
#define FLAG_TCS_NEW 0x04    // a new full R,G,B,C set is in this packet
#define FLAG_TOF_OK  0x08    // all 8 VL53L0X answered at start-up

// command.out bits
#define OUT_LED_BLUE   0x01
#define OUT_LED_GREEN  0x02
#define OUT_TCS_FLOOR  0x04   // floor TCS3200 white LEDs on
#define OUT_TCS_FRONT  0x08   // front TCS3200 white LEDs on
#define OUT_BUZZER     0x10
// command.ctl bits
#define CTL_ZERO_ODOM  0x01   // zero encoders and yaw
#define CTL_GYRO_CAL   0x02   // re-measure gyro bias (robot must be still for 1 s)

struct __attribute__((packed)) Telemetry {
  uint32_t ms;         // Mega millis()
  int32_t  encL, encR; // accumulated encoder ticks (forward = +)
  float    yaw;        // rad, integrated L3G4200D gyro Z, CCW +, unwrapped
  float    gz;         // rad/s
  uint16_t us[2];      // mm: front, back   (2500 = no echo)
  uint16_t tof[8];     // mm: front, fl, fr, lf, lr, rf, rr, back (30..1200; 1200 = nothing in range)
  uint16_t line[8];    // QTR-8RC decay time in us (0..2500, white ~100-300, black ~2500), line[0] = right-most
  uint16_t tcsFloor[4];// R, G, B, Clear  (edges per 6 ms window)
  uint16_t tcsFront[4];
  uint16_t vbat_mv;
  uint8_t  keys;
  uint8_t  flags;
  uint8_t  seq;
};

struct __attribute__((packed)) Command {
  int16_t  vL, vR;     // wheel speed set-points, mm/s
  uint8_t  servoL, servoR;   // degrees 0..180
  uint8_t  out;        // OUT_* bits
  uint8_t  ctl;        // CTL_* bits (edge triggered by the brain)
};

struct __attribute__((packed)) TextMsg {
  char line[2][21];    // not NUL-terminated; padded with spaces
};

// GuardianBot v3 - Mega <-> ESP32-S3 link protocol (identical copy in esp32s3_brain/)
// Frame: 0xA5 0x5A <type> <len> <payload...> <sum8>   (sum8 = 8-bit sum of type, len and payload)
// Both CPUs are little-endian, payloads are packed structs.
#pragma once
#include <stdint.h>

#define PKT_TELEMETRY 0x54   // 'T'  Mega -> ESP, every 20 ms (50 Hz)
#define PKT_COMMAND   0x43   // 'C'  ESP -> Mega, every control cycle (motors stop after 250 ms of silence)

// buttons / flags bits
#define BTN_START   0x01
#define BTN_CAL     0x02
#define FLAG_LOWBAT 0x01
#define FLAG_GYRO_OK 0x02
#define FLAG_TCS_NEW 0x04    // a new full R,G,B,C set is in this packet

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
  float    yaw;        // rad, integrated MPU6050 gyro Z, CCW +, unwrapped
  float    gz;         // rad/s
  uint16_t us[4];      // mm: front, left, right, back   (2500 = no echo)
  uint16_t ir[5];      // mm: front, fl, fr, left, right (800 = nothing in range, 100 = closer than 10 cm)
  uint16_t gs[5];      // raw ADC 0..1023, gs0 = right-most
  uint16_t tcsFloor[4];// R, G, B, Clear  (edges per 6 ms window)
  uint16_t tcsFront[4];
  uint16_t vbat_mv;
  uint8_t  buttons;
  uint8_t  flags;
  uint8_t  seq;
};

struct __attribute__((packed)) Command {
  int16_t  vL, vR;     // wheel speed set-points, mm/s
  uint8_t  servoL, servoR;   // degrees 0..180
  uint8_t  out;        // OUT_* bits
  uint8_t  ctl;        // CTL_* bits (edge triggered by the brain)
};

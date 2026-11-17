// VL53L0X time-of-flight sensor (GY-VL53L0XV2 module) for Wokwi - GuardianBot v4
//
// Enough of the register map for the Pololu "VL53L0X" Arduino library: init() (model ID, SPAD info,
// reference calibration), setAddress(), setMeasurementTimingBudget(), startContinuous() and reading the
// result registers. XSHUT works like on the module (10 k pull-up): LOW = sensor off and reset to 0x29.
// The "distance" slider sets what the sensor sees; over 1200 mm it reports 8190 (no target).
#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REG_SYSRANGE_START 0x00
#define REG_INT_CLEAR 0x0B
#define REG_INT_STATUS 0x13
#define REG_RANGE_STATUS 0x14
#define REG_ADDRESS 0x8A
#define REG_MODEL_ID 0xC0

typedef struct {
  pin_t xshut;
  uint8_t addr, ptr, reg[256];
  bool first, cont, ready, powered;
  uint16_t latched;
  uint32_t a_dist, a_noise, seed;
  timer_t meas;
} chip_t;

static void reset(chip_t *c) {
  memset(c->reg, 0, sizeof(c->reg));
  c->addr = 0x29;
  c->reg[REG_MODEL_ID] = 0xEE;
  c->reg[0xC2] = 0x10;           // revision
  c->reg[0x01] = 0xFF;           // SYSTEM_SEQUENCE_CONFIG
  c->reg[0x50] = 0x06;           // PRE_RANGE_CONFIG_VCSEL_PERIOD (14 PCLK)
  c->reg[0x70] = 0x04;           // FINAL_RANGE_CONFIG_VCSEL_PERIOD (10 PCLK)
  c->reg[0x51] = 0x00; c->reg[0x52] = 0x96;
  c->reg[0x71] = 0x01; c->reg[0x72] = 0x2C;
  c->reg[0x46] = 0x0A;           // MSRC timeout
  c->reg[0x91] = 0x3C;           // stop variable
  c->reg[0x92] = 0x2C;           // SPAD count 44, non-aperture
  c->reg[0xF8] = 0x00; c->reg[0xF9] = 0x80;   // OSC_CALIBRATE_VAL
  c->reg[REG_RANGE_STATUS] = 0x0B << 3;
  c->cont = c->ready = false;
  c->ptr = 0;
  timer_stop(c->meas);
}

static uint16_t measure(chip_t *c) {
  uint32_t d = attr_read(c->a_dist);
  if (d == 0 || d > 1200) return 8190;
  uint32_t n = attr_read(c->a_noise);
  c->seed = c->seed * 1103515245u + 12345u;
  int jitter = n ? (int)((c->seed >> 16) % (2 * n + 1)) - (int)n : 0;
  int v = (int)d + jitter;
  return (uint16_t)(v < 20 ? 20 : v);
}

static uint8_t read_reg(chip_t *c, uint8_t r) {
  switch (r) {
    case REG_INT_STATUS: return c->ready ? 0x04 : 0x00;     // "new sample ready"
    case 0x83: return c->reg[0x83] | 0x10;                   // SPAD info handshake: never 0
    case REG_RANGE_STATUS + 10: c->latched = measure(c); return c->latched >> 8;
    case REG_RANGE_STATUS + 11: return c->latched & 0xFF;
    default: return c->reg[r];
  }
}

static void write_reg(chip_t *c, uint8_t r, uint8_t v) {
  c->reg[r] = v;
  if (r == REG_ADDRESS) c->addr = v & 0x7F;
  else if (r == REG_INT_CLEAR && (v & 1)) c->ready = false;
  else if (r == REG_SYSRANGE_START && c->reg[0xFF] == 0x00) {
    if (v & 0x02) { c->cont = true; c->ready = false; timer_start(c->meas, 33000, true); }
    else if (v & 0x01) {
      if (c->cont) { c->cont = false; timer_stop(c->meas); }
      c->ready = true;                                       // single shot / reference calibration: done at once
    }
    c->reg[REG_SYSRANGE_START] = v & ~0x01;
  }
}

static bool on_connect(void *u, uint32_t address, bool read) {
  chip_t *c = u;
  if (!c->powered || address != c->addr) return false;       // NACK: not us (or switched off by XSHUT)
  c->first = !read;
  return true;
}
static uint8_t on_read(void *u) { chip_t *c = u; return read_reg(c, c->ptr++); }
static bool on_write(void *u, uint8_t data) {
  chip_t *c = u;
  if (c->first) { c->ptr = data; c->first = false; }
  else write_reg(c, c->ptr++, data);
  return true;
}
static void on_meas(void *u) { ((chip_t *)u)->ready = true; }
static void on_xshut(void *u, pin_t pin, uint32_t value) {
  chip_t *c = u;
  if (value && !c->powered) reset(c);
  c->powered = value;
}

void chip_init(void) {
  chip_t *c = calloc(1, sizeof(chip_t));
  c->a_dist = attr_init("distance", 300);
  c->a_noise = attr_init("noise", 2);
  c->seed = 12345;
  timer_config_t tc = { .user_data = c, .callback = on_meas };
  c->meas = timer_init(&tc);
  c->xshut = pin_init("XSHUT", INPUT_PULLUP);                // the module has a pull-up on XSHUT
  pin_init("GPIO1", INPUT);
  reset(c);
  c->powered = pin_read(c->xshut);
  pin_watch_config_t wc = { .user_data = c, .edge = BOTH, .pin_change = on_xshut };
  pin_watch(c->xshut, &wc);
  i2c_config_t ic = { .user_data = c, .address = 0, .scl = pin_init("SCL", INPUT), .sda = pin_init("SDA", INPUT),
                      .connect = on_connect, .read = on_read, .write = on_write };
  i2c_init(&ic);                                             // address 0 = see every request (address can change)
}

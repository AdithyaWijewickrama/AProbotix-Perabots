// L3G4200D 3-axis gyro (GY-50 module, SDO high -> address 0x69) for Wokwi - GuardianBot v4
// WHO_AM_I = 0xD3, CTRL_REG1..5, OUT_X/Y/Z (X and Y read 0). Sub-address bit 7 = auto-increment.
// Full scale from CTRL_REG4 FS bits: 250 / 500 / 2000 dps = 8.75 / 17.5 / 70 mdps per digit.
#include "wokwi-api.h"
#include <stdlib.h>
#include <string.h>

typedef struct {
  uint8_t reg[64], ptr;
  bool first, inc;
  int16_t z;
  uint32_t a_rate, a_bias, seed;
} chip_t;

static int16_t sample(chip_t *c) {
  static const float mdps[4] = {8.75f, 17.5f, 70.0f, 70.0f};
  float s = mdps[(c->reg[0x23] >> 4) & 3] / 1000.0f;
  c->seed = c->seed * 1103515245u + 12345u;
  int noise = (int)((c->seed >> 16) % 7) - 3;
  float v = (float)(int32_t)attr_read(c->a_rate) / s + (float)(int32_t)attr_read(c->a_bias) + noise;
  if (v > 32767) v = 32767;
  if (v < -32768) v = -32768;
  return (int16_t)v;
}
static uint8_t read_reg(chip_t *c, uint8_t r) {
  bool on = c->reg[0x20] & 0x08;
  switch (r) {
    case 0x0F: return 0xD3;                       // WHO_AM_I
    case 0x27: return on ? 0x0F : 0x00;           // STATUS_REG: new data
    case 0x28: case 0x29: case 0x2A: case 0x2B: return 0;
    case 0x2C: c->z = on ? sample(c) : 0; return c->z & 0xFF;
    case 0x2D: return (uint8_t)((uint16_t)c->z >> 8);
    default: return c->reg[r & 0x3F];
  }
}
static bool on_connect(void *u, uint32_t address, bool read) { ((chip_t *)u)->first = !read; return true; }
static uint8_t on_read(void *u) {
  chip_t *c = u;
  uint8_t v = read_reg(c, c->ptr);
  if (c->inc) c->ptr = (c->ptr + 1) & 0x3F;
  return v;
}
static bool on_write(void *u, uint8_t d) {
  chip_t *c = u;
  if (c->first) { c->ptr = d & 0x3F; c->inc = d & 0x80; c->first = false; return true; }
  if (c->ptr >= 0x20 && c->ptr <= 0x24) c->reg[c->ptr] = d;
  if (c->inc) c->ptr = (c->ptr + 1) & 0x3F;
  return true;
}

void chip_init(void) {
  chip_t *c = calloc(1, sizeof(chip_t));
  c->reg[0x20] = 0x07;                            // power-down after reset
  c->a_rate = attr_init("rate", 0);
  c->a_bias = attr_init("bias", 8);
  c->seed = 777;
  pin_init("SDO", INPUT); pin_init("CS", INPUT);
  i2c_config_t ic = { .user_data = c, .address = 0x69, .scl = pin_init("SCL", INPUT), .sda = pin_init("SDA", INPUT),
                      .connect = on_connect, .read = on_read, .write = on_write };
  i2c_init(&ic);
}

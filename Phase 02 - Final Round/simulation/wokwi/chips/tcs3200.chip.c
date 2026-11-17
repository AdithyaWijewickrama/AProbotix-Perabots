// TCS3200 / GY-31 colour sensor for Wokwi - GuardianBot v4
// OUT is a 50 % square wave. Filter S2 S3: LL red, LH blue, HL clear, HH green. Scaling S0 S1:
// LL power-down (OUT low), LH 2 %, HL 20 % (as wired on the PCB deck), HH 100 %.
// At 20 %: f = 200 Hz + 80 Hz per colour step (white 255 -> ~20 kHz). LED pin LOW = only room light (x0.3).
#include "wokwi-api.h"
#include <stdlib.h>

typedef struct {
  pin_t s0, s1, s2, s3, out, led;
  uint32_t a_r, a_g, a_b;
  timer_t t;
  bool level;
} chip_t;

static double freq(chip_t *c) {
  bool s0 = pin_read(c->s0), s1 = pin_read(c->s1), s2 = pin_read(c->s2), s3 = pin_read(c->s3);
  if (!s0 && !s1) return 0;
  double r = attr_read(c->a_r), g = attr_read(c->a_g), b = attr_read(c->a_b);
  double v = !s2 && !s3 ? r : (!s2 && s3 ? b : (s2 && s3 ? g : (r + g + b) / 3.0 * 1.15));
  double f = 200.0 + 80.0 * v;
  if (!pin_read(c->led)) f *= 0.3;
  double scale = s0 && s1 ? 5.0 : (s0 ? 1.0 : 0.1);
  return f * scale;
}

static void tick(void *u) {
  chip_t *c = u;
  double f = freq(c);
  if (f <= 0) { c->level = false; pin_write(c->out, LOW); timer_start(c->t, 1000, false); return; }
  c->level = !c->level;
  pin_write(c->out, c->level);
  timer_start_ns(c->t, (uint64_t)(5e8 / f), false);
}

void chip_init(void) {
  chip_t *c = calloc(1, sizeof(chip_t));
  c->s0 = pin_init("S0", INPUT); c->s1 = pin_init("S1", INPUT);
  c->s2 = pin_init("S2", INPUT); c->s3 = pin_init("S3", INPUT);
  c->led = pin_init("LED", INPUT_PULLUP);                    // module LEDs are on unless the pin is pulled low
  c->out = pin_init("OUT", OUTPUT_LOW);
  c->a_r = attr_init("red", 200); c->a_g = attr_init("green", 200); c->a_b = attr_init("blue", 200);
  timer_config_t tc = { .user_data = c, .callback = tick };
  c->t = timer_init(&tc);
  timer_start(c->t, 100, false);
}

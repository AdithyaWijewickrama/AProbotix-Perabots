// Pololu QTR-8RC reflectance array for Wokwi - GuardianBot v4
// RC sensors: the MCU drives a line HIGH (charges the capacitor), then makes it an input; the line stays
// HIGH for a time that grows with how dark the floor is (white ~150 us, black ~2400 us), then falls.
// Sensor 1 is the right-most (robot frame), pitch 9.525 mm. LEDON LOW = emitters off = everything reads dark.
#include "wokwi-api.h"
#include <stdlib.h>
#include <math.h>

#define N 8
#define PITCH 9.525
#define T_WHITE 150.0
#define T_BLACK 2400.0
#define HOLD_LOW 300                   // us the line is held low after it falls, then released

struct chip;
typedef struct {
  struct chip *c;
  int i;
  pin_t pin;
  timer_t t;
  int phase;                           // 0 idle, 1 holding high, 2 holding low
} sensor_t;

typedef struct chip {
  sensor_t s[N];
  pin_t ledon;
  uint32_t a_pos, a_on, a_w;
} chip_t;

static double decay_us(chip_t *c, int i) {
  if (!pin_read(c->ledon)) return T_BLACK + 200;
  if (!attr_read(c->a_on)) return T_WHITE;
  double y = (i - 3.5) * PITCH;                  // sensor 1 (i = 0) is the right-most: y = -33.3 mm
  double pos = (double)(int32_t)attr_read(c->a_pos), half = attr_read(c->a_w) / 2.0;
  double spot = 3.0;                             // each sensor sees about a 6 mm spot
  double lo = fmax(y - spot, pos - half), hi = fmin(y + spot, pos + half);
  double cover = hi > lo ? (hi - lo) / (2 * spot) : 0.0;
  return T_WHITE + cover * (T_BLACK - T_WHITE);
}

static void on_timer(void *u) {
  sensor_t *s = u;
  if (s->phase == 1) {                           // capacitor discharged: line falls
    s->phase = 2;
    pin_mode(s->pin, OUTPUT_LOW);
    timer_start(s->t, HOLD_LOW, false);
  } else {                                       // let go, ready for the next read
    s->phase = 0;
    pin_mode(s->pin, INPUT);
  }
}

static void on_edge(void *u, pin_t pin, uint32_t value) {
  sensor_t *s = u;
  if (!value || s->phase) return;
  s->phase = 1;                                  // MCU is charging the line: keep it high after it lets go
  pin_mode(s->pin, OUTPUT_HIGH);
  timer_start(s->t, (uint32_t)(10 + decay_us(s->c, s->i)), false);
}

void chip_init(void) {
  chip_t *c = calloc(1, sizeof(chip_t));
  static const char *names[N] = {"1", "2", "3", "4", "5", "6", "7", "8"};
  c->ledon = pin_init("LEDON", INPUT_PULLUP);
  c->a_pos = attr_init("line_mm", 0);
  c->a_on = attr_init("line_on", 1);
  c->a_w = attr_init("width_mm", 20);
  for (int i = 0; i < N; i++) {
    sensor_t *s = &c->s[i];
    s->c = c; s->i = i;
    s->pin = pin_init(names[i], INPUT);
    timer_config_t tc = { .user_data = s, .callback = on_timer };
    s->t = timer_init(&tc);
    pin_watch_config_t wc = { .user_data = s, .edge = RISING, .pin_change = on_edge };
    pin_watch(s->pin, &wc);
  }
}

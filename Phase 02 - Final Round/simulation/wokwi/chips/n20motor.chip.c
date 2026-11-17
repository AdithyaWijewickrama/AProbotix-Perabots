// One DRV8833 H-bridge channel + N20 6 V 300 rpm encoder gear motor, for Wokwi - GuardianBot v4
// IN1/IN2 as on the DRV8833: PWM on one input = drive (fast decay), both HIGH = brake, both LOW = coast,
// SLP LOW = driver asleep (coast). The chip measures the PWM duty every 1 ms, runs a first-order motor
// model (300 rpm at 6 V, slower with load) and outputs the hall encoder in quadrature on ENC_A / ENC_B
// (forward: A leads B, the way mega_body.ino counts up). Prints the wheel speed when it changes.
#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct {
  pin_t in1, in2, slp, a, b;
  uint32_t a_vbat, a_load, a_cpr, a_rev;
  timer_t ctl, edge;
  uint64_t last_ns, hi1, hi2, win0;
  bool l1, l2, edge_on;
  double rpm, printed;
  int64_t pos;
  int id;
  uint32_t ticks;
} chip_t;

static int nchips = 0;

static void account(chip_t *c, uint64_t now) {
  uint64_t dt = now - c->last_ns;
  if (c->l1) c->hi1 += dt;
  if (c->l2) c->hi2 += dt;
  c->last_ns = now;
}
static void on_in(void *u, pin_t pin, uint32_t value) {
  chip_t *c = u;
  account(c, get_sim_nanos());
  if (pin == c->in1) c->l1 = value; else c->l2 = value;
}

static void write_enc(chip_t *c) {
  static const uint8_t A[4] = {0, 1, 1, 0}, B[4] = {0, 0, 1, 1};
  int k = (int)(c->pos & 3);
  pin_write(c->a, A[k]);
  pin_write(c->b, B[k]);
}

static double counts_per_s(chip_t *c) {
  double cps = c->rpm / 60.0 * attr_read(c->a_cpr);
  return attr_read(c->a_rev) ? -cps : cps;
}

static void on_edge(void *u) {
  chip_t *c = u;
  double cps = counts_per_s(c);
  if (fabs(cps) < 2.0) { c->edge_on = false; return; }
  c->pos += cps > 0 ? 1 : -1;
  write_enc(c);
  timer_start_ns(c->edge, (uint64_t)(1e9 / fabs(cps)), false);
}

static void on_ctl(void *u) {
  chip_t *c = u;
  uint64_t now = get_sim_nanos();
  account(c, now);
  double win = (double)(now - c->win0);
  double d1 = win > 0 ? c->hi1 / win : c->l1, d2 = win > 0 ? c->hi2 / win : c->l2;
  c->hi1 = c->hi2 = 0;
  c->win0 = now;
  bool awake = pin_read(c->slp);
  double vbat = attr_read(c->a_vbat) / 10.0, load = attr_read(c->a_load) / 100.0;
  double v = awake ? (d1 - d2) * vbat : 0.0;
  bool brake = awake && d1 > 0.98 && d2 > 0.98;
  double target = 300.0 * v / 6.0 * (1.0 - 0.6 * load);
  double tau = brake ? 0.010 : (awake ? 0.040 : 0.060);       // seconds (gearbox friction stops a coasting N20 fast)
  c->rpm += (target - c->rpm) * (0.001 / tau);
  if (fabs(c->rpm) < 0.5 && fabs(target) < 0.5) c->rpm = 0;
  if (!c->edge_on && fabs(counts_per_s(c)) >= 2.0) { c->edge_on = true; timer_start(c->edge, 50, false); }
  if (++c->ticks % 250 == 0 && fabs(c->rpm - c->printed) > 5) {
    printf("motor %d: %.0f rpm (%.2f m/s on a 43 mm wheel), duty %.2f/%.2f, %s\n", c->id, c->rpm,
           c->rpm / 60.0 * 3.14159 * 0.043, d1, d2, awake ? "awake" : "sleep");
    c->printed = c->rpm;
  }
}

void chip_init(void) {
  chip_t *c = calloc(1, sizeof(chip_t));
  c->id = ++nchips;
  c->in1 = pin_init("IN1", INPUT); c->in2 = pin_init("IN2", INPUT); c->slp = pin_init("SLP", INPUT_PULLDOWN);
  c->a = pin_init("ENC_A", OUTPUT_LOW); c->b = pin_init("ENC_B", OUTPUT_LOW);
  c->a_vbat = attr_init("vbat_dV", 74); c->a_load = attr_init("load", 20);
  c->a_cpr = attr_init("cpr", 2800); c->a_rev = attr_init("reverse", 0);
  c->l1 = pin_read(c->in1); c->l2 = pin_read(c->in2);
  c->last_ns = c->win0 = get_sim_nanos();
  pin_watch_config_t w1 = { .user_data = c, .edge = BOTH, .pin_change = on_in };
  pin_watch(c->in1, &w1);
  pin_watch(c->in2, &w1);
  timer_config_t t1 = { .user_data = c, .callback = on_ctl };
  c->ctl = timer_init(&t1);
  timer_config_t t2 = { .user_data = c, .callback = on_edge };
  c->edge = timer_init(&t2);
  timer_start(c->ctl, 1000, true);
}

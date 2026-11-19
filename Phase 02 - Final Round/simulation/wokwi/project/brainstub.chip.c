// "Brain stub" for Wokwi - stands in for the ESP32-S3 brain on the Mega's Serial2 link (250000 baud).
// Every 20 ms it sends a Command frame built from its sliders (wheel speeds, servos, LEDs ...), once a
// second an OLED text frame, and it decodes every Telemetry frame from the Mega and prints a summary
// twice a second. Frames and structs are exactly firmware/mega_body/protocol.h.
#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PKT_TELEMETRY 0x54
#define PKT_COMMAND 0x43
#define PKT_TEXT 0x58

typedef struct __attribute__((packed)) {
  uint32_t ms; int32_t encL, encR; float yaw, gz;
  uint16_t us[2], tof[8], line[8], tcsFloor[4], tcsFront[4], vbat_mv;
  uint8_t keys, flags, seq;
} telemetry_t;
typedef struct __attribute__((packed)) { int16_t vL, vR; uint8_t servoL, servoR, out, ctl; } command_t;

typedef struct {
  uart_dev_t uart;
  timer_t t;
  uint32_t a_en, a_vl, a_vr, a_sl, a_sr, a_out, a_zero;
  uint8_t q[256];
  int qn;
  bool busy;
  uint32_t ticks, frames, bad;
  int rx_state, rx_pos, rx_len, rx_type;
  uint8_t rx[128];
  telemetry_t last;
} chip_t;

static void pump(chip_t *c) {
  if (c->busy || c->qn == 0) return;
  static uint8_t out[256];
  memcpy(out, c->q, c->qn);
  int n = c->qn;
  c->qn = 0;
  c->busy = uart_write(c->uart, out, n);
}
static void on_done(void *u) { chip_t *c = u; c->busy = false; pump(c); }

static void send(chip_t *c, uint8_t type, const void *p, uint8_t len) {
  if (c->qn + len + 5 > (int)sizeof(c->q)) return;
  uint8_t *d = c->q + c->qn, s = type + len;
  d[0] = 0xA5; d[1] = 0x5A; d[2] = type; d[3] = len;
  memcpy(d + 4, p, len);
  for (int i = 0; i < len; i++) s += d[4 + i];
  d[4 + len] = s;
  c->qn += len + 5;
  pump(c);
}

static void print_tele(chip_t *c) {
  telemetry_t *t = &c->last;
  printf("T#%3u %6lums enc %ld/%ld yaw %+.1f us %u/%u tof", t->seq, (unsigned long)t->ms, (long)t->encL, (long)t->encR,
         t->yaw * 57.2958f, t->us[0], t->us[1]);
  for (int i = 0; i < 8; i++) printf(" %u", t->tof[i]);
  printf(" line");
  for (int i = 0; i < 8; i++) printf(" %u", t->line[i]);
  printf(" tcsF %u,%u,%u,%u tcsP %u,%u,%u,%u bat %umV keys %X flags %X (%lu frames, %lu bad)\n",
         t->tcsFloor[0], t->tcsFloor[1], t->tcsFloor[2], t->tcsFloor[3], t->tcsFront[0], t->tcsFront[1],
         t->tcsFront[2], t->tcsFront[3], t->vbat_mv, t->keys, t->flags, (unsigned long)c->frames, (unsigned long)c->bad);
}

static void on_rx(void *u, uint8_t b) {
  chip_t *c = u;
  switch (c->rx_state) {
    case 0: if (b == 0xA5) c->rx_state = 1; break;
    case 1: c->rx_state = b == 0x5A ? 2 : 0; break;
    case 2: c->rx_type = b; c->rx_state = 3; break;
    case 3: c->rx_len = b; c->rx_pos = 0; c->rx_state = b <= (int)sizeof(c->rx) ? (b ? 4 : 5) : 0; break;
    case 4: c->rx[c->rx_pos++] = b; if (c->rx_pos >= c->rx_len) c->rx_state = 5; break;
    case 5: {
      uint8_t s = c->rx_type + c->rx_len;
      for (int i = 0; i < c->rx_len; i++) s += c->rx[i];
      if (s == b && c->rx_type == PKT_TELEMETRY && c->rx_len == sizeof(telemetry_t)) {
        memcpy(&c->last, c->rx, sizeof(telemetry_t));
        c->frames++;
      } else c->bad++;
      c->rx_state = 0;
    } break;
  }
}

static void on_tick(void *u) {
  chip_t *c = u;
  c->ticks++;
  if (attr_read(c->a_en)) {
    command_t cmd = { (int16_t)(int32_t)attr_read(c->a_vl), (int16_t)(int32_t)attr_read(c->a_vr),
                      (uint8_t)attr_read(c->a_sl), (uint8_t)attr_read(c->a_sr), (uint8_t)attr_read(c->a_out),
                      (uint8_t)(attr_read(c->a_zero) ? 0x01 : 0x00) };
    send(c, PKT_COMMAND, &cmd, sizeof(cmd));
    if (c->ticks % 50 == 1) {
      char txt[2][21];
      char a[32], b[32];
      snprintf(a, sizeof(a), "BRAIN STUB  t=%lus", (unsigned long)(c->ticks / 50));
      snprintf(b, sizeof(b), "vL %d vR %d", cmd.vL, cmd.vR);
      memset(txt, ' ', sizeof(txt));
      memcpy(txt[0], a, strlen(a) > 21 ? 21 : strlen(a));
      memcpy(txt[1], b, strlen(b) > 21 ? 21 : strlen(b));
      send(c, PKT_TEXT, txt, sizeof(txt));
    }
  }
  if (c->ticks % 25 == 0 && c->frames) print_tele(c);
}

void chip_init(void) {
  chip_t *c = calloc(1, sizeof(chip_t));
  c->a_en = attr_init("enable", 1);
  c->a_vl = attr_init("v_left", 0); c->a_vr = attr_init("v_right", 0);
  c->a_sl = attr_init("servo_left", 90); c->a_sr = attr_init("servo_right", 90);
  c->a_out = attr_init("outputs", 0); c->a_zero = attr_init("zero", 0);
  uart_config_t uc = { .user_data = c, .rx = pin_init("RX", INPUT), .tx = pin_init("TX", INPUT_PULLUP),
                       .baud_rate = 250000, .rx_data = on_rx, .write_done = on_done };
  c->uart = uart_init(&uc);
  timer_config_t tc = { .user_data = c, .callback = on_tick };
  c->t = timer_init(&tc);
  timer_start(c->t, 20000, true);
  printf("brain stub: Command every 20 ms on 250000 baud, telemetry summary every 0.5 s\n");
}

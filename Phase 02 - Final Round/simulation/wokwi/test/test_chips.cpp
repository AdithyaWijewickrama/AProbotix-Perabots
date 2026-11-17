// test_chips.cpp - unit tests for the GuardianBot Wokwi custom chips (run by run_tests.sh).
// Each chip runs against mock_wokwi; the ToF test drives it with the REAL Pololu VL53L0X library,
// the same calls and order as tofInit() / tofPoll() in firmware/mega_body/mega_body.ino.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include "mock_wokwi.h"
#include "Wire.h"
#include "VL53L0X.h"

extern "C" { void vl53_init(); void gyro_init(); void tcs_init(); void qtr_init(); void motor_init(); void brain_init(); }
using namespace mock;
static int fails = 0;
#define CHECK(cond, ...) do { if (cond) printf("  ok    "); else { printf("  FAIL  "); fails++; } printf(__VA_ARGS__); printf("\n"); } while (0)

static void test_tof() {
  printf("VL53L0X x3 with the Pololu library (XSHUT start-up, new addresses, continuous mode)\n");
  const uint32_t dist[3] = {250, 1500, 640};
  int xs[3];
  for (int i = 0; i < 3; i++) {
    xs[i] = net("XS" + std::to_string(i));
    begin_chip(vl53_init, {{"XSHUT", xs[i]}}, {{"distance", dist[i]}, {"noise", 2}});
  }
  VL53L0X tof[3];
  for (int i = 0; i < 3; i++) mcu_drive(xs[i], 0);
  delay(10);
  bool ok[3];
  for (int i = 0; i < 3; i++) {
    mcu_drive(xs[i], -1);                       // release: module pull-up wakes the sensor
    delay(3);
    tof[i].setTimeout(40);
    ok[i] = tof[i].init();
    if (ok[i]) {
      tof[i].setAddress(0x30 + i);
      bool b = tof[i].setMeasurementTimingBudget(33000);
      tof[i].startContinuous(0);
      CHECK(b, "sensor %d: init ok, address 0x%02X, timing budget accepted", i, 0x30 + i);
    } else CHECK(false, "sensor %d: init failed", i);
  }
  uint16_t last[3] = {0, 0, 0};
  int fresh[3] = {0, 0, 0};
  uint32_t t0 = millis();
  while (millis() - t0 < 300) {                  // tofPoll(): never waits
    for (int i = 0; i < 3; i++) {
      if (!ok[i] || (tof[i].readReg(0x13) & 0x07) == 0) continue;
      last[i] = tof[i].readReg16Bit(0x14 + 10);
      tof[i].writeReg(0x0B, 0x01);
      fresh[i]++;
    }
    delay(1);
  }
  CHECK(abs((int)last[0] - 250) <= 2, "sensor 0 reads %u mm (set 250)", last[0]);
  CHECK(last[1] == 8190, "sensor 1 reads %u (set 1500 mm = out of range -> 8190)", last[1]);
  CHECK(abs((int)last[2] - 640) <= 2, "sensor 2 reads %u mm (set 640)", last[2]);
  CHECK(fresh[0] >= 8 && fresh[0] <= 10, "sensor 0 gave %d new readings in 300 ms (~30 Hz)", fresh[0]);
  set_attr(0, "distance", 90);
  delay(40);
  tof[0].readReg(0x13);
  uint16_t v = tof[0].readReg16Bit(0x14 + 10);
  CHECK(abs((int)v - 90) <= 2, "sensor 0 follows the slider: %u mm (set 90)", v);
  Wire.beginTransmission(0x29);
  CHECK(Wire.endTransmission() != 0, "no sensor left on the default address 0x29");
}

static void test_gyro() {
  printf("L3G4200D\n");
  int c = chip_count();
  begin_chip(gyro_init, {}, {{"rate", (uint32_t)90}, {"bias", 8}});
  Wire.beginTransmission(0x69); Wire.write(0x0F); Wire.endTransmission(false);
  Wire.requestFrom(0x69, 1);
  int who = Wire.read();
  CHECK(who == 0xD3, "WHO_AM_I = 0x%02X", who);
  auto wr = [](uint8_t r, uint8_t v) { Wire.beginTransmission(0x69); Wire.write(r); Wire.write(v); Wire.endTransmission(); };
  wr(0x20, 0x6F); wr(0x23, 0x10);
  Wire.beginTransmission(0x69); Wire.write(0x2C | 0x80); Wire.endTransmission(false);
  Wire.requestFrom(0x69, 2);
  int lo = Wire.read(), hi = Wire.read();
  int16_t raw = (int16_t)((hi << 8) | lo);
  float dps = (raw - 8) * 0.0175f;
  CHECK(fabs(dps - 90) < 0.2, "90 deg/s reads raw %d = %.2f deg/s after the bias", raw, dps);
  set_attr(c, "rate", (uint32_t)-200);
  Wire.beginTransmission(0x69); Wire.write(0x2C | 0x80); Wire.endTransmission(false);
  Wire.requestFrom(0x69, 2);
  lo = Wire.read(); hi = Wire.read();
  raw = (int16_t)((hi << 8) | lo);
  CHECK(fabs((raw - 8) * 0.0175f + 200) < 0.2, "-200 deg/s reads %.2f deg/s", (raw - 8) * 0.0175f);
}

static int count_rising(int n, double us) {
  int c = 0, prev = level(n);
  for (double t = 0; t < us; t += 2) { run_us(2); int l = level(n); if (l && !prev) c++; prev = l; }
  return c;
}
static void test_tcs() {
  printf("TCS3200 (S0 = 5 V, S1 = GND as on the PCB deck)\n");
  int s0 = net("S0"), s1 = net("S1"), s2 = net("S2"), s3 = net("S3"), out = net("TCS_OUT"), led = net("TCS_LED");
  begin_chip(tcs_init, {{"S0", s0}, {"S1", s1}, {"S2", s2}, {"S3", s3}, {"OUT", out}, {"LED", led}},
             {{"red", 220}, {"green", 40}, {"blue", 30}});
  mcu_drive(s0, 1); mcu_drive(s1, 0); mcu_drive(led, 1);
  const char *nm[4] = {"R", "G", "B", "C"};
  const int S2[4] = {0, 1, 0, 1}, S3[4] = {0, 1, 1, 0};
  int cnt[4];
  for (int k = 0; k < 4; k++) {
    mcu_drive(s2, S2[k]); mcu_drive(s3, S3[k]);
    run_us(1000);                                  // 1 ms settle, then count 6 ms (as mega_body does)
    cnt[k] = count_rising(out, 6000);
  }
  printf("        counts in 6 ms: R %d  G %d  B %d  C %d\n", cnt[0], cnt[1], cnt[2], cnt[3]);
  CHECK(abs(cnt[0] - (int)((200 + 80 * 220) * 0.006)) <= 2, "red count matches 200 + 80 x 220 Hz");
  CHECK(cnt[0] > 3 * cnt[1] && cnt[0] > 3 * cnt[2], "a red piece reads red");
  mcu_drive(led, 0);
  mcu_drive(s2, 0); mcu_drive(s3, 0); run_us(1000);
  int dark = count_rising(out, 6000);
  CHECK(abs(dark - (int)(cnt[0] * 0.3)) <= 2, "LEDs off -> %d (about 30 %%)", dark);
  (void)nm;
}

static void test_qtr() {
  printf("QTR-8RC (real qtrRead() timing: charge 10 us, release, time the fall, 2.5 ms limit)\n");
  int ledon = net("QTR_LED"), s[8];
  std::map<std::string, int> m = {{"LEDON", ledon}};
  for (int i = 0; i < 8; i++) { s[i] = net("QTR" + std::to_string(i + 1)); m[std::to_string(i + 1)] = s[i]; }
  int chip = chip_count();
  begin_chip(qtr_init, m, {{"line_mm", 0}, {"line_on", 1}, {"width_mm", 20}});
  auto read = [&](uint16_t *us) {
    mcu_drive(ledon, 1);
    for (int i = 0; i < 8; i++) mcu_drive(s[i], 1);
    run_us(10);
    for (int i = 0; i < 8; i++) mcu_drive(s[i], -1);
    for (int i = 0; i < 8; i++) us[i] = 2500;
    uint8_t pending = 0xFF;
    for (int t = 0; t < 2500 && pending; t++) {
      run_us(1);
      for (int i = 0; i < 8; i++) if ((pending & (1 << i)) && !level(s[i])) { us[i] = t; pending &= ~(1 << i); }
    }
    run_us(20000 - 2510);
  };
  uint16_t us[8];
  read(us);
  printf("        line centred: "); for (int i = 0; i < 8; i++) printf("%u ", us[i]); printf("\n");
  CHECK(us[3] > 2000 && us[4] > 2000 && us[0] < 300 && us[7] < 300, "middle sensors 4 and 5 see the black line, the ends see white");
  set_attr(chip, "line_mm", (uint32_t)-30);
  read(us);
  printf("        line 30 mm right: "); for (int i = 0; i < 8; i++) printf("%u ", us[i]); printf("\n");
  CHECK(us[0] > 1500 && us[7] < 300, "line on the right: sensor 1 (right-most) is dark");
  int c0 = conflicts();
  read(us);
  CHECK(conflicts() == c0, "no bus fights between the MCU and the sensor");
}

static int qdec_count = 0, qstate = 0;
static void test_motor() {
  printf("DRV8833 + N20 (31 kHz PWM like mega_body, QDEC like the encoder ISR)\n");
  int in1 = net("AIN1"), in2 = net("AIN2"), slp = net("SLP"), a = net("ENC_A"), b = net("ENC_B");
  begin_chip(motor_init, {{"IN1", in1}, {"IN2", in2}, {"SLP", slp}, {"ENC_A", a}, {"ENC_B", b}},
             {{"vbat_dV", 74}, {"load", 20}, {"cpr", 2800}, {"reverse", 0}});
  static const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};
  mcu_drive(slp, 1); mcu_drive(in2, 0);
  auto pwm = [&](double duty, double ms, bool count) {
    const double per = 32.0;                       // us (31.25 kHz)
    for (double t = 0; t < ms * 1000; t += per) {
      mcu_drive(in1, 1);
      for (double k = 0; k < per * duty; k += 4) { run_us(4); if (count) { qstate = ((qstate << 2) | (level(a) | level(b) << 1)) & 15; qdec_count += QDEC[qstate]; } }
      mcu_drive(in1, 0);
      for (double k = per * duty; k < per; k += 4) { run_us(4); if (count) { qstate = ((qstate << 2) | (level(a) | level(b) << 1)) & 15; qdec_count += QDEC[qstate]; } }
    }
  };
  pwm(0.6, 300, false);                            // spin up
  qstate = level(a) | level(b) << 1;
  qdec_count = 0;
  pwm(0.6, 500, true);
  double cps = qdec_count / 0.5, rpm = cps / 2800 * 60;
  double expect = 300 * (0.6 * 7.4 / 6) * (1 - 0.6 * 0.2);
  CHECK(fabs(rpm - expect) / expect < 0.05, "60 %% PWM: %.0f rpm, model says %.0f rpm; counts up going forward", rpm, expect);
  mcu_drive(slp, 0);
  pwm(0.6, 600, false);
  int before = qdec_count;
  pwm(0.6, 200, true);
  CHECK(abs(qdec_count - before) < 20, "SLP low (brain silent): the wheel stops");
}

static void test_brain() {
  printf("Brain stub (Command every 20 ms, decodes Telemetry)\n");
  begin_chip(brain_init, {}, {{"enable", 1}, {"v_left", 200}, {"v_right", (uint32_t)-150}, {"servo_left", 30}});
  auto &cap = uart_captured(0);
  cap.clear();
  run_us(105000);
  int cmds = 0, texts = 0;
  for (size_t i = 0; i + 4 < cap.size(); i++) {
    if (cap[i] != 0xA5 || cap[i + 1] != 0x5A) continue;
    uint8_t ty = cap[i + 2], n = cap[i + 3], s = ty + n;
    if (i + 4 + n >= cap.size()) break;
    for (int k = 0; k < n; k++) s += cap[i + 4 + k];
    if (s != cap[i + 4 + n]) continue;
    if (ty == 0x43 && n == 8) {
      int16_t vl, vr; memcpy(&vl, &cap[i + 4], 2); memcpy(&vr, &cap[i + 6], 2);
      if (cmds == 0) CHECK(vl == 200 && vr == -150 && cap[i + 8] == 30, "Command frame: vL %d vR %d servoL %u", vl, vr, cap[i + 8]);
      cmds++;
    }
    if (ty == 0x58 && n == 42) texts++;
    i += 4 + n;
  }
  CHECK(cmds == 5, "%d Command frames in 105 ms (one per 20 ms)", cmds);
  CHECK(texts == 1, "one OLED text frame per second");
  uint8_t t[77] = {0};
  uint32_t ms = 1234; memcpy(t, &ms, 4);
  uint16_t tof0 = 321; memcpy(t + 24, &tof0, 2);
  uint8_t f[82] = {0xA5, 0x5A, 0x54, 77};
  memcpy(f + 4, t, 77);
  uint8_t s = 0x54 + 77; for (int k = 0; k < 77; k++) s += t[k]; f[81] = s;
  uart_inject(0, f, 82);
  printf("        (next line is the stub's own print of the injected telemetry frame)\n");
  run_us(500000);
}

int main() {
  test_tof(); test_gyro(); test_tcs(); test_qtr(); test_motor(); test_brain();
  printf("\n%s (%d failure%s)\n", fails ? "SOME TESTS FAILED" : "ALL CHIP TESTS PASSED", fails, fails == 1 ? "" : "s");
  return fails ? 1 : 0;
}

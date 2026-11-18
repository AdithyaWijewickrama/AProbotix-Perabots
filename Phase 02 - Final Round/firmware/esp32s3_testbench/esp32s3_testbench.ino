// =============================================================================
// GuardianBot v4.1 - ESP32-S3 TEST BENCH firmware (web page over Wi-Fi)  -  NOT FOR COMPETITION RUNS
//
// Flash this instead of esp32s3_brain to test the robot from a phone or laptop:
//   1. join the Wi-Fi network "GuardianBot-test" (password below)
//   2. open http://192.168.4.1
// Tabs: Sensors (live), Drive (joystick, servos, outputs), Camera (OV7670 picture, 48x32 grid, white
// balance, lens FOV), Calibrate (1 m drive -> ENC_CPR, 360 deg spin, gripper angles, ToF offsets), Log (CSV).
// Calibration is saved in the S3 flash ("gb" namespace) and the brain firmware loads it at start-up.
//
// Rule 4.2 (no remote control): Wi-Fi exists ONLY in this firmware. Re-flash esp32s3_brain before any run.
// The Mega (mega_body) is unchanged: this firmware talks to it with the same Command / Telemetry frames.
// Safety: if the page stops sending for 300 ms the wheels stop; the Mega stops on its own after 250 ms.
// Board settings: as esp32s3_brain ("ESP32S3 Dev Module", 16MB flash, OPI PSRAM, USB CDC On Boot).
// =============================================================================
#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>
#include <string.h>
#include "protocol.h"
#include "colour.h"
#include "cam_ov7670.h"
#include "esp_heap_caps.h"
#include "web_page.h"           // index.html, made by tools/make_page_h.py from web/index.html

// ---------------------------------------------------------------- settings
static const char *AP_SSID = "GuardianBot-test";
static const char *AP_PASS = "aprobotix26";       // at least 8 characters
#define LINK_RX_PIN 41
#define LINK_TX_PIN 42
#define LINK_BAUD 250000UL
static const float R_WHEEL = 0.0215f, HALF_TRACK = 0.062f;
static const int V_LIMIT = 600;                   // mm/s, hard cap on anything the page asks for
static const uint32_t DEADMAN_MS = 300;
#define CAM_W 48
#define CAM_H 32
static const float CROP_FOV = 0.90f;              // same crop as the brain (CAM_FOV)
static const int LOG_MAX = 3000;                  // 60 s at 50 Hz

HardwareSerial Link(2);
WebServer web(80);
Preferences prefs;

// ---------------------------------------------------------------- calibration (shared with the brain)
struct Cal {
  float cpr = 2800, lens = 0.90f;
  uint8_t grip[4] = {30, 90, 150, 90};
  int16_t tofo[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  float whiteF[3] = {300, 300, 300}, whiteX[3] = {300, 300, 300};
  CamGain cam;
} cal;
void calLoad() {
  prefs.begin("gb", true);
  cal.cpr = prefs.getFloat("cpr", cal.cpr);
  cal.lens = prefs.getFloat("lens", cal.lens);
  prefs.getBytes("grip", cal.grip, 4);
  prefs.getBytes("tofo", cal.tofo, sizeof(cal.tofo));
  prefs.getBytes("wF", cal.whiteF, sizeof(cal.whiteF));
  prefs.getBytes("wX", cal.whiteX, sizeof(cal.whiteX));
  prefs.getBytes("cg", &cal.cam, sizeof(cal.cam));
  prefs.end();
}
void calSave() {
  prefs.begin("gb", false);
  prefs.putFloat("cpr", cal.cpr);
  prefs.putFloat("lens", cal.lens);
  prefs.putBytes("grip", cal.grip, 4);
  prefs.putBytes("tofo", cal.tofo, sizeof(cal.tofo));
  prefs.putBytes("wF", cal.whiteF, sizeof(cal.whiteF));
  prefs.putBytes("wX", cal.whiteX, sizeof(cal.whiteX));
  prefs.putBytes("cg", &cal.cam, sizeof(cal.cam));
  prefs.end();
}

// ---------------------------------------------------------------- link to the Mega
Telemetry tele = {};
uint32_t teleMs = 0, teleCount = 0, teleBad = 0, teleFps = 0;
Command cmd = {0, 0, 90, 90, 0, 0};
uint32_t webCmdMs = 0;
int16_t webVL = 0, webVR = 0;
float vMeasL = 0, vMeasR = 0;
int32_t prevEncL = 0, prevEncR = 0;
uint32_t prevEncMs = 0;

void sendFrame(uint8_t type, const void *p, uint8_t n) {
  uint8_t f[4 + 64 + 1];
  f[0] = 0xA5; f[1] = 0x5A; f[2] = type; f[3] = n;
  memcpy(f + 4, p, n);
  uint8_t s = type + n;
  for (uint8_t i = 0; i < n; i++) s += f[4 + i];
  f[4 + n] = s;
  Link.write(f, n + 5);
}
void showText(const char *l1, const char *l2) {
  TextMsg m;
  memset(&m, ' ', sizeof(m));
  memcpy(m.line[0], l1, strnlen(l1, 21));
  memcpy(m.line[1], l2, strnlen(l2, 21));
  sendFrame(PKT_TEXT, &m, sizeof(m));
}
void onTelemetry();
void linkPoll() {
  static uint8_t buf[128], st = 0, type = 0, len = 0, pos = 0;
  while (Link.available()) {
    uint8_t b = Link.read();
    switch (st) {
      case 0: if (b == 0xA5) st = 1; break;
      case 1: st = (b == 0x5A) ? 2 : 0; break;
      case 2: type = b; st = 3; break;
      case 3: len = b; pos = 0; st = (len <= sizeof(buf)) ? (len ? 4 : 5) : 0; break;
      case 4: buf[pos++] = b; if (pos >= len) st = 5; break;
      case 5: {
        uint8_t s = type + len;
        for (uint8_t i = 0; i < len; i++) s += buf[i];
        st = 0;
        if (s == b && type == PKT_TELEMETRY && len == sizeof(Telemetry)) {
          memcpy(&tele, buf, sizeof(tele));
          teleMs = millis(); teleCount++;
          onTelemetry();
        } else teleBad++;
      } break;
    }
  }
}

// ---------------------------------------------------------------- camera (core 0)
static uint16_t *camFull = nullptr;               // 160 x 120 RGB565 (PSRAM)
static uint16_t camGridBuf[CAM_W * CAM_H];
static int camFW = 160, camFH = 120;
static volatile uint32_t camFrames = 0;
static portMUX_TYPE camMux = portMUX_INITIALIZER_UNLOCKED;
bool camOk = false;
void cameraTask(void *) {
  static uint16_t grid[CAM_W * CAM_H];
  uint16_t *full = (uint16_t *)heap_caps_malloc(160 * 120 * 2, MALLOC_CAP_SPIRAM);
  for (;;) {
    int w = 0, h = 0;
    if (!full || !camGrab(grid, CAM_W, CAM_H, CROP_FOV < cal.lens ? CROP_FOV : cal.lens, cal.lens, full, &w, &h)) { vTaskDelay(5); continue; }
    portENTER_CRITICAL(&camMux);
    memcpy(camGridBuf, grid, sizeof(grid));
    if (camFull && w * h <= 160 * 120) { memcpy(camFull, full, w * h * 2); camFW = w; camFH = h; }
    camFrames++;
    portEXIT_CRITICAL(&camMux);
  }
}

// ---------------------------------------------------------------- telemetry log (PSRAM)
struct LogRow { Telemetry t; int16_t vl, vr; uint8_t sl, sr; };
LogRow *logBuf = nullptr;
int logRows = 0, logWant = 0;
bool logOn = false;

// ---------------------------------------------------------------- routines (run here, 50 Hz, on each telemetry frame)
enum { R_NONE, R_DRIVE1M, R_SPIN360, R_WB };
int routine = R_NONE, rPhase = 0;
uint32_t rT0 = 0;
int32_t rEnc0L = 0, rEnc0R = 0;
float rYaw0 = 0, rAcc[6];
int rN = 0;
char rName[12] = "", rMsg[160] = "-";
bool rResult = false;
float mmPerTick() { return 2 * PI * R_WHEEL * 1000.0f / cal.cpr; }

void startRoutine(const String &n) {
  rPhase = 0; rT0 = millis(); rResult = false; rN = 0;
  memset(rAcc, 0, sizeof(rAcc));
  rEnc0L = tele.encL; rEnc0R = tele.encR; rYaw0 = tele.yaw;
  if (n == "drive1m") routine = R_DRIVE1M;
  else if (n == "spin360") routine = R_SPIN360;
  else if (n == "wb") routine = R_WB;
  else { routine = R_NONE; return; }
  strncpy(rName, n.c_str(), sizeof(rName) - 1);
  snprintf(rMsg, sizeof(rMsg), "running...");
}
void stopRoutine(const char *why) {
  if (routine != R_NONE && why) snprintf(rMsg, sizeof(rMsg), "%s", why);
  routine = R_NONE;
  cmd.vL = cmd.vR = 0;
}
void routineStep() {
  float dl = (tele.encL - rEnc0L) * mmPerTick(), dr = (tele.encR - rEnc0R) * mmPerTick();
  float dyaw = (tele.yaw - rYaw0) * 57.2958f;
  uint32_t t = millis() - rT0;
  if (routine == R_DRIVE1M) {
    float d = (dl + dr) / 2, left = 1000 - d;
    if (left <= 0 || t > 15000) {
      cmd.vL = cmd.vR = 0;
      rResult = true;
      snprintf(rMsg, sizeof(rMsg), "Done: encoders say %.0f mm (L %.0f, R %.0f), heading drift %.1f deg. Measure the real distance below.",
               d, dl, dr, dyaw);
      routine = R_NONE;
      return;
    }
    float v = fminf(200, fmaxf(40, left * 1.5f));
    float steer = -dyaw * 4;                        // hold the heading on the gyro (mm/s per degree)
    cmd.vL = (int16_t)(v - steer); cmd.vR = (int16_t)(v + steer);
    snprintf(rMsg, sizeof(rMsg), "driving... %.0f mm", d);
  } else if (routine == R_SPIN360) {
    float left = 360 - dyaw;
    if (left <= 0.5f || t > 15000) {
      cmd.vL = cmd.vR = 0;
      float track = (dr - dl) / (2 * PI * dyaw / 360.0f) / 2;
      rResult = true;
      snprintf(rMsg, sizeof(rMsg), "Done: gyro %.1f deg. Encoders imply half-track %.1f mm (design 62). "
               "If the robot does not face the start direction, the gyro scale is off.", dyaw, track);
      routine = R_NONE;
      return;
    }
    float w = fminf(120, fmaxf(25, left * 1.2f)) * PI / 180;    // rad/s
    int16_t v = (int16_t)(w * HALF_TRACK * 1000);
    cmd.vL = -v; cmd.vR = v;
    snprintf(rMsg, sizeof(rMsg), "turning... %.0f deg", dyaw);
  } else if (routine == R_WB) {
    cmd.vL = cmd.vR = 0;
    cmd.out |= OUT_TCS_FLOOR | OUT_TCS_FRONT;
    if (t < 300) return;                            // LEDs settle
    if (tele.flags & FLAG_TCS_NEW) {
      for (int k = 0; k < 3; k++) { rAcc[k] += tele.tcsFloor[k]; rAcc[3 + k] += tele.tcsFront[k]; }
      rN++;
    }
    if (rN >= 15) {
      for (int k = 0; k < 3; k++) { cal.whiteF[k] = fmaxf(1, rAcc[k] / rN); cal.whiteX[k] = fmaxf(1, rAcc[3 + k] / rN); }
      float cr = 0, cg = 0, cb = 0; int cn = 0;
      portENTER_CRITICAL(&camMux);
      for (int v = CAM_H / 2; v < CAM_H; v++)
        for (int u = CAM_W / 4; u < 3 * CAM_W / 4; u++) {
          float r, g, b; camRGB(camGridBuf[v * CAM_W + u], CamGain(), r, g, b);
          cr += r; cg += g; cb += b; cn++;
        }
      portEXIT_CRITICAL(&camMux);
      if (camOk && cr > 30 * cn && cg > 30 * cn && cb > 30 * cn) {
        float m = fmaxf(cr, fmaxf(cg, cb));
        cal.cam.r = m / cr; cal.cam.g = m / cg; cal.cam.b = m / cb;
      }
      calSave();
      cmd.out &= ~(OUT_TCS_FLOOR | OUT_TCS_FRONT);
      rResult = true;
      snprintf(rMsg, sizeof(rMsg), "Saved. Floor white %.0f/%.0f/%.0f, front %.0f/%.0f/%.0f, camera gains %.2f/%.2f/%.2f",
               cal.whiteF[0], cal.whiteF[1], cal.whiteF[2], cal.whiteX[0], cal.whiteX[1], cal.whiteX[2], cal.cam.r, cal.cam.g, cal.cam.b);
      routine = R_NONE;
    }
  }
}

// called for every telemetry frame (50 Hz): speeds, routines, deadman, command out, log
void onTelemetry() {
  uint32_t now = millis();
  if (prevEncMs) {
    float dt = (now - prevEncMs) / 1000.0f;
    if (dt > 0.005f) {
      vMeasL = 0.6f * vMeasL + 0.4f * (tele.encL - prevEncL) * mmPerTick() / dt;
      vMeasR = 0.6f * vMeasR + 0.4f * (tele.encR - prevEncR) * mmPerTick() / dt;
    }
  }
  prevEncL = tele.encL; prevEncR = tele.encR; prevEncMs = now;
  if (routine != R_NONE) routineStep();
  else if (now - webCmdMs > DEADMAN_MS) { cmd.vL = cmd.vR = 0; }
  else { cmd.vL = webVL; cmd.vR = webVR; }
  sendFrame(PKT_COMMAND, &cmd, sizeof(cmd));
  cmd.ctl = 0;
  if (logOn && logBuf) {
    logBuf[logRows] = {tele, cmd.vL, cmd.vR, cmd.servoL, cmd.servoR};
    if (++logRows >= logWant) logOn = false;
  }
}

// ---------------------------------------------------------------- web API (same as tools/testbench_sim)
String jarr(const uint16_t *a, int n) { String s = "["; for (int i = 0; i < n; i++) { if (i) s += ","; s += a[i]; } return s + "]"; }
String jarr16(const int16_t *a, int n) { String s = "["; for (int i = 0; i < n; i++) { if (i) s += ","; s += a[i]; } return s + "]"; }
String jf3(const float *a) { char b[64]; snprintf(b, sizeof(b), "[%.0f,%.0f,%.0f]", a[0], a[1], a[2]); return b; }
int argi(const char *k, int d) { return web.hasArg(k) ? web.arg(k).toInt() : d; }

void hState() {
  float fr, fg, fb, xr, xg, xb;
  uint16_t cf[4], cx[4];
  memcpy(cf, (const void *)tele.tcsFloor, sizeof(cf)); memcpy(cx, (const void *)tele.tcsFront, sizeof(cx));
  tcsRGB(cf, cal.whiteF, fr, fg, fb); tcsRGB(cx, cal.whiteX, xr, xg, xb);
  static uint32_t lastFrames = 0, lastT = 0, camFps = 0;
  if (millis() - lastT >= 1000) { camFps = camFrames - lastFrames; lastFrames = camFrames; lastT = millis(); }
  uint16_t us[2], tof[8], line[8];
  memcpy(us, (const void *)tele.us, sizeof(us)); memcpy(tof, (const void *)tele.tof, sizeof(tof)); memcpy(line, (const void *)tele.line, sizeof(line));
  char b[700];
  String s;
  s.reserve(1600);
  snprintf(b, sizeof(b), "{\"fw\":\"test bench v4.1\",\"ms\":%lu,\"link\":{\"ok\":%s,\"fps\":%lu,\"bad\":%lu,\"age\":%lu},"
           "\"enc\":[%ld,%ld],\"v\":[%d,%d],\"yaw\":%.2f,\"gz\":%.2f,\"vbat\":%u,\"keys\":%u,\"flags\":%u,",
           (unsigned long)tele.ms, (millis() - teleMs < 300) ? "true" : "false", (unsigned long)teleFps, (unsigned long)teleBad,
           (unsigned long)(millis() - teleMs), (long)tele.encL, (long)tele.encR, (int)vMeasL, (int)vMeasR,
           tele.yaw * 57.2958f, tele.gz * 57.2958f, tele.vbat_mv, tele.keys, tele.flags);
  s += b;
  s += "\"us\":" + jarr(us, 2) + ",\"tof\":" + jarr(tof, 8) + ",\"line\":" + jarr(line, 8);
  s += ",\"tcsFloor\":" + jarr(cf, 4) + ",\"tcsFront\":" + jarr(cx, 4);
  snprintf(b, sizeof(b), ",\"floorRGB\":[%.0f,%.0f,%.0f],\"frontRGB\":[%.0f,%.0f,%.0f],\"floorCls\":%d,\"frontCls\":%d,"
           "\"cmd\":{\"vl\":%d,\"vr\":%d,\"sl\":%u,\"sr\":%u,\"out\":%u},\"cam\":{\"ok\":%s,\"pid\":%u,\"fps\":%lu},\"psram\":%u,",
           fr, fg, fb, xr, xg, xb, classify(fr, fg, fb), classify(xr, xg, xb), cmd.vL, cmd.vR, cmd.servoL, cmd.servoR, cmd.out,
           camOk ? "true" : "false", camPID, (unsigned long)camFps, (unsigned)ESP.getPsramSize());
  s += b;
  snprintf(b, sizeof(b), "\"routine\":{\"name\":\"%s\",\"running\":%s,\"result\":%s,\"msg\":\"%s\"},"
           "\"log\":{\"on\":%s,\"rows\":%d},\"cal\":{\"cpr\":%.1f,\"lens\":%.4f,\"grip\":[%u,%u,%u,%u],\"tofo\":",
           rName, routine != R_NONE ? "true" : "false", rResult ? "true" : "false", rMsg, logOn ? "true" : "false", logRows,
           cal.cpr, cal.lens, cal.grip[0], cal.grip[1], cal.grip[2], cal.grip[3]);
  s += b;
  s += jarr16(cal.tofo, 8) + ",\"whiteF\":" + jf3(cal.whiteF) + ",\"whiteX\":" + jf3(cal.whiteX) + "}}";
  web.send(200, "application/json", s);
}
void hCmd() {
  webVL = constrain(argi("vl", 0), -V_LIMIT, V_LIMIT);
  webVR = constrain(argi("vr", 0), -V_LIMIT, V_LIMIT);
  cmd.servoL = constrain(argi("sl", cmd.servoL), 0, 180);
  cmd.servoR = constrain(argi("sr", cmd.servoR), 0, 180);
  if (routine != R_WB) cmd.out = argi("out", cmd.out) & 0x1F;
  webCmdMs = millis();
  if ((webVL || webVR) && routine != R_NONE) stopRoutine("stopped: you took over the wheels");
  web.send(200, "text/plain", "ok");
}
void hStop() { webVL = webVR = 0; stopRoutine("STOPPED"); web.send(200, "text/plain", "stopped"); }
void hCtl() {
  if (argi("zero", 0)) cmd.ctl |= CTL_ZERO_ODOM;
  if (argi("gyro", 0)) cmd.ctl |= CTL_GYRO_CAL;
  web.send(200, "text/plain", "ok");
}
void hFrame() {
  static uint8_t out[4 + 160 * 120 * 2];
  int w, h;
  portENTER_CRITICAL(&camMux);
  w = camFW; h = camFH;
  out[0] = w & 0xFF; out[1] = w >> 8; out[2] = h & 0xFF; out[3] = h >> 8;
  if (camFull) memcpy(out + 4, camFull, w * h * 2);
  portEXIT_CRITICAL(&camMux);
  web.send_P(200, "application/octet-stream", (const char *)out, 4 + w * h * 2);
}
void hGrid() {
  static uint16_t g[CAM_W * CAM_H];
  portENTER_CRITICAL(&camMux);
  memcpy(g, camGridBuf, sizeof(g));
  portEXIT_CRITICAL(&camMux);
  String s;
  s.reserve(CAM_W * CAM_H * 7 + 200);
  s = "{\"w\":48,\"h\":32,\"px\":[";
  char cls[CAM_W * CAM_H + 1];
  for (int i = 0; i < CAM_W * CAM_H; i++) {
    if (i) s += ",";
    s += g[i];
    float r, gg, b; camRGB(g[i], cal.cam, r, gg, b);
    int k = classify(r, gg, b);
    cls[i] = k == BLUE ? 'B' : k == GREEN ? 'G' : k == FLOOR ? 'F' : '.';
  }
  cls[CAM_W * CAM_H] = 0;
  char b[160];
  snprintf(b, sizeof(b), "],\"cls\":\"%s", "");
  s += b; s += cls;
  float crop = CROP_FOV < cal.lens ? CROP_FOV : cal.lens;
  snprintf(b, sizeof(b), "\",\"gain\":[%.3f,%.3f,%.3f],\"lens\":%.4f,\"crop\":%.4f}", cal.cam.r, cal.cam.g, cal.cam.b, cal.lens, crop);
  s += b;
  web.send(200, "application/json", s);
}
void hRoutine() { startRoutine(web.arg("name")); web.send(200, "text/plain", "ok"); }
void hSet() {
  String k = web.arg("key");
  if (k == "cpr") cal.cpr = constrain(web.arg("val").toFloat(), 100.0f, 20000.0f);
  else if (k == "lens") cal.lens = constrain(web.arg("val").toFloat(), 0.2f, 2.5f);
  else if (k == "grip") { int i = argi("i", -1); if (i >= 0 && i < 4) cal.grip[i] = constrain(argi("val", 90), 0, 180); }
  else { web.send(400, "text/plain", "unknown key"); return; }
  calSave();
  web.send(200, "text/plain", "saved");
}
void hTofZero() {
  int i = argi("i", -1), ref = argi("ref", 100);
  if (i < 0 || i > 7 || tele.tof[i] >= 1200) { web.send(400, "text/plain", "no reading"); return; }
  cal.tofo[i] = (int16_t)(ref - (int)tele.tof[i]);
  calSave();
  web.send(200, "text/plain", "saved");
}
void hLog() {
  if (!logBuf) { web.send(500, "text/plain", "no PSRAM"); return; }
  logWant = constrain(argi("secs", 30), 1, 60) * 50;
  logRows = 0; logOn = true;
  web.send(200, "text/plain", "recording");
}
void hLogCsv() {
  web.setContentLength(CONTENT_LENGTH_UNKNOWN);
  web.send(200, "text/csv", "");
  web.sendContent("ms,encL,encR,yaw_deg,gz_dps,us_f,us_b,tof_f,tof_fl,tof_fr,tof_lf,tof_lr,tof_rf,tof_rr,tof_b,"
                  "q1,q2,q3,q4,q5,q6,q7,q8,fR,fG,fB,fC,xR,xG,xB,xC,vbat_mv,keys,flags,cmd_vl,cmd_vr,servo_l,servo_r\n");
  char b[320];
  for (int i = 0; i < logRows; i++) {
    const Telemetry &t = logBuf[i].t;
    int n = snprintf(b, sizeof(b), "%lu,%ld,%ld,%.2f,%.2f,%u,%u", (unsigned long)t.ms, (long)t.encL, (long)t.encR,
                     t.yaw * 57.2958f, t.gz * 57.2958f, t.us[0], t.us[1]);
    for (int k = 0; k < 8; k++) n += snprintf(b + n, sizeof(b) - n, ",%u", t.tof[k]);
    for (int k = 0; k < 8; k++) n += snprintf(b + n, sizeof(b) - n, ",%u", t.line[k]);
    for (int k = 0; k < 4; k++) n += snprintf(b + n, sizeof(b) - n, ",%u", t.tcsFloor[k]);
    for (int k = 0; k < 4; k++) n += snprintf(b + n, sizeof(b) - n, ",%u", t.tcsFront[k]);
    snprintf(b + n, sizeof(b) - n, ",%u,%u,%u,%d,%d,%u,%u\n", t.vbat_mv, t.keys, t.flags, logBuf[i].vl, logBuf[i].vr,
             logBuf[i].sl, logBuf[i].sr);
    web.sendContent(b);
  }
  web.sendContent("");
}

// ---------------------------------------------------------------- setup / loop
void setup() {
  Serial.begin(115200);
  Link.setRxBufferSize(1024);
  Link.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
  calLoad();
  cmd.servoL = cal.grip[1]; cmd.servoR = cal.grip[3];
  camFull = (uint16_t *)heap_caps_malloc(160 * 120 * 2, MALLOC_CAP_SPIRAM);
  logBuf = (LogRow *)heap_caps_malloc(sizeof(LogRow) * LOG_MAX, MALLOC_CAP_SPIRAM);
  camOk = camBegin();
  if (camOk) xTaskCreatePinnedToCore(cameraTask, "cam", 4096, nullptr, 1, nullptr, 0);
  Serial.printf("test bench: PSRAM %u bytes, camera %s (PID 0x%02X)\n", (unsigned)ESP.getPsramSize(), camOk ? "ok" : "NOT FOUND", camPID);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  web.on("/", []() { web.send_P(200, "text/html", INDEX_HTML); });
  web.on("/api/state", hState);
  web.on("/api/cmd", hCmd);
  web.on("/api/stop", hStop);
  web.on("/api/ctl", hCtl);
  web.on("/api/frame", hFrame);
  web.on("/api/grid", hGrid);
  web.on("/api/routine", hRoutine);
  web.on("/api/set", hSet);
  web.on("/api/tofzero", hTofZero);
  web.on("/api/log", hLog);
  web.on("/api/log.csv", hLogCsv);
  web.begin();
  Serial.printf("Wi-Fi \"%s\" up: open http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}

void loop() {
  linkPoll();
  web.handleClient();
  static uint32_t lastS = 0, lastCount = 0;
  if (millis() - lastS >= 1000) {
    teleFps = teleCount - lastCount; lastCount = teleCount; lastS = millis();
    char l2[22];
    snprintf(l2, sizeof(l2), "%s %s", WiFi.softAPIP().toString().c_str(), WiFi.softAPgetStationNum() ? "PAGE" : "");
    showText("TEST FIRMWARE WIFI ON", l2);
    if (millis() - teleMs > 1000) Serial.println("no telemetry from the Mega - check the link cable");
  }
  delay(1);
}

// =============================================================================
// GuardianBot v4.1 - ESP32-S3 N16R8 "brain + eye" firmware (OV7670 camera)
// PeraBots 2026 University Category, Final Round - The Nine Seals of Pera
//
// One ESP32-S3 (dual core, 8 MB octal PSRAM) with an OV7670 camera on the mast is the whole brain:
//   core 0: camera task - grabs 160x120 RGB565 frames, crops them to CAM_FOV and averages them to the
//           48x32 grid that the maze code projects onto the floor
//   core 1: the mission - a line-by-line port of simulation/controllers/main_controller
// The Arduino Mega 2560 does all real-time I/O and sends a telemetry frame every 20 ms
// (one frame = one step()). The brain sends wheel speeds, servo angles and OLED text back.
//
//   Section 1  occupancy grid (2 cm) from 8 VL53L0X + 2 HC-SR04, A* with a clearance cost,
//              straight 45-degree segments, wall-based position fixes
//   Section 2  9x4 maze: wall voting from every ray, heading fix from the side ToF pairs,
//              camera + floor TCS3200 tile votes, Dijkstra over (cell, heading) that only goes
//              where it can SEE possible dead ends, biased towards the exit
//   Section 3  clue decoding (tiles + turns of the shortest path), Lo Shu board, line following
//              on the QTR-8RC with crossing fixes, rear-pocket pick and place, ToF board
//              inspection, front TCS3200 colour check
//
// Keypad (on the Mega): K1 START (detects START area or maze entrance)   K2 same as K1
//                       K3 Section 3 restart with the clues saved in flash
//                       K4 CAL: white-balance both TCS3200 AND the camera (white paper under / in front)
// Board: "ESP32S3 Dev Module" (Arduino-ESP32 core 3.x): Flash Size 16MB, PSRAM "OPI PSRAM",
//        USB CDC On Boot "Enabled" (log on the USB-C "USB" port at 115200). Wi-Fi/BT never started.
// The web test page is a SEPARATE firmware (esp32s3_testbench): never flash it for a competition run.
// =============================================================================
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include "protocol.h"
#include "config.h"
#ifndef SIL
#include "esp_heap_caps.h"
#endif
#include "cam_ov7670.h"

#define PI_F  3.14159265f
#define HP_F  1.57079633f

HardwareSerial Link(2);
void show(const char *l1, const char *l2);
#ifdef SIL
HardwareSerial CamPipe(3);       // software-in-the-loop: Webots camera frames arrive on a pipe
#endif

// big buffers go to the 8 MB PSRAM of the N16R8 (internal RAM is too small for the 2 cm A* grid)
void *bigAlloc(size_t n) {
#ifndef SIL
  void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM);
  if (p) return p;
#endif
  return malloc(n);
}
Preferences prefs;

// ============================================================ state
float px = START_X, py = START_Y, yaw = HP_F, yawOff = HP_F;
float tSim = 0;
float D[N_SENS];
float GSV[N_GS];
Telemetry tele;
bool haveTele = false;
int32_t lastEncL = 0, lastEncR = 0;
bool firstStep = true;
float lastL = 0, lastR = 0;          // commanded wheel speeds, m/s
Command out = {0, 0, SERVO_L_CLOSED, SERVO_R_CLOSED, 0, 0};
float whiteF[3] = {TCS_WHITE_DEFAULT, TCS_WHITE_DEFAULT, TCS_WHITE_DEFAULT};
float whiteX[3] = {TCS_WHITE_DEFAULT, TCS_WHITE_DEFAULT, TCS_WHITE_DEFAULT};   // TCS white balance (counts on white)
void (*senseHook)() = nullptr;
CamGain camGain;                     // camera white balance (K4 on white paper)

// ---- camera frames
uint16_t camImg[CAM_W * CAM_H];
bool camNew = false;
uint32_t camFrames = 0;
#ifdef SIL
void camPoll() {                 // SIL: 'V' frames (same format the old esp32cam_vision sent) on a pipe
  static uint8_t buf[CAM_W * CAM_H * 2];
  static uint8_t st = 0, sum = 0;
  static uint16_t len = 0, pos = 0;
  while (CamPipe.available()) {
    uint8_t b = CamPipe.read();
    switch (st) {
      case 0: if (b == 0xA5) st = 1; break;
      case 1: st = (b == 0x5A) ? 2 : 0; break;
      case 2: if (b == 'V') { sum = b; st = 3; } else st = 0; break;
      case 3: sum += b; st = 4; break;
      case 4: len = b; sum += b; st = 5; break;
      case 5: len |= b << 8; sum += b; pos = 0; st = (len == sizeof(buf)) ? 6 : 0; break;
      case 6: buf[pos++] = b; sum += b; if (pos >= len) st = 7; break;
      case 7:
        if (b == sum) { memcpy(camImg, buf, sizeof(buf)); camNew = true; camFrames++; }
        st = 0; break;
    }
  }
}
#else
// ---- real camera: OV7670 on the S3 (cam_ov7670.h), task on core 0
static uint16_t camGrid[CAM_W * CAM_H];
static volatile bool camGridNew = false;
static portMUX_TYPE camMux = portMUX_INITIALIZER_UNLOCKED;
void cameraTask(void *) {
  static uint16_t grid[CAM_W * CAM_H];
  for (;;) {
    if (!camGrab(grid, CAM_W, CAM_H, CAM_FOV, LENS_HFOV)) { vTaskDelay(5); continue; }
    portENTER_CRITICAL(&camMux);
    memcpy(camGrid, grid, sizeof(grid));
    camGridNew = true;
    portEXIT_CRITICAL(&camMux);
  }
}
void camPoll() {
  if (!camGridNew) return;
  portENTER_CRITICAL(&camMux);
  memcpy(camImg, camGrid, sizeof(camImg));
  camGridNew = false;
  portEXIT_CRITICAL(&camMux);
  camNew = true; camFrames++;
}
#endif

static inline float wrapA(float a) {
  while (a > PI_F) a -= 2 * PI_F;
  while (a < -PI_F) a += 2 * PI_F;
  return a;
}
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

void logf_(const char *fmt, ...) {
  char buf[200];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  Serial.printf("[%7.2f] %s\n", tSim, buf);
}

// ============================================================ link
bool readTelemetry(uint32_t timeoutMs) {
  static uint8_t buf[128];
  static uint8_t st = 0, type = 0, len = 0, pos = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
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
            return true;
          }
        } break;
      }
    }
    delay(0);
  }
  return false;
}

void sendCommand() {
  uint8_t f[4 + sizeof(Command) + 1];
  f[0] = 0xA5; f[1] = 0x5A; f[2] = PKT_COMMAND; f[3] = sizeof(Command);
  memcpy(f + 4, &out, sizeof(Command));
  uint8_t s = PKT_COMMAND + sizeof(Command);
  for (uint8_t i = 0; i < sizeof(Command); i++) s += f[4 + i];
  f[4 + sizeof(Command)] = s;
  Link.write(f, sizeof(f));
}

// ============================================================ step: one 20 ms control cycle
void step() {
  out.vL = (int16_t)lroundf(lastL * 1000.0f);
  out.vR = (int16_t)lroundf(lastR * 1000.0f);
  sendCommand();
  out.ctl = 0;
  if (!readTelemetry(200)) {
    logf_("link lost - waiting for the Mega");
    while (!readTelemetry(1000)) sendCommand();
  }
  tSim += STEP_S;
  float y2 = wrapA(tele.yaw + yawOff);
  if (!firstStep) {
    float mpt = 2.0f * PI_F * R_WHEEL / ENC_CPR;
    float d = ((tele.encL - lastEncL) + (tele.encR - lastEncR)) * 0.5f * mpt;
    float mid = yaw + wrapA(y2 - yaw) * 0.5f;
    px += d * cosf(mid);
    py += d * sinf(mid);
  }
  firstStep = false;
  lastEncL = tele.encL; lastEncR = tele.encR; yaw = y2;
  for (int i = 0; i < 2; i++) D[US_FRONT + i] = tele.us[i] / 1000.0f;
  for (int i = 0; i < 8; i++) D[TOF_FRONT + i] = (tele.tof[i] + (tele.tof[i] < 1200 ? TOF_OFF_MM[i] : 0)) / 1000.0f;
  for (int i = 0; i < N_GS; i++) GSV[i] = QTR_TIMEOUT_US - tele.line[i];      // white high, black low
  camPoll();
}

float gyroZ() { return tele.gz; }
float rngMax(int n) { return SENS[n].maxr - 0.005f; }

// ============================================================ motion primitives
void wheels(float a, float b) {          // wheel linear speeds m/s with cap and slew
  float big = fmaxf(fabsf(a), fabsf(b));
  if (big > WHEEL_VMAX) { a *= WHEEL_VMAX / big; b *= WHEEL_VMAX / big; }
  float dv = WHEEL_ACC * STEP_S;
  a = clampf(a, lastL - dv, lastL + dv);
  b = clampf(b, lastR - dv, lastR + dv);
  lastL = a; lastR = b;
}
void setVW(float v, float w) {
  if (v != 0) w = fabsf(v) > 0.12f ? clampf(w, -W_RUN, W_RUN) : 0.0f;
  wheels(v - w * HALF_TRACK, v + w * HALF_TRACK);
}
void driveLR(float v, float w) { wheels(v - w * HALF_TRACK, v + w * HALF_TRACK); }

void fullStop() {
  while (fabsf(lastL) > 0.002f || fabsf(lastR) > 0.002f) {
    setVW(0, 0); step();
    if (senseHook) senseHook();
  }
}
void stopNow(int n = 4) { for (int i = 0; i < n; i++) { setVW(0, 0); step(); } }

void turnTo(float target, float tol = 0.02f) {
  fullStop();
  uint32_t n = 0;
  while (true) {
    step();
    if (senseHook) senseHook();
    float e = wrapA(target - yaw);
    if (fabsf(e) < tol && fabsf(gyroZ()) < 0.3f) break;
    if (++n > 400) break;                                  // 8 s safety
    float w = fminf(TURN_WMAX, sqrtf(2 * TURN_ALPHA * fabsf(e)));
    w = fmaxf(w, 0.5f);
    setVW(0, e > 0 ? w : -w);
  }
  fullStop();
}

void ray(int n, float &ox, float &oy, float &a) {
  float c = cosf(yaw), s = sinf(yaw);
  ox = px + c * SENS[n].x - s * SENS[n].y;
  oy = py + s * SENS[n].x + c * SENS[n].y;
  a = yaw + SENS[n].a;
}
bool nearCardinal(float tol) {
  float k = roundf(yaw / HP_F);
  return fabsf(wrapA(yaw - k * HP_F)) < tol;
}

void waitS(float sec) { for (int i = 0; i < (int)(sec / STEP_S); i++) { driveLR(0, 0); step(); } }

// ============================================================ colour
#include "colour.h"   // FLOOR / BLUE / GREEN / JUNK, tcsRGB(), classify(): shared with the test bench
int floorColour() { uint16_t c[4]; memcpy(c, (const void *)tele.tcsFloor, sizeof(c)); float r, g, b; tcsRGB(c, whiteF, r, g, b); return classify(r, g, b); }
int frontColour() { uint16_t c[4]; memcpy(c, (const void *)tele.tcsFront, sizeof(c)); float r, g, b; tcsRGB(c, whiteX, r, g, b); return classify(r, g, b); }

// ============================================================ SECTION 2: maze model
static const int DC[4] = {0, 1, 0, -1}, DR[4] = {1, 0, -1, 0};
static const float HANG[4] = {HP_F, 0.0f, -HP_F, PI_F};
enum { UNK = 0, OPEN = 1, WALL = 2 };
int hz[COLS][ROWS + 1][2], vt[COLS + 1][ROWS][2];
int votes[COLS][ROWS][3];
int8_t seen[COLS][ROWS];          // -1 = not decided, else FLOOR/BLUE/GREEN
bool mapDirty = true, exploring = true;

void mazeInit() {
  memset(hz, 0, sizeof(hz)); memset(vt, 0, sizeof(vt)); memset(votes, 0, sizeof(votes));
  memset(seen, -1, sizeof(seen));
  for (int c = 0; c < COLS; c++) { hz[c][0][1] = hz[c][ROWS][1] = 999; }
  for (int r = 0; r < ROWS; r++) { vt[0][r][1] = vt[COLS][r][1] = 999; }
}
int st_(const int *e) { if (e[0] == 0 && e[1] == 0) return UNK; return e[1] > e[0] ? WALL : OPEN; }
int *edgeRef(int c, int r, int h) {
  if (h == 0) return hz[c][r + 1];
  if (h == 2) return hz[c][r];
  if (h == 1) return vt[c + 1][r];
  return vt[c][r];
}
int edge(int c, int r, int h) { return st_(edgeRef(c, r, h)); }
bool inGrid(int c, int r) { return c >= 0 && c < COLS && r >= 0 && r < ROWS; }
void cellOf(float x, float y, int &c, int &r) { c = (int)floorf((x - MX0) / CELL); r = (int)floorf((y - MY0) / CELL); }
void centre(int c, int r, float &x, float &y) { x = MX0 + (c + 0.5f) * CELL; y = MY0 + (r + 0.5f) * CELL; }
void vote(int *e, bool wall) { int b = st_(e); e[wall ? 1 : 0]++; if (st_(e) != b) mapDirty = true; }

void mazeRay(int n, bool fix) {
  float ox, oy, a; ray(n, ox, oy, a);
  float d = D[n], mx = rngMax(n);
  bool hit = d < mx;
  if (SENS[n].sonar && (!hit || !nearCardinal(0.12f))) return;
  if (!SENS[n].sonar && d <= TOF_MIN) fix = false;
  float L = hit ? d : mx, ux = cosf(a), uy = sinf(a);
  if (fabsf(ux) > 1e-3f) {
    for (int k = 0; k <= COLS; k++) {
      float t = (MX0 + k * CELL - ox) / ux;
      if (t <= 0 || t > L + 0.03f) continue;
      float yy = oy + uy * t;
      int r = (int)floorf((yy - MY0) / CELL);
      if (r < 0 || r >= ROWS || fabsf(yy - MY0 - (r + 0.5f) * CELL) > 0.085f) continue;
      if (t < L - 0.02f) vote(vt[k][r], false);
      else if (hit) {
        vote(vt[k][r], true);
        if (fix && fabsf(uy) < 0.06f) {
          float face = MX0 + k * CELL - copysignf(0.006f, ux), err = face - (ox + ux * d);
          if (fabsf(err) < 0.03f) px += 0.25f * err;
        }
      }
    }
  }
  if (fabsf(uy) > 1e-3f) {
    for (int k = 0; k <= ROWS; k++) {
      float t = (MY0 + k * CELL - oy) / uy;
      if (t <= 0 || t > L + 0.03f) continue;
      float xx = ox + ux * t;
      int c = (int)floorf((xx - MX0) / CELL);
      if (c < 0 || c >= COLS || fabsf(xx - MX0 - (c + 0.5f) * CELL) > 0.085f) continue;
      if (t < L - 0.02f) vote(hz[c][k], false);
      else if (hit) {
        vote(hz[c][k], true);
        if (fix && fabsf(ux) < 0.06f) {
          float face = MY0 + k * CELL - copysignf(0.006f, uy), err = face - (oy + uy * d);
          if (fabsf(err) < 0.03f) py += 0.25f * err;
        }
      }
    }
  }
}

void decideColour(int c, int r) {
  int f = votes[c][r][0], b = votes[c][r][1], g = votes[c][r][2], tot = f + b + g;
  if (b >= 5 && b > g && b >= 0.25f * tot) seen[c][r] = BLUE;
  else if (g >= 5 && g > b && g >= 0.25f * tot) seen[c][r] = GREEN;
  else seen[c][r] = FLOOR;
  if (seen[c][r] != FLOOR) logf_("tile %s at c%d r%d", seen[c][r] == BLUE ? "BLUE" : "GREEN", c + 1, r + 1);
}

// pixel -> floor point table (every 2nd pixel), same as the Webots controller
#define MAX_PIX (CAM_W * CAM_H / 4)
int16_t pixIdx[MAX_PIX]; float pixGx[MAX_PIX], pixGy[MAX_PIX]; int nPix = 0;
void buildPix() {
  float f = (CAM_W / 2.0f) / tanf(CAM_FOV / 2);
  nPix = 0;
  for (int v = 1; v < CAM_H; v += 2)
    for (int u = 1; u < CAM_W; u += 2) {
      float x = f, y = -(u + 0.5f - CAM_W / 2.0f), z = -(v + 0.5f - CAM_H / 2.0f);
      float x2 = x * cosf(CAM_PITCH) + z * sinf(CAM_PITCH), z2 = -x * sinf(CAM_PITCH) + z * cosf(CAM_PITCH);
      if (z2 >= -0.05f * sqrtf(x * x + y * y + z * z)) continue;
      float t = -CAM_Z / z2, gx = CAM_X + t * x2, gy = t * y;
      if (gx < 0.75f && nPix < MAX_PIX) { pixIdx[nPix] = v * CAM_W + u; pixGx[nPix] = gx; pixGy[nPix] = gy; nPix++; }
    }
}

// is the centre of cell (c,r) visible from (cx,cy) through open walls only?
bool canSee(float cx, float cy, int tc, int tr) {
  float tx, ty; centre(tc, tr, tx, ty);
  int n = (int)(hypotf(tx - cx, ty - cy) / 0.04f) + 1;
  int pc = -1, pr = -1;
  for (int k = 0; k <= n; k++) {
    float f = (float)k / n;
    int c, r; cellOf(cx + (tx - cx) * f, cy + (ty - cy) * f, c, r);
    if (!inGrid(c, r)) { if (pc >= 0) return false; continue; }
    if (pc < 0) { if ((c != 0 || r != 0) && k > 0) return false; pc = c; pr = r; continue; }
    if (c != pc || r != pr) {
      int dc = c - pc, dr = r - pr;
      if (abs(dc) + abs(dr) != 1) return false;
      int h = 0; while (h < 4 && !(DC[h] == dc && DR[h] == dr)) h++;
      if (edge(pc, pr, h) != OPEN) return false;
      pc = c; pr = r;
    }
  }
  return true;
}

void look() {
  if (fabsf(gyroZ()) > 1.0f) return;
  static int hits[COLS][ROWS][3];
  memset(hits, 0, sizeof(hits));
  bool any = false;
  float c_ = cosf(yaw), s_ = sinf(yaw);
  if (camNew) {
    camNew = false;
    float camx = px + c_ * CAM_X, camy = py + s_ * CAM_X;
    int8_t ok[COLS][ROWS]; memset(ok, -1, sizeof(ok));
    for (int i = 0; i < nPix; i++) {
      uint16_t v = camImg[pixIdx[i]];
      float r, g, b;
      camRGB(v, camGain, r, g, b);
      int k = classify(r, g, b);
      if (k == JUNK) continue;
      float wx = px + c_ * pixGx[i] - s_ * pixGy[i], wy = py + s_ * pixGx[i] + c_ * pixGy[i];
      float fx = (wx - MX0) / CELL, fy = (wy - MY0) / CELL;
      int c = (int)floorf(fx), rr = (int)floorf(fy);
      if (!inGrid(c, rr) || fabsf(fx - c - 0.5f) > 0.36f || fabsf(fy - rr - 0.5f) > 0.36f) continue;
      if (ok[c][rr] < 0) ok[c][rr] = canSee(camx, camy, c, rr);
      if (ok[c][rr]) { hits[c][rr][k]++; any = true; }
    }
  }
  if (tele.flags & FLAG_TCS_NEW) {
    int k = floorColour();
    if (k != JUNK) {
      float fx = (px + c_ * TCS_FLOOR_X - s_ * TCS_FLOOR_Y - MX0) / CELL, fy = (py + s_ * TCS_FLOOR_X + c_ * TCS_FLOOR_Y - MY0) / CELL;
      int c = (int)floorf(fx), r = (int)floorf(fy);
      if (inGrid(c, r) && fabsf(fx - c - 0.5f) <= 0.35f && fabsf(fy - r - 0.5f) <= 0.35f) { hits[c][r][k] += 3; any = true; }
    }
  }
  if (!any) return;
  for (int c = 0; c < COLS; c++) for (int r = 0; r < ROWS; r++) {
    int s = hits[c][r][0] + hits[c][r][1] + hits[c][r][2];
    if (!s) continue;
    for (int j = 0; j < 3; j++) votes[c][r][j] += hits[c][r][j];
    if (seen[c][r] < 0 && votes[c][r][0] + votes[c][r][1] + votes[c][r][2] >= 12) { decideColour(c, r); mapDirty = true; }
  }
}

bool axisSens(int n) { return n == TOF_FRONT || n == TOF_LF || n == TOF_RF || n == TOF_BACK; }

// Two ToF sensors per side see the same straight wall: their difference is the robot's angle to it.
// Slowly removes gyro drift; does nothing while turning or without a wall close by.
void sideAlign() {
  if (!nearCardinal(0.15f) || fabsf(gyroZ()) > 0.2f) return;
  const int pr[2][2] = {{TOF_LF, TOF_LR}, {TOF_RF, TOF_RR}};
  for (int k = 0; k < 2; k++) {
    float df = D[pr[k][0]], dr = D[pr[k][1]];
    if (df > TOF_MIN && df < 0.20f && dr > TOF_MIN && dr < 0.20f && fabsf(df - dr) < 0.03f) {
      float theta = atan2f(dr - df, SIDE_PAIR_DX) * (k == 0 ? 1.0f : -1.0f);
      float kk = roundf(yaw / HP_F);
      float err = wrapA(yaw - kk * HP_F) - theta;
      yawOff -= 0.02f * err;
      yaw = wrapA(yaw - 0.02f * err);
    }
  }
}

void mazeSense() {
  if (py < 1.2f) return;
  bool fix = nearCardinal(0.05f);
  for (int n = 0; n < N_SENS; n++) mazeRay(n, fix && axisSens(n));
  sideAlign();
  look();
}

int nOpen(int c, int r) { int k = 0; for (int h = 0; h < 4; h++) k += edge(c, r, h) == OPEN; return k; }
bool needed(int c, int r) {
  if (seen[c][r] >= 0 || nOpen(c, r) >= 2) return false;
  for (int h = 0; h < 4; h++) if (edge(c, r, h) != WALL) return true;
  return false;
}

// Dijkstra over (cell, heading). Returns path length (states) into pathC/R/H, 0 = none.
#define NST (COLS * ROWS * 4)
int pathC[NST], pathR[NST], pathH[NST], pathN = 0;
float bonusArr[COLS][ROWS];
bool useBonus = false;
float bonusMax = 0;

void endBonus() {
  int dist[COLS][ROWS]; memset(dist, -1, sizeof(dist));
  int qc[COLS * ROWS], qr[COLS * ROWS], qh = 0, qt = 0;
  dist[END_C][END_R] = 0; qc[qt] = END_C; qr[qt++] = END_R;
  while (qh < qt) {
    int c = qc[qh], r = qr[qh++];
    for (int h = 0; h < 4; h++) {
      int nc = c + DC[h], nr = r + DR[h];
      if (inGrid(nc, nr) && dist[nc][nr] < 0 && edge(c, r, h) != WALL) {
        dist[nc][nr] = dist[c][r] + 1; qc[qt] = nc; qr[qt++] = nr;
      }
    }
  }
  bonusMax = 0;
  for (int c = 0; c < COLS; c++) for (int r = 0; r < ROWS; r++) {
    bonusArr[c][r] = dist[c][r] < 0 ? 0 : dist[c][r];
    bonusMax = fmaxf(bonusMax, bonusArr[c][r]);
  }
}

int sid(int c, int r, int h) { return (c * ROWS + r) * 4 + h; }
bool plan(int c0, int r0, int h0, bool (*goal)(int, int, int), bool allowUnk, bool bonus) {
  static float best[NST]; static int prev[NST]; static bool done[NST];
  for (int i = 0; i < NST; i++) { best[i] = 1e9f; prev[i] = -1; done[i] = false; }
  int s0 = sid(c0, r0, h0);
  best[s0] = 0;
  int pick = -1; float pickSc = 1e9f;
  while (true) {                       // simple O(N^2) Dijkstra, 144 states
    int s = -1; float bd = 1e9f;
    for (int i = 0; i < NST; i++) if (!done[i] && best[i] < bd) { bd = best[i]; s = i; }
    if (s < 0) break;
    done[s] = true;
    int h = s % 4, r = (s / 4) % ROWS, c = s / 4 / ROWS;
    if (goal(c, r, h)) {
      float sc = bd - (bonus ? bonusArr[c][r] : 0);
      if (sc < pickSc) { pick = s; pickSc = sc; }
      if (!bonus) break;
    }
    if (pick >= 0 && bonus && bd > pickSc + bonusMax) break;
    int nx[4]; float nk[4]; int nn = 0;
    nx[nn] = sid(c, r, (h + 1) % 4); nk[nn++] = 0.7f;
    nx[nn] = sid(c, r, (h + 3) % 4); nk[nn++] = 0.7f;
    nx[nn] = sid(c, r, (h + 2) % 4); nk[nn++] = 1.0f;
    int e = edge(c, r, h);
    if (e == OPEN || (allowUnk && e == UNK)) {
      int nc = c + DC[h], nr = r + DR[h];
      if (inGrid(nc, nr)) { nx[nn] = sid(nc, nr, h); nk[nn++] = e == OPEN ? 1.0f : 3.0f; }
    }
    for (int i = 0; i < nn; i++)
      if (bd + nk[i] < best[nx[i]]) { best[nx[i]] = bd + nk[i]; prev[nx[i]] = s; }
  }
  if (pick < 0) { pathN = 0; return false; }
  int tmp[NST], n = 0;
  for (int s = pick; s >= 0; s = prev[s]) tmp[n++] = s;
  pathN = n;
  for (int i = 0; i < n; i++) {
    int s = tmp[n - 1 - i];
    pathH[i] = s % 4; pathR[i] = (s / 4) % ROWS; pathC[i] = s / 4 / ROWS;
  }
  return true;
}
// stand where the camera looks at an undecided possible dead end 1 or 2 cells ahead
bool exploreGoal(int c, int r, int h) {
  int nc = c + DC[h], nr = r + DR[h];
  if (!inGrid(nc, nr) || edge(c, r, h) == WALL) return false;
  if (needed(nc, nr)) return true;
  int fc = nc + DC[h], fr = nr + DR[h];
  return edge(c, r, h) == OPEN && inGrid(fc, fr) && edge(nc, nr, h) == OPEN && needed(fc, fr);
}
bool endGoal(int c, int r, int h) { return c == END_C && r == END_R; }

void showMajority() {
  int nb = 0, ng = 0;
  for (int c = 0; c < COLS; c++) for (int r = 0; r < ROWS; r++) { nb += seen[c][r] == BLUE; ng += seen[c][r] == GREEN; }
  bool blue = nb >= ng;
  out.out = (out.out & ~(OUT_LED_BLUE | OUT_LED_GREEN)) | (blue ? OUT_LED_BLUE : OUT_LED_GREEN);
  logf_("explored. blue=%d green=%d -> %s", nb, ng, blue ? "BLUE" : "GREEN");
}

bool nextPlan(int c, int r, int h) {
  if (exploring) {
    endBonus();
    if (plan(c, r, h, exploreGoal, false, true)) return true;
    exploring = false;
    showMajority();
  }
  if (plan(c, r, h, endGoal, false, false)) return true;
  return plan(c, r, h, endGoal, true, false);
}
int fwdCount() { int k = 0; for (int i = 1; i < pathN; i++) { if (pathH[i] != pathH[i - 1]) break; k++; } return k; }

void enterMaze() {
  senseHook = mazeSense;
  turnTo(HP_F, 0.008f);
  while (true) {
    step(); mazeSense();
    float rem = 1.455f - py;
    if (rem < 0.004f) break;
    setVW(fminf(V_MAX, sqrtf(2 * ACC * rem) + 0.01f), 0);
  }
  fullStop();
}

void runMaze() {
  senseHook = mazeSense;
  int h = 0, wait = 0;
  bool running = false;
  float lastMove = tSim, lx = px, ly = py;
  while (true) {
    step(); mazeSense();
    int ax = DC[h], ay = DR[h];
    if (hypotf(px - lx, py - ly) > 0.03f) { lastMove = tSim; lx = px; ly = py; }
    if (running) {
      int cc, cr; cellOf(px, py, cc, cr);
      cc = max(0, min(COLS - 1, cc)); cr = max(0, min(ROWS - 1, cr));
      float cx, cy; centre(cc, cr, cx, cy);
      float along = (px - cx) * ax + (py - cy) * ay;
      int dc, dr;
      if (py < MY0) { dc = 0; dr = 0; }
      else if (along > 0.02f && edge(cc, cr, h) == OPEN) { dc = cc + ax; dr = cr + ay; }
      else { dc = cc; dr = cr; }
      bool ok = nextPlan(dc, dr, h);
      int k = ok ? fwdCount() : 0;
      float tx, ty; centre(dc + ax * k, dr + ay * k, tx, ty);
      float rem = (tx - px) * ax + (ty - py) * ay;
      if (rem > 0.004f) setVW(fminf(V_MAX, sqrtf(2 * ACC * rem) + 0.01f), 0);
      if (rem <= 0.004f || D[US_FRONT] < US_STOP || D[TOF_FRONT] < TOF_STOP) { fullStop(); running = false; wait = 0; }
    } else {
      setVW(0, 0);
      int c, r; cellOf(px, py, c, r);
      if (!inGrid(c, r)) { c = 0; r = 0; }
      if (!nextPlan(c, r, h)) { logf_("no path?? stuck at c%d r%d", c + 1, r + 1); return; }
      if (pathN == 1) {
        if (c == END_C && r == END_R && !exploring) return;
        wait++;
        if (wait > 6) {                      // looking at the cells ahead: decide on the votes so far
          int cc = c, cr = r;
          for (int k = 0; k < 2; k++) {
            if (edge(cc, cr, h) != OPEN) break;
            cc += DC[h]; cr += DR[h];
            if (inGrid(cc, cr) && seen[cc][cr] < 0) decideColour(cc, cr);
          }
        }
        if (wait > 12) {
          int cc = c + DC[h], cr = r + DR[h];
          for (int k = 0; k < 2; k++) { if (inGrid(cc, cr) && seen[cc][cr] < 0) seen[cc][cr] = FLOOR; cc += DC[h]; cr += DR[h]; }
        }
        continue;
      }
      h = pathH[1];
      if (fabsf(wrapA(HANG[h] - yaw)) > 0.02f) turnTo(HANG[h], 0.02f);
      running = true;
    }
    if (tSim - lastMove > 6.0f) {
      logf_("wiggle, no progress for 6s");
      for (int i = 0; i < 20; i++) { setVW(-0.08f, 0); step(); }
      fullStop();
      lastMove = tSim; running = false;
    }
  }
}

// ============================================================ SECTION 1: grid + A*
#define G 0.02f
#define NX 122
#define NY 68
static const float R_HARD = 0.086f, R_INF = 0.096f, R_SOFT = 0.14f;   // 75 mm body + grid margin
uint8_t occ[NX * NY], wallCell[NX * NY];       // 8 KB each: internal RAM
float cellX(int k);
float cellY(int k);
float *clearD;                                 // 33 KB  (PSRAM)
bool newObs = false;
struct Off { int8_t di, dj; float dd; };
Off offs[300]; int nOff = 0;

void mark(int i, int j) {
  if (i < 0 || i >= NX || j < 0 || j >= NY || occ[j * NX + i]) return;
  occ[j * NX + i] = 1; newObs = true;
  for (int k = 0; k < nOff; k++) {
    int a = i + offs[k].di, b = j + offs[k].dj;
    if (a >= 0 && a < NX && b >= 0 && b < NY && offs[k].dd < clearD[b * NX + a]) clearD[b * NX + a] = offs[k].dd;
  }
}
void markRect(float x0, float y0, float x1, float y1) {
  for (int j = max(0, (int)(y0 / G)); j <= min(NY - 1, (int)(y1 / G)); j++)
    for (int i = max(0, (int)(x0 / G)); i <= min(NX - 1, (int)(x1 / G)); i++) mark(i, j);
}
void gridInit() {
  memset(occ, 0, sizeof(occ));
  for (int k = 0; k < NX * NY; k++) clearD[k] = 9.0f;
  nOff = 0;
  for (int dj = -8; dj <= 8; dj++) for (int di = -8; di <= 8; di++) {
    float dd = hypotf(di, dj) * G;
    if (dd <= R_SOFT + 1e-6f && nOff < 300) offs[nOff++] = {(int8_t)di, (int8_t)dj, dd};
  }
  markRect(0, 0, 2.44f, 0.006f); markRect(0, 0, 0.006f, 1.22f); markRect(2.434f, 0, 2.44f, 1.22f);
  markRect(0, 1.214f, 0.101f, 1.36f); markRect(0.339f, 1.214f, 2.44f, 1.36f);
  memcpy(wallCell, occ, sizeof(occ));
}

// drop sensed obstacles within rad (arena walls stay) and rebuild the clearance map
void forgetNear(float rad) {
  for (int k = 0; k < NX * NY; k++)
    if (occ[k] && !wallCell[k] && hypotf(cellX(k) - px, cellY(k) - py) < rad) occ[k] = 0;
  static uint8_t keep[NX * NY];
  memcpy(keep, occ, sizeof(occ));
  memset(occ, 0, sizeof(occ));
  for (int k = 0; k < NX * NY; k++) clearD[k] = 9.0f;
  for (int k = 0; k < NX * NY; k++) if (keep[k]) mark(k % NX, k / NX);
  newObs = true;
}

void s1Sense() {
  bool fix = nearCardinal(0.05f);
  for (int n = 0; n < N_SENS; n++) {
    float d = D[n];
    if (d >= rngMax(n) || (SENS[n].sonar && d > US_MAP_MAX)) continue;
    if (!SENS[n].sonar && d > TOF_MAP_MAX) continue;     // ToF: noisy far away
    float ox, oy, a; ray(n, ox, oy, a);
    float hx = ox + d * cosf(a), hy = oy + d * sinf(a);
    if (fix && d > TOF_MIN && axisSens(n)) {
      const float faces[4] = {0.006f, 2.434f, 0.006f, 1.214f};
      for (int f = 0; f < 4; f++) {
        bool isx = f < 2;
        if (isx && fabsf(sinf(a)) < 0.06f && fabsf(hx - faces[f]) < 0.03f) px += 0.25f * (faces[f] - hx);
        else if (!isx && fabsf(cosf(a)) < 0.06f && fabsf(hy - faces[f]) < 0.03f &&
                 (faces[f] < 1 || hx < 0.09f || hx > 0.35f)) py += 0.25f * (faces[f] - hy);
      }
    }
    if (hx > 0.012f && hx < 2.428f && hy > 0.012f && hy < 1.208f) {
      mark((int)(hx / G), (int)(hy / G));
      if (SENS[n].sonar) mark((int)((hx + 0.01f * cosf(a)) / G), (int)((hy + 0.01f * sinf(a)) / G));
    }
  }
}

// A* -> path cells in apath (start excluded), returns count
int apath[NX * NY / 4], apathN = 0;
struct PQ { float f, d; int k; };
PQ *heap; int heapN = 0;                       // 100 KB (PSRAM)
void hpush(PQ e) { int i = heapN++; heap[i] = e; while (i && heap[(i - 1) / 2].f > heap[i].f) { PQ t = heap[i]; heap[i] = heap[(i - 1) / 2]; heap[(i - 1) / 2] = t; i = (i - 1) / 2; } }
PQ hpop() {
  PQ top = heap[0]; heap[0] = heap[--heapN]; int i = 0;
  while (true) {
    int l = 2 * i + 1, r = l + 1, m = i;
    if (l < heapN && heap[l].f < heap[m].f) m = l;
    if (r < heapN && heap[r].f < heap[m].f) m = r;
    if (m == i) break;
    PQ t = heap[i]; heap[i] = heap[m]; heap[m] = t; i = m;
  }
  return top;
}
static float *bestA; static int *prevA;         // 33 + 33 KB (PSRAM)
void gridAlloc() {
  clearD = (float *)bigAlloc(sizeof(float) * NX * NY);
  heap = (PQ *)bigAlloc(sizeof(PQ) * NX * NY);
  bestA = (float *)bigAlloc(sizeof(float) * NX * NY);
  prevA = (int *)bigAlloc(sizeof(int) * NX * NY);
}

bool astar(float sx, float sy, float gx, float gy, float hard = R_HARD) {
  int si = sx / G, sj = sy / G, gi = gx / G, gj = gy / G;
  int s = sj * NX + si, g = gj * NX + gi;
  for (int k = 0; k < NX * NY; k++) { bestA[k] = 1e9f; prevA[k] = -1; }
  bestA[s] = 0; heapN = 0; hpush({0, 0, s});
  static const int8_t DI[8] = {1, -1, 0, 0, 1, 1, -1, -1}, DJ[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  while (heapN) {
    PQ e = hpop();
    if (e.k == g) {
      int n = 0;
      for (int k = g; k != s && k >= 0 && n < (int)(sizeof(apath) / sizeof(int)); k = prevA[k]) apath[n++] = k;
      for (int i = 0; i < n / 2; i++) { int t = apath[i]; apath[i] = apath[n - 1 - i]; apath[n - 1 - i] = t; }
      apathN = n;
      return true;
    }
    if (e.d > bestA[e.k] || heapN >= NX * NY - 8) continue;
    int i = e.k % NX, j = e.k / NX;
    for (int q = 0; q < 8; q++) {
      int a = i + DI[q], b = j + DJ[q];
      if (a < 0 || a >= NX || b < 0 || b >= NY) continue;
      int k2 = b * NX + a;
      float cl = clearD[k2];
      if (cl < hard) continue;
      float m = cl < R_INF ? 25.0f : cl < R_SOFT ? 1.0f + 2.5f * (R_SOFT - cl) / (R_SOFT - R_INF) : 1.0f;
      float d2 = e.d + (q < 4 ? 1.0f : 1.41421356f) * m;
      if (d2 < bestA[k2]) {
        bestA[k2] = d2; prevA[k2] = e.k;
        int adx = abs(a - gi), ady = abs(b - gj);
        hpush({d2 + max(adx, ady) + 0.41421356f * min(adx, ady), d2, k2});
      }
    }
  }
  apathN = 0;
  return false;
}
float cellX(int k) { return (k % NX + 0.5f) * G; }
float cellY(int k) { return (k / NX + 0.5f) * G; }

bool lineOk(float x0, float y0, float x1, float y1) {
  float L = hypotf(x1 - x0, y1 - y0);
  int n = (int)(L / 0.01f) + 1;
  for (int k = 0; k <= n; k++) {
    float f = (float)k / n, x = x0 + (x1 - x0) * f, y = y0 + (y1 - y0) * f;
    int i = x / G, j = y / G;
    if (i < 0 || i >= NX || j < 0 || j >= NY) return false;
    if (clearD[j * NX + i] < (f * L < 0.08f ? R_HARD : R_INF)) return false;
  }
  return true;
}
void pickSeg(float &hd, float &dist) {
  const float Q = PI_F / 4;
  for (int k = apathN - 1; k >= 0; k--) {
    float dx = cellX(apath[k]) - px, dy = cellY(apath[k]) - py, L = hypotf(dx, dy);
    if (L < 0.03f) break;
    float a = atan2f(dy, dx), q = roundf(a / Q) * Q;
    if (fabsf(L * sinf(a - q)) > 0.016f) continue;
    float d = L * cosf(a - q);
    if (lineOk(px, py, px + d * cosf(q), py + d * sinf(q))) { hd = wrapA(q); dist = d; return; }
  }
  int k = min(apathN - 1, 2);
  hd = atan2f(cellY(apath[k]) - py, cellX(apath[k]) - px);
  dist = hypotf(cellY(apath[k]) - py, cellX(apath[k]) - px);
}

const float S1_GX = 0.22f, S1_GY = 1.27f;
bool s1Done() { return py > 1.12f && fabsf(px - 0.22f) < 0.03f; }

void runS1() {
  senseHook = s1Sense;
  int stuck = 0, fails = 0;
  while (!s1Done()) {
    bool ok = astar(px, py, S1_GX, S1_GY);
    if (!ok || apathN == 0) ok = astar(px, py, S1_GX, S1_GY, 0.078f);   // squeeze: body radius + 3 mm
    if (!ok || apathN == 0) {
      if (++fails >= 2) {
        logf_("S1: no way out on the map at %.3f %.3f - forgetting obstacles within 35 cm", px, py);
        forgetNear(0.35f); fails = 0;
      }
      turnTo(wrapA(yaw + 1.6f)); newObs = false; continue;
    }
    fails = 0;
    newObs = false;
    float hd, dist; pickSeg(hd, dist);
    if (fabsf(wrapA(hd - yaw)) > 0.01f) { turnTo(hd, 0.008f); if (newObs) continue; }
    float x0 = px, y0 = py, moved = 0;
    while (true) {
      step(); s1Sense();
      if (s1Done()) return;
      moved = (px - x0) * cosf(hd) + (py - y0) * sinf(hd);
      float rem = dist - moved;
      if (rem < 0.004f) break;
      if (newObs) { newObs = false; if (!lineOk(px, py, x0 + dist * cosf(hd), y0 + dist * sinf(hd))) break; }
      if (D[US_FRONT] < US_STOP || D[TOF_FRONT] < TOF_STOP) break;
      setVW(fminf(V_MAX, sqrtf(2 * ACC * rem) + 0.01f), 0);
    }
    fullStop();
    stuck = moved < 0.01f ? stuck + 1 : 0;
    if (stuck >= 3) {
      for (int i = 0; i < 15; i++) { setVW(-0.08f, 0); step(); s1Sense(); }
      fullStop(); stuck = 0;
    }
  }
}

// ============================================================ start detection
// Fix 1 (test plan): the robot may be placed a little crooked, and the gyro only knows how far it has turned.
// While it stands still, the two side ToF pairs (66 mm apart) give its angle to a straight wall beside it
// (the east wall in the START area, the chamber / maze walls at the maze entrance). Same signs as sideAlign().
void startHeading(int n = 15) {
  float sum[2] = {0, 0}; int cnt[2] = {0, 0};
  const int pr[2][2] = {{TOF_LF, TOF_LR}, {TOF_RF, TOF_RR}};
  for (int i = 0; i < n; i++) {
    step();
    for (int k = 0; k < 2; k++) {
      float df = D[pr[k][0]], dr = D[pr[k][1]];
      if (df > TOF_MIN && df < 0.25f && dr > TOF_MIN && dr < 0.25f) {
        sum[k] += atan2f(dr - df, SIDE_PAIR_DX) * (k == 0 ? 1.0f : -1.0f);
        cnt[k]++;
      }
    }
  }
  float th = 0; int m = 0;
  for (int k = 0; k < 2; k++)
    if (cnt[k] >= n / 2 && fabsf(sum[k] / cnt[k]) < 0.5f) { th += sum[k] / cnt[k]; m++; }
  if (!m) { logf_("start heading: no wall beside the robot - assuming it is straight"); return; }
  th /= m;
  float k = roundf(yaw / HP_F);
  float c = wrapA(k * HP_F + th - yaw);
  yawOff += c;
  yaw = wrapA(yaw + c);
  logf_("start heading: placed %.1f deg off straight (from the side ToF pairs)", th * 180.0f / PI_F);
}

int whereAmI() {
  startHeading();
  float k = roundf(yaw / HP_F);
  turnTo(k * HP_F); stopNow(3);
  float wd[4] = {9, 9, 9, 9}, wox[4] = {0}, woy[4] = {0};
  const int ax[4] = {TOF_FRONT, TOF_LF, TOF_RF, TOF_BACK};
  for (int i = 0; i < 4; i++) {
    float ox, oy, a; ray(ax[i], ox, oy, a);
    int q = ((int)roundf(wrapA(a) / HP_F) % 4 + 4) % 4;
    wd[q] = D[ax[i]]; wox[q] = ox - px; woy[q] = oy - py;
  }
  float e = wd[0], w = wd[2], so = wd[3];
  if (!(e < 0.3f && so < 0.3f) && w < 0.2f) {
    px = 0.101f + w - wox[2];
    turnTo(-HP_F, 0.01f); stopNow(3);
    py = D[US_FRONT] < 0.3f ? 1.214f + D[US_FRONT] + SENS[US_FRONT].x : 1.275f;
    logf_("starting at the maze entrance y~%.3f", py);
    return 2;
  }
  float xs = 2.434f - e - wox[0], ys = 0.006f + wd[3] - woy[3];
  if (xs > 2.12f && xs < 2.44f && ys > 0 && ys < 0.32f) { px = xs; py = ys; }
  else { px = START_X; py = START_Y; }
  logf_("start pose %.3f %.3f", px, py);
  return 1;
}

// ============================================================ SECTION 3
struct P2 { float x, y; };
static const float XT[4] = {0.50f, 0.90f, 1.30f, 1.70f};
static const float YL[4] = {2.60f, 2.90f, 3.20f, 3.50f};
static const float ROW_X[3] = {1.50f, 1.10f, 0.70f};
static const float COL_Y[3] = {2.75f, 3.05f, 3.35f};
static const int MAGIC[3][3] = {{6, 1, 8}, {7, 5, 3}, {2, 9, 4}};
P2 POS_CELL[10];
const P2 EXIT_JUNC = {2.21f, 3.05f}, COLLECT_JUNC = {2.006f, 3.05f}, BOARD_OUT = {0.50f, 3.05f}, END_PT = {0.13f, 3.05f};
const P2 COLLECT_G = {2.006f, 2.70f}, COLLECT_B = {2.006f, 3.40f};
struct Seg { P2 a, b; };
Seg LINES[12]; int nLines = 0;
float GS_WHITE = 350, GS_BLACK = 0;
float GS_Y[N_GS];

void s3Init() {
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) POS_CELL[MAGIC[i][j]] = {ROW_X[i], COL_Y[j]};
  nLines = 0;
  LINES[nLines++] = {{2.21f, 2.44f}, {2.21f, 3.05f}};
  LINES[nLines++] = {{1.70f, 3.05f}, {2.21f, 3.05f}};
  LINES[nLines++] = {{2.006f, 2.70f}, {2.006f, 3.40f}};
  LINES[nLines++] = {{0.13f, 3.05f}, {0.50f, 3.05f}};
  for (int i = 0; i < 4; i++) LINES[nLines++] = {{0.50f, YL[i]}, {1.70f, YL[i]}};
  for (int i = 0; i < 4; i++) LINES[nLines++] = {{XT[i], 2.60f}, {XT[i], 3.50f}};
  for (int i = 0; i < N_GS; i++) GS_Y[i] = (i - (N_GS - 1) / 2.0f) * GS_PITCH;
}

float gsDark(int i) { float span = fmaxf(20.f, GS_WHITE - GS_BLACK); return clampf((GS_WHITE - GSV[i]) / span, 0, 1); }
bool lineReading(float &m, int &nd) {
  float s = 0, sy = 0; nd = 0;
  for (int i = 0; i < N_GS; i++) { float d = gsDark(i); s += d; sy += d * GS_Y[i]; nd += d > 0.5f; }
  if (s < 0.6f) { nd = 0; return false; }
  m = sy / s; return true;
}
void calibrateLine() {
  float lo = 1e9, hi = -1e9, t0 = yaw;
  const float tg[3] = {t0 + 0.35f, t0 - 0.35f, t0};
  for (int k = 0; k < 3; k++) {
    turnTo(wrapA(tg[k]), 0.03f);
    for (int i = 0; i < 3; i++) { step(); for (int j = 0; j < N_GS; j++) { lo = fminf(lo, GSV[j]); hi = fmaxf(hi, GSV[j]); } }
  }
  if (hi - lo > 60) { GS_WHITE = hi; GS_BLACK = lo; }
  logf_("[S3] line sensors calibrated: white=%.0f black=%.0f", GS_WHITE, GS_BLACK);
}

void lineCorrect(const Seg &sg) {
  float L = hypotf(sg.b.x - sg.a.x, sg.b.y - sg.a.y), ux = (sg.b.x - sg.a.x) / L, uy = (sg.b.y - sg.a.y) / L;
  float nx = -uy, ny = ux, c = cosf(yaw), s = sinf(yaw), lx = -s, ly = c, den = nx * lx + ny * ly;
  if (fabsf(den) < 0.7f) return;
  float m; int nd;
  if (!lineReading(m, nd) || nd >= N_GS - 1) return;
  float sx = px + c * LINE_X, sy = py + s * LINE_X;
  float e = m - (-(nx * (sx - sg.a.x) + ny * (sy - sg.a.y)) / den);
  if (fabsf(e) < 0.07f) { px -= 0.25f * e * lx; py -= 0.25f * e * ly; }
}

uint32_t crossDone = 0;      // bit per line index
void crossingCorrect(float ux, float uy) {
  float m; int nd;
  lineReading(m, nd);
  if (nd < N_GS - 1) return;
  float c = cosf(yaw), s = sinf(yaw), sx = px + c * LINE_X, sy = py + s * LINE_X;
  for (int i = 0; i < nLines; i++) {
    const Seg &L = LINES[i];
    bool vert = fabsf(L.b.x - L.a.x) < 1e-6f;
    float err;
    if (fabsf(ux) > 0.9f && vert && sy >= fminf(L.a.y, L.b.y) - 0.01f && sy <= fmaxf(L.a.y, L.b.y) + 0.01f) err = L.a.x - sx;
    else if (fabsf(uy) > 0.9f && !vert && sx >= fminf(L.a.x, L.b.x) - 0.01f && sx <= fmaxf(L.a.x, L.b.x) + 0.01f) err = L.a.y - sy;
    else continue;
    if (fabsf(err) < 0.10f && !(crossDone & (1u << i))) {
      crossDone |= 1u << i;
      if (fabsf(ux) > 0.9f) px += 0.8f * err; else py += 0.8f * err;
    }
  }
}

bool onSeg(P2 p, P2 a, P2 b, float tol = 0.006f) {
  if (fabsf(a.x - b.x) < 1e-6f) return fabsf(p.x - a.x) < tol && p.y >= fminf(a.y, b.y) - tol && p.y <= fmaxf(a.y, b.y) + tol;
  return fabsf(p.y - a.y) < tol && p.x >= fminf(a.x, b.x) - tol && p.x <= fmaxf(a.x, b.x) + tol;
}
int segOf(P2 p, P2 q) {
  for (int i = 0; i < nLines; i++) if (onSeg(p, LINES[i].a, LINES[i].b, 0.03f) && onSeg(q, LINES[i].a, LINES[i].b, 0.006f)) return i;
  return -1;
}
void stopDrive() { while (fabsf(lastL) > 0.002f || fabsf(lastR) > 0.002f) { driveLR(0, 0); step(); } driveLR(0, 0); step(); }

void driveLine(float tx, float ty, float vMax = 0.20f, bool reverse = false, bool follow = true, float stopTol = 0.004f) {
  crossDone = 0;
  float x0 = px, y0 = py, L = hypotf(tx - x0, ty - y0);
  if (L < 0.003f) return;
  float ux = (tx - x0) / L, uy = (ty - y0) / L;
  int sg = follow ? segOf({x0, y0}, {tx, ty}) : -1;
  float hd = atan2f(uy, ux) + (reverse ? PI_F : 0);
  if (fabsf(wrapA(hd - yaw)) > 0.06f) turnTo(wrapA(hd), 0.012f);
  for (int n = 0; n < 3000; n++) {
    step();
    if (sg >= 0) { lineCorrect(LINES[sg]); crossingCorrect(ux, uy); }
    float rem = (tx - px) * ux + (ty - py) * uy;
    if (rem < stopTol) break;
    float lat = -(tx - px) * uy + (ty - py) * ux, e = wrapA(hd - yaw);
    float v = fminf(vMax, sqrtf(2 * 1.2f * rem) + 0.015f);
    driveLR(reverse ? -v : v, clampf(4.0f * e + 12.0f * lat, -1.5f, 1.5f));
  }
  stopDrive();
}

// ---- route graph on the black lines (Dijkstra over node + arrival direction)
#define MAXN 64
P2 nodes[MAXN]; int nNodes = 0;
int addNode(P2 p) {
  p.x = roundf(p.x * 10000) / 10000; p.y = roundf(p.y * 10000) / 10000;
  for (int i = 0; i < nNodes; i++) if (fabsf(nodes[i].x - p.x) < 1e-4f && fabsf(nodes[i].y - p.y) < 1e-4f) return i;
  if (nNodes < MAXN) nodes[nNodes++] = p;
  return nNodes - 1;
}
P2 snapP(P2 p) {
  P2 best = p; float bd = 1e9;
  for (int i = 0; i < nLines; i++) {
    const Seg &L = LINES[i]; P2 q;
    if (fabsf(L.b.x - L.a.x) < 1e-6f) q = {L.a.x, clampf(p.y, fminf(L.a.y, L.b.y), fmaxf(L.a.y, L.b.y))};
    else q = {clampf(p.x, fminf(L.a.x, L.b.x), fmaxf(L.a.x, L.b.x)), L.a.y};
    float d = hypotf(q.x - p.x, q.y - p.y);
    if (d < bd) { bd = d; best = q; }
  }
  return best;
}
int routeOut[MAXN], routeN = 0;
bool route(P2 a, P2 b) {
  a = snapP(a); b = snapP(b);
  nNodes = 0;
  for (int i = 0; i < nLines; i++) { addNode(LINES[i].a); addNode(LINES[i].b); }
  for (int i = 0; i < 4; i++) { for (int j = 0; j < 4; j++) addNode({XT[i], YL[j]}); for (int j = 0; j < 3; j++) addNode({XT[i], COL_Y[j]}); }
  for (int i = 0; i < 3; i++) for (int j = 0; j < 4; j++) addNode({ROW_X[i], YL[j]});
  addNode(COLLECT_JUNC);
  int ia = addNode(a), ib = addNode(b);
  // adjacency: consecutive nodes along each line
  static int adj[MAXN][8]; static float adjW[MAXN][8]; static int nAdj[MAXN];
  memset(nAdj, 0, sizeof(nAdj));
  for (int l = 0; l < nLines; l++) {
    int on[MAXN], n = 0;
    for (int i = 0; i < nNodes; i++) if (onSeg(nodes[i], LINES[l].a, LINES[l].b)) on[n++] = i;
    for (int i = 1; i < n; i++) for (int j = i; j > 0; j--) {
      float dj = hypotf(nodes[on[j]].x - LINES[l].a.x, nodes[on[j]].y - LINES[l].a.y);
      float dk = hypotf(nodes[on[j - 1]].x - LINES[l].a.x, nodes[on[j - 1]].y - LINES[l].a.y);
      if (dj < dk) { int t = on[j]; on[j] = on[j - 1]; on[j - 1] = t; } else break;
    }
    for (int i = 0; i + 1 < n; i++) {
      int p = on[i], q = on[i + 1];
      float d = hypotf(nodes[p].x - nodes[q].x, nodes[p].y - nodes[q].y);
      if (nAdj[p] < 8) { adj[p][nAdj[p]] = q; adjW[p][nAdj[p]++] = d; }
      if (nAdj[q] < 8) { adj[q][nAdj[q]] = p; adjW[q][nAdj[q]++] = d; }
    }
  }
  // Dijkstra over (node, arrival direction 0..4), 0.08 m penalty per turn
  const int ND = 5;
  static float best[MAXN * 5]; static int prev[MAXN * 5]; static bool done[MAXN * 5];
  for (int i = 0; i < MAXN * ND; i++) { best[i] = 1e9; prev[i] = -1; done[i] = false; }
  best[ia * ND + 4] = 0;
  int goal = -1;
  while (true) {
    int s = -1; float bd = 1e9;
    for (int i = 0; i < nNodes * ND; i++) if (!done[i] && best[i] < bd) { bd = best[i]; s = i; }
    if (s < 0) break;
    done[s] = true;
    int n = s / ND, hd = s % ND;
    if (n == ib) { goal = s; break; }
    for (int k = 0; k < nAdj[n]; k++) {
      int m = adj[n][k];
      float dx = nodes[m].x - nodes[n].x, dy = nodes[m].y - nodes[n].y;
      int h2 = fabsf(dx) > fabsf(dy) ? (dx > 0 ? 0 : 1) : (dy > 0 ? 2 : 3);
      float d2 = bd + adjW[n][k] + (hd != 4 && hd != h2 ? 0.08f : 0);
      if (d2 < best[m * ND + h2]) { best[m * ND + h2] = d2; prev[m * ND + h2] = s; }
    }
  }
  if (goal < 0) { routeN = 0; return false; }
  int tmp[MAXN * 5], n = 0;
  for (int s = goal; s >= 0; s = prev[s]) tmp[n++] = s / ND;
  routeN = n;
  for (int i = 0; i < n; i++) routeOut[i] = tmp[n - 1 - i];
  return true;
}
bool go(P2 t, float vMax = 0.20f) {
  if (!route({px, py}, t)) { logf_("[S3] no route to %.2f %.2f", t.x, t.y); return false; }
  for (int i = 1; i < routeN; i++) driveLine(nodes[routeOut[i]].x, nodes[routeOut[i]].y, vMax);
  return true;
}

// ---- clues
void mazeClues(int &nb, int &ng, int &R, int &L) {
  nb = ng = 0;
  for (int c = 0; c < COLS; c++) for (int r = 0; r < ROWS; r++) { nb += seen[c][r] == BLUE; ng += seen[c][r] == GREEN; }
  // Dijkstra on (cells, turns) lexicographic, entering cell (0,0) heading +y
  static int bc[NST], bt[NST], bR[NST], bL[NST]; static bool done[NST];
  for (int i = 0; i < NST; i++) { bc[i] = 1 << 20; bt[i] = 0; done[i] = false; }
  int s0 = sid(0, 0, 0); bc[s0] = 0; bt[s0] = 0; bR[s0] = bL[s0] = 0;
  int res = -1;
  while (true) {
    int s = -1;
    for (int i = 0; i < NST; i++) if (!done[i] && bc[i] < (1 << 20) &&
        (s < 0 || bc[i] < bc[s] || (bc[i] == bc[s] && bt[i] < bt[s]))) s = i;
    if (s < 0) break;
    done[s] = true;
    int h = s % 4, r = (s / 4) % ROWS, c = s / 4 / ROWS;
    if (c == END_C && r == END_R) { res = s; break; }
    for (int h2 = 0; h2 < 4; h2++) {
      if (edge(c, r, h2) != OPEN || h2 == (h + 2) % 4) continue;
      int nc = c + DC[h2], nr = r + DR[h2];
      if (!inGrid(nc, nr)) continue;
      int t = sid(nc, nr, h2), cells = bc[s] + 1, turns = bt[s] + (h2 != h);
      if (!done[t] && (cells < bc[t] || (cells == bc[t] && turns < bt[t]))) {
        bc[t] = cells; bt[t] = turns;
        bR[t] = bR[s] + (h2 == (h + 1) % 4); bL[t] = bL[s] + (h2 == (h + 3) % 4);
      }
    }
  }
  R = res >= 0 ? bR[res] : 0; L = res >= 0 ? bL[res] : 0;
  logf_("[S3] clues: blue tiles=%d green tiles=%d right turns=%d left turns=%d (path %d cells)", nb, ng, R, L, res >= 0 ? bc[res] : -1);
}

int board[10];   // 0 empty, BLUE, GREEN
int winningCell(int clan, int line[3]) {
  int lines[8][3];
  int n = 0;
  for (int j = 0; j < 3; j++) { lines[n][0] = MAGIC[0][j]; lines[n][1] = MAGIC[1][j]; lines[n++][2] = MAGIC[2][j]; }  // along the entry
  for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) lines[n][j] = MAGIC[i][j]; n++; }
  lines[n][0] = 6; lines[n][1] = 5; lines[n++][2] = 4;
  lines[n][0] = 8; lines[n][1] = 5; lines[n++][2] = 2;
  for (int k = 0; k < n; k++) {
    int mine = 0, empty = 0, e = 0;
    for (int j = 0; j < 3; j++) { if (board[lines[k][j]] == clan) mine++; else if (!board[lines[k][j]]) { empty++; e = lines[k][j]; } }
    if (mine == 2 && empty == 1) { memcpy(line, lines[k], sizeof(int) * 3); return e; }
  }
  return 0;
}

// ---- gripper
void gripSet(bool closed, float sec = 0.6f) {
  out.servoL = closed ? SERVO_L_CLOSED : SERVO_L_OPEN;
  out.servoR = closed ? SERVO_R_CLOSED : SERVO_R_OPEN;
  waitS(sec);
}
void pickPiece(int clan) {
  P2 cp = clan == BLUE ? COLLECT_B : COLLECT_G;
  float sgn = cp.y > COLLECT_JUNC.y ? 1 : -1;
  go(COLLECT_JUNC);
  driveLine(cp.x, cp.y - sgn * 0.135f, 0.10f);
  turnTo(wrapA(atan2f(-sgn, 0)), 0.01f);
  gripSet(false);
  driveLine(cp.x, cp.y - sgn * POCKET_OFF, 0.05f, true, false);
  gripSet(true, 0.8f);
  logf_("[S3] picked %s piece", clan == BLUE ? "BLUE" : "GREEN");
  driveLine(COLLECT_JUNC.x, COLLECT_JUNC.y, 0.12f);
}
void placePiece(int pos) {
  P2 c = POS_CELL[pos];
  float ax = c.x + 0.20f;
  go({ax, c.y}, 0.14f);
  turnTo(0.0f, 0.008f);
  driveLine(c.x + POCKET_OFF, c.y, 0.05f, true, false);
  gripSet(false, 0.6f);
  logf_("[S3] piece released in position %d at (%.2f, %.2f)", pos, c.x, c.y);
  driveLine(ax, c.y, 0.08f, false, false);
  gripSet(true, 0.4f);
}

// ---- inspection: drive the two inner long lines, the side ToF pairs find occupied cells
int inspectBoard(int occOut[10]) {
  int hits[10] = {0};
  for (int k = 0; k < 2; k++) {
    float y = k == 0 ? 2.90f : 3.20f;
    go({1.70f, y}); turnTo(PI_F, 0.01f);
    const float tx = 0.52f;
    Seg sg = {{0.50f, y}, {1.70f, y}};
    for (int n = 0; px > tx + 0.004f && n < 3000; n++) {
      step(); lineCorrect(sg);
      for (int s = TOF_LF; s <= TOF_RR; s++) {
        float d = D[s];
        if (d >= 0.20f || d <= TOF_MIN) continue;
        float ox, oy, a; ray(s, ox, oy, a);
        float hx = ox + d * cosf(a), hy = oy + d * sinf(a);
        for (int p = 1; p <= 9; p++) if (hypotf(hx - POS_CELL[p].x, hy - POS_CELL[p].y) < 0.06f) hits[p]++;
      }
      float e = wrapA(PI_F - yaw), lat = -(y - py);
      driveLR(fminf(0.12f, sqrtf(2 * 1.2f * (px - tx)) + 0.015f), clampf(4 * e + 12 * lat, -1.5f, 1.5f));
    }
    stopDrive();
  }
  int n = 0;
  for (int p = 1; p <= 9; p++) if (hits[p] >= 3) occOut[n++] = p;    // 50 Hz here vs 60 Hz in the simulation
  logf_("[S3] inspection: %d occupied positions", n);
  return n;
}
int lookColour(int pos) {
  P2 c = POS_CELL[pos];
  go({c.x + 0.20f, c.y}, 0.12f);
  turnTo(PI_F, 0.01f);
  out.out |= OUT_TCS_FRONT;
  driveLine(c.x + 0.108f, c.y, 0.05f, false, false);
  int nb = 0, ng = 0;
  for (int i = 0; i < 15; i++) { step(); int k = frontColour(); nb += k == BLUE; ng += k == GREEN; }
  out.out &= ~OUT_TCS_FRONT;
  int col = nb > ng ? BLUE : ng > nb ? GREEN : 0;
  logf_("[S3] front colour sensor at position %d reads %s (%d/%d)", pos, col == BLUE ? "BLUE" : col == GREEN ? "GREEN" : "?", nb, ng);
  driveLine(c.x + 0.20f, c.y, 0.06f, true, false);
  return col;
}

void runPhase3(bool restart) {
  logf_("========== SECTION 3: BATTLE OF THE TWO GUARDIANS ==========");
  int nb, ng, R, L;
  if (restart) {
    prefs.begin("gb", true);
    nb = prefs.getInt("nb", 0); ng = prefs.getInt("ng", 0); R = prefs.getInt("R", 0); L = prefs.getInt("L", 0);
    prefs.end();
    logf_("[S3] restart - clues from flash: blue=%d green=%d right=%d left=%d", nb, ng, R, L);
  } else {
    mazeClues(nb, ng, R, L);
    prefs.begin("gb", false);
    prefs.putInt("nb", nb); prefs.putInt("ng", ng); prefs.putInt("R", R); prefs.putInt("L", L);
    prefs.end();
  }
  int clan = nb >= ng ? BLUE : GREEN;
  out.out = (out.out & ~(OUT_LED_BLUE | OUT_LED_GREEN)) | (clan == BLUE ? OUT_LED_BLUE : OUT_LED_GREEN);
  logf_("[S3] robot plays for the %s", clan == BLUE ? "Blue Sentinels (cylinder)" : "Green Keepers (cube)");
  {
    char l1[24], l2[24];
    snprintf(l1, sizeof(l1), "B%d G%d R%d L%d", nb, ng, R, L);
    snprintf(l2, sizeof(l2), "S3 %s", clan == BLUE ? "BLUE cylinder" : "GREEN cube");
    show(l1, l2);
  }

  memset(board, 0, sizeof(board));
  int clueCol[4] = {BLUE, GREEN, BLUE, GREEN}, clueVal[4] = {nb, ng, R, L};
  int unresolvedB = 0, unresolvedG = 0;
  for (int i = 0; i < 4; i++) {
    int v = clueVal[i];
    if (v >= 1 && v <= 9 && !board[v]) { board[v] = clueCol[i]; logf_("[S3] clue %d -> position %d", i + 1, v); }
    else { clueCol[i] == BLUE ? unresolvedB++ : unresolvedG++; logf_("[S3] clue %d value %d unusable", i + 1, v); }
  }

  out.out &= ~OUT_TCS_FLOOR;
  turnTo(HP_F, 0.01f);
  driveLine(EXIT_JUNC.x, 2.44f, 0.12f, false, false);
  calibrateLine();
  driveLine(EXIT_JUNC.x, EXIT_JUNC.y, 0.18f);
  pickPiece(clan);

  if (unresolvedB + unresolvedG) {
    int occ_[10], n = inspectBoard(occ_);
    for (int i = 0; i < n; i++) {
      int p = occ_[i];
      if (board[p]) continue;
      if (unresolvedB && !unresolvedG) board[p] = BLUE;
      else if (unresolvedG && !unresolvedB) board[p] = GREEN;
      else { int c = lookColour(p); board[p] = c ? c : (unresolvedB >= unresolvedG ? BLUE : GREEN); }
      if (board[p] == BLUE && unresolvedB) unresolvedB--;
      if (board[p] == GREEN && unresolvedG) unresolvedG--;
    }
    if (n) for (int p = 1; p <= 9; p++) {
      bool seenP = false;
      for (int i = 0; i < n; i++) seenP |= occ_[i] == p;
      if (board[p] && !seenP) { logf_("[S3] position %d expected a piece but none was seen", p); board[p] = 0; }
    }
  }
  for (int i = 2; i >= 0; i--) {
    char row[16]; int k = 0;
    for (int j = 0; j < 3; j++) { int v = board[MAGIC[i][j]]; row[k++] = v == BLUE ? 'B' : v == GREEN ? 'G' : '.'; row[k++] = ' '; }
    row[k] = 0; logf_("        %s", row);
  }
  int line[3] = {0, 0, 0};
  int win = winningCell(clan, line);
  if (!win) {
    const int pref[9] = {1, 5, 6, 8, 7, 3, 2, 9, 4};
    for (int i = 0; i < 9; i++) if (!board[pref[i]]) { win = pref[i]; break; }
    logf_("[S3] no winning move - placing in position %d", win);
  } else logf_("[S3] winning move: position %d completes %d-%d-%d", win, line[0], line[1], line[2]);
  placePiece(win);
  go(BOARD_OUT, 0.18f);
  driveLine(END_PT.x, END_PT.y, 0.12f);
  logf_("[S3] reached the END zone  t=%.1fs", tSim);
  { char l2[24]; snprintf(l2, sizeof(l2), "END  t=%.1f s", tSim); show("MISSION COMPLETE", l2); }
  logf_("==================== MISSION COMPLETE ====================");
  out.out |= OUT_BUZZER; waitS(0.5f); out.out &= ~OUT_BUZZER;
}

// ============================================================ white balance (CAL button)
void whiteBalance() {
  out.out |= OUT_TCS_FLOOR | OUT_TCS_FRONT;
  float f[3] = {0, 0, 0}, x[3] = {0, 0, 0}; int n = 0;
  for (int i = 0; i < 100 && n < 20; i++) {
    step();
    if (tele.flags & FLAG_TCS_NEW) { for (int k = 0; k < 3; k++) { f[k] += tele.tcsFloor[k]; x[k] += tele.tcsFront[k]; } n++; }
  }
  if (n) for (int k = 0; k < 3; k++) { whiteF[k] = fmaxf(1, f[k] / n); whiteX[k] = fmaxf(1, x[k] / n); }
  // camera: average the lower middle of the picture (the white paper on the floor in front of the robot)
  float cr = 0, cg = 0, cb = 0; int cn = 0;
  for (int v = CAM_H / 2; v < CAM_H; v++)
    for (int u = CAM_W / 4; u < 3 * CAM_W / 4; u++) {
      uint16_t p = camImg[v * CAM_W + u];
      float r, g, b; camRGB(p, CamGain(), r, g, b);
      cr += r; cg += g; cb += b; cn++;
    }
  if (cn && cr > 30 * cn && cg > 30 * cn && cb > 30 * cn) {
    float m = fmaxf(cr, fmaxf(cg, cb));
    camGain.r = m / cr; camGain.g = m / cg; camGain.b = m / cb;     // equalise the three channels on white
  }
  prefs.begin("gb", false);
  prefs.putBytes("wF", whiteF, sizeof(whiteF)); prefs.putBytes("wX", whiteX, sizeof(whiteX));
  prefs.putBytes("cg", &camGain, sizeof(camGain));
  prefs.end();
  logf_("camera gains R %.2f G %.2f B %.2f", camGain.r, camGain.g, camGain.b);
  logf_("white balance floor %.0f %.0f %.0f  front %.0f %.0f %.0f", whiteF[0], whiteF[1], whiteF[2], whiteX[0], whiteX[1], whiteX[2]);
  out.out &= ~OUT_TCS_FRONT;
}

// ============================================================ OLED text (2 x 21 characters, shown by the Mega)
void show(const char *l1, const char *l2) {
  static char last[42];
  TextMsg m;
  memset(&m, ' ', sizeof(m));
  memcpy(m.line[0], l1, strnlen(l1, 21));
  memcpy(m.line[1], l2, strnlen(l2, 21));
  if (!memcmp(last, &m, sizeof(m))) return;
  memcpy(last, &m, sizeof(m));
  uint8_t f[4 + sizeof(TextMsg) + 1];
  f[0] = 0xA5; f[1] = 0x5A; f[2] = PKT_TEXT; f[3] = sizeof(TextMsg);
  memcpy(f + 4, &m, sizeof(m));
  uint8_t s = PKT_TEXT + sizeof(TextMsg);
  for (uint8_t i = 0; i < sizeof(TextMsg); i++) s += f[4 + i];
  f[4 + sizeof(TextMsg)] = s;
  Link.write(f, sizeof(f));
}

// ============================================================ calibration saved by the test bench (esp32s3_testbench)
void loadCalibration() {
  prefs.begin("gb", true);
  float cpr = prefs.getFloat("cpr", 0), lens = prefs.getFloat("lens", 0);
  uint8_t grip[4];
  if (prefs.getBytes("grip", grip, 4) == 4) { SERVO_L_OPEN = grip[0]; SERVO_L_CLOSED = grip[1]; SERVO_R_OPEN = grip[2]; SERVO_R_CLOSED = grip[3]; }
  prefs.getBytes("tofo", TOF_OFF_MM, sizeof(TOF_OFF_MM));
  prefs.end();
  if (cpr > 100) ENC_CPR = cpr;
  if (lens > 0.2f && lens < 2.5f) { LENS_HFOV = lens; CAM_FOV = lens < 0.90f ? lens : 0.90f; }
  out.servoL = SERVO_L_CLOSED; out.servoR = SERVO_R_CLOSED;
  logf_("calibration: ENC_CPR %.0f  lens %.2f rad  grip L %u/%u R %u/%u", ENC_CPR, LENS_HFOV, SERVO_L_OPEN, SERVO_L_CLOSED,
        SERVO_R_OPEN, SERVO_R_CLOSED);
}

// ============================================================ main
void setup() {
  Serial.begin(115200);
  Link.setRxBufferSize(1024);
  Link.begin(LINK_BAUD, SERIAL_8N1, LINK_RX_PIN, LINK_TX_PIN);
#ifdef SIL
  // camera frames come from the Webots emulator
  loadCalibration();
#else
  loadCalibration();
  if (camBegin()) { xTaskCreatePinnedToCore(cameraTask, "cam", 4096, nullptr, 1, nullptr, 0); logf_("camera PID 0x%02X", camPID); }
  else logf_("CAMERA NOT FOUND - tiles will come from the floor colour sensor only");
  if (!psramFound()) logf_("NO PSRAM: set Tools > PSRAM > OPI PSRAM (N16R8)");
#endif
  gridAlloc();
  buildPix();
  prefs.begin("gb", true);
  prefs.getBytes("wF", whiteF, sizeof(whiteF));
  prefs.getBytes("wX", whiteX, sizeof(whiteX));
  prefs.getBytes("cg", &camGain, sizeof(camGain));
  prefs.end();
  mazeInit(); gridInit(); s3Init();
  delay(500);
  logf_("GuardianBot v4.1 brain ready - K1 start, K3 Section 3 restart, K4 white balance");
}

void loop() {
  // idle: keep the Mega alive, wait for a key
  setVW(0, 0); step();
  static uint8_t lastKeys = 0;
  uint8_t k = tele.keys, rise = k & ~lastKeys;
  lastKeys = k;
  char l2[24];
  static uint32_t lastFrames = 0, lastCheck = 0;
  static bool camAlive = false;
  if (millis() - lastCheck > 1000) { camAlive = camFrames != lastFrames; lastFrames = camFrames; lastCheck = millis(); }
  snprintf(l2, sizeof(l2), "cam %s ToF %s", camAlive ? "ok" : "--", (tele.flags & FLAG_TOF_OK) ? "ok" : "CHECK");
  show("READY 1:GO 3:S3 4:CAL", l2);
  if (rise & (KEY_START | KEY_MAZE | KEY_S3)) {
    bool s3 = rise & KEY_S3;
    show(s3 ? "SECTION 3 RESTART" : "GO", "keep still 1.4 s");
    out.out = OUT_TCS_FLOOR; out.ctl = CTL_ZERO_ODOM | CTL_GYRO_CAL;
    for (int i = 0; i < 70; i++) step();                // gyro bias re-measure (robot still, 1.4 s)
    firstStep = true; yawOff = HP_F; tSim = 0;
    step();
    if (s3) {
      px = MX0 + (END_C + 0.5f) * CELL; py = MY0 + (END_R + 0.5f) * CELL;
      runPhase3(true);
    } else {
      mazeInit(); gridInit(); exploring = true;
      int sec = whereAmI();
      show(sec == 1 ? "SECTION 1" : "MAZE ENTRANCE START", "");
      if (sec == 1) { runS1(); logf_("section 1 done t=%.1fs", tSim); }
      show("SECTION 2 maze", "");
      enterMaze();
      runMaze();
      logf_("section 2 maze done t=%.1fs", tSim);
      runPhase3(false);
    }
    senseHook = nullptr;
    out.out &= ~(OUT_TCS_FLOOR | OUT_TCS_FRONT);
  }
  if (rise & KEY_CAL) { show("CAL: white paper", "under + in front"); whiteBalance(); }
}

/* c_section1.ino - Trial One: cross the fallen stones
 * 100 mm occupancy grid of Section 1 built from ultrasonic hits while driving,
 * 8-neighbour A* to the maze entrance, re-planned whenever something blocks the way. */

static uint8_t s1grid[S1_COLS][S1_ROWS];          /* 0 free, 1 obstacle, 2 inflated */

bool s1in(int c, int r) { return c >= 0 && c < S1_COLS && r >= 0 && r < S1_ROWS; }

void s1Mark(float x, float y) {
  int c = (int)(x / S1_GRID_MM), r = (int)(y / S1_GRID_MM);
  if (!s1in(c, r)) return;
  if (s1grid[c][r] == 1) return;
  s1grid[c][r] = 1;
  for (int dc = -S1_INFLATE; dc <= S1_INFLATE; dc++)
    for (int dr = -S1_INFLATE; dr <= S1_INFLATE; dr++)
      if (s1in(c + dc, r + dr) && s1grid[c + dc][r + dr] == 0) s1grid[c + dc][r + dr] = 2;
}

void s1Init() {
  memset(s1grid, 0, sizeof s1grid);
  /* the wall between Section 1 and the maze, except the entrance door */
  for (int c = 0; c < S1_COLS; c++) {
    float cx = (c + 0.5f) * S1_GRID_MM;
    if (fabs(cx - S1_GOAL_X_MM) > 180) s1Mark(cx, 1215);
  }
}

/* record the three forward-looking ultrasonic hits into the grid */
void s1Sense() {
  readFLR();
  float th = poseTh * DEG_TO_RAD;
  if (usF < 600) { float d = usF + US_F_OFFSET_MM; s1Mark(poseX + d * cos(th), poseY + d * sin(th)); }
  if (usL < 400) { float d = usL + US_SIDE_OFFSET_MM; s1Mark(poseX + d * cos(th + HALF_PI), poseY + d * sin(th + HALF_PI)); }
  if (usR < 400) { float d = usR + US_SIDE_OFFSET_MM; s1Mark(poseX + d * cos(th - HALF_PI), poseY + d * sin(th - HALF_PI)); }
}

/* A*: returns number of waypoints written to wx/wy (mm), 0 if no path */
static int16_t  aParent[S1_COLS * S1_ROWS];
static uint16_t aG[S1_COLS * S1_ROWS];
static uint8_t  aClosed[S1_COLS * S1_ROWS];

int s1Plan(float *wx, float *wy, int maxW) {
  int sc = constrain((int)(poseX / S1_GRID_MM), 0, S1_COLS - 1), sr = constrain((int)(poseY / S1_GRID_MM), 0, S1_ROWS - 1);
  int gc = S1_GOAL_X_MM / S1_GRID_MM, gr = S1_GOAL_Y_MM / S1_GRID_MM;
  const int N = S1_COLS * S1_ROWS;
  for (int i = 0; i < N; i++) { aG[i] = 0xFFFF; aParent[i] = -1; aClosed[i] = 0; }
  int s = sc * S1_ROWS + sr, g = gc * S1_ROWS + gr;
  aG[s] = 0;
  for (;;) {
    int best = -1; uint32_t bf = 0xFFFFFFFF;
    for (int i = 0; i < N; i++) if (!aClosed[i] && aG[i] != 0xFFFF) {
      int c = i / S1_ROWS, r = i % S1_ROWS;
      uint32_t f = aG[i] + 10 * (uint32_t)max(abs(c - gc), abs(r - gr));
      if (f < bf) { bf = f; best = i; }
    }
    if (best < 0) return 0;
    if (best == g) break;
    aClosed[best] = 1;
    int c = best / S1_ROWS, r = best % S1_ROWS;
    for (int dc = -1; dc <= 1; dc++) for (int dr = -1; dr <= 1; dr++) {
      if (!dc && !dr) continue;
      int nc = c + dc, nr = r + dr;
      if (!s1in(nc, nr)) continue;
      if (s1grid[nc][nr] == 1 && !(nc == sc && nr == sr)) continue;
      if (dc && dr && (s1grid[c + dc][r] == 1 || s1grid[c][r + dr] == 1)) continue;   /* no corner cutting */
      uint16_t step = (dc && dr) ? 14 : 10;
      if (s1grid[nc][nr] == 2) step += 12;                                              /* inflated: allowed but costly */
      int ni = nc * S1_ROWS + nr;
      if (aG[best] + step < aG[ni]) { aG[ni] = aG[best] + step; aParent[ni] = best; }
    }
  }
  /* reconstruct, then keep only direction changes */
  static int16_t chain[S1_COLS * S1_ROWS]; int n = 0;
  for (int i = g; i >= 0 && n < N; i = aParent[i]) chain[n++] = i;
  int nw = 0, pdc = 99, pdr = 99;
  for (int k = n - 2; k >= 0 && nw < maxW; k--) {
    int c = chain[k] / S1_ROWS, r = chain[k] % S1_ROWS, pc = chain[k + 1] / S1_ROWS, pr = chain[k + 1] % S1_ROWS;
    int dc = c - pc, dr = r - pr;
    if (dc != pdc || dr != pdr || k == 0) {
      if (nw && k != 0) { wx[nw - 1] = (pc + 0.5f) * S1_GRID_MM; wy[nw - 1] = (pr + 0.5f) * S1_GRID_MM; }
      else { wx[nw] = (pc + 0.5f) * S1_GRID_MM; wy[nw] = (pr + 0.5f) * S1_GRID_MM; nw++; }
      pdc = dc; pdr = dr;
    }
  }
  wx[nw] = S1_GOAL_X_MM; wy[nw] = S1_GOAL_Y_MM; nw++;
  return nw;
}

void runSection1() {
  logf("===== SECTION 1: FALLEN STONES =====");
  s1Init();
  float wx[16], wy[16];
  unsigned long t0 = millis();
  int replans = 0;
  while (true) {
    float dx = S1_GOAL_X_MM - poseX, dy = S1_GOAL_Y_MM - poseY;
    if (sqrt(dx * dx + dy * dy) < 70) break;
    if (millis() - t0 > 90000) { logf("S1 timeout - pushing on"); break; }
    s1Sense();
    int nw = s1Plan(wx, wy, 16);
    if (!nw) { logf("S1: no path! clearing inflation and retrying"); for (int c = 0; c < S1_COLS; c++) for (int r = 0; r < S1_ROWS; r++) if (s1grid[c][r] == 2) s1grid[c][r] = 0; nw = s1Plan(wx, wy, 16); if (!nw) { driveStraight(-100, SPEED_SLOW, poseTh); continue; } }
    logf("S1 plan: %d waypoints from (%.0f,%.0f)", nw, poseX, poseY);
    /* follow the first waypoint(s), re-plan if something appears in front */
    bool blocked = false;
    for (int i = 0; i < nw && !blocked; i++) {
      float tx = wx[i], ty = wy[i];
      float ddx = tx - poseX, ddy = ty - poseY, d = sqrt(ddx * ddx + ddy * ddy);
      if (d < 20) continue;
      float h = atan2(ddy, ddx) * RAD_TO_DEG;
      if (fabs(wrapDeg(h - poseTh)) > 4) turnTo(h);
      /* drive in 100 mm chunks so we can scan and bail out */
      while (d > 15) {
        float chunk = min(d, 120.0f);
        int stopAt = (i == nw - 1) ? 0 : 150;
        float gone = driveStraight(chunk, (d > 300) ? SPEED_FAST : SPEED_NORMAL, h, false, stopAt);
        s1Sense();
        if (usF < 170 && i != nw - 1) { logf("S1 blocked at (%.0f,%.0f) F=%d", poseX, poseY, usF); blocked = true; replans++; break; }
        if (gone < chunk - 20 && usF < 250) { blocked = true; break; }
        ddx = tx - poseX; ddy = ty - poseY; d = sqrt(ddx * ddx + ddy * ddy);
      }
    }
    if (!blocked) break;
    if (replans > 25) { logf("S1 too many replans"); break; }
  }
  /* face the maze entrance and go in to the first cell centre */
  turnTo(90);
  logf("S1 done t=%.1fs at (%.0f,%.0f)", (millis() - t0) / 1000.0, poseX, poseY);
}

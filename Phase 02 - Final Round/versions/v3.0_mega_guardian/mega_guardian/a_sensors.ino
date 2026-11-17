/* a_sensors.ino - ultrasonic, TCS3200 colour, line array, MPU6050, buttons, battery */

void sensorsInit() {
  const uint8_t trig[] = { PIN_US_F_TRIG, PIN_US_L_TRIG, PIN_US_R_TRIG, PIN_US_B_TRIG };
  const uint8_t echo[] = { PIN_US_F_ECHO, PIN_US_L_ECHO, PIN_US_R_ECHO, PIN_US_B_ECHO };
  for (int i = 0; i < 4; i++) { pinMode(trig[i], OUTPUT); digitalWrite(trig[i], LOW); pinMode(echo[i], INPUT); }
  const uint8_t cs[] = { PIN_CF_S0, PIN_CF_S1, PIN_CF_S2, PIN_CF_S3, PIN_CF_LED, PIN_CG_S0, PIN_CG_S1, PIN_CG_S2, PIN_CG_S3, PIN_CG_LED };
  for (uint8_t i = 0; i < sizeof cs; i++) pinMode(cs[i], OUTPUT);
  pinMode(PIN_CF_OUT, INPUT); pinMode(PIN_CG_OUT, INPUT);
  /* TCS3200 frequency scaling 20 % (S0=1, S1=0) */
  digitalWrite(PIN_CF_S0, HIGH); digitalWrite(PIN_CF_S1, LOW);
  digitalWrite(PIN_CG_S0, HIGH); digitalWrite(PIN_CG_S1, LOW);
  digitalWrite(PIN_CF_LED, HIGH); digitalWrite(PIN_CG_LED, LOW);
}

/* ------------------------------------------------------------ ultrasonic */
int usPing(uint8_t trig, uint8_t echo) {
  digitalWrite(trig, LOW); delayMicroseconds(3);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long us = pulseIn(echo, HIGH, US_TIMEOUT_US);
  if (us == 0) return US_MAX_MM;
  long mm = us * 10 / 58;            /* 58 us per cm */
  return (int)min(mm, (long)US_MAX_MM);
}
int usMedian(uint8_t trig, uint8_t echo) {
  int a = usPing(trig, echo); delayMicroseconds(300);
  int b = usPing(trig, echo); delayMicroseconds(300);
  int c = usPing(trig, echo);
  if (a > b) { int t = a; a = b; b = t; }
  if (b > c) { int t = b; b = c; c = t; }
  if (a > b) { int t = a; a = b; b = t; }
  return b;
}
/* read all four (about 4 x 3 x <=12 ms worst case; typically ~15 ms) */
void readUltrasonics(bool median) {
  if (median) {
    usF = usMedian(PIN_US_F_TRIG, PIN_US_F_ECHO); usL = usMedian(PIN_US_L_TRIG, PIN_US_L_ECHO);
    usR = usMedian(PIN_US_R_TRIG, PIN_US_R_ECHO); usB = usMedian(PIN_US_B_TRIG, PIN_US_B_ECHO);
  } else {
    usF = usPing(PIN_US_F_TRIG, PIN_US_F_ECHO); usL = usPing(PIN_US_L_TRIG, PIN_US_L_ECHO);
    usR = usPing(PIN_US_R_TRIG, PIN_US_R_ECHO); usB = usPing(PIN_US_B_TRIG, PIN_US_B_ECHO);
  }
}
/* cheap: only the three we steer with */
void readFLR() {
  usF = usPing(PIN_US_F_TRIG, PIN_US_F_ECHO);
  usL = usPing(PIN_US_L_TRIG, PIN_US_L_ECHO);
  usR = usPing(PIN_US_R_TRIG, PIN_US_R_ECHO);
}

/* ------------------------------------------------------------- TCS3200 */
/* returns pulse period in us for one filter (lower = brighter) */
unsigned long tcsChannel(uint8_t s2, uint8_t s3, uint8_t out, bool b2, bool b3) {
  digitalWrite(s2, b2); digitalWrite(s3, b3);
  delayMicroseconds(300);
  unsigned long acc = 0; int n = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 8 && n < 6) { unsigned long p = pulseIn(out, LOW, 6000); if (p) { acc += p; n++; } }
  return n ? acc / n : 6000;
}
/* R,G,B as "brightness" 1/period (bigger = more of that colour) */
void tcsReadRGB(uint8_t s2, uint8_t s3, uint8_t out, float *r, float *g, float *b) {
  unsigned long pr = tcsChannel(s2, s3, out, LOW,  LOW);    /* red   filter */
  unsigned long pb = tcsChannel(s2, s3, out, LOW,  HIGH);   /* blue  filter */
  unsigned long pg = tcsChannel(s2, s3, out, HIGH, HIGH);   /* green filter */
  *r = 100000.0f / pr; *g = 100000.0f / pg; *b = 100000.0f / pb;
}
/* classify against calibration: normalise by white, then compare ratios.
 * returns TILE_BLUE / TILE_GREEN / TILE_NONE  */
int classifyColour(const ColourCal *cal, float r, float g, float b) {
  float nr = r / cal->whiteR, ng = g / cal->whiteG, nb = b / cal->whiteB;
  float sum = nr + ng + nb; if (sum < 0.05f) return TILE_NONE;
  nr /= sum; ng /= sum; nb /= sum;         /* chromaticity, white = 1/3 each */
  float bScore = nb / (nr + 1e-3f);        /* blue tile: B >> R */
  float gScore = ng / (nr + 1e-3f);        /* green tile: G >> R, and G > B */
  if (bScore > cal->blueScore && nb >= ng) return TILE_BLUE;
  if (gScore > cal->greenScore && ng > nb) return TILE_GREEN;
  return TILE_NONE;
}
int readFloorColour() {
  float r, g, b; tcsReadRGB(PIN_CF_S2, PIN_CF_S3, PIN_CF_OUT, &r, &g, &b);
  return classifyColour(&calF, r, g, b);
}
int readGripperColour() {
  digitalWrite(PIN_CG_LED, HIGH); delay(20);
  float r, g, b; tcsReadRGB(PIN_CG_S2, PIN_CG_S3, PIN_CG_OUT, &r, &g, &b);
  digitalWrite(PIN_CG_LED, LOW);
  return classifyColour(&calG, r, g, b);
}
/* 3-step calibration: white, then blue (CAL press), then green (CAL press). 10 s timeout per step keeps defaults. */
bool waitCalPress(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (digitalRead(PIN_BTN_CAL) == LOW) { delay(30); while (digitalRead(PIN_BTN_CAL) == LOW); delay(200); return true; }
  }
  return false;
}
void calibrateColour(ColourCal *cal, uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t out, uint8_t led, const char *name) {
  (void)s0; (void)s1;
  digitalWrite(led, HIGH); delay(30);
  float r, g, b;
  /* white reference = what the sensor sees now (floor / empty pocket) */
  float sr = 0, sg = 0, sb = 0;
  for (int i = 0; i < 5; i++) { tcsReadRGB(s2, s3, out, &r, &g, &b); sr += r; sg += g; sb += b; }
  cal->whiteR = sr / 5; cal->whiteG = sg / 5; cal->whiteB = sb / 5;
  logf("%s white: R%.1f G%.1f B%.1f  - press CAL over BLUE (10 s)", name, cal->whiteR, cal->whiteG, cal->whiteB);
  beep(1);
  float bScoreBlue = 0, gScoreGreen = 0, bScoreGreen = 0, gScoreBlue = 0;
  if (waitCalPress(10000)) {
    tcsReadRGB(s2, s3, out, &r, &g, &b);
    float nr = r / cal->whiteR, ng = g / cal->whiteG, nb = b / cal->whiteB, s = nr + ng + nb;
    nr /= s; ng /= s; nb /= s; bScoreBlue = nb / (nr + 1e-3f); gScoreBlue = ng / (nr + 1e-3f);
    logf("%s blue sample: chroma R%.2f G%.2f B%.2f (bScore %.2f)", name, nr, ng, nb, bScoreBlue);
    beep(1);
  }
  logf("press CAL over GREEN (10 s)");
  if (waitCalPress(10000)) {
    tcsReadRGB(s2, s3, out, &r, &g, &b);
    float nr = r / cal->whiteR, ng = g / cal->whiteG, nb = b / cal->whiteB, s = nr + ng + nb;
    nr /= s; ng /= s; nb /= s; gScoreGreen = ng / (nr + 1e-3f); bScoreGreen = nb / (nr + 1e-3f);
    logf("%s green sample: chroma R%.2f G%.2f B%.2f (gScore %.2f)", name, nr, ng, nb, gScoreGreen);
    beep(1);
  }
  /* threshold half-way between the coloured sample and white (score 1.0) */
  if (bScoreBlue > 1.05f)  cal->blueScore  = 1.0f + (bScoreBlue - 1.0f) * 0.5f;
  if (gScoreGreen > 1.05f) cal->greenScore = 1.0f + (gScoreGreen - 1.0f) * 0.5f;
  (void)gScoreBlue; (void)bScoreGreen;
  logf("%s thresholds: blueScore %.2f greenScore %.2f", name, cal->blueScore, cal->greenScore);
  digitalWrite(led, led == PIN_CF_LED ? HIGH : LOW);
}

/* ----------------------------------------------------------- line array */
void readLine() {
  lineRaw[0] = analogRead(PIN_LINE0); lineRaw[1] = analogRead(PIN_LINE1); lineRaw[2] = analogRead(PIN_LINE2);
  lineRaw[3] = analogRead(PIN_LINE3); lineRaw[4] = analogRead(PIN_LINE4);
}
uint8_t lineBits() {            /* bit4 = leftmost ... bit0 = rightmost ; 1 = black */
  readLine();
  uint8_t b = 0;
  for (int i = 0; i < 5; i++) if (lineRaw[i] > lineThresh) b |= (1 << (4 - i));
  return b;
}
/* weighted position -2..+2 (positive = line is to the RIGHT). returns false if no line */
bool linePosition(float *pos) {
  readLine();
  float w = 0, s = 0;
  for (int i = 0; i < 5; i++) { float v = constrain((float)(lineRaw[i] - lineThresh / 2) / (float)lineThresh, 0.0f, 1.0f); w += v * (i - 2); s += v; }
  if (s < 0.3f) return false;
  *pos = w / s; return true;
}

/* -------------------------------------------------------------- MPU6050 */
void imuInit() {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();  /* wake */
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();  /* gyro +-500 dps */
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();  /* DLPF 44 Hz */
  delay(50);
  imuCalibrate(400);
}
float imuGyroZ() {              /* deg/s, bias-corrected */
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2);
  if (Wire.available() < 2) return 0;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return raw / 65.5f - gyroBiasZ;
}
void imuCalibrate(int n) {
  double s = 0; gyroBiasZ = 0;
  for (int i = 0; i < n; i++) { s += imuGyroZ(); delay(2); }
  gyroBiasZ = s / n;
}
void imuZeroHeading() { lastOdoUs = micros(); }

/* --------------------------------------------------------------- misc */
float readBattery() { return analogRead(PIN_VBAT) * (5.0 / 1023.0) * VBAT_DIVIDER; }

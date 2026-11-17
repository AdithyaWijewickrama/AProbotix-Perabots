/* b_motion.ino - encoders, odometry (gyro heading + encoder distance), drive primitives, gripper */

/* --------------------------------------------------------- encoders */
void isrEncL() { if (digitalRead(PIN_ENC_LA) == digitalRead(PIN_ENC_LB)) encL++; else encL--; }
void isrEncR() { if (digitalRead(PIN_ENC_RA) != digitalRead(PIN_ENC_RB)) encR++; else encR--; }

void motionInit() {
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENC_LA, INPUT_PULLUP); pinMode(PIN_ENC_LB, INPUT_PULLUP);
  pinMode(PIN_ENC_RA, INPUT_PULLUP); pinMode(PIN_ENC_RB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LA), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RA), isrEncR, CHANGE);
  stopMotors();
}

/* ------------------------------------------------------------ motors */
void setMotor(uint8_t en, uint8_t in1, uint8_t in2, int pwm) {
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm > 0 && pwm < PWM_MIN_MOVE) pwm = PWM_MIN_MOVE;
  if (pwm < 0 && pwm > -PWM_MIN_MOVE) pwm = -PWM_MIN_MOVE;
  if (pwm > 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else if (pwm < 0) { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); }
  else { digitalWrite(in1, LOW); digitalWrite(in2, LOW); }
  analogWrite(en, abs(pwm));
}
void setMotors(int l, int r) { setMotor(PIN_ENA, PIN_IN1, PIN_IN2, l); setMotor(PIN_ENB, PIN_IN3, PIN_IN4, r); }
void stopMotors() {
  digitalWrite(PIN_IN1, HIGH); digitalWrite(PIN_IN2, HIGH); digitalWrite(PIN_IN3, HIGH); digitalWrite(PIN_IN4, HIGH);  /* brake */
  analogWrite(PIN_ENA, 255); analogWrite(PIN_ENB, 255);
  delay(60);
  analogWrite(PIN_ENA, 0); analogWrite(PIN_ENB, 0);
  digitalWrite(PIN_IN1, LOW); digitalWrite(PIN_IN2, LOW); digitalWrite(PIN_IN3, LOW); digitalWrite(PIN_IN4, LOW);
}

/* ---------------------------------------------------------- odometry */
long lastL = 0, lastR = 0;
float wrapDeg(float a) { while (a > 180) a -= 360; while (a <= -180) a += 360; return a; }

/* call often (every few ms). heading from the gyro, distance from the encoders */
void odoUpdate() {
  unsigned long now = micros();
  float dt = (now - lastOdoUs) * 1e-6f; lastOdoUs = now;
  if (dt > 0.2f) dt = 0;                                  /* first call after a pause */
  poseTh = wrapDeg(poseTh + imuGyroZ() * dt);
  long l, r; noInterrupts(); l = encL; r = encR; interrupts();
  float dl = (l - lastL) * MM_PER_COUNT, dr = (r - lastR) * MM_PER_COUNT;
  lastL = l; lastR = r;
  float d = (dl + dr) * 0.5f;
  poseX += d * cos(poseTh * DEG_TO_RAD);
  poseY += d * sin(poseTh * DEG_TO_RAD);
}
void odoReset() { noInterrupts(); encL = encR = 0; interrupts(); lastL = lastR = 0; lastOdoUs = micros(); }
float travelledMM() { long l, r; noInterrupts(); l = encL; r = encR; interrupts(); return (l + r) * 0.5f * MM_PER_COUNT; }

/* ------------------------------------------------------------- turns */
/* turn in place to an absolute heading (deg) using the gyro */
void turnTo(float target) {
  unsigned long t0 = millis(), settled = 0;
  for (;;) {
    odoUpdate();
    float e = wrapDeg(target - poseTh);
    if (fabs(e) < TURN_TOL_DEG) { if (!settled) settled = millis(); if (millis() - settled > 60) break; }
    else settled = 0;
    int pwm = (int)(KP_TURN * e);
    if (abs(pwm) < TURN_MIN_PWM) pwm = (pwm > 0) ? TURN_MIN_PWM : -TURN_MIN_PWM;
    pwm = constrain(pwm, -SPEED_TURN, SPEED_TURN);
    setMotors(-pwm, pwm);                 /* +e = turn left (CCW) */
    if (millis() - t0 > 4000) { logf("turn timeout e=%.1f", e); break; }
    delay(3);
  }
  stopMotors();
}
void turnBy(float deg) { turnTo(poseTh + deg); }
float snapHeading(float h) { return wrapDeg(90.0f * roundf(h / 90.0f)); }   /* nearest cardinal */

/* -------------------------------------------------------- straight drive */
/* Drive `mm` forward (negative = backwards) holding `head` (deg).
 * Options:  wallCentre  - use L/R ultrasonics to stay centred in a maze cell
 *           frontStopMM - stop early when the front ultrasonic reads <= this (0 = ignore)
 *           sampleFloor - keep the strongest floor tile colour seen in *tileOut
 * Returns distance actually travelled (mm). */
float driveStraight(float mm, int speed, float head, bool wallCentre, int frontStopMM, int *tileOut) {
  odoReset();
  bool fwd = mm >= 0; float goal = fabs(mm);
  unsigned long t0 = millis(), lastUs = 0, lastCs = 0, lastProgress = millis(); float lastDist = 0;
  int tileVotes[3] = { 0, 0, 0 };
  for (;;) {
    odoUpdate();
    float dist = fabs(travelledMM());
    if (dist >= goal) break;
    if (dist - lastDist > 5) { lastDist = dist; lastProgress = millis(); }
    if (millis() - lastProgress > 1500) { logf("drive stalled at %.0f/%.0f mm", dist, goal); break; }
    /* speed profile: ramp down over the last 60 mm */
    float rem = goal - dist;
    int v = speed;
    if (rem < 60) v = max(PWM_MIN_MOVE + 20, (int)(speed * rem / 60.0f));
    if (dist < 40) v = min(v, PWM_MIN_MOVE + 40 + (int)dist);
    if (!fwd) v = -v;
    /* heading hold */
    float corr = KP_HEAD * wrapDeg(head - poseTh);
    /* wall centring + front stop every 25 ms */
    if (millis() - lastUs > 25) {
      lastUs = millis();
      if (fwd) readFLR(); else usF = US_MAX_MM;
      if (wallCentre && fwd) {
        int tl = usL + (int)US_SIDE_OFFSET_MM, tr = usR + (int)US_SIDE_OFFSET_MM;   /* robot centre to wall */
        bool hl = tl < WALL_SEEN_MM + 40, hr = tr < WALL_SEEN_MM + 40;
        float e = 0;
        if (hl && hr) e = (tr - tl) * 0.5f;              /* +e: closer to left wall -> steer right */
        else if (hl) e = WALL_CENTRE_MM - tl;
        else if (hr) e = tr - WALL_CENTRE_MM;
        corr += -KP_WALL * e;                            /* positive corr = turn left */
        corr = constrain(corr, -60.0f, 60.0f);
      }
      if (frontStopMM > 0 && fwd && usF <= frontStopMM) { break; }
    }
    if (tileOut && millis() - lastCs > 60) { lastCs = millis(); int t = readFloorColour(); if (t) tileVotes[t]++; }
    setMotors(v - (int)corr, v + (int)corr);
    if (millis() - t0 > 8000) { logf("drive timeout"); break; }
  }
  stopMotors();
  if (tileOut) { *tileOut = TILE_NONE; if (tileVotes[TILE_BLUE] >= 2 || tileVotes[TILE_GREEN] >= 2) *tileOut = tileVotes[TILE_BLUE] >= tileVotes[TILE_GREEN] ? TILE_BLUE : TILE_GREEN; }
  odoUpdate();
  return travelledMM();
}

/* drive to a world point (mm) along a straight line: turn, then drive */
void driveToPoint(float x, float y, int speed, int frontStopMM) {
  float dx = x - poseX, dy = y - poseY, d = sqrt(dx * dx + dy * dy);
  if (d < 8) return;
  float h = atan2(dy, dx) * RAD_TO_DEG;
  if (fabs(wrapDeg(h - poseTh)) > 3) turnTo(h);
  driveStraight(d, speed, h, false, frontStopMM);
}

/* ----------------------------------------------------- line following */
#define LF_LOST     0
#define LF_CROSS    1   /* full crossing (all sensors black) */
#define LF_T_LEFT   2   /* branch to the left */
#define LF_T_RIGHT  3
#define LF_DIST     4   /* maxMM reached */
#define LF_FRONT    5   /* front obstacle */

/* Follow the line until a junction / distance / obstacle. The array is
 * LINE_OFFSET_MM ahead of the axle; after a junction is seen we roll on so the
 * axle stops on the junction (ready to pivot).  */
void (*lfHook)() = 0;          /* optional callback run every ~30 ms while following */

int followLine(int speed, float maxMM, int frontStopMM, bool stopOnJunction) {
  odoReset();
  float lastPos = 0; unsigned long t0 = millis(), lastUs = 0; int result = LF_DIST;
  int lostCount = 0, branchL = 0, branchR = 0;
  for (;;) {
    odoUpdate();
    if (fabs(travelledMM()) >= maxMM) { result = LF_DIST; break; }
    uint8_t bits = lineBits();
    float pos;
    bool have = linePosition(&pos);
    if (stopOnJunction) {
      if ((bits & 0x1F) == 0x1F || (bits & 0x1B) == 0x1B) { result = LF_CROSS; break; }
      if (bits & 0x10) branchL++; else branchL = 0;
      if (bits & 0x01) branchR++; else branchR = 0;
      if (branchL >= 3 && (bits & 0x1C) == 0x1C) { result = LF_T_LEFT; break; }
      if (branchR >= 3 && (bits & 0x07) == 0x07) { result = LF_T_RIGHT; break; }
    }
    if (!have) { if (++lostCount > 40) { result = LF_LOST; break; } pos = lastPos > 0 ? 2 : -2; }
    else { lostCount = 0; lastPos = pos; }
    float corr = KP_LINE * pos;                      /* line to the right -> turn right */
    setMotors(speed + (int)corr, speed - (int)corr);
    if (millis() - lastUs > 30) {
      lastUs = millis();
      if (frontStopMM) { usF = usPing(PIN_US_F_TRIG, PIN_US_F_ECHO); if (usF <= frontStopMM) { result = LF_FRONT; break; } }
      if (lfHook) lfHook();
    }
    if (millis() - t0 > 20000) break;
    delay(2);
  }
  if (result == LF_CROSS || result == LF_T_LEFT || result == LF_T_RIGHT) {
    /* roll the axle onto the junction */
    driveStraight(LINE_OFFSET_MM, SPEED_SLOW, snapHeading(poseTh));
  } else stopMotors();
  return result;
}

/* turn by 90-degree multiples relative to the nearest cardinal heading, then re-acquire the line */
void lineTurn(int quarterTurnsLeft) {
  float target = snapHeading(poseTh) + 90.0f * quarterTurnsLeft;
  turnTo(target);
}

/* ------------------------------------------------------------- gripper */
void gripperInit() {
  servoJaw.attach(PIN_SERVO_JAW); servoLift.attach(PIN_SERVO_LIFT);
  servoJaw.write(JAW_OPEN_DEG); servoLift.write(LIFT_DOWN_DEG);
}
void jawOpen()   { servoJaw.write(JAW_OPEN_DEG);   delay(400); }
void jawClose()  { servoJaw.write(JAW_CLOSED_DEG); delay(500); }
void liftUp()    { for (int a = LIFT_DOWN_DEG; a <= LIFT_UP_DEG; a += 2) { servoLift.write(a); delay(15); } }
void liftDown()  { for (int a = LIFT_UP_DEG; a >= LIFT_DOWN_DEG; a -= 2) { servoLift.write(a); delay(15); } }

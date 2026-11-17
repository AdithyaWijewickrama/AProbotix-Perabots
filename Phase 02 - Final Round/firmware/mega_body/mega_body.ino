// =============================================================================
// GuardianBot v3 - Arduino Mega 2560 "body" firmware
// PeraBots 2026 University Category, Final Round
//
// Job: everything that must be on time, every time.
//   * 4x quadrature decoding of both N20 hall encoders (interrupts)
//   * 100 Hz wheel speed PI + feed-forward on the L298N, with slew limit and
//     a 250 ms command watchdog (robot stops if the brain goes quiet)
//   * MPU6050 gyro Z at 200 Hz -> yaw (bias learned at boot and re-learned
//     automatically whenever the wheels are still)
//   * 4x HC-SR04 round-robin, echo timed with pin-change interrupts (never blocks)
//   * 5x Sharp GP2Y0A21 (median of 3) and 5x TCRT5000 line sensors (ADC)
//   * 2x TCS3200 colour sensors, frequency counted with pin-change interrupts
//   * 2 gripper servos, buttons, LEDs, buzzer, battery monitor
//   * 50 Hz binary telemetry to the ESP32-S3 brain, commands back (protocol.h)
//
// Libraries: Servo, Wire (both ship with the Arduino IDE). Board: Arduino Mega 2560.
// =============================================================================
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "pins.h"
#include "protocol.h"

// ----------------------------------------------------------------- tuning
// Encoder counts per WHEEL revolution: 7 pulses x 4 edges x gear ratio.
// N20 12 V "300 rpm" = 1:100 -> 2800. Measure: turn a wheel 10 times by hand, read encL/10.
static const float ENC_CPR     = 2800.0f;
static const float WHEEL_D_MM  = 43.0f;
static const int8_t ENC_L_SIGN = +1, ENC_R_SIGN = -1;     // flip if a wheel counts backwards
static const int8_t MOT_L_SIGN = +1, MOT_R_SIGN = +1;     // flip if a wheel turns backwards
static const float KFF = 0.30f;    // PWM per (mm/s): 255 PWM ~ 650 mm/s free running at 12 V
static const float KP  = 0.60f;    // PWM per (mm/s) error
static const float KI  = 6.0f;     // PWM per (mm/s * s)
static const int   PWM_MIN = 28;   // L298N + N20 static friction compensation
static const int   PWM_SLEW = 40;  // max PWM change per 10 ms
static const uint16_t VBAT_LOW_MV = 10200;   // 3S: 3.4 V/cell under load

// ----------------------------------------------------------------- state
Servo servoL, servoR;
volatile int32_t encL = 0, encR = 0;
volatile uint8_t encLState = 0, encRState = 0;
static const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};

// sonar
volatile uint32_t usRise[4];
volatile uint16_t usWidth[4];
volatile uint8_t usDone = 0;
uint16_t usMm[4] = {2500, 2500, 2500, 2500};
uint8_t usIdx = 0;
uint32_t usT0 = 0;
const uint8_t US_TRIG[4] = {PIN_US_F_TRIG, PIN_US_L_TRIG, PIN_US_R_TRIG, PIN_US_B_TRIG};

// colour
volatile uint16_t tcsCntFloor = 0, tcsCntFront = 0;
uint8_t lastPINB = 0;
uint16_t tcsFloor[4], tcsFront[4];
uint8_t tcsPhase = 0;        // 0..3 = R, G, B, C
uint32_t tcsT0 = 0;
bool tcsCounting = false, tcsNew = false;

// gyro
bool gyroOk = false;
float gyroBias = 0, yaw = 0, gz = 0;
uint32_t gyroT = 0;
const float GYRO_SCALE = (PI / 180.0f) / 65.5f;    // +-500 dps

// wheels
Command cmd = {0, 0, 90, 90, 0, 0};
uint32_t lastCmdMs = 0;
uint8_t lastCtl = 0;
int32_t pL = 0, pR = 0;
float iL = 0, iR = 0, vMeasL = 0, vMeasR = 0;
int pwmL = 0, pwmR = 0;
uint32_t pidT = 0;

// misc
uint16_t irMm[5], gsRaw[5], vbat = 12000;
uint8_t buttons = 0, seq = 0;
uint32_t lowBatSince = 0, teleT = 0;

// ================================================================ interrupts
void isrEncL() {
  encLState = ((encLState << 2) | ((PINE >> 4) & 0x03)) & 0x0F;   // D2 = PE4, D3 = PE5
  encL += QDEC[encLState];
}
void isrEncR() {
  encRState = ((encRState << 2) | ((PIND >> 2) & 0x03)) & 0x0F;   // D19 = PD2, D18 = PD3
  encR += QDEC[encRState];
}

ISR(PCINT0_vect) {                 // PB2..PB7: TCS outputs and sonar echoes
  uint8_t now = PINB;
  uint8_t ch = now ^ lastPINB;
  lastPINB = now;
  if ((ch & _BV(3)) && (now & _BV(3))) tcsCntFloor++;
  if ((ch & _BV(2)) && (now & _BV(2))) tcsCntFront++;
  if (ch & 0xF0) {
    uint32_t t = micros();
    for (uint8_t i = 0; i < 4; i++) {
      uint8_t m = _BV(4 + i);
      if (!(ch & m)) continue;
      if (now & m) usRise[i] = t;
      else { { uint32_t w = t - usRise[i]; usWidth[i] = (uint16_t)(w > 60000UL ? 60000UL : w); } usDone |= (1 << i); }
    }
  }
}

// ================================================================ helpers
void motorWrite(int pwm, uint8_t en, uint8_t a, uint8_t b) {
  if (pwm > 0) { digitalWrite(a, HIGH); digitalWrite(b, LOW); }
  else if (pwm < 0) { digitalWrite(a, LOW); digitalWrite(b, HIGH); }
  else { digitalWrite(a, HIGH); digitalWrite(b, HIGH); }            // active brake
  analogWrite(en, min(255, abs(pwm)));
}

int16_t readGyroZ() {
  Wire.beginTransmission(0x68);
  Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(0x68, 2) != 2) return 0;
  return (int16_t)((Wire.read() << 8) | Wire.read());
}

void mpuWrite(uint8_t reg, uint8_t v) {
  Wire.beginTransmission(0x68); Wire.write(reg); Wire.write(v); Wire.endTransmission();
}

void gyroCalibrate() {
  long s = 0;
  for (int i = 0; i < 400; i++) { s += readGyroZ(); delay(2); }
  gyroBias = s / 400.0f;
}

void gyroInit() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  Wire.beginTransmission(0x68);
  gyroOk = (Wire.endTransmission() == 0);
  if (!gyroOk) return;
  mpuWrite(0x6B, 0x01);   // wake, PLL on gyro X
  mpuWrite(0x1A, 0x03);   // DLPF 44 Hz
  mpuWrite(0x1B, 0x08);   // +-500 dps
  mpuWrite(0x19, 0x04);   // 200 Hz sample rate
  delay(100);
  gyroCalibrate();
}

uint16_t sharpMm(uint8_t pin) {
  // median of 3 conversions, then the GP2Y0A21 power-law fit d[cm] = 27.86 * V^-1.15
  int a = analogRead(pin), b = analogRead(pin), c = analogRead(pin);
  int m = max(min(a, b), min(max(a, b), c));
  float v = m * (5.0f / 1023.0f);
  if (v < 0.35f) return 800;                 // nothing within 80 cm
  float d = 278.6f * pow(v, -1.15f);         // mm
  if (d > 800) d = 800;
  if (d < 100) d = 100;                      // blind zone: report the 10 cm floor value
  return (uint16_t)d;
}

void tcsSetFilter(uint8_t ph) {
  // TCS3200: S2 S3 = LL red, HH green, LH blue, HL clear
  static const uint8_t S2[4] = {LOW, HIGH, LOW, HIGH};
  static const uint8_t S3[4] = {LOW, HIGH, HIGH, LOW};
  digitalWrite(PIN_TCS_S2, S2[ph]);
  digitalWrite(PIN_TCS_S3, S3[ph]);
}

// ================================================================ link
uint8_t rxBuf[64];
uint8_t rxPos = 0, rxLen = 0, rxType = 0;
enum { RX_H1, RX_H2, RX_TYPE, RX_LEN, RX_DATA, RX_SUM } rxState = RX_H1;

void handleCommand(const Command &c) {
  cmd = c;
  lastCmdMs = millis();
  uint8_t rise = c.ctl & ~lastCtl;
  lastCtl = c.ctl;
  if (rise & CTL_ZERO_ODOM) {
    noInterrupts(); encL = encR = 0; interrupts();
    pL = pR = 0; yaw = 0;
  }
  if ((rise & CTL_GYRO_CAL) && gyroOk) gyroCalibrate();
}

void linkPoll() {
  while (Serial2.available()) {
    uint8_t b = Serial2.read();
    switch (rxState) {
      case RX_H1: if (b == 0xA5) rxState = RX_H2; break;
      case RX_H2: rxState = (b == 0x5A) ? RX_TYPE : RX_H1; break;
      case RX_TYPE: rxType = b; rxState = RX_LEN; break;
      case RX_LEN: rxLen = b; rxPos = 0; rxState = (b <= sizeof(rxBuf)) ? (b ? RX_DATA : RX_SUM) : RX_H1; break;
      case RX_DATA: rxBuf[rxPos++] = b; if (rxPos >= rxLen) rxState = RX_SUM; break;
      case RX_SUM: {
        uint8_t s = rxType + rxLen;
        for (uint8_t i = 0; i < rxLen; i++) s += rxBuf[i];
        if (s == b && rxType == PKT_COMMAND && rxLen == sizeof(Command)) {
          Command c; memcpy(&c, rxBuf, sizeof(c)); handleCommand(c);
        }
        rxState = RX_H1;
      } break;
    }
  }
}

// non-blocking transmit queue (Serial2 TX buffer is only 64 bytes)
uint8_t txBuf[128];
uint8_t txLen = 0, txPos = 0;

void txPump() {
  while (txPos < txLen && Serial2.availableForWrite() > 0) Serial2.write(txBuf[txPos++]);
}

void sendTelemetry() {
  if (txPos < txLen) return;               // previous frame still going out: skip this one
  Telemetry t;
  t.ms = millis();
  noInterrupts(); t.encL = encL * ENC_L_SIGN; t.encR = encR * ENC_R_SIGN; interrupts();
  t.yaw = yaw; t.gz = gz;
  for (uint8_t i = 0; i < 4; i++) t.us[i] = usMm[i];
  for (uint8_t i = 0; i < 5; i++) { t.ir[i] = irMm[i]; t.gs[i] = gsRaw[i]; }
  for (uint8_t i = 0; i < 4; i++) { t.tcsFloor[i] = tcsFloor[i]; t.tcsFront[i] = tcsFront[i]; }
  t.vbat_mv = vbat;
  t.buttons = buttons;
  t.flags = (lowBatSince && millis() - lowBatSince > 2000 ? FLAG_LOWBAT : 0) | (gyroOk ? FLAG_GYRO_OK : 0) |
            (tcsNew ? FLAG_TCS_NEW : 0);
  tcsNew = false;
  t.seq = seq++;
  uint8_t n = sizeof(t);
  txBuf[0] = 0xA5; txBuf[1] = 0x5A; txBuf[2] = PKT_TELEMETRY; txBuf[3] = n;
  memcpy(txBuf + 4, &t, n);
  uint8_t s = PKT_TELEMETRY + n;
  for (uint8_t i = 0; i < n; i++) s += txBuf[4 + i];
  txBuf[4 + n] = s;
  txLen = n + 5; txPos = 0;
  txPump();
}

// ================================================================ setup / loop
void setup() {
  Serial.begin(115200);                       // USB debug
  Serial2.begin(LINK_BAUD);
  pinMode(PIN_IN1, OUTPUT); pinMode(PIN_IN2, OUTPUT); pinMode(PIN_IN3, OUTPUT); pinMode(PIN_IN4, OUTPUT);
  pinMode(PIN_ENA, OUTPUT); pinMode(PIN_ENB, OUTPUT);
  TCCR3B = (TCCR3B & 0xF8) | 0x02;            // D5 PWM -> 3.9 kHz (quieter, smoother N20 torque)
  TCCR4B = (TCCR4B & 0xF8) | 0x02;            // D6 PWM -> 3.9 kHz
  motorWrite(0, PIN_ENA, PIN_IN1, PIN_IN2);
  motorWrite(0, PIN_ENB, PIN_IN3, PIN_IN4);

  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrEncR, CHANGE);

  for (uint8_t i = 0; i < 4; i++) { pinMode(US_TRIG[i], OUTPUT); digitalWrite(US_TRIG[i], LOW); }
  pinMode(PIN_US_F_ECHO, INPUT); pinMode(PIN_US_L_ECHO, INPUT); pinMode(PIN_US_R_ECHO, INPUT); pinMode(PIN_US_B_ECHO, INPUT);
  pinMode(PIN_TCS_S2, OUTPUT); pinMode(PIN_TCS_S3, OUTPUT);
  pinMode(PIN_TCS_FLOOR_OUT, INPUT); pinMode(PIN_TCS_FRONT_OUT, INPUT);
  pinMode(PIN_TCS_FLOOR_LED, OUTPUT); pinMode(PIN_TCS_FRONT_LED, OUTPUT);
  lastPINB = PINB;
  PCMSK0 = _BV(PCINT2) | _BV(PCINT3) | _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6) | _BV(PCINT7);
  PCICR |= _BV(PCIE0);

  pinMode(PIN_BTN_START, INPUT_PULLUP); pinMode(PIN_BTN_CAL, INPUT_PULLUP);
  pinMode(PIN_LED_BLUE, OUTPUT); pinMode(PIN_LED_GREEN, OUTPUT); pinMode(PIN_BUZZER, OUTPUT);

  servoL.attach(PIN_SERVO_L, 500, 2500);
  servoR.attach(PIN_SERVO_R, 500, 2500);
  servoL.write(cmd.servoL); servoR.write(cmd.servoR);

  digitalWrite(PIN_LED_BLUE, HIGH); digitalWrite(PIN_LED_GREEN, HIGH);
  gyroInit();                                  // keep the robot still for ~1 s after power-on
  digitalWrite(PIN_LED_BLUE, LOW); digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_BUZZER, HIGH); delay(60); digitalWrite(PIN_BUZZER, LOW);
  Serial.println(gyroOk ? F("GuardianBot body ready") : F("GuardianBot body ready - MPU6050 NOT FOUND"));
  pidT = gyroT = teleT = micros();
  teleT = millis();
}

void wheelPid() {
  noInterrupts(); int32_t l = encL * ENC_L_SIGN, r = encR * ENC_R_SIGN; interrupts();
  const float dt = 0.010f;
  const float mmPerTick = PI * WHEEL_D_MM / ENC_CPR;
  vMeasL = 0.6f * vMeasL + 0.4f * (l - pL) * mmPerTick / dt;
  vMeasR = 0.6f * vMeasR + 0.4f * (r - pR) * mmPerTick / dt;
  pL = l; pR = r;
  bool alive = millis() - lastCmdMs < 250;
  float tL = alive ? cmd.vL : 0, tR = alive ? cmd.vR : 0;
  int outL, outR;
  if (tL == 0 && tR == 0 && fabs(vMeasL) < 20 && fabs(vMeasR) < 20) {
    iL = iR = 0; outL = outR = 0;
  } else {
    float eL = tL - vMeasL, eR = tR - vMeasR;
    iL = constrain(iL + eL * dt, -40.0f, 40.0f);
    iR = constrain(iR + eR * dt, -40.0f, 40.0f);
    float uL = KFF * tL + KP * eL + KI * iL;
    float uR = KFF * tR + KP * eR + KI * iR;
    outL = (int)uL + (uL > 1 ? PWM_MIN : uL < -1 ? -PWM_MIN : 0);
    outR = (int)uR + (uR > 1 ? PWM_MIN : uR < -1 ? -PWM_MIN : 0);
  }
  pwmL = constrain(outL, pwmL - PWM_SLEW, pwmL + PWM_SLEW);
  pwmR = constrain(outR, pwmR - PWM_SLEW, pwmR + PWM_SLEW);
  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);
  motorWrite(pwmL * MOT_L_SIGN, PIN_ENA, PIN_IN1, PIN_IN2);
  motorWrite(pwmR * MOT_R_SIGN, PIN_ENB, PIN_IN3, PIN_IN4);
}

void loop() {
  uint32_t nowUs = micros();
  uint32_t nowMs = millis();
  linkPoll();
  txPump();

  // ---- gyro 200 Hz
  if (gyroOk && nowUs - gyroT >= 5000) {
    float dt = (nowUs - gyroT) * 1e-6f;
    gyroT = nowUs;
    int16_t raw = readGyroZ();
    bool still = cmd.vL == 0 && cmd.vR == 0 && fabs(vMeasL) < 5 && fabs(vMeasR) < 5;
    if (still && fabs(raw - gyroBias) < 40) gyroBias += 0.002f * (raw - gyroBias);   // zero-velocity update
    gz = (raw - gyroBias) * GYRO_SCALE;
    if (still && fabs(gz) < 0.01f) gz = 0;
    yaw += gz * dt;
  }

  // ---- wheels 100 Hz
  if (nowUs - pidT >= 10000) { pidT += 10000; wheelPid(); }

  // ---- sonar: one sensor at a time, 15 ms slot each (no cross-talk)
  if (nowUs - usT0 >= 15000) {
    uint8_t m = 1 << usIdx;
    if (usDone & m) { { float mm = usWidth[usIdx] / 5.83f; usMm[usIdx] = (uint16_t)(mm > 2500 ? 2500 : mm); } usDone &= ~m; }
    else usMm[usIdx] = 2500;                                        // lost echo = no reading
    usIdx = (usIdx + 1) & 3;
    usT0 = nowUs;
    digitalWrite(US_TRIG[usIdx], HIGH); delayMicroseconds(10); digitalWrite(US_TRIG[usIdx], LOW);
  }

  // ---- colour: 1 ms settle + 6 ms count per filter, both sensors at once
  if (!tcsCounting && nowUs - tcsT0 >= 1000) {
    noInterrupts(); tcsCntFloor = tcsCntFront = 0; interrupts();
    tcsCounting = true; tcsT0 = nowUs;
  } else if (tcsCounting && nowUs - tcsT0 >= 6000) {
    noInterrupts(); tcsFloor[tcsPhase] = tcsCntFloor; tcsFront[tcsPhase] = tcsCntFront; interrupts();
    tcsPhase = (tcsPhase + 1) & 3;
    if (tcsPhase == 0) tcsNew = true;
    tcsSetFilter(tcsPhase);
    tcsCounting = false; tcsT0 = nowUs;
  }

  // ---- telemetry 50 Hz (analog sensors sampled just before sending)
  if (nowMs - teleT >= 20) {
    teleT += 20;
    const uint8_t IRP[5] = {PIN_IR_FRONT, PIN_IR_FL, PIN_IR_FR, PIN_IR_LEFT, PIN_IR_RIGHT};
    const uint8_t GSP[5] = {PIN_GS0, PIN_GS1, PIN_GS2, PIN_GS3, PIN_GS4};
    for (uint8_t i = 0; i < 5; i++) { irMm[i] = sharpMm(IRP[i]); gsRaw[i] = analogRead(GSP[i]); }
    vbat = (uint16_t)(analogRead(PIN_VBAT) * (5000.0f * 3.0f / 1023.0f));
    if (vbat > 6000 && vbat < VBAT_LOW_MV) { if (!lowBatSince) lowBatSince = nowMs; } else lowBatSince = 0;
    buttons = (digitalRead(PIN_BTN_START) == LOW ? BTN_START : 0) | (digitalRead(PIN_BTN_CAL) == LOW ? BTN_CAL : 0);
    bool alive = nowMs - lastCmdMs < 250;
    digitalWrite(PIN_LED_BLUE, (cmd.out & OUT_LED_BLUE) ? HIGH : LOW);
    digitalWrite(PIN_LED_GREEN, (cmd.out & OUT_LED_GREEN) ? HIGH : LOW);
    digitalWrite(PIN_TCS_FLOOR_LED, (!alive || (cmd.out & OUT_TCS_FLOOR)) ? HIGH : LOW);
    digitalWrite(PIN_TCS_FRONT_LED, (!alive || (cmd.out & OUT_TCS_FRONT)) ? HIGH : LOW);
    bool lowBeep = lowBatSince && nowMs - lowBatSince > 2000 && (nowMs / 250) % 8 == 0;
    digitalWrite(PIN_BUZZER, ((cmd.out & OUT_BUZZER) && alive) || lowBeep ? HIGH : LOW);
    servoL.write(cmd.servoL);
    servoR.write(cmd.servoR);
    sendTelemetry();
    if (!alive && (nowMs / 500) % 2 == 0 && nowMs % 1000 < 20) {      // 1 Hz USB status when no brain
      Serial.print(F("no brain | us ")); Serial.print(usMm[0]); Serial.print(' '); Serial.print(usMm[1]);
      Serial.print(' '); Serial.print(usMm[2]); Serial.print(' '); Serial.print(usMm[3]);
      Serial.print(F(" ir ")); for (uint8_t i = 0; i < 5; i++) { Serial.print(irMm[i]); Serial.print(' '); }
      Serial.print(F("gs ")); for (uint8_t i = 0; i < 5; i++) { Serial.print(gsRaw[i]); Serial.print(' '); }
      Serial.print(F("enc ")); Serial.print(encL); Serial.print(' '); Serial.print(encR);
      Serial.print(F(" yaw ")); Serial.print(yaw * 57.3f); Serial.print(F(" vbat ")); Serial.println(vbat);
    }
  }
}

// =============================================================================
// GuardianBot v4.0 - Arduino Mega 2560 "body" firmware
// PeraBots 2026 University Category, Final Round
//
// Job: everything that must be on time, every time.
//   * 4x quadrature decoding of both N20 hall encoders (interrupts)
//   * 100 Hz wheel speed PI + feed-forward on the DRV8833 (31 kHz PWM), slew limit,
//     250 ms command watchdog (driver goes to sleep if the brain goes quiet)
//   * L3G4200D gyro Z at 200 Hz -> yaw (bias learned at boot and whenever the wheels are still)
//   * 8 x VL53L0X time-of-flight in continuous mode, polled without blocking
//   * Pololu QTR-8RC line array (8 x RC decay times, 2.5 ms window)
//   * 2 x HC-SR04 and 2 x TCS3200 timed with pin-change interrupts (never blocks)
//   * 2 gripper servos, 1x4 keypad, 128x32 OLED (text from the brain), LEDs, buzzer, battery
//   * 50 Hz binary telemetry to the ESP32-CAM brain, commands back (protocol.h)
//
// Libraries (Arduino Library Manager): "VL53L0X" by Pololu, "SSD1306Ascii" by Bill Greiman.
// Built in: Servo, Wire. Board: Arduino Mega 2560.
// =============================================================================
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>
#include "pins.h"
#include "protocol.h"

// ----------------------------------------------------------------- tuning
// Encoder counts per WHEEL revolution = 7 pulses x 4 edges x gear ratio.
// The 6 V 300 rpm N20 ratio is not printed: turn a wheel 10 times by hand, read "enc" on USB, divide by 10.
static const float ENC_CPR     = 2800.0f;
static const float WHEEL_D_MM  = 43.0f;
static const int8_t ENC_L_SIGN = +1, ENC_R_SIGN = -1;     // flip if a wheel counts backwards
static const int8_t MOT_L_SIGN = +1, MOT_R_SIGN = +1;     // flip if a wheel turns backwards
static const int   PWM_MAX = 205;  // 80 % of 2S (8.4 V) ~= 6.7 V: keeps the 6 V N20 motors healthy
static const float KFF = 0.33f;    // PWM per (mm/s): 205 PWM ~ 620 mm/s free running
static const float KP  = 0.60f;    // PWM per (mm/s) error
static const float KI  = 6.0f;     // PWM per (mm/s * s)
static const int   PWM_MIN = 18;   // static friction compensation
static const int   PWM_SLEW = 30;  // max PWM change per 10 ms
static const uint16_t VBAT_LOW_MV = 6800;    // 2S: 3.4 V/cell under load

// ----------------------------------------------------------------- state
Servo servoL, servoR;
VL53L0X tof[8];
SSD1306AsciiWire oled;
volatile int32_t encL = 0, encR = 0;
volatile uint8_t encLState = 0, encRState = 0;
static const int8_t QDEC[16] = {0, +1, -1, 0, -1, 0, 0, +1, +1, 0, 0, -1, 0, -1, +1, 0};

// sonar (2) on PB4/PB5
volatile uint32_t usRise[2];
volatile uint16_t usWidth[2];
volatile uint8_t usDone = 0;
uint16_t usMm[2] = {2500, 2500};
uint8_t usIdx = 0;
uint32_t usT0 = 0;
const uint8_t US_TRIG[2] = {PIN_US_F_TRIG, PIN_US_B_TRIG};

// ToF
uint16_t tofMm[8] = {1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200};
bool tofOk[8];
uint8_t tofIdx = 0;

// line
uint16_t lineUs[8];

// colour
volatile uint16_t tcsCntFloor = 0, tcsCntFront = 0;
uint8_t lastPINB = 0;
uint16_t tcsFloor[4], tcsFront[4];
uint8_t tcsPhase = 0;
uint32_t tcsT0 = 0;
bool tcsCounting = false, tcsNew = false;

// gyro (L3G4200D, +-500 dps, 17.5 mdps/digit)
bool gyroOk = false;
float gyroBias = 0, yaw = 0, gz = 0;
uint32_t gyroT = 0;
const float GYRO_SCALE = 0.0175f * (PI / 180.0f);
const uint8_t GYRO_ADDR = 0x69;

// wheels
Command cmd = {0, 0, 90, 90, 0, 0};
uint32_t lastCmdMs = 0;
uint8_t lastCtl = 0;
int32_t pL = 0, pR = 0;
float iL = 0, iR = 0, vMeasL = 0, vMeasR = 0;
int pwmL = 0, pwmR = 0;
uint32_t pidT = 0;

// misc
uint16_t vbat = 8000;
uint8_t keys = 0, seq = 0;
uint32_t lowBatSince = 0, teleT = 0;
char oledText[2][22];
bool oledDirty = true;

// ================================================================ interrupts
void isrEncL() {
  encLState = ((encLState << 2) | ((PINE >> 4) & 0x03)) & 0x0F;   // D2 = PE4, D3 = PE5
  encL += QDEC[encLState];
}
void isrEncR() {
  encRState = ((encRState << 2) | ((PIND >> 2) & 0x03)) & 0x0F;   // D19 = PD2, D18 = PD3
  encR += QDEC[encRState];
}

ISR(PCINT0_vect) {                 // PB2..PB5: TCS outputs and sonar echoes
  uint8_t now = PINB;
  uint8_t ch = now ^ lastPINB;
  lastPINB = now;
  if ((ch & _BV(3)) && (now & _BV(3))) tcsCntFloor++;
  if ((ch & _BV(2)) && (now & _BV(2))) tcsCntFront++;
  if (ch & 0x30) {
    uint32_t t = micros();
    for (uint8_t i = 0; i < 2; i++) {
      uint8_t m = _BV(4 + i);
      if (!(ch & m)) continue;
      if (now & m) usRise[i] = t;
      else { uint32_t w = t - usRise[i]; usWidth[i] = (uint16_t)(w > 60000UL ? 60000UL : w); usDone |= (1 << i); }
    }
  }
}

// ================================================================ helpers
void motorWrite(int pwm, uint8_t in1, uint8_t in2) {
  // DRV8833 "fast decay": PWM on one input, the other LOW; both LOW = coast, both HIGH = brake
  pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
  if (pwm > 0) { analogWrite(in1, pwm); digitalWrite(in2, LOW); }
  else if (pwm < 0) { digitalWrite(in1, LOW); analogWrite(in2, -pwm); }
  else { digitalWrite(in1, HIGH); digitalWrite(in2, HIGH); }   // brake
}

void i2cWrite(uint8_t addr, uint8_t reg, uint8_t v) {
  Wire.beginTransmission(addr); Wire.write(reg); Wire.write(v); Wire.endTransmission();
}
int16_t readGyroZ() {
  Wire.beginTransmission(GYRO_ADDR);
  Wire.write(0x2C | 0x80);                  // OUT_Z_L with auto-increment
  if (Wire.endTransmission(false) != 0) return 0;
  if (Wire.requestFrom(GYRO_ADDR, (uint8_t)2) != 2) return 0;
  uint8_t lo = Wire.read(), hi = Wire.read();
  return (int16_t)((hi << 8) | lo);
}
void gyroCalibrate() {
  long s = 0;
  for (int i = 0; i < 300; i++) { s += readGyroZ(); delay(5); }
  gyroBias = s / 300.0f;
}
void gyroInit() {
  Wire.beginTransmission(GYRO_ADDR);
  gyroOk = (Wire.endTransmission() == 0);
  if (!gyroOk) return;
  i2cWrite(GYRO_ADDR, 0x20, 0x6F);   // CTRL_REG1: ODR 200 Hz, BW 50 Hz, normal mode, X Y Z on
  i2cWrite(GYRO_ADDR, 0x23, 0x10);   // CTRL_REG4: +-500 dps
  delay(100);
  gyroCalibrate();
}

void tofInit() {
  for (uint8_t i = 0; i < 8; i++) { pinMode(PIN_XSHUT[i], OUTPUT); digitalWrite(PIN_XSHUT[i], LOW); }
  delay(10);
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(PIN_XSHUT[i], INPUT);                 // release: the module pulls XSHUT up to 2.8 V
    delay(3);
    tof[i].setTimeout(40);
    tofOk[i] = tof[i].init();
    if (tofOk[i]) {
      tof[i].setAddress(0x30 + i);
      tof[i].setMeasurementTimingBudget(33000);   // ~30 Hz per sensor
      tof[i].startContinuous(0);
    }
  }
}
// One sensor per call: read it only if a new measurement is ready (never waits)
void tofPoll() {
  uint8_t i = tofIdx;
  tofIdx = (tofIdx + 1) & 7;
  if (!tofOk[i]) return;
  if ((tof[i].readReg(0x13) & 0x07) == 0) return;          // RESULT_INTERRUPT_STATUS: nothing new yet
  uint16_t mm = tof[i].readReg16Bit(0x14 + 10);            // RESULT_RANGE_STATUS + 10
  tof[i].writeReg(0x0B, 0x01);                             // SYSTEM_INTERRUPT_CLEAR
  if (mm >= 8190 || mm > 1200) mm = 1200;                  // out of range
  if (mm < 30) mm = 30;
  tofMm[i] = mm;
}

void qtrRead() {
  // RC sensors: charge the 8 lines, then time how long each takes to fall (longer = darker)
  digitalWrite(PIN_QTR_LEDON, HIGH);
  PORTA = 0xFF; DDRA = 0xFF;
  delayMicroseconds(10);
  DDRA = 0x00; PORTA = 0x00;
  uint32_t t0 = micros();
  uint8_t pending = 0xFF;
  for (uint8_t i = 0; i < 8; i++) lineUs[i] = 2500;
  while (pending) {
    uint32_t dt = micros() - t0;
    if (dt >= 2500) break;
    uint8_t low = ~PINA & pending;
    if (low) {
      for (uint8_t i = 0; i < 8; i++) if (low & (1 << i)) lineUs[i] = (uint16_t)dt;
      pending &= ~low;
    }
  }
}

void tcsSetFilter(uint8_t ph) {
  static const uint8_t S2[4] = {LOW, HIGH, LOW, HIGH};   // R G B C
  static const uint8_t S3[4] = {LOW, HIGH, HIGH, LOW};
  digitalWrite(PIN_TCS_S2, S2[ph]);
  digitalWrite(PIN_TCS_S3, S3[ph]);
}

void oledShow() {
  if (!oledDirty) return;
  oledDirty = false;
  oled.setCursor(0, 0); oled.print(oledText[0]); oled.clearToEOL();
  oled.setCursor(0, 1); oled.print(oledText[1]); oled.clearToEOL();
  oled.setCursor(0, 2); oled.print(F("bat ")); oled.print(vbat / 1000.0f, 2); oled.print(F(" V ")); oled.clearToEOL();
  oled.setCursor(0, 3); oled.print(F("1:GO 2:MAZE 3:S3 4:CAL")); oled.clearToEOL();
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
        } else if (s == b && rxType == PKT_TEXT && rxLen == sizeof(TextMsg)) {
          for (uint8_t l = 0; l < 2; l++) { memcpy(oledText[l], rxBuf + 21 * l, 21); oledText[l][21] = 0; }
          oledDirty = true;
        }
        rxState = RX_H1;
      } break;
    }
  }
}

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
  for (uint8_t i = 0; i < 2; i++) t.us[i] = usMm[i];
  for (uint8_t i = 0; i < 8; i++) { t.tof[i] = tofMm[i]; t.line[i] = lineUs[i]; }
  for (uint8_t i = 0; i < 4; i++) { t.tcsFloor[i] = tcsFloor[i]; t.tcsFront[i] = tcsFront[i]; }
  t.vbat_mv = vbat;
  t.keys = keys;
  bool allTof = true;
  for (uint8_t i = 0; i < 8; i++) allTof &= tofOk[i];
  t.flags = (lowBatSince && millis() - lowBatSince > 2000 ? FLAG_LOWBAT : 0) | (gyroOk ? FLAG_GYRO_OK : 0) |
            (tcsNew ? FLAG_TCS_NEW : 0) | (allTof ? FLAG_TOF_OK : 0);
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
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_DRV_SLEEP, OUTPUT); digitalWrite(PIN_DRV_SLEEP, LOW);
  TCCR3B = (TCCR3B & 0xF8) | 0x01;            // D5: 31 kHz (silent, smooth N20 torque)
  TCCR4B = (TCCR4B & 0xF8) | 0x01;            // D6 D7 D8: 31 kHz
  motorWrite(0, PIN_AIN1, PIN_AIN2);
  motorWrite(0, PIN_BIN1, PIN_BIN2);

  pinMode(PIN_ENC_L_A, INPUT_PULLUP); pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP); pinMode(PIN_ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrEncR, CHANGE);

  for (uint8_t i = 0; i < 2; i++) { pinMode(US_TRIG[i], OUTPUT); digitalWrite(US_TRIG[i], LOW); }
  pinMode(PIN_US_F_ECHO, INPUT); pinMode(PIN_US_B_ECHO, INPUT);
  pinMode(PIN_TCS_S2, OUTPUT); pinMode(PIN_TCS_S3, OUTPUT);
  pinMode(PIN_TCS_FLOOR_OUT, INPUT); pinMode(PIN_TCS_FRONT_OUT, INPUT);
  pinMode(PIN_TCS_FLOOR_LED, OUTPUT); pinMode(PIN_TCS_FRONT_LED, OUTPUT);
  pinMode(PIN_QTR_LEDON, OUTPUT);
  lastPINB = PINB;
  PCMSK0 = _BV(PCINT2) | _BV(PCINT3) | _BV(PCINT4) | _BV(PCINT5);
  PCICR |= _BV(PCIE0);

  for (uint8_t i = 0; i < 4; i++) pinMode(PIN_KEY[i], INPUT_PULLUP);
  pinMode(PIN_LED_BLUE, OUTPUT); pinMode(PIN_LED_GREEN, OUTPUT); pinMode(PIN_BUZZER, OUTPUT);
  servoL.attach(PIN_SERVO_L, 500, 2500);
  servoR.attach(PIN_SERVO_R, 500, 2500);
  servoL.write(cmd.servoL); servoR.write(cmd.servoR);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);
  oled.begin(&Adafruit128x32, 0x3C);
  oled.setFont(System5x7);
  oled.clear();
  oled.println(F("GuardianBot v4.0"));
  oled.println(F("starting - keep still"));
  digitalWrite(PIN_LED_BLUE, HIGH); digitalWrite(PIN_LED_GREEN, HIGH);
  tofInit();
  gyroInit();                                   // robot still for ~1.5 s after power-on
  digitalWrite(PIN_LED_BLUE, LOW); digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_BUZZER, HIGH); delay(60); digitalWrite(PIN_BUZZER, LOW);
  uint8_t nt = 0;
  for (uint8_t i = 0; i < 8; i++) nt += tofOk[i];
  snprintf(oledText[0], 22, "ToF %u/8 gyro %s", nt, gyroOk ? "ok" : "MISSING");
  snprintf(oledText[1], 22, "waiting for brain...");
  oledDirty = true;
  Serial.println(oledText[0]);
  pidT = gyroT = usT0 = tcsT0 = micros();
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
  digitalWrite(PIN_DRV_SLEEP, alive ? HIGH : LOW);
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
  motorWrite(pwmL * MOT_L_SIGN, PIN_AIN1, PIN_AIN2);
  motorWrite(pwmR * MOT_R_SIGN, PIN_BIN1, PIN_BIN2);
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
    if (still && fabs(raw - gyroBias) < 60) gyroBias += 0.002f * (raw - gyroBias);   // zero-velocity update
    gz = (raw - gyroBias) * GYRO_SCALE;
    if (still && fabs(gz) < 0.01f) gz = 0;
    yaw += gz * dt;
  }

  // ---- wheels 100 Hz
  if (nowUs - pidT >= 10000) { pidT += 10000; wheelPid(); }

  // ---- ToF: one sensor per pass
  tofPoll();

  // ---- sonar: front / back alternately, 25 ms slot each
  if (nowUs - usT0 >= 25000) {
    uint8_t m = 1 << usIdx;
    if (usDone & m) { float mm = usWidth[usIdx] / 5.83f; usMm[usIdx] = (uint16_t)(mm > 2500 ? 2500 : mm); usDone &= ~m; }
    else usMm[usIdx] = 2500;
    usIdx ^= 1;
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

  // ---- telemetry 50 Hz (line array read just before sending)
  if (nowMs - teleT >= 20) {
    teleT += 20;
    qtrRead();
    vbat = (uint16_t)(analogRead(PIN_VBAT) * (5000.0f * 3.0f / 1023.0f));
    if (vbat > 5000 && vbat < VBAT_LOW_MV) { if (!lowBatSince) lowBatSince = nowMs; } else lowBatSince = 0;
    keys = 0;
    for (uint8_t i = 0; i < 4; i++) if (digitalRead(PIN_KEY[i]) == LOW) keys |= 1 << i;
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
    if (nowMs % 500 < 20) oledDirty = true;        // refresh the battery line twice a second
    oledShow();
    if (!alive && nowMs % 1000 < 20) {             // 1 Hz USB status when no brain
      Serial.print(F("no brain | us ")); Serial.print(usMm[0]); Serial.print(' '); Serial.print(usMm[1]);
      Serial.print(F(" tof ")); for (uint8_t i = 0; i < 8; i++) { Serial.print(tofMm[i]); Serial.print(' '); }
      Serial.print(F("line ")); for (uint8_t i = 0; i < 8; i++) { Serial.print(lineUs[i]); Serial.print(' '); }
      Serial.print(F("enc ")); Serial.print(encL); Serial.print(' '); Serial.print(encR);
      Serial.print(F(" yaw ")); Serial.print(yaw * 57.3f); Serial.print(F(" keys ")); Serial.print(keys);
      Serial.print(F(" vbat ")); Serial.println(vbat);
    }
  }
}

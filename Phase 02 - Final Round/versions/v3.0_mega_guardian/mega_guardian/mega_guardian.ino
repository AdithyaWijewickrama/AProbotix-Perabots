/*
 * GuardianBot v3  -  PeraBots 2026 University Final, "The Nine Seals of Pera"
 * Team AProbotix.   Arduino Mega 2560 + L298N + 4x HC-SR04 + 2x TCS3200 +
 * 5-ch TCRT5000 line array + MPU6050 + 2 metal servos (jaw, lift).
 *
 * Tabs:  config.h      pins / geometry / tunables
 *        game_logic.h  maze + board logic (pure C, unit-tested on PC)
 *        a_sensors.ino ultrasonic, colour, line, IMU, buttons, battery
 *        b_motion.ino  encoders, odometry, drive / turn primitives, gripper
 *        c_section1.ino obstacle field (occupancy grid + A*)
 *        d_section2.ino maze exploration, tiles, turn counting
 *        e_section3.ino board: pick piece, inspect, place, END
 *
 * Buttons:  START short press  -> run from the selected stage
 *           CAL   short press  -> cycle stage 1 / 2 / 3 (LEDs blink the stage number)
 *           CAL   hold 1.5 s   -> 2-minute calibration (gyro, line, colours)
 * Clues are kept in EEPROM so a judge-ordered restart at Section 3 still works.
 */
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include "config.h"
#include "game_logic.h"
#include "protos.h"

/* ------------------------------------------------------------ globals */
Servo servoJaw, servoLift;

volatile long encL = 0, encR = 0;          /* encoder counts */
float poseX = 0, poseY = 0, poseTh = 0;     /* mm, mm, degrees (0 = +x, 90 = +y) */
float gyroBiasZ = 0;                        /* deg/s */
unsigned long lastOdoUs = 0;

int usF = US_MAX_MM, usL = US_MAX_MM, usR = US_MAX_MM, usB = US_MAX_MM;  /* mm */
int lineRaw[5];                             /* analog */
int lineThresh = LINE_THRESH;

ColourCal calF, calG;                       /* colour calibration, see config.h */

int clueBlue = 0, clueGreen = 0, clueRight = 0, clueLeft = 0;
int myClan = CLAN_BLUE;
uint8_t stage = 1;                          /* 1 = full run, 2 = start at maze, 3 = start at board */
MazeMap maze;

/* ------------------------------------------------------------ logging */
void logf(const char *fmt, ...) {
  char buf[96];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
  Serial.print('['); Serial.print(millis() / 1000.0, 1); Serial.print("] "); Serial.println(buf);
  Serial1.print('['); Serial1.print(millis() / 1000.0, 1); Serial1.print("] "); Serial1.println(buf);
}

void beep(int n, int ms) {
  for (int i = 0; i < n; i++) { digitalWrite(PIN_BUZZER, HIGH); delay(ms); digitalWrite(PIN_BUZZER, LOW); delay(ms); }
}

void showStage() {
  for (int i = 0; i < stage; i++) {
    digitalWrite(PIN_LED_BLUE, HIGH); digitalWrite(PIN_LED_GREEN, HIGH); delay(150);
    digitalWrite(PIN_LED_BLUE, LOW);  digitalWrite(PIN_LED_GREEN, LOW);  delay(150);
  }
}

void saveClues() {
  EEPROM.update(EE_ADDR_MAGIC, EE_MAGIC);
  EEPROM.update(EE_ADDR_CLUES + 0, clueBlue);  EEPROM.update(EE_ADDR_CLUES + 1, clueGreen);
  EEPROM.update(EE_ADDR_CLUES + 2, clueRight); EEPROM.update(EE_ADDR_CLUES + 3, clueLeft);
  logf("clues saved: B=%d G=%d R=%d L=%d", clueBlue, clueGreen, clueRight, clueLeft);
}
bool loadClues() {
  if (EEPROM.read(EE_ADDR_MAGIC) != EE_MAGIC) return false;
  clueBlue = EEPROM.read(EE_ADDR_CLUES); clueGreen = EEPROM.read(EE_ADDR_CLUES + 1);
  clueRight = EEPROM.read(EE_ADDR_CLUES + 2); clueLeft = EEPROM.read(EE_ADDR_CLUES + 3);
  return true;
}
void saveCal() {
  int a = EE_ADDR_CAL;
  EEPROM.put(a, calF); a += sizeof(calF);
  EEPROM.put(a, calG); a += sizeof(calG);
  EEPROM.put(a, lineThresh);
}
void loadCal() {
  int a = EE_ADDR_CAL;
  EEPROM.get(a, calF); a += sizeof(calF);
  EEPROM.get(a, calG); a += sizeof(calG);
  EEPROM.get(a, lineThresh);
  if (isnan(calF.whiteR) || calF.whiteR <= 0 || lineThresh < 50 || lineThresh > 1000) {
    /* defaults until the first calibration */
    calF.whiteR = calF.whiteG = calF.whiteB = 1.0f; calF.blueScore = calF.greenScore = 1.25f;
    calG = calF; lineThresh = LINE_THRESH;
  }
}

/* --------------------------------------------------------------- setup */
void setup() {
  Serial.begin(LOG_BAUD); Serial1.begin(LOG_BAUD);
  pinMode(PIN_BTN_START, INPUT_PULLUP); pinMode(PIN_BTN_CAL, INPUT_PULLUP);
  pinMode(PIN_LED_BLUE, OUTPUT); pinMode(PIN_LED_GREEN, OUTPUT); pinMode(PIN_BUZZER, OUTPUT);
  motionInit();
  sensorsInit();
  gripperInit();
  loadCal();
  Wire.begin(); Wire.setClock(400000);
  imuInit();
  logf("GuardianBot v3 ready. Vbat=%.2f V", readBattery());
  if (readBattery() < VBAT_LOW) { logf("LOW BATTERY"); beep(5, 60); }
  beep(2);
  showStage();
}

/* ---------------------------------------------------------------- loop */
void loop() {
  /* idle: wait for a button */
  if (digitalRead(PIN_BTN_CAL) == LOW) {
    unsigned long t0 = millis();
    while (digitalRead(PIN_BTN_CAL) == LOW) { if (millis() - t0 > 1500) break; }
    if (millis() - t0 > 1500) { runCalibration(); while (digitalRead(PIN_BTN_CAL) == LOW); }
    else { stage = stage % 3 + 1; logf("stage -> %d", stage); showStage(); }
    delay(200);
    return;
  }
  if (digitalRead(PIN_BTN_START) == LOW) {
    delay(30); if (digitalRead(PIN_BTN_START) != LOW) return;
    while (digitalRead(PIN_BTN_START) == LOW);
    delay(300);                                  /* hands away */
    runMission(stage);
    stage = 1;                                   /* next START = full run unless changed */
    showStage();
  }
}

/* ------------------------------------------------------------- mission */
void runMission(uint8_t fromStage) {
  logf("===== MISSION START from stage %d =====", fromStage);
  imuZeroHeading();
  if (fromStage <= 1) {
    poseX = S1_START_X_MM; poseY = S1_START_Y_MM; poseTh = S1_START_HEAD;
    runSection1();
  }
  if (fromStage <= 2) {
    if (fromStage == 2) { /* judges placed us at the maze entrance, facing in */ poseTh = 90; }
    runSection2();           /* fills clue* and saves to EEPROM */
  } else {
    if (!loadClues()) { logf("no stored clues! using 0/0/0/0"); }
    else logf("restart at Section 3 with stored clues B=%d G=%d R=%d L=%d", clueBlue, clueGreen, clueRight, clueLeft);
  }
  runSection3(fromStage == 3);
  stopMotors();
  logf("===== MISSION COMPLETE =====");
  beep(3, 150);
}

/* ---------------------------------------------------------- calibration */
void runCalibration() {
  logf("CALIBRATION: keep the robot still (gyro)...");
  beep(1);
  imuCalibrate();
  logf("gyro bias %.3f deg/s", gyroBiasZ);
  /* line: robot standing with the array half on a black line is ideal, but we
   * also accept "white only": threshold = white*2.5 clipped */
  int mn = 1023, mx = 0;
  for (int i = 0; i < 100; i++) { readLine(); for (int k = 0; k < 5; k++) { mn = min(mn, lineRaw[k]); mx = max(mx, lineRaw[k]); } delay(5); }
  lineThresh = (mx - mn > 150) ? (mn + mx) / 2 : constrain(mn * 2 + 100, 150, 900);
  logf("line: min %d max %d -> thresh %d", mn, mx, lineThresh);
  /* colour: floor sensor over WHITE floor now; press CAL again over a BLUE tile,
   * again over a GREEN tile (skip either by waiting 10 s). Same for the gripper
   * sensor with the two pieces. */
  calibrateColour(&calF, PIN_CF_S0, PIN_CF_S1, PIN_CF_S2, PIN_CF_S3, PIN_CF_OUT, PIN_CF_LED, "floor");
  calibrateColour(&calG, PIN_CG_S0, PIN_CG_S1, PIN_CG_S2, PIN_CG_S3, PIN_CG_OUT, PIN_CG_LED, "gripper");
  saveCal();
  logf("calibration saved");
  beep(2);
}

/*
 * config.h  -  GuardianBot v3  -  pins, geometry and tunables (Arduino Mega 2560)
 * Change things HERE, not in the other tabs.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* ============================== PINS ================================== */
/* L298N motor driver (ENA/ENB must be PWM pins) */
#define PIN_ENA   5      /* left  PWM */
#define PIN_IN1   22
#define PIN_IN2   23
#define PIN_ENB   6      /* right PWM */
#define PIN_IN3   24
#define PIN_IN4   25

/* Quadrature encoders: channel A on external-interrupt pins, B on plain pins */
#define PIN_ENC_LA  2
#define PIN_ENC_LB  4
#define PIN_ENC_RA  3
#define PIN_ENC_RB  7

/* HC-SR04 ultrasonic (trig, echo) : Front, Left, Right, Back */
#define PIN_US_F_TRIG 26
#define PIN_US_F_ECHO 27
#define PIN_US_L_TRIG 28
#define PIN_US_L_ECHO 29
#define PIN_US_R_TRIG 30
#define PIN_US_R_ECHO 31
#define PIN_US_B_TRIG 32
#define PIN_US_B_ECHO 33

/* TCS3200 colour sensor #1 : FLOOR (points down, front of the robot, ~8 mm above floor) */
#define PIN_CF_S0  34
#define PIN_CF_S1  35
#define PIN_CF_S2  36
#define PIN_CF_S3  37
#define PIN_CF_OUT 38
#define PIN_CF_LED 39
/* TCS3200 colour sensor #2 : GRIPPER (in the pocket, looks at the held piece) */
#define PIN_CG_S0  40
#define PIN_CG_S1  41
#define PIN_CG_S2  42
#define PIN_CG_S3  43
#define PIN_CG_OUT 44
#define PIN_CG_LED 45

/* 5-channel TCRT5000 line array, left -> right, analog inputs */
#define PIN_LINE0  A8
#define PIN_LINE1  A9
#define PIN_LINE2  A10
#define PIN_LINE3  A11
#define PIN_LINE4  A12

/* Servos (metal gear, e.g. MG995 / MG996R) */
#define PIN_SERVO_JAW   8
#define PIN_SERVO_LIFT  9

/* UI */
#define PIN_BTN_START 10   /* to GND, internal pull-up */
#define PIN_BTN_CAL   11
#define PIN_LED_BLUE  12
#define PIN_LED_GREEN 13
#define PIN_BUZZER    52

/* Battery sense: 2S LiPo through 100k (top) / 33k (bottom) divider */
#define PIN_VBAT  A0
#define VBAT_DIVIDER  ((100.0 + 33.0) / 33.0)
#define VBAT_LOW      6.8          /* warn below this (2S) */

/* MPU6050 on I2C (SDA 20, SCL 21), AD0 low -> 0x68 */
#define MPU_ADDR 0x68

/* Serial1 (TX1=18, RX1=19) -> ESP32-S3 log bridge (optional).  Serial = USB. */
#define LOG_BAUD 115200

/* ============================ GEOMETRY ================================ */
#define WHEEL_DIAMETER_MM   40.0
#define TRACK_MM            120.0     /* distance between the two wheel centres */
#define COUNTS_PER_REV      300.0     /* 12 CPR magnetic enc. x 50:1 at 2x decoding = 300 (change if different) */
#define MM_PER_COUNT        (3.14159265 * WHEEL_DIAMETER_MM / COUNTS_PER_REV)

/* sensor offsets from the wheel axle centre (mm, +x forward) */
#define US_F_OFFSET_MM      60.0      /* front ultrasonic face ahead of the axle */
#define US_SIDE_OFFSET_MM   60.0      /* side ultrasonic faces from the robot centre line */
#define LINE_OFFSET_MM      55.0      /* line array ahead of the axle */
#define FLOOR_CS_OFFSET_MM  50.0      /* floor colour sensor ahead of the axle */
#define POCKET_OFFSET_MM    75.0      /* centre of a held piece ahead of the axle */
#define ROBOT_HALF_LEN_MM   80.0      /* axle to front-most point */

/* ============================ ARENA =================================== */
/* World frame = rulebook Figure 1, metres -> we use mm. Origin = arena corner
 * nearest the START zone, +x along the 2.44 m side, +y along the 3.66 m side
 * (same frame as the Webots simulation).                                      */
#define S1_START_X_MM   2314
#define S1_START_Y_MM   126
#define S1_START_HEAD   90        /* degrees, +y */
#define S1_GOAL_X_MM    220       /* in front of the maze entrance door */
#define S1_GOAL_Y_MM    1080
#define S1_GRID_MM      100       /* occupancy cell */
#define S1_COLS         25
#define S1_ROWS         13
#define S1_INFLATE      1         /* cells of inflation around an obstacle (robot radius ~75 mm) */

/* Section 2 maze: 9 x 4 cells, 250 mm pitch, entry cell (0,0) reached heading +y */
#define MZ_CELL_MM      250
#define MZ_ENTRY_C      0
#define MZ_ENTRY_R      0
#define MZ_EXIT_C       8
#define MZ_EXIT_R       3
#define MZ_EXIT_HEAD    H_N       /* direction of the exit door out of the exit cell */
#define WALL_SEEN_MM    180       /* ultrasonic < this at cell centre => wall on that side */
#define WALL_CENTRE_MM  119       /* ideal distance robot-centre -> wall  ( (238)/2 ) */
#define FRONT_STOP_MM   (WALL_CENTRE_MM - US_F_OFFSET_MM)   /* front sensor reading when centred in a cell with a wall ahead */

/* Section 3 board, Figure 1 (mm).  Transverse lines x, long lines y.          */
#define B_XT0  500
#define B_XT1  900
#define B_XT2  1300
#define B_XT3  1700
#define B_YL0  2600
#define B_YL1  2900
#define B_YL2  3200
#define B_YL3  3500
#define B_ENTRY_Y       3050      /* the entry/exit line runs along y = 3050 */
#define B_EXIT_JUNC_X   2210      /* T junction after the maze exit */
#define B_COLLECT_X     2006      /* crossing with the collection line */
#define B_COLLECT_LEN   350       /* mm from the crossing to a collection point */
#define B_END_X         130

/* ============================ MOTION ================================== */
#define PWM_MAX         255
#define PWM_MIN_MOVE    60        /* below this the L298N + motor does not move */
#define SPEED_FAST      200
#define SPEED_NORMAL    150
#define SPEED_SLOW      100
#define SPEED_TURN      130
#define KP_HEAD         4.0       /* PWM per degree of heading error while driving straight */
#define KP_WALL         0.8       /* PWM per mm of wall-centring error (maze) */
#define KP_LINE         35.0      /* PWM per unit of line position error (-2..2) */
#define TURN_TOL_DEG    2.0
#define KP_TURN         3.5
#define TURN_MIN_PWM    85

/* ============================ SENSORS ================================= */
#define US_TIMEOUT_US   12000     /* ~2 m */
#define US_MAX_MM       2000
#define LINE_THRESH     500       /* analog: > this = black (overwritten by calibration) */
#define TCS_READ_MS     40        /* time per channel read */
#define NO_PROGRESS_MS  12000     /* rulebook stops us after 15 s of no progress -> self-recover before */

/* ============================ GRIPPER ================================= */
#define JAW_OPEN_DEG    120
#define JAW_CLOSED_DEG  35
#define LIFT_DOWN_DEG   20
#define LIFT_UP_DEG     70

/* ============================ EEPROM ================================== */
#define EE_MAGIC        0x5A
#define EE_ADDR_MAGIC   0
#define EE_ADDR_CLUES   1        /* 4 bytes: blue, green, right, left */
#define EE_ADDR_CAL     8        /* calibration block */

/* TCS3200 calibration record (white reference + chroma thresholds), stored in EEPROM */
struct ColourCal { float whiteR, whiteG, whiteB; float blueScore, greenScore; };

#endif

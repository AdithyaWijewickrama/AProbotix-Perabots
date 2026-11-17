# GuardianBot — build guide & bill of materials

PeraBots 2026 University Final ("Nine Seals of Pera"). This is the real robot the
Webots model `protos/GuardianBot.proto` is built from — same dimensions, sensor
positions, wheel size, encoder resolution and mass.

## 1. Why this shape

| Rule / arena fact | Design answer |
|---|---|
| Max 20 × 20 × 20 cm | 130 mm Ø × ~105 mm tall (147 mm long with gripper arms open) |
| Maze cell clear width 238 mm | Round body, wheels on the centre line → spins in place with 54 mm to spare each side. Nothing sticks out when turning, so no wall hits |
| Obstacle gaps | Widest-robot check on your cube layout: up to 228 mm Ø passes; 130 mm leaves a big margin |
| Pieces: 50 mm cube / 50 mm Ø cylinder, must be picked and placed | Rear **capture pocket** (62 mm wide, 50 mm deep) fully inside the body circle + two servo arms that close behind the piece. Both piece shapes self-centre. No lift needed — piece slides on the floor |
| Black 20 mm lines, coloured tiles, varying lighting | 8-ch IR line array (own IR emitters, ambient-immune), camera for tiles, colour sensor with its own white LED for the held piece, **CAL button** for the 2-min calibration |
| 2 mm section edges | 9 mm ground clearance, 40 mm wheels, steel ball caster |
| Collisions cost marks | 10 ToF sensors on a ring (every 23°) + encoders + IMU heading |

Old TwoWheelRobot problems this fixes: the ToF sensors faced the opposite way to
how the controller drove the robot (the controller drove "backwards" with yaw+π,
the sensors were mounted on the other end) → wall hits; the gripper fingers were
fixed boxes 48 mm apart (a 50 mm piece can't fit) → no pick-up; no line sensors.

## 2. Bill of materials (best-performance build)

Prices are approximate list prices (USD) — check your supplier. ✔ = you said you already have it.

### Drive
| # | Part | Qty | Why this one | ≈ $ |
|---|---|---|---|---|
| 1 | **Pololu 50:1 Micro Metal Gearmotor HPCB 6V, extended back shaft** (#3073) ✔ | 2 | 650 rpm, 0.74 kg·cm, 9.5 g. With 40 mm wheels → 1.36 m/s top speed (plenty: controller runs 0.2–0.3 m/s) with enough torque for 3–5 m/s² accel on a 0.23 kg robot. 30:1 is faster but weaker at low speed; 100:1 too slow | 2 × 22 |
| 2 | **Pololu Magnetic Encoder Pair Kit, 12 CPR, 2.7–18 V** (#4761) ✔ | 1 pr | 617 counts per wheel rev = 0.2 mm per count. Modelled exactly in Webots (`resolution 0.01018`) | 10 |
| 3 | **Pololu Wheel 40×7 mm pair, white** (#1452) | 1 pr | Silicone tyre = best grip on the arena vinyl/flex; 40 mm clears 2 mm edges | 7 |
| 4 | Pololu Micro Metal Gearmotor Bracket pair, black (#989) | 1 pr | Rigid, lightweight mounts | 5 |
| 5 | **Pololu Ball Caster with 3/8″ Metal Ball** (#951) | 1 | Low friction, rolls over edges. Shim to 12.4 mm | 3 |
| 6 | PTFE glide dots (≈ 7 mm) | 2 | Rear anti-tip skids, 1.5 mm off the floor | 1 |

### Brains & power
| # | Part | Qty | Why | ≈ $ |
|---|---|---|---|---|
| 7 | **ESP32-S3-DevKitC-1 N16R8** ✔ | 1 | Dual-core 240 MHz + FPU, 2 I²C buses, enough ADC pins for the 8 line channels, hardware PCNT for encoders, MCPWM for motors | 15 |
| 8 | **Seeed XIAO ESP32S3 Sense** (OV2640/OV3660 cam) | 1 | Dedicated vision co-processor: does tile colour detection on-board, sends results to the main S3 over UART. Keeps the main loop fast. 21×17.5 mm, ~8 g | 14 |
| 9 | **Pololu TB6612FNG Dual Motor Driver Carrier** (#713) | 1 | 1.2 A cont / 3.2 A peak per channel, handles the 1.5 A stall of HPCB motors (DRV8835 is too close to its limit) | 5 |
| 10 | **Pololu 5V 2.5A Step-Down Regulator D24V22F5** (#2858) | 1 | Clean 5 V for servo + logic; servo spikes won't brown-out the ESP32 | 9 |
| 11 | **2S 7.4 V LiPo, 450–550 mAh, ≥30C** (e.g. GNB / Tattu 450 mAh) | 2 | ~28 g each; 15+ min of run time. Bring 2 charged — battery swap is the only thing allowed in the 2-min calibration slot | 2 × 8 |
| 12 | XT30 connector pair, slide switch (SS12D10), 1N5819 / P-MOSFET reverse protection, 10k/3.3k divider for battery voltage | 1 set | Safety + battery monitoring | 4 |
| 13 | LiPo balance charger (e.g. ISDT/SkyRC) + LiPo bag | 1 | — | — |

Motors run from battery voltage (7.4–8.4 V) through the TB6612 — cap PWM at ~75 % in firmware so they see ≤6.3 V average and keep the brushes healthy.

### Sensors
| # | Part | Qty | Why | ≈ $ |
|---|---|---|---|---|
| 14 | **Pololu VL53L4CD ToF carrier** (#3692) | 10 | Best short-range ToF for walls: up to 1.2–1.3 m, 1 mm resolution, up to 100 Hz, much better ambient-light rejection than VL53L0X, 0.5 g. Positions: front 0°, ±23°, ±46°, ±69°, ±90° and one in the pocket facing back (also confirms a held piece) | 10 × 11 |
| 15 | **MCP23017 I/O expander** (breakout or SOIC on the main PCB) | 1 | Drives the 10 XSHUT pins to give each ToF its own I²C address at boot | 3 |
| 16 | **Pololu QTRX-MD-08A** 8-ch analog reflectance array, 8 mm pitch (#4448) | 1 | 61×20 mm, 3 g, low current, dimmable emitters (CTRL pin) for calibration. 64 mm span covers the 20 mm line with room to correct | 9 |
| 17 | **Adafruit BNO085 9-DoF IMU** (#4754) | 1 | On-chip sensor fusion "game rotation vector" — drift-free heading for 90° turns without magnetometer trouble | 25 |
| 18 | **TCS34725 RGB colour sensor** (Adafruit #1334) | 1 | On the pocket back wall with its own white LED: tells blue cylinder from green cube regardless of room light | 8 |
| 19 | 2 × 12 mm tactile buttons (START, CAL), WS2812B or 2 × 3 mm LEDs (blue/green clan indicator), buzzer | 1 set | CAL button is required by rule 5.3 | 2 |

### Gripper
| # | Part | Qty | Why | ≈ $ |
|---|---|---|---|---|
| 20 | **MG90S metal-gear micro servo** (or Emax ES08MA II) | 1 | 1.8 kg·cm, 13 g, metal gears don't strip when an arm is blocked | 4 |
| 21 | 3 mm steel shaft (2 × 55 mm), 2 × 3×7×3 mm bearings / brass bushings, 1 mm steel push-rods + Z-bends | 1 set | Arm hinges through the top plate; one servo drives both arms through push-rods | 3 |
| 22 | TPU (or rubber) pads on arm inner faces | 2 | Grip without scratching pieces (rule 4.3) | — |

### Structure
| # | Part | Qty | Why | ≈ $ |
|---|---|---|---|---|
| 23 | **Bottom deck = custom 1.6 mm PCB, 130 mm Ø with rear notch** (JLCPCB/PCBWay) | 1 | Chassis *and* wiring in one: TB6612, MCP23017, regulator footprints, JST-SH sockets for ToF. Lightest, neatest, judges score construction quality | 5–10 |
| 24 | **Top deck = 1.5 mm carbon-fibre plate, 130 mm Ø** | 1 | Stiff, ~20 g. (Budget alt: 3 mm acrylic or a second PCB) | 8 |
| 25 | PETG 3-D prints: pocket walls, gripper arms, ToF brackets (ring), camera mast, caster shim | 1 set | PETG is tougher than PLA and doesn't creep | — |
| 26 | M2.5 × 48 mm aluminium standoffs | 4 | Deck spacing: 62 mm top deck clears the 50 mm pieces | 2 |
| 27 | M2/M2.5 screws & nuts, JST-SH 4-pin (Qwiic) cables, 26 AWG silicone wire, heat-shrink | — | — | 5 |

**Mass estimate ≈ 225 g. Total ≈ $330** (≈ $270 without the parts you already have).

## 3. Layout (top view, + x = forward)

```
               front
          ToF  ToF  ToF            camera mast (XIAO Sense) at x=48 mm, 92 mm high, 23° down
      ToF   [ line array ]  ToF    line array under deck, x=30–50 mm
   ToF   battery 2S  (x 15–45)  ToF ball caster x=55 mm
  ToF ==[motor]==  O  ==[motor]== ToF   <- wheels on the centre line, 111 mm track
      skid |  pocket 62 mm   | skid
           |  (piece here)   |
         arm ->           <- arm   hinges at x=-50 mm, servo on top deck
               rear
```

Top deck: ESP32-S3 DevKit (rear-centre), BNO085 near the turning centre, TB6612 and
regulator front-left/right, START/CAL buttons and power switch at the back, clan LEDs at the front.

## 4. Wiring (ESP32-S3-DevKitC-1)

| Function | Pins |
|---|---|
| Line array 8 × analog (all on ADC1) + emitter CTRL | GPIO 1, 2, 4, 5, 6, 7, 8, 9 + GPIO 21 |
| Battery voltage (10k/3.3k divider) | GPIO 10 (ADC1) |
| TB6612: PWMA, AIN1, AIN2 / PWMB, BIN1, BIN2 (STBY → 3V3) | GPIO 11, 12, 13 / 14, 15, 16 |
| Encoders L A/B, R A/B (PCNT units) | GPIO 17, 18 / 38, 39 |
| I²C bus 0: 5 × ToF + BNO085 | SDA 40 / SCL 41 |
| I²C bus 1: 5 × ToF + TCS34725 + MCP23017 (drives XSHUT 0–9) | SDA 42 / SCL 47 |
| Gripper servo PWM (LEDC) | GPIO 48 |
| UART1 to XIAO ESP32S3 Sense | TX 43 / RX 44 (program the S3 through its native-USB port) |
| START button / CAL button | GPIO 3 / GPIO 0 (in parallel with the on-board BOOT key) |
| WS2812 clan LED / buzzer (via NPN, base pull-down) | GPIO 45 / GPIO 46 |

(GPIO 35–37 are taken by the octal PSRAM on N16R8 boards and 19/20 are the native USB — leave them free. Don't hold CAL while powering on, or the S3 enters download mode.)

## 5. Firmware notes that matter on the real arena

* **Calibration button (rule 5.3):** hold CAL → robot sweeps the line array across the line and records min/max per channel, captures white-floor and tile RGB from the camera, and stores them in NVS flash. Must finish in < 2 min.
* Use the BNO085 heading for turns and encoders for distance; fuse wall distances from ToF to correct drift (the Webots controller already does this).
* Ramp acceleration (≤ 3 m/s²); the robot is short, so harder starts can lift the front.
* Gripper: open = 0°, closed ≈ 60° (1.05 rad). Confirm capture with pocket ToF < 10 mm **and** TCS34725 colour.
* Pick-up = drive up to the collection point, spin 180°, reverse 10 cm into the piece, close. Place = reverse until the piece centre (33 mm behind the axle) is over the board intersection, open, drive away forward.

## 6. Webots files

* `make_guardianbot.py` → regenerates `protos/GuardianBot.proto` (change a dimension here, not in the proto).
* `apply_guardianbot.py` → switches worlds to GuardianBot and updates `main_controller.py` geometry/sign conventions (originals saved as `*.bak_twowheel`).
* `worlds/guardian_hw_test.wbt` + `controllers/guardian_hw_test` → gripper / sensor self-test.
* Device names: `left/right_wheel_motor`, `left/right_wheel_sensor`, `ds_*` (10 ToF, mm), `gs0..gs7` (line), `cam`, `piece_cam`, `gripper::left` (coupled with `gripper::right`), `imu`, `gyro`, `accel`, `led_blue`, `led_green`.

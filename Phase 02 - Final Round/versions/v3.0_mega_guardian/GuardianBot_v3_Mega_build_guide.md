# GuardianBot v3 — Arduino Mega build guide, parts list and competition procedure

PeraBots 2026 University Final, *The Nine Seals of Pera* — team AProbotix.
This replaces the v2 design (custom PCB / ESP32-S3 / TB6612) with a robot built from the parts
you already own: **Arduino Mega 2560, L298N, 2 × colour sensors, 2 × metal-gear servos, HC-SR04s,
2S LiPo**, plus a short shopping list that stays well inside Rs 15 000.
Firmware: `firmware/mega_guardian/` (see section 6). Logic unit test: `firmware/test/`.

---

## 1. Design in one page

| Rule / arena fact | Design answer |
|---|---|
| 20 × 20 × 20 cm limit | 160 mm long × 150 mm wide × ~140 mm tall, two-deck acrylic (or 3 mm plywood) chassis |
| Maze cell 238 mm clear, walls 100 mm high | Differential drive with the wheels on the centre line → spins in place; 4 × HC-SR04 at **45 mm** centre height see walls *and* the 50 mm game pieces |
| Obstacle field, 10 cm cubes, unknown layout | Occupancy grid (10 cm cells) built from the three forward ultrasonics + A* to the maze door, re-planned when blocked. Gyro (MPU6050) keeps headings honest |
| Tiles only in dead-end cells, lighting varies | Colour sensor #1 (TCS3200) under the front, 8 mm above the floor, with its own white LEDs → immune to room light. 2-minute CAL button routine (rule 5.3) stores white/blue/green references in EEPROM |
| Count R/L turns on the *shortest* path | Full map of the maze → Dijkstra (fewest cells, then fewest turns); turns in the exit cell not counted (rule 3.2) — unit-tested in `test_logic.c` |
| Nine-Point Board, 20 mm black lines | 5-ch TCRT5000 array 55 mm ahead of the axle: line following + junction counting; all board moves are scripted on the known Figure-1 geometry |
| Pick & place 50 mm cube / cylinder | Front **pocket gripper**: servo 1 closes two jaws (cube can't twist, cylinder is cradled), servo 2 lifts the whole jaw assembly ~25 mm so the piece clears the lines and never drags |
| Clue → piece mapping, Lo Shu numbering | `game_logic.h` builds the board from the 4 clues; if a clue value is unusable (0, >9, duplicate) the robot inspects the inner grid lines with the side ultrasonics *before* picking (rulebook 3.3) |
| Restart at Section 3 (rule 5.5) | Clues are written to EEPROM at the end of the maze; CAL button selects start stage 1/2/3 |
| 2 mm section edges | 65 mm wheels, 10 mm ground clearance, ball caster at the back |

Why **front** pocket (v2 had a rear pocket): the Mega + L298N stack is bulky, and a front pocket lets
the same colour sensor #2 look at the piece while the robot *approaches* it (verification) and while it
is held. It also makes the "place 150 mm in front of a line" move trivial.

---

## 2. Parts

### 2.1 Already have (per your message + photo)
| Part | Qty | Used for |
|---|---|---|
| Arduino Mega 2560 | 1 | main controller |
| L298N dual H-bridge | 1 (you have 2) | drive motors (second one is a spare) |
| Colour sensor (TCS3200 assumed) | 2 | #1 floor tiles, #2 piece verification |
| Metal-gear servos (MG995/MG996R assumed) | 2 | jaw, lift |
| HC-SR04 | 4 | F / L / R / B |
| 2S LiPo + XT60/XT30, LiPo voltage alarm (in photo) | 1 | power; the 3-digit alarm goes on the balance lead |
| LM2596 buck with display (in photo) | 1 | 5 V rail for servos + sensors |
| ESP32-S3 (any) | 1 | optional: Serial1 → Bluetooth/WiFi log viewer (NOT a controller, keeps rule 4.2 clean — it only *listens*) |
| Pololu HPCB 50:1 micro gearmotors + magnetic encoders (from v2 BOM) | 2 | drive — **confirm** |
| HC-05, DHT11, NodeMCU, pot (in photo) | – | not needed |

### 2.2 To buy (Sri Lanka). Prices are Nov-2026 web prices, rounded; check stock before travelling.
| # | Part | Qty | Where | ≈ LKR |
|---|---|---|---|---|
| 1 | **MPU-6050** gyro/accel module | 1 | tronic.lk (code MD0122, Rs 680); lk-tronics.com; duino.lk | 700 |
| 2 | **5-ch TCRT5000 line array** (TCRT5000L, analog or digital) | 1 | tronic.lk / duino.lk / lk-tronics / alphatronic.lk | 600–900 |
| 3 | **Wheels 65 mm rubber** (for 3 mm D-shaft N20/HPCB: "N20 wheel 43 mm" or 65 mm with 3 mm hub) | 1 pair | tronic.lk / duino.lk | 500–900 |
| 4 | Ball caster 3/8" or 1/2" (metal ball, with holder) | 1 | tronic.lk / duino.lk | 350–600 |
| 5 | N20 / micro-metal-gearmotor **mounting brackets** | 1 pair | tronic.lk / duino.lk | 300–500 |
| 6 | **3 mm acrylic sheet** 300 × 300 mm (or 3 mm plywood) + laser cutting | 1 | laser cutter at Peradeniya E-Fac / any sign shop in Kandy or Colombo (Pettah) | 800–1 500 |
| 7 | M3 standoffs 40 mm (×4) + 25 mm (×4), M3/M2 screws & nuts assortment | 1 set | tronic.lk / Pettah | 600–900 |
| 8 | Tactile push buttons (START, CAL) + 2 × 5 mm LEDs (blue, green) + buzzer + resistors | 1 set | any | 300 |
| 9 | Dupont jumpers (M-F, F-F, 20 cm, 40 pcs each) + 1 m silicone wire | 1 | any | 600 |
| 10 | Rocker/slide switch 3 A, XT60 → wire pigtail, 1000 µF 16 V capacitor (motor supply) | 1 | tronic.lk / Pettah | 400 |
| 11 | Second 2S LiPo (450–1000 mAh, 25C+) — swap allowed in calibration (rule 5.3) | 1 | tronic.lk / RC shops Colombo | 2 500–3 500 |
| 12 | PETG/PLA 3-D printed gripper jaws, pocket, sensor brackets (≈120 g filament) | 1 set | university printer / online print service | 1 500–2 500 |
| 13 | Spare HC-SR04 ×2, spare TCS3200 ×1 | – | tronic.lk | 1 200 |
| | **Total (with spares)** | | | **≈ 10 500 – 14 000** |

If you do **not** have the Pololu motors with encoders, add 2 × **N20 6 V 300 rpm with encoder**
(alphatronic.lk Rs 1 950 each, SKU MGE11) → +Rs 3 900; drop item 13 to stay under budget.
300 rpm × 65 mm wheel ≈ 1.0 m/s top speed, which is plenty; 600 rpm (Rs 2 250) is faster but weaker.

Shops: **tronic.lk** (Colombo 11 / Nugegoda, online, best stock), **duino.lk** (Moratuwa, online),
**lk-tronics.com**, **alphatronic.lk**, **micro.lk**; in Kandy the electronics shops around
Peradeniya Road / Kandy City Centre carry Arduino basics; Pettah (Colombo) for hardware and switches.
Galle/Gampaha: order online — delivery is 1–3 days.

**Raspberry Pi 400:** the board *can* be removed from the keyboard, but it is a custom full-keyboard-
width PCB (≈ 28 cm long) — it cannot fit the 20 cm limit. Not usable; don't buy it.

---

## 3. Mechanical layout (top view, front = +x)

```
            ┌──────────── 150 mm ────────────┐
            │  [US-L]                        │
            │   ┌──────┐   ┌──────────┐      │  deck 2 (top): Mega, L298N, LM2596, MPU6050,
   [jaw L]  │   │ LIFT │   │  Arduino │      │  buttons, LEDs, battery strap
 ┌─┐        │   │servo │   │   Mega   │  [US-B]
 │ │ pocket │   └──────┘   └──────────┘      │
 │ │ ◄── piece sits here, colour sensor #2   │  deck 1 (bottom): motors on the centre line,
 └─┘ at the pocket back, US-F above pocket   │  L298N, wheels, caster at back, line array
   [jaw R]  │   [US-R]                       │  and colour sensor #1 under the front lip
            └────────────────────────────────┘
   Wheel axle: 80 mm from the front-most point. Ball caster 50 mm behind the axle.
```

Heights: ultrasonic centres **45 mm** above floor (wall 100 mm, cube 100 mm, piece 50 mm all visible;
beam does not hit the floor inside 40 cm). Line array 3–5 mm above floor. Colour sensor #1 LEDs
8–10 mm above floor, shrouded with black tape so room light can't enter.

Gripper: two jaws hinged on the lift arm, closed by servo 1 through a small crank. Jaw opening
63 mm, closed on a 50 mm piece. Lift servo 2 raises the jaw+pocket assembly 25 mm. Pocket back wall
holds colour sensor #2 at ~15 mm from the held piece.

Centre of mass: battery under the Mega over the axle. Keep the gripper light (PETG, 2 mm walls).

---

## 4. Wiring

Power: LiPo (7.4 V) → switch → **L298N +12V input** (motors) and → **LM2596 set to 5.0 V** →
servos, sensors, line array, 5 V pin of the Mega (or Mega VIN from the LiPo — either, not both).
Put a 1000 µF cap across the L298N input. Common GND everywhere. Remove the L298N 5V-EN jumper only
if you feed the L298N logic from the Mega 5 V; simplest: leave the jumper, don't use its 5 V output.
Servos must be on the LM2596 (2–3 A), never on the Mega's regulator.

| Mega pin | Goes to | Mega pin | Goes to |
|---|---|---|---|
| 5 / 22 / 23 | L298N ENA / IN1 / IN2 (left) | 6 / 24 / 25 | L298N ENB / IN3 / IN4 (right) |
| 2 / 4 | left encoder A / B | 3 / 7 | right encoder A / B |
| 26 / 27 | US-F trig / echo | 28 / 29 | US-L trig / echo |
| 30 / 31 | US-R trig / echo | 32 / 33 | US-B trig / echo |
| 34 35 36 37 / 38 / 39 | TCS3200 #1 S0 S1 S2 S3 / OUT / LED | 40 41 42 43 / 44 / 45 | TCS3200 #2 S0 S1 S2 S3 / OUT / LED |
| A8 … A12 | line array OUT1 … OUT5 (left → right) | 8 / 9 | jaw servo / lift servo signal |
| 20 / 21 | MPU6050 SDA / SCL (3.3 V or 5 V board) | 10 / 11 | START / CAL buttons to GND |
| 12 / 13 | blue / green LED (+ 330 Ω) | 52 | buzzer (active) |
| A0 | battery divider: LiPo+ —100k— A0 —33k— GND | 18 / 19 | TX1/RX1 → ESP32-S3 (optional log) |

Encoders: if a wheel counts backwards, swap its A/B wires (or swap motor leads). Verify with the
serial log: both `encL`/`encR` must increase when pushed forward.

---

## 5. Build order (≈ 2 evenings)
1. Cut decks (DXF in `hardware/acrylic/` is for v2 — for v3 use a 150 × 160 mm rectangle with 50 mm
   corner radii; mark the axle line 80 mm from the front, motor bracket holes, caster holes).
2. Fit motors + wheels + caster. Check it rolls straight when pushed; wheel track = 120 mm.
3. Mount L298N and Mega on deck 2, battery between decks on a Velcro strap.
4. Print and fit the gripper. Set servo horns at the mid positions in `config.h`
   (`JAW_OPEN/CLOSED`, `LIFT_UP/DOWN`) *before* fixing the horns.
5. Mount the four HC-SR04 at 45 mm, line array and colour sensor #1 under the front.
6. Wire per section 4, flash `mega_guardian.ino`, open the serial monitor at 115200.
7. Bench tests (robot on a box): START → observe log; wheels forward; `turnTo` 90° ×4 lands
   back on heading; push a wall in front of US-F and read ≈ distance.
8. Floor tests: 1 m straight (tune `KP_HEAD`, `COUNTS_PER_REV`, `WHEEL_DIAMETER_MM`);
   maze corridor from cardboard + 12 mm walls (tune `WALL_SEEN_MM`, `KP_WALL`);
   a printed Nine-Point Board on paper (tune `KP_LINE`, `LINE_OFFSET_MM`).

---

## 6. Firmware (`firmware/mega_guardian/`)

| File | What |
|---|---|
| `mega_guardian.ino` | setup/loop, buttons, stage selection, calibration, EEPROM, mission sequencer |
| `config.h` | **all pins and tunables** |
| `game_logic.h` | pure-C: maze map, Dijkstra path + turn count, Lo Shu board, winning move, fallback |
| `protos.h` | prototypes for functions with default arguments (Arduino IDE quirk) |
| `a_sensors.ino` | HC-SR04, TCS3200 read/classify/calibrate, line array, MPU6050, battery |
| `b_motion.ino` | encoders, gyro+encoder odometry, `driveStraight`, `turnTo`, `followLine`, gripper |
| `c_section1.ino` | occupancy grid + A* + re-planning |
| `d_section2.ino` | frontier exploration, wall sensing, tiles, clue computation, exit |
| `e_section3.ino` | collection, inspection, placement, END |

Libraries: only `Wire`, `Servo`, `EEPROM` (all built-in). Board: Arduino Mega 2560.
Host test: `cd firmware/test && gcc -I../mega_guardian test_logic.c -o test_logic && ./test_logic`
(21 checks, all passing).

Buttons: **START** short = run from the selected stage. **CAL** short = cycle stage 1→2→3
(LEDs blink the stage number). **CAL** hold 1.5 s = calibration routine:
gyro bias (keep still) → line threshold (array over the line edge) → floor colour: white now,
press CAL on a blue tile, press CAL on a green tile → gripper colour: empty, blue piece, green piece
(each step times out after 10 s and keeps the old value). Everything is saved to EEPROM.

Log: everything is printed on USB serial *and* Serial1 (115200). Flash the ESP32-S3 with any
"UART → Bluetooth SPP / WiFi telnet bridge" sketch to read the log from a phone during practice.
It must only receive; do not wire its TX to the Mega during the competition (rule 4.2 / 7.2).

---

## 7. Competition-day procedure

**Practice session (30–40 min)** — in this order, don't skip:
1. Hold CAL → full calibration on the practice arena (gyro, line, tiles, pieces). Repeat under
   the brightest and darkest spots; the TCS3200 LEDs make it robust but check the log numbers.
2. Run stage 3 alone (CAL ×2 → START) with hand-entered clues? No — set stage 3 and let it use the
   EEPROM clues from the last maze run; verify pick/place/END.
3. Run stage 2 alone, check tile detection and the printed clue numbers against your own count.
4. One full run. Note the times.
5. Charge/swap the battery; hand over the robot.

**Official 2-minute calibration:** swap in the fresh battery; hold CAL; place the floor sensor on
white, then on a blue tile, then a green tile when the buzzer asks (judges normally let you touch the
tiles near the start; if not, skip with the 10 s timeout — the practice-arena values stay).

**15-minute period, 3 attempts:** START at the start zone. If stopped in Section 1 or 2 the judge
will restart you at the next trial's start: press CAL once (stage 2) or twice (stage 3), then START.
Stage 3 uses the clues saved by the last completed maze run.

Tie-breaker priorities (rulebook 6.5) are completion > progress > time > collisions, so the firmware
favours **finishing with zero collisions** over raw speed: `SPEED_NORMAL` is conservative. Raise it
only after the course is reliable.

---

## 8. Known simplifications / what to test first
* `S1_GOAL`, `S1_START`, maze entry at y = 1330 mm and the Section 3 line geometry come from the
  rulebook Figure 1 as used by the Webots model. Re-measure on the practice arena and edit `config.h`.
* Section 3 junction counting assumes the line layout of Figure 1 (exit T at (2210, 3050), collection
  crossing at x = 2006, board edge at x = 1700). If the practice arena differs, adjust the constants
  in `e_section3.ino` top block.
* Side-ultrasonic inspection flags a position as occupied when the reading is < 130 mm while passing
  a cell centre. Pieces are 150 mm from the line, so expect ~90 mm readings.
* The floor colour sensor samples while *entering* a cell and again at the centre; tiles are usually
  centred, but if the tiles are small, slow `SPEED_NORMAL` for the last cell of a dead end.

# GuardianBot v2: build guide and parts list

PeraBots 2026 University Final, "Nine Seals of Pera".
This is the real robot behind the Webots model `simulation/protos/GuardianBot.proto`. Every
dimension comes from `hardware/cad/guardianbot_geometry.py`. The same file drives the Webots
model, the laser-cut acrylic, the 3-D printed parts and the main PCB, so they all match.

The sensors are the low-cost set you asked for: 4 x HC-SR04 ultrasonic and 4 x Sharp GP2Y0A41SK0F IR.
The motors (Pololu HPCB with encoders) and the ESP32-S3 are parts you already have.

---

## 1. How the design meets the rules

| Rule / arena fact | Design answer |
|---|---|
| Must fit 20 x 20 x 20 cm | 134 mm across the ultrasonic ring, 130 mm body, 122 mm tall |
| Maze cell is 238 mm wide inside | Round body with the wheels on the centre line, so it spins in place with 50 mm to spare |
| Obstacle field gaps | A robot up to 228 mm wide fits through your cube layout, so 135 mm has plenty of margin |
| Pieces: 50 mm cube and 50 mm cylinder | Rear pocket that narrows to 55 mm (a cube cannot twist), opens to 63 mm, and has 2 servo arms. The piece slides on the floor; no lift is needed |
| Black 20 mm grid lines | 8-channel IR line array; the firmware corrects position on lines and intersections |
| Coloured tiles, changing lighting | Camera (XIAO ESP32S3 Sense) sees tiles from a distance. The CAL button runs the 2-minute calibration (rule 5.3) |
| Walls and cubes are 100 mm tall; pieces are 50 mm | Ultrasonics sit at 83 mm, so they see walls and cubes but not pieces. The Sharp IRs sit at 35-40 mm, so they also see pieces when inspecting the board |
| 2 mm section edges | 9 mm ground clearance, 40 mm wheels, steel ball caster |

### What the simulation shows (Webots R2025a, your maze and Figure-1 Section 3)

- Section 1 done at about 26 s. Section 2 explored, with all 5 tiles found (3 blue, 2 green), by about 87 s.
- Full run including Section 3 takes about 182 s with zero collisions.
- In Section 3 the robot takes the cylinder from the collection point. Your maze gives the same number for both cylinder clues (3 right turns and 3 blue tiles), so the clues are not enough on their own. The robot then:
  1. drives the two inner grid lines and finds the occupied cells with the side Sharps;
  2. checks one piece's colour with the camera;
  3. places its piece in position 7, 5 mm from the cell centre;
  4. drives to END.
- When the clues fully describe the board (tested with clues 3/2/5/4 and 3/4/5/2), the robot places the piece directly, without inspecting.
- Restart in Section 3 (rule 5.5): the robot stores its clues after the maze. Use `simulation/worlds/section3_start.wbt` (controller argument `section3`).

---

## 2. Bill of materials

✔ = you already have it. Prices are rough single-unit prices in USD, for comparison only.

### Drive
| Part | Qty | Notes | ≈ $ |
|---|---|---|---|
| Pololu 50:1 Micro Metal Gearmotor HPCB 6V, extended shaft (#3073) ✔ | 2 | 650 rpm and 0.74 kg·cm, so about 1.3 m/s top speed on 40 mm wheels | – |
| Pololu magnetic encoder pair, 12 CPR (#4761) ✔ | 1 pr | 617 counts per wheel turn | – |
| Pololu wheel 40 x 7 mm pair (#1452) | 1 pr | Silicone tyre | 7 |
| Pololu micro metal gearmotor bracket pair (#989) | 1 pr | 2 x M2 screws each, 18 mm apart | 5 |
| Pololu ball caster, 3/8" metal ball | 1 | Printed 1.6 mm spacer sets the height | 3 |
| PTFE glide domes, M3 screw-in (about 7 mm) | 2 | Rear anti-tip, 1.5 mm off the floor | 1 |

### Brains and power
| Part | Qty | Notes | ≈ $ |
|---|---|---|---|
| ESP32-S3-DevKitC-1 N16R8 ✔ | 1 | Plugs into female headers J1/J3 on the main PCB | – |
| Seeed XIAO ESP32S3 Sense | 1 | Tile camera. Sends results to the main S3 as an I2C slave at 0x42 | 14 |
| TB6612FNG (SSOP-24) | 1 | Motor driver, soldered on the PCB | 2 |
| MCP23017-E/SO (SOIC-28) | 1 | 16 extra I/O for ultrasonic triggers, motor direction, buttons and LEDs | 1.5 |
| Pololu D24V22F5 5 V 2.5 A buck regulator | 1 | Plugs into U4 (pins PG, EN, VIN, GND, VOUT; check its silkscreen) | 9 |
| AO3401A P-MOSFET (SOT-23) | 1 | Reverse-battery protection | 0.1 |
| 2S 7.4 V LiPo, 450–550 mAh, 30C+, XT30 | 2 | Bring 2 charged; a battery swap is allowed during calibration | 2 x 8 |
| XT30 PCB male + cable female | 1 | | 1 |
| Panel/slide power switch, at least 3 A | 1 | On J21 | 1 |

### Sensors (low-cost set)
| Part | Qty | Where | ≈ $ |
|---|---|---|---|
| HC-SR04 ultrasonic ✔ | 4 | Front, left, right, back, hung under the crown at 83 mm | – |
| Sharp GP2Y0A41SK0F (4–30 cm) | 4 | Front-left and front-right at 45°, left and right at 90°, mounted vertically | 4 x 9 |
| Pololu QTRX-MD-08A line array (8 ch, 8 mm pitch) | 1 | Under the base, 40 mm ahead of the axle | 9 |
| TCS34725 colour sensor (Qwiic) | 1 | Behind the window in the pocket back wall | 8 |
| BNO085 IMU (Qwiic) | 1 | On top of the crown | 25 |

A cheaper IMU option is an MPU-6050 at about $3, but expect heading drift.

### Gripper
| Part | Qty | Notes | ≈ $ |
|---|---|---|---|
| MG90S metal-gear micro servo | 1 | In the printed cradle on the PCB | 4 |
| 3 mm steel rod, cut to 2 x 60 mm | 1 | Arm hinge pins | 1 |
| Flanged bushing, 3 mm ID x 4 mm OD | 4 | In PCB holes H9/H10 and in the pocket | 1 |
| 1 mm steel push-rod with Z-bends | 2 | Servo horn to the hinge levers | 1 |
| M2 grub screws | 4 | Arm hubs and levers | – |

### Structure and hardware
| Part | Qty | Notes |
|---|---|---|
| Main PCB, 130 mm round, 2 layers, 1.6 mm (JLCPCB) | 1 | `hardware/pcb/guardianbot_main_gerbers_JLCPCB.zip` |
| 3 mm black acrylic base plate | 1 | `hardware/acrylic/acrylic_base_3mm_black.dxf` |
| 3 mm clear acrylic crown | 1 | `hardware/acrylic/acrylic_crown_3mm_clear.dxf` |
| PETG prints: pocket, 2 arms, 2 hinge levers, servo cradle, 4 Sharp brackets, 4 HC-SR04 hangers, camera mast, caster spacer | 1 set | `hardware/3d_prints/*.stl` |
| TPU 95A gripper pads | 2 | `gripper_pad.stl` |
| M3 x 48 mm brass standoffs (base to PCB) | 4 | |
| M3 x 35 mm brass standoffs (PCB to crown) | 4 | |
| Heat-set inserts: M2.5 x 4 (pocket), M2 x 3 (brackets) | 6 + 12 | |
| M2/M2.5/M3 screws, JST-XH crimp housings, Qwiic cables, 26 AWG silicone wire | – | |
| Hook-and-loop battery strap, 10 mm | 1 | Goes through the base-plate slots |

**Estimated total: about $230. About $120 more if you also order PCB assembly.**
**Estimated mass: about 270 g.**

---

## 3. Main PCB (`hardware/pcb/`)

- KiCad 7 project: `guardianbot_main.kicad_pro`, `.kicad_sch` and `.kicad_pcb`. There is also a PDF of the schematic and a 3-D STEP of the board.
- The board is fully routed, with GND pours on both layers. DRC is clean apart from silkscreen warnings. Rules: 0.2 mm tracks, 0.15 mm clearance, 0.6/0.3 mm vias, which is JLCPCB's standard 2-layer service.
- Order it by uploading `guardianbot_main_gerbers_JLCPCB.zip` to JLCPCB with 2 layers and 1.6 mm thickness; colour is up to you.
- The connectors for the IR sensors, motors, line array, XT30 and power switch are on the **bottom** side, so cables go down to the base. The ultrasonic, camera, servo and Qwiic connectors are on top.

| Connector | Pins (pin 1 first) |
|---|---|
| J4/J5/J6/J7 ultrasonic F/L/R/B | 5V, TRIG, ECHO, GND (straight HC-SR04 cable). Each echo has a 10k/20k divider down to 3.3 V |
| J8/J9/J10/J11 Sharp FL/FR/L/R | Vo, GND, 5V (standard Sharp JST order), plus 10 µF each |
| J12/J13 motor L/R | M1, M2, 3V3, GND, A, B. **Match these to the labels on your encoder board** |
| J14 line array | 3V3, GND, CTRL, OUT1…OUT8 |
| J15 servo | S, +5V, GND |
| J16/J17 Qwiic | GND, 3V3, SDA, SCL |
| J18 XIAO | 5V, GND, SDA, SCL |
| J20 / J21 | Battery XT30 / external power switch |

### ESP32-S3 pin map (as routed)
| GPIO | Function | GPIO | Function |
|---|---|---|---|
| 1,2,4,5,6,7,8,9 | Line sensors 1–8 (ADC1) | 15 / 16 | PWMA / PWMB |
| 10 / 11 / 12 / 13 | Sharp FL / L / R / FR (ADC) | 17,18 / 21,47 | Left encoder A,B / right encoder A,B |
| 14 | Battery voltage (100k/33k divider) | 40 / 41 / 42 / 39 | Ultrasonic echo F / L / R / B |
| 3 / 46 | I2C SDA / SCL (4.7k pull-ups) | 48 | Servo PWM |

| MCP23017 (0x20) | Function |
|---|---|
| GPA0–3 | Ultrasonic TRIG F/L/R/B |
| GPA4 | Line array CTRL |
| GPA5 / GPA6 | START / CAL buttons (use internal pull-ups) |
| GPA7 | TB6612 STBY |
| GPB0–3 | AIN1, AIN2, BIN1, BIN2 |
| GPB4 / GPB5 | Blue / green clan LED |

**Check the DevKit header pinout against your board's silkscreen before ordering.**
The design uses the standard ESP32-S3-DevKitC-1 pinout and a 22.86 mm row spacing.

---

## 4. Build order

1. **Base plate.** Laser-cut it, then fit the motor brackets, the caster with its spacer, the line array (M2 with 2 mm spacers) and the PTFE skids.
2. **Sharp brackets.** Screw the 4 brackets to the base and mount the Sharps vertically, lens facing out.
3. **Pocket and gripper.**
   - Put the inserts in the pocket and screw it to the base tabs.
   - Push the hinge pins through the bushings and fit the arms (grub screws) and the TPU pads.
4. **Standoffs and battery.** Fit the 48 mm standoffs and strap the LiPo on.
5. **PCB.**
   - Solder the PCB, put the DevKit in its sockets, screw on the servo cradle and fit the MG90S.
   - Fit the hinge levers on the pins above the PCB and link them to the servo horn with the push-rods. The arms should be parallel (open) at 0° and closed at about 60°.
6. **Crown.** Fit the 35 mm standoffs, the clear crown, the 4 HC-SR04 hangers, the camera mast with the XIAO, and the BNO085 on top.
7. **First test.** Run the `guardian_hw_test` routine: it prints every sensor, then picks up, carries and releases a piece.

### Firmware notes for the real arena

- **Ultrasonics.** Fire one at a time, at least 25 ms apart, to avoid cross-talk. A lost echo means "unknown", not "free". The Webots controller treats lost echoes the same way.
- **Sharps.** Use a lookup table from volts to cm. Readings under 4 cm fold back, so treat them as "closer than 4 cm".
- **CAL button.** Sweeps the line array across a line to learn its minimum and maximum, samples the floor and tile colours with the camera, and saves everything to NVS flash.
- **Section 3 restart.** Save the 4 clues to NVS at the end of the maze. On a restart in Section 3, load them and run Section 3 only.

---

## 5. Files

See the folder map in [`../README.md`](../README.md).

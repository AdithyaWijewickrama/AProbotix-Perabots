# GuardianBot v4.1 firmware

| Folder | What it is |
|---|---|
| `mega_body/` | **Arduino Mega 2560** (real-time body): `mega_body.ino`, `pins.h` (pin map = the PCB deck), `protocol.h` (link frames) |
| `esp32s3_brain/` | **ESP32-S3 N16R8 + OV7670** = brain + eye (competition firmware, no Wi-Fi). Core 1 runs the mission (Sections 1–3); core 0 runs the camera task (160 × 120 frame → 48 × 32 grid). `config.h` holds geometry, speeds and servo angles; `cam_ov7670.h` the camera pins and driver; `colour.h` the colour classes; `protocol.h` is an identical copy |
| `esp32s3_testbench/` | **Test-bench firmware** for the same S3: a Wi-Fi access point with a web page for sensors, manual drive, camera view, calibration and CSV logging. Calibration is saved in flash (namespace `gb`) and loaded by the brain. `web/index.html` is the page; `tools/make_page_h.py` turns it into `web_page.h` |
| `hostcheck/` | `check.sh`: compiles all three sketches on a PC and checks that the shared headers are identical with stub Arduino headers (`g++ -Wall -Wextra`) |
| (see also) | `simulation/wokwi/`: the Mega firmware in the Wokwi circuit simulator with every sensor, plus a simavr co-simulation test |
| `sil/` | `build_sil.sh`: builds the brain as a PC program. `simulation/controllers/mega_sil` plays the Mega and the camera in Webots, so the exact brain firmware is tested against the simulated arena |

Older firmware is recorded in `versions/`: v4.0 (ESP32-CAM brain) in `versions/v4.0_esp32cam_pcbdeck/firmware/` and tag `v4.0.1`; v3.3 (Mega + ESP32-S3 + a separate ESP32-CAM) in tag `v3.3-pcb`.

`protocol.h`, `cam_ov7670.h` and `colour.h` are copied into each sketch folder (the Arduino IDE can't include from `../`). Edit them in `esp32s3_brain/` and copy them; `hostcheck/check.sh` fails if the copies differ.

## Boards and libraries

- **Mega:** board "Arduino Mega or Mega 2560". Libraries from the Library Manager: **VL53L0X** by Pololu and **SSD1306Ascii** by Bill Greiman. `Servo` and `Wire` are built in.
- **ESP32-S3 (both sketches):**
  - Board "ESP32S3 Dev Module" (esp32 core 3.x).
  - Flash Size **16MB**, PSRAM **OPI PSRAM**, Partition scheme "16M Flash (3MB APP/9.9MB FATFS)", USB CDC On Boot **Enabled**.
  - Upload through the USB-C port marked USB (hold BOOT, tap RST if it doesn't connect). No extra libraries: the camera driver (`esp_camera`) is part of the core.
  - `esp32s3_brain` never starts Wi-Fi or Bluetooth (rule 4.2). Its debug log is on the USB port at 115200.
  - `esp32s3_testbench` starts the Wi-Fi access point **GuardianBot-test** (password `aprobotix26`); the page is at http://192.168.4.1. Use it only on the bench.

## Link protocol (`protocol.h`)

- **Link:** 250000 baud, 8N1. Mega Serial2 (TX2 D16 / RX2 D17) ↔ ESP32-S3 GPIO41 (RX) / GPIO42 (TX).
- **Level shifting on the PCB deck:**
  - A 1k8/3k3 divider takes Mega TX down to 3.2 V.
  - A 2N7000 with two 10 k pull-ups takes ESP TX up to 5 V. The 3.3 V comes from the S3 board's own regulator (5th wire of the link cable).
- **Frame format:** `A5 5A type len payload sum8`.
- **Telemetry** (`T`, Mega → ESP, every 20 ms):
  - time
  - encoder ticks
  - gyro yaw and yaw rate
  - 2 sonar distances (mm)
  - 8 ToF distances (mm)
  - 8 QTR-8RC decay times (µs, 0–2500)
  - R, G, B, Clear counts from both TCS3200
  - battery mV, keypad bits, flags
- **Command** (`C`, ESP → Mega, every step):
  - left and right wheel speed (mm/s)
  - 2 servo angles
  - LED, colour-sensor LED and buzzer bits
  - zero-odometry and gyro-calibration triggers
- **Text** (`X`, ESP → Mega, about once a second): two 20-character lines for the OLED.

The Mega puts the DRV8833 to sleep if no command arrives for 250 ms.

## Keypad (1 × 4)

| Key | Action |
|---|---|
| K1 GO | Start. The brain detects whether it is in the START area or at the maze entrance |
| K2 MAZE | Same as K1 (use it when the robot is placed at the maze) |
| K3 S3 | Section 3 restart with the clues saved in flash |
| K4 CAL | White-balance both TCS3200 **and the camera** (white paper under and in front of the robot); saved in flash |

## First power-up checklist

1. **Mega only.** Flash `mega_body`.
   - With only USB connected, the OLED shows "ToF n/8 gyro ok". If n < 8, check that sensor's cable and XSHUT wire.
   - The Serial Monitor at 115200 prints one status line per second (sonar, ToF, line, encoders, yaw, keys, battery).
2. **Encoders.** Turn each wheel forward by hand. `enc` must count **up** on both wheels; otherwise flip `ENC_L_SIGN` / `ENC_R_SIGN`. Both are +1 for motors wired as on the PCB deck: the right encoder is read with A/B swapped to undo the mirrored motor.
3. **Encoder counts.** Measure `ENC_CPR`: turn a wheel 10 times by hand, read `enc` and divide by 10. The gear ratio of the 300 rpm N20 is not printed. Easier: the test page's **Calibrate → Drive 1 m** (step 4). Set the value in `mega_body.ino`; the brain reads it from flash (`config.h` holds the default).
4. **Test bench.** Flash `esp32s3_testbench`, plug in the link cable and power everything from the LiPo. Open http://192.168.4.1:
   - **Sensors** tab: every reading moves when you wave a hand or a card.
   - **Camera** tab: camera ID 0x76 and a live picture. Use the FOV helper (needs ≥ 52°).
   - **Calibrate** tab: Drive 1 m, Spin 360°, gripper angles, ToF offsets. Saved in flash.
5. **Brain.** Flash `esp32s3_brain`. The OLED shows "READY 1:GO 3:S3 4:CAL".
6. **Motors.** Wheels off the ground: press K1. Both wheels must turn **forward** first; otherwise flip `MOT_*_SIGN`. Press reset to stop.
7. **Gripper.** If not set on the test page, set the gripper angles (`SERVO_*` in `config.h`). Open means the arms are parallel to the pocket walls. Closed means they hold a 50 mm cube firmly.
8. **Colour sensors and camera.** Press K4 with white paper under and in front of the robot.

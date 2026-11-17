# GuardianBot v4 firmware

| Folder | What it is |
|---|---|
| `mega_body/` | **Arduino Mega 2560** (real-time body): `mega_body.ino`, `pins.h` (pin map = the PCB deck), `protocol.h` (link frames) |
| `esp32cam_brain/` | **ESP32-CAM (AI-Thinker, OV2640)** = brain + eye. Core 1 runs the mission (Sections 1–3); core 0 runs the camera task (160 × 120 frame → 48 × 32 grid). `config.h` holds geometry, speeds and servo angles; `protocol.h` is an identical copy |
| `hostcheck/` | `check.sh`: compiles both sketches on a PC with stub Arduino headers (`g++ -Wall -Wextra`) |
| (see also) | `simulation/wokwi/`: the Mega firmware in the Wokwi circuit simulator with every sensor, plus a simavr co-simulation test |
| `sil/` | `build_sil.sh`: builds the brain as a PC program. `simulation/controllers/mega_sil` plays the Mega and the camera in Webots, so the exact brain firmware is tested against the simulated arena |

The older firmware (v3.3: Mega + ESP32-S3 + a separate ESP32-CAM vision board) is kept in git tag `v3.3-pcb` and recorded in `versions/`.

## Boards and libraries

- **Mega:** board "Arduino Mega or Mega 2560". Libraries from the Library Manager: **VL53L0X** by Pololu and **SSD1306Ascii** by Bill Greiman. `Servo` and `Wire` are built in.
- **ESP32-CAM:**
  - Board "AI Thinker ESP32-CAM" (esp32 core 3.x), PSRAM enabled.
  - Upload through the ESP32-CAM-MB adapter. No extra libraries.
  - Wi-Fi and Bluetooth are never started.
  - The debug log is on UART0 (the adapter's USB) at 115200.

## Link protocol (`protocol.h`)

- **Link:** 250000 baud, 8N1. Mega Serial2 (TX2 D16 / RX2 D17) ↔ ESP32-CAM GPIO15 (RX) / GPIO14 (TX). The SD card is not used, so these pins are free.
- **Level shifting on the PCB deck:**
  - A 1k8/3k3 divider takes Mega TX down to 3.2 V.
  - A 2N7000 with two 10 k pull-ups takes ESP TX up to 5 V. The 3.3 V comes from the ESP32-CAM's own regulator (5th wire of the link cable).
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
| K4 CAL | White-balance both TCS3200 (white paper under and in front of the robot) |

## First power-up checklist

1. **Mega only.** Flash `mega_body`.
   - With only USB connected, the OLED shows "ToF n/8 gyro ok". If n < 8, check that sensor's cable and XSHUT wire.
   - The Serial Monitor at 115200 prints one status line per second (sonar, ToF, line, encoders, yaw, keys, battery).
2. **Encoders.** Turn each wheel forward by hand. `enc` must count **up** on both wheels; otherwise flip `ENC_L_SIGN` / `ENC_R_SIGN`. Both are +1 for motors wired as on the PCB deck: the right encoder is read with A/B swapped to undo the mirrored motor.
3. **Encoder counts.** Measure `ENC_CPR`: turn a wheel 10 times by hand, read `enc` and divide by 10. The gear ratio of the 300 rpm N20 is not printed. Set the value in **both** `mega_body.ino` and `esp32cam_brain/config.h`.
4. **Brain.** Flash `esp32cam_brain`, plug in the link cable and power everything from the LiPo. The OLED shows "READY 1:GO 3:S3 4:CAL".
5. **Motors.** Wheels off the ground: press K1. Both wheels must turn **forward** first; otherwise flip `MOT_*_SIGN`. Press reset to stop.
6. **Gripper.** Set the gripper angles (`SERVO_*` in `config.h`). Open means the arms are parallel to the pocket walls. Closed means they hold a 50 mm cube firmly.
7. **Colour sensors.** Press K4 with white paper under and in front of the robot.

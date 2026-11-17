# GuardianBot v3 firmware

| Folder | What it is |
|---|---|
| `mega_body/` | **Arduino Mega 2560**: `mega_body.ino`, `pins.h` (pin map), `protocol.h` (link frames) |
| `esp32s3_brain/` | **ESP32-S3**: `esp32s3_brain.ino` (Sections 1–3), `config.h` (geometry, speeds, servo angles), `protocol.h` (identical copy) |
| `esp32cam_vision/` | **ESP32-CAM (AI-Thinker, OV2640)**: crops each frame to the brain's field of view and sends a 48 × 32 RGB565 grid (`A5 5A 'V' seq len16 data sum8`, 921600 baud, about 15 fps) to ESP32-S3 GPIO15. The `'L' n` command sets the flash LED |
| `hostcheck/` | `check.sh`: compiles both sketches on a PC with stub Arduino headers (`g++ -Wall -Wextra`) |
| `sil/` | `build_sil.sh`: builds the brain as a PC program. `simulation/controllers/mega_sil` plays both the Mega and the ESP32-CAM in Webots, so the exact firmware is tested against the simulated arena |
| (moved) | the earlier Mega-only draft now lives in `versions/v3.0_mega_guardian/` |

## Link protocol (`protocol.h`)

- **Link:** 250000 baud, 8N1, through the level shifter. Mega TX2/RX2 ↔ ESP32-S3 GPIO18/GPIO17.
- **Frame format:** `A5 5A type len payload sum8`.
- **Telemetry** (`T`, Mega → ESP, 20 ms):
  - time
  - encoder ticks
  - gyro yaw and yaw rate
  - 4 sonar distances (mm)
  - 5 Sharp distances (mm)
  - 5 line-sensor values
  - R, G, B, Clear counts from both TCS3200
  - battery voltage, buttons, flags
- **Command** (`C`, ESP → Mega, every step):
  - left and right wheel speed (mm/s)
  - 2 servo angles
  - LED, colour-sensor LED and buzzer bits
  - zero-odometry and gyro-calibration triggers

The Mega stops the motors if no command arrives for 250 ms.

## First power-up checklist

1. Flash `mega_body`. With only USB connected, open the Serial Monitor at 115200. You get one status line per second ("no brain | us … ir … gs … enc … yaw … vbat").
2. Turn each wheel forward by hand. `enc` must count **up** on both wheels; otherwise flip `ENC_L_SIGN` / `ENC_R_SIGN`.
3. Flash `esp32s3_brain`, wire the link and power everything from the LiPo. The ESP USB log prints "GuardianBot brain ready".
4. Wheels off the ground: press START. Both wheels must turn **forward** first; otherwise flip `MOT_*_SIGN`. Press reset to stop.
5. Set the gripper angles (`SERVO_*` in `config.h`): open means the arms are parallel to the pocket walls, closed means they hold a 50 mm cube firmly.
6. Measure `ENC_CPR` (10 hand turns) and set it in **both** `mega_body.ino` and `config.h`.

# GuardianBot v4 in Wokwi (circuit simulation, like Tinkercad)

This folder runs the **real Arduino Mega firmware** (`firmware/mega_body`) on a virtual Mega. Every part is wired
exactly as on the PCB deck (pin map = `firmware/mega_body/pins.h`), and you can drag sliders while it runs: ToF
distances, gyro turn rate, line position, colours, battery voltage and the brain's wheel commands.

Wokwi has the Mega, the OLED, the HC-SR04, servos, buttons, LEDs, a buzzer and a logic analyzer. The parts it
doesn't have are **custom chips** in `chips/`:

| Chip | What it simulates | Sliders |
|---|---|---|
| `vl53l0x` ×8 | GY-VL53L0XV2 time-of-flight sensor: enough registers for the Pololu library; XSHUT and address change work like the real module | distance (mm; over 1200 = nothing), noise |
| `l3g4200d` | GY-50 gyro (0x69): WHO_AM_I, CTRL_REG1/4, OUT_Z with auto-increment | turn rate (deg/s), zero-rate offset |
| `tcs3200` ×2 | colour sensor: 50 % square wave, S0–S3 scaling and filters, LED pin | red, green, blue |
| `qtr8rc` | Pololu QTR-8RC: the RC decay timing the firmware measures (white ~150 µs, black ~2400 µs) | line position (mm), line on/off, line width |
| `n20motor` ×2 | one DRV8833 channel + N20 6 V 300 rpm motor + hall encoder: reads the 31 kHz PWM and outputs quadrature | battery, load, counts per turn, encoder reversed |
| `brainstub` | stands in for the ESP32-S3 brain on Serial2 (250 kbaud): sends Command frames every 20 ms, OLED text, decodes and prints the telemetry | send on/off (watchdog test), wheel speeds, servos, LEDs/buzzer, zero odometry |

Keys K1–K4 also answer to the keyboard keys **1–4**. The slide potentiometer is the battery: A0 = VBAT / 3, so
mid-travel ≈ 7.5 V.

## Run it on wokwi.com (no install)

1. Go to wokwi.com, start a new **Arduino Mega** project.
2. Upload every file in `project/`: drag them onto the editor, or use the file menu's upload option. That is
   `sketch.ino` (= `mega_body.ino`), `pins.h`, `protocol.h`, `diagram.json`, `libraries.txt` and the
   `*.chip.c` / `*.chip.json` pairs.
3. Press the green play button. The chips compile in the browser.
   - The OLED shows `ToF 8/8 gyro ok`.
   - The chip console prints a telemetry line twice a second.
4. Click a part to move its slider. For example:
   - **Brain stub:** set the left and right wheel speeds to 250. Both motors print about 110 rpm, and the encoder counts in the telemetry go up.
   - **Brain stub:** set "send" to 0. The Mega's 250 ms watchdog puts the DRV8833 to sleep.
   - **Gyro:** set 90 deg/s. The yaw in the telemetry rises 90° per second.
   - **QTR:** move the line. The `line` values follow.

After changing the firmware, run `python make_wokwi.py` again to refresh `project/`.

## Run it in VS Code (Wokwi for VS Code extension)

1. Build the firmware into `build/`. Either:
   - `arduino-cli compile -b arduino:avr:mega --output-dir build project`, or
   - `./build_hex.sh` (Linux/WSL with avr-gcc).
2. `python make_wokwi.py --wasm` compiles the chips to `chips/*.chip.wasm`. This needs clang with wasi-libc. The `.wasm` files are already in the repo.
3. Open this folder in VS Code and run **Wokwi: Start Simulator**. `wokwi.toml` points at the firmware and the chips.

## Checked without Wokwi

Wokwi itself can't run here, so the project is checked two ways on a PC (Linux / WSL):

- `test/run_tests.sh`: unit tests of every chip.
  - The ToF test drives three sensors with the **real Pololu VL53L0X library**: XSHUT start-up, new addresses, continuous mode, reading at ~30 Hz.
  - The other chips are tested with the firmware's own timing and decoding: gyro, TCS3200 counts, QTR RC timing, motor speed and encoder direction, brain stub frames.
- `test/run_cosim.sh`: a **co-simulation**. The compiled Mega firmware runs in simavr, an AVR simulator. It is wired to the same chips through `diagram.json`, and the test checks:
  - boot: all 8 ToF on their own address, gyro found;
  - the 50 Hz telemetry: ToF, sonar, QTR, colour and battery values all match the sliders;
  - wheel speed control with the brain stub's commands;
  - a 90° gyro turn;
  - a key press;
  - the link watchdog.

  It passes 14 of 14.

The co-simulation found a real bug, which is now fixed in `mega_body.ino`. The right encoder is read with A and B
swapped, which already undoes the mirrored motor, so `ENC_R_SIGN` must be +1. With the old −1, the right wheel's
speed control saw the wrong sign and ran the motor at full speed.

Electrical checks of the deck (link levels and timing, battery divider, buzzer driver, power-rail dips) are in
`hardware/spice/`.

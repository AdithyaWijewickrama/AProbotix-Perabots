# Fixes after the Phase C findings (test plan Phase F)

Each fix changes both the Python controller and the ESP32-S3 firmware. After each fix, every test world is re-run.
A change is kept only if it is faster or equal on most worlds and breaks nothing. Full tables: `results/fixN_compare.md`.
The Phase C results before any fix are kept in `results/baseline_phaseC/`.

## Fix 1 – crooked start (kept)

**What changed:**
- `simulation/controllers/main_controller/main_controller.py`:
  - The heading now comes from "turned since power-on + 90°", like the real gyro. The simulator's absolute compass is no longer used.
  - New `start_heading()`.
- `firmware/esp32s3_brain/esp32s3_brain.ino`: new `startHeading()`, called first in `whereAmI()`.
- **How it works:** standing still for 15 steps (0.24 s), each side ToF pair (66 mm apart) that sees a wall within 25 cm gives the robot's angle to that wall. The heading is corrected before the robot straightens up. The formula and signs are the same as the existing `sideAlign()`.
- **Test set:** 2 crooked maze-entrance starts were added (`w30_maze_crooked` +12°, `w31_maze_crooked` −14°). `gen.py` has a new `CROOK=deg` option.

**Measured vs true start angle:**

| World | True | Measured |
|---|---|---|
| w01 | −16.1° | −16.0° (firmware), −16.3° (Python) |
| w30 | +12° | +12.2° |
| w00 | 0° | −0.2° |

**Result (44 runs):**
- **6 fixed:** the firmware now finishes all three crooked starts that failed (w01, w02, w26_hard), and w26_hard_straight's firmware run is now fully right.
- **Python:** w15 (stuck at the maze entrance) and w27_hard (wrong tile count) are now right too.
- **Times:** almost all within ±1 s. A few runs are 3–12 s slower (w00 firmware +11.8 s, w28 +7.4 s) or faster (w11, w12 about −3 s). These come from slightly different paths, not from the fix's 0.24 s pause.
- **One run broke:** w26_hard_straight (Python) now misses the tile in the dead end beside the maze exit (c8 r4). That is the same cell w27 missed before, so it's the tile-counting blind spot fix 2 addresses. Fix 1 only changed the path enough to hit it.
- **Not yet covered:** a crooked **Section 3 restart**. The restart still assumes the robot faces the exit door exactly.

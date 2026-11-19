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

## Fix 2 – tile counting (kept)

**Causes found:**
- **Missed tile:** the dead end beside the maze exit (c8 r4) was never looked at. Its west wall had exactly **one "open" vote and zero "wall" votes** in the robot's wall map, from a single stray ToF ray. That was enough to give the cell two openings, so it could not be a dead end.
- **Double count (firmware):** one tile was counted in two neighbouring cells (c4 r3 and c4 r2). The camera spilled the tile into the open cell next to it.

**What changed (Python `main_controller.py` and firmware `esp32s3_brain.ino`, same rules):**
- `sure_open()` / `sureOpen()`: an opening is "sure" only with at least 3 open votes and more than twice as many open as wall votes.
- `needed()`: a cell is ruled out as a dead end only by **two sure openings**. Otherwise the robot goes and looks.
- `tile_totals()` / `tileTotals()`: tiles are only in dead ends, so a tile in a cell with 2+ sure openings isn't counted. The log says so: "tile at c4 r2 ignored: not a dead end".

**Result (44 runs vs fix 1):**
- **2 fixed, 0 broken.** **All 44 runs are now fully right:** they finish, with the right tile count, the piece on a correct cell, and no collisions.
- **Time:** mean END change +0.2 s, median 0 s. The extra looks cost up to 10 s in a few worlds; others got a little faster.

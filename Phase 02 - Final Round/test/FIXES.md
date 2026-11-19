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

## Fix 3 – tight maze entrance → turned out to be a test-tool bug, plus two more exploration bugs (kept)

**1. The "tight entrance" was an illegal arena.**
- **What the sweep showed:** w15 was swept over 8 start angles. The robot still got stuck in 2 of 8 runs (Python −15°, firmware −10°), at the same spot below the gap.
- **The geometry:** the cubes beside the gap leave our 15 cm robot a corridor only **3.5 cm** wide. The ToF noise (1–3 cm) is too large to find it, and the VL53L0X can't read closer than 3 cm.
- **The rulebook (4.1)** allows robots up to **20 × 20 cm** and says they must be able to cross the obstacle field. A 20 cm robot can't pass w15 at all, so **w15 isn't a legal arena**. My generator was too lenient at the gap.
- **Fixed in the test tools:**
  - `gen.py` now checks the route with **exact geometry** (every wall and cube is a rectangle, 1 cm grid) for a robot of **10 cm radius**.
  - Every truth file has `legal_20cm_route`. The w15 worlds are kept as **stress tests** and marked "ILLEGAL ARENA" in the reports, and they don't count in the totals.
  - All other worlds, including the official arena, pass.
- **No robot change for this:** a "creep through the gap" manoeuvre was tried and removed. It can't work at that accuracy, and it isn't needed on legal arenas.

**2. New tight worlds found two real exploration bugs.** The five new worlds are `w32`–`w35_tight` and `w37_tight`: 43 cubes, legal under the strict rule. In w37_tight the Python controller missed 2 blue tiles.
- **Bug A, "no route" was taken as "explored".** The camera planner only used openings already seen as open. When the cells still to check lay behind unmapped edges, it found no route and stopped exploring.
  - **Fix:** if no route exists through known openings, re-plan through unknown edges before calling the maze explored.
- **Bug B, phantom wall.** The only real opening into an 8-cell block had **one "wall" vote and zero "open"**, from a single stray ray. This is the mirror image of fix 2.
  - **Fix:** `sure_wall()` / `sureWall()`. A wall is sure only with at least 3 wall votes and more than twice as many wall as open votes. Weak walls are treated as unknown when planning a looking route, and only sure walls close a cell in `needed()`.
- Both fixes are in `main_controller.py` and `esp32s3_brain.ino` (`next_plan` / `nextPlan`, `plan`, `explore_goal` / `exploreGoal`, `needed`).

**Result (63 runs vs fix 2):**
- **1 fixed** (w37_tight Python, now also 19 s faster because the right clues need no board inspection), **0 broken**.
- **All 53 runs on legal arenas are fully right.**
- **Time:** mean change +0.2 s, median 0 s. A few runs are 10–16 s slower because the robot now looks behind weak walls; a few are 5–10 s faster.

## Fix 4 – faster Section 3: camera board look on a tilt servo, no stops at crossings (kept, released as v4.2)

**Where the time went (Phase C):** Section 3 took about 81 s against a best possible ~22 s. Most of it was "below top
speed" (line legs at 0.12–0.16 m/s that braked to a stop at every crossing) and the ToF inspection: two slow passes
along the inner board lines, done in most worlds because two clues often point at the same cell.

**1. Driving on the lines** (`main_controller.py` and `esp32s3_brain.ino`):
- `go()` keeps only the turning points of a route, so the robot drives straight through crossings without stopping.
- Line legs at **0.25 m/s** (were 0.12–0.20), starting with a **1.2 m/s² ramp**: a straight jump to full speed made the
  wheels spin, and the robot lost 10 cm of position on a short leg and missed the END zone.
- **While the piece is in the gripper: 0.16 m/s and 0.6 m/s².** At 0.25 m/s the piece slipped out of the gripper in 5 of 6 test runs.
- The ToF inspection pass runs at 0.20 m/s when it is still needed (was 0.10–0.12).

**2. Camera board look with a tilt servo** (the SG90 comparison from the plan):
- **Without tilt the camera cannot do it.** In the maze view (28.6° down) it sees 13–70 cm ahead; the board is 50–150 cm
  from the exit junction. Looking from spots on the board lines found 1 complete board out of 45 (92 of 180 pieces).
- **With the camera lifted 14° by an SG90 / MG90S (29° hard stop):** from the exit junction, the place the robot passes
  anyway, one look reads **all 4 pieces on all 45 test boards**. Details: `test/results/fix4_camera_study.md`.
- **It self-calibrates.** Pitch and heading are fitted to the board's black grid lines in every frame, so servo error,
  floor tilt and odometry don't matter: still 45/45 with ±3.4° pitch error, ±2.9° heading error or ±3 cm position error.
  Without the fit, 0.9° of pitch error already broke 35 of 45 boards.
- **Needs a 96 × 64 grid.** At the maze grid's 48 × 32 the far row is 1–2 pixels: only 10 of 45 boards. The firmware's
  camera task averages the same 160 × 120 frame to 96 × 64 while the look asks for it.
- **A piece hidden behind another:** the board always has 2 cylinders and 2 cubes. If one colour is missing and
  exactly one empty cell lies in a seen piece's shadow, the missing piece is there (13 of 60 runs).
- **Safe fallback:** if the look is not complete (1 of 60 runs: two pieces hidden, w05_loops) or there is no tilt servo,
  the robot does the ToF inspection as before.
- Simulation: the camera sits on a `cam_tilt` hinge (SG90-like speed, 29° stop) with a second `cam_hi` 96 × 64 camera on the
  same head. Firmware: `tiltSet()` drives the servo from **S3 GPIO21** (LEDC 50 Hz), `boardLook()` is the C++ port.
  In the software-in-the-loop test `mega_sil` plays the servo and sends the 96 × 64 grid as 'W' frames.

**Result (64 runs vs fix 3):**
- **All 54 runs on legal arenas are fully right.** 59 runs faster, 0 slower.
- **Section 3: 81 s → 36 s on average** (Python 80.9 → 35.5 s, firmware 83.6 → 35.8 s).
- **Whole mission:** mean −46 s (Python), −60 s (firmware). The official arena: Python 153.6 → 93.1 s, firmware 187.7 → 105.8 s.
- **2 illegal stress arenas broke** (w15_c5, w15_cm10, Python): they have no route for a 20 cm robot (Fix 3) and the robot now
  gets stuck in Section 1 instead of squeezing through by luck. Not counted, as agreed in Fix 3.
- **Not done yet:** the tilt servo needs a new camera mast (hardware), a crooked start in a Section 3 restart, and a
  second look for boards where two pieces hide behind others.

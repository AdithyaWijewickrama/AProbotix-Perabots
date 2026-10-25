# Test report – AProbotix Phase 1 controller

Generated 2026-10-25 19:16 from the sim lab database (`test/tools/report.py`).
Controller tested: `controllers/main_controller/main_controller.py`, code hash `64b46444bf`.
Webots R2025a. Every run was headless at fast mode, judged by a supervisor robot added only in the test copy of the world.

## 1. Summary

| | |
|---|---|
| Test worlds | 45 |
| Passed (right LED colour, stopped in end cell, 0 collisions) | **45 / 45** |
| Collisions in total | 0 |
| Max body tilt | 1.4 deg |
| Tile counts exactly right | 45 / 45 |
| S1 start zone (full run) | mean 83.8 s, min 65.8, max 99.3 (22 worlds) |
| Hard arenas, S1 start (full run) | mean 82.3 s, min 60.7, max 96.0 (10 worlds) |
| Maze entrance start | mean 64.2 s, min 47.0, max 77.8 (13 worlds) |
| Section 1 alone | mean 21.1 s, min 17.6, max 25.7 |
| Official dev arena | 65.8 s total, Section 1 19.3 s |

## 2. How we tested

- **Worlds.** 45 in total, all in `test/worlds/` and openable in Webots:
  - the official dev arena: S1 start, plus starts in the chamber and in c1r1;
  - 21 random arenas with an S1 start and a random start heading;
  - 11 of those again with the robot placed at the maze entrance (rule 4.6, Section 2 attempts);
  - 10 "hard" arenas with 30-40 cubes, 8-9 tile slots and more maze loops.
- **How the random arenas are built** (`test/tools/harness/gen.py`). They keep everything the rulebook fixes: outer walls, gap, chamber, 9x4 maze with 25 cm cells, 10 cm cubes.
  - Cubes are placed so that a 25 cm wide passage from the start to the gap always exists.
  - The maze is random, and tiles sit only at dead ends.
  - The majority colour is never a tie.
- **Judge** (`test/tools/harness/judge.py`). A supervisor robot that exists only in the test copies and is never in the submission. It records:
  - the true position, distance travelled and maximum tilt;
  - any contact with walls or cubes (counted as a collision);
  - the Section 1 time (robot fully past y = 1.336);
  - the finish time (whole body inside the end cell and stopped for 1 s).
- **Pass rule.** A run passes only when all of these hold:
  - the controller lit the majority colour equal to the truth;
  - it finished in the end cell;
  - it had 0 collisions;
  - it tilted less than 10 deg;
  - it did not crash.
- **One run per world.** The simulation is deterministic, so the same world and the same settings always give the same result.

## 3. Final controller, every world

| world | truth B/G | seen B/G | result | S1 s | decision s | total s | dist m | coll | tilt |
|---|---|---|---|---|---|---|---|---|---|
| seed22 (hard) | 1/2 | 1/2 | PASS | 19.7 | 76.1 | **96.0** | 15.12 | 0 | 0.8 |
| seed23 (hard) | 1/2 | 1/2 | PASS | 22.2 | 68.6 | **89.1** | 14.01 | 0 | 1.2 |
| seed24 (hard) | 1/4 | 1/4 | PASS | 17.6 | 90.3 | **93.7** | 14.16 | 0 | 0.1 |
| seed25 (hard) | 1/0 | 1/0 | PASS | 24.1 | 71.3 | **89.2** | 13.69 | 0 | 0.1 |
| seed26 (hard) | 0/3 | 0/3 | PASS | 18.3 | 65.2 | **67.8** | 10.75 | 0 | 1.4 |
| seed27 (hard) | 1/0 | 1/0 | PASS | 21.6 | 71.2 | **71.1** | 10.95 | 0 | 0.4 |
| seed28 (hard) | 1/0 | 1/0 | PASS | 24.9 | 60.8 | **66.5** | 10.71 | 0 | 0.1 |
| seed29 (hard) | 3/0 | 3/0 | PASS | 19.4 | 74.4 | **94.8** | 14.27 | 0 | 0.5 |
| seed30 (hard) | 0/1 | 0/1 | PASS | 18.2 | 55.3 | **60.7** | 10.01 | 0 | 1.3 |
| seed31 (hard) | 1/2 | 1/2 | PASS | 23.3 | 86.8 | **94.5** | 14.69 | 0 | 0.1 |
| dev_s2 | 2/3 | 2/3 | PASS |  | 44.6 | **47.0** | 6.69 | 0 | 0.9 |
| dev_s2@0.22,1.455,1.5708 | 2/3 | 2/3 | PASS |  | 47.2 | **49.5** | 6.69 | 0 | 0.1 |
| seed01_s2 | 2/1 | 2/1 | PASS |  | 69.0 | **75.4** | 11.05 | 0 | 1.1 |
| seed02_s2 | 3/0 | 3/0 | PASS |  | 52.4 | **65.9** | 9.80 | 0 | 0.6 |
| seed03_s2 | 1/0 | 1/0 | PASS |  | 58.4 | **67.9** | 10.71 | 0 | 0.9 |
| seed04_s2 | 2/1 | 2/1 | PASS |  | 63.8 | **72.1** | 11.30 | 0 | 0.8 |
| seed05_s2 | 2/3 | 2/3 | PASS |  | 49.7 | **49.7** | 7.60 | 0 | 0.9 |
| seed07_s2 | 0/3 | 0/3 | PASS |  | 63.1 | **66.8** | 10.62 | 0 | 0.5 |
| seed09_s2 | 0/3 | 0/3 | PASS |  | 58.9 | **62.6** | 9.71 | 0 | 0.8 |
| seed10_s2 | 1/0 | 1/0 | PASS |  | 57.2 | **60.9** | 9.22 | 0 | 0.9 |
| seed13_s2 | 1/2 | 1/2 | PASS |  | 50.7 | **66.7** | 10.26 | 0 | 0.9 |
| seed16_s2 | 0/3 | 0/3 | PASS |  | 71.8 | **71.8** | 10.49 | 0 | 1.1 |
| seed19_s2 | 3/0 | 3/0 | PASS |  | 66.3 | **77.8** | 11.90 | 0 | 1.1 |
| dev | 2/3 | 2/3 | PASS | 19.3 | 63.4 | **65.8** | 9.62 | 0 | 0.1 |
| seed01 | 2/1 | 2/1 | PASS | 25.7 | 92.8 | **99.3** | 14.63 | 0 | 0.1 |
| seed02 | 3/0 | 3/0 | PASS | 21.7 | 72.4 | **85.9** | 13.04 | 0 | 0.1 |
| seed03 | 1/0 | 1/0 | PASS | 20.1 | 76.6 | **86.2** | 13.60 | 0 | 0.1 |
| seed04 | 2/1 | 2/1 | PASS | 19.1 | 81.0 | **89.2** | 14.11 | 0 | 1.2 |
| seed05 | 2/3 | 2/3 | PASS | 22.0 | 69.9 | **69.8** | 10.76 | 0 | 1.2 |
| seed06 | 0/1 | 0/1 | PASS | 25.6 | 72.5 | **93.6** | 14.43 | 0 | 0.9 |
| seed07 | 0/3 | 0/3 | PASS | 25.6 | 86.9 | **90.6** | 14.29 | 0 | 0.1 |
| seed08 | 1/2 | 1/2 | PASS | 20.2 | 76.2 | **82.6** | 13.19 | 0 | 0.1 |
| seed09 | 0/3 | 0/3 | PASS | 24.4 | 81.5 | **85.2** | 13.29 | 0 | 0.1 |
| seed10 | 1/0 | 1/0 | PASS | 19.6 | 75.1 | **78.7** | 12.31 | 0 | 0.1 |
| seed11 | 1/0 | 1/0 | PASS | 21.0 | 73.2 | **79.2** | 12.51 | 0 | 1.4 |
| seed12 | 0/1 | 0/1 | PASS | 21.5 | 79.1 | **83.8** | 13.27 | 0 | 0.7 |
| seed13 | 1/2 | 1/2 | PASS | 18.8 | 67.6 | **83.5** | 13.03 | 0 | 0.1 |
| seed14 | 0/3 | 0/3 | PASS | 19.6 | 74.2 | **81.3** | 12.55 | 0 | 0.1 |
| seed15 | 0/1 | 0/1 | PASS | 21.1 | 72.4 | **73.7** | 11.82 | 0 | 0.1 |
| seed16 | 0/3 | 0/3 | PASS | 19.2 | 89.1 | **89.1** | 13.21 | 0 | 0.5 |
| seed17 | 1/4 | 1/4 | PASS | 19.5 | 91.2 | **91.1** | 13.88 | 0 | 1.2 |
| seed18 | 3/0 | 3/0 | PASS | 19.9 | 73.4 | **88.2** | 13.26 | 0 | 0.1 |
| seed19 | 3/0 | 3/0 | PASS | 21.8 | 86.1 | **97.7** | 15.19 | 0 | 0.9 |
| seed20 | 1/2 | 1/2 | PASS | 17.9 | 68.0 | **71.6** | 11.29 | 0 | 0.4 |
| seed21 | 1/2 | 1/2 | PASS | 23.0 | 78.4 | **78.3** | 12.11 | 0 | 0.4 |

## 4. Settings we compared

Each candidate was run on the same worlds as the reference, and only those shared worlds are compared.
The reference is the controller before the last change ("default": R_INF 0.09, R_SOFT 0.15).

| config | worlds | passed | mean change vs reference | faster on | slower on |
|---|---|---|---|---|---|
| LAM=0 | 2 | 2/2 | +8.45 s (+13.7%) | 0 | 1 |
| LAM=0.5 | 2 | 2/2 | +1.20 s (+1.9%) | 0 | 1 |
| R_INF=0.075 | 22 | 22/22 | -1.40 s (-1.6%) | 14 | 2 |
| R_INF=0.075,R_SOFT=0.12 | 45 | 45/45 | -1.36 s (-1.7%) | 27 | 5 |
| R_SOFT=0.12 | 45 | 45/45 | -0.85 s (-1.1%) | 24 | 6 |
| R_SOFT=0.2 | 22 | 22/22 | +0.72 s (+0.8%) | 8 | 13 |
| TURN180_COST=0.7 | 13 | 13/13 | +0.08 s (+0.1%) | 0 | 1 |
| TURN180_COST=1.4 | 13 | 13/13 | +0.17 s (+0.3%) | 0 | 1 |
| TURN90_COST=0.5 | 13 | 13/13 | +0.00 s (+0.0%) | 0 | 0 |
| TURN90_COST=1 | 13 | 13/13 | +0.64 s (+1.0%) | 0 | 4 |
| VOTES_NEEDED=16 | 13 | 13/13 | +0.26 s (+0.4%) | 0 | 3 |
| VOTES_NEEDED=8 | 13 | 13/13 | +0.00 s (+0.0%) | 0 | 0 |

Adopted: **R_INF 0.075 + R_SOFT 0.12** (now the final controller). Turn costs and the camera vote threshold gave no gain, so they were kept.

Earlier tuning, done with the command-line harness before the lab existed:
- acceleration MAX_DV 1.0 → 4.0 was about 10% faster; at 10 the robot rocked up to 9 deg;
- exploration bias LAM: 1.0 beat 0.5 and 0 (0 was up to 33% slower on some mazes).

## 5. Physics findings that shaped the design

Scripts are in `test/tools/experiments/` (calib.py, calib2.py).

- **Driving wheels first is unstable.** A heading error of 0.02 rad grew to 0.3-0.6 rad within 0.5 m, even with strong steering correction.
- **Driving caster first locks onto the world axes.**
  - Within about 20 deg of an axis the robot snapped back onto it.
  - Errors of 0.03-0.06 rad decayed to 0.000 with no steering at all.
  - The robot therefore drives caster first, and mostly in straight lines along the axes and diagonals.
- **Odometry.**
  - Straight runs with equal wheel speeds matched the true distance exactly.
  - Steering while moving made the wheels slip: up to 45 cm of position error in Section 1.
  - Pivots turn about a point about 2.5 cm from the axle, and that point is what the controller tracks.
  - With all of this, the position error stayed under about 1 cm for a whole run.

## 6. Problems found during testing (all fixed)

- **Camera pointed at the ceiling.** The earlier version had the wrong tilt sign: R2025a cameras look along +X.
- **Tiles "seen" through walls.** Dark wall faces were counted as floor of the cell behind them, which marked unseen cells as empty. Fixed by only counting pixels whose line of sight crosses known-open edges.
- **Wrong start detection.** A cube near the start zone made the robot think it was at the maze entrance. Fixed by requiring both the east and south walls to be close.
- **Webots closed mid-run.** One run (seed 22, hard) ended at sim time about 55 s because Webots closed underneath it. The same world passes when rerun, and the lab now reports such runs as ABORTED.

## 7. Reproduce

- **Sim lab** (web UI): double click `test/tools/lab/start_lab.bat`, open http://localhost:8765, pick worlds, run and compare.
- **Command line:** `test/tools/harness/batch.ps1 -seeds 1,2,3` or `-mode s2`.
- **Watch any world:** open `test/worlds/<name>.wbt` in Webots R2025a and press Play. These worlds use `test/controllers/test_runner`, which runs the real `controllers/main_controller/main_controller.py`.
- **Raw data:** `test/results/final_controller.csv` and `test/results/all_runs.csv` (every run in the lab, including the setting sweeps).

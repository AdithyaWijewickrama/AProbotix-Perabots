# GuardianBot test plan

Status: **Phases A–C done** (19 Nov 2026, 32 test worlds; results in `test/PHASE_C_FINDINGS.md`).
**Next:** the fixes, in this order: crooked start, tile counting, tight maze entrance, faster Section 3 (with an SG90 camera-tilt comparison).
It runs after v4.1 (ESP32-S3 + OV7670 brain).
It builds on the Phase 01 test harness (`Phase 01 - Online Round/test`), used only as a reference and never changed.

## Phase A: Test arenas (Final Round rules)

`test/tools/gen.py` makes random but legal arenas:

- **Section 1:** different obstacle counts and layouts, always with a passage wide enough for the robot.
- **Section 2:** random 9 × 4 mazes, with and without loops, with tiles only in dead ends. Some mazes are "deceptive": dead ends near the exit, long side branches.
- **Section 3:** the clues come from the maze (tiles and turns), so the target positions change with every maze.
- **Starts:** the START area, the maze entrance (Section 2 restart) and the Section 3 restart.
- **Batches:** about 30 worlds: 12 full missions, 8 maze starts, 6 Section 3 starts and 4 hard ones. The official arena stays as world 0.
- **Truth file:** the generator writes `truth.json` for each world: obstacles, walls, tiles, clues and the correct answer.

## Phase B: Judge and "best path" reference

The judge (a supervisor robot only in the test copies) records:

- the true path and speed;
- every turn and stop;
- collisions;
- the section times;
- a power estimate from the motor model.

For each section, a reference planner that knows the true map computes the best possible run:

- **Section 1:** the shortest safe path through the real obstacles, timed with the robot's real limits (top speed, acceleration, turn rate). It reports the minimum time, distance and number of turns.
- **Section 2:** two references:
  - the **all-knowing reference**: the shortest route that lets the camera or floor sensor check every dead end, then reaches the exit;
  - the **fair reference**: the best a robot could do while discovering the maze as it goes.
- **Section 3:** the fewest moves and the shortest path to place the pieces correctly.

"Best" means **time first**, as the rules rank by time. Distance, turns and power are reported next to it.

## Phase C: Why the path differs from the best one

For each run the gap is split into causes:

- not knowing the map (detours, backtracking);
- safety margins and diagonal-only moves;
- replanning stops;
- turn-in-place time;
- slow approaches (acceleration);
- extra viewing stops.

The outputs are a picture per run (the real path over the reference path, with turns and stops marked, plus a Section 2 coverage map) and a report table.

## Phase D: Section 2, were all cells checked?

For each world, check that every dead-end cell was seen by the camera or the floor sensor, and that every skipped cell is provably not a dead end. Flag any wrong or missed tile count, fix the exploration rules if needed, and re-run all worlds.

## Phase E: Section 3, find the pieces with the camera, moving as little as possible

1. Model what the camera can see from the board entrance and a few spots around it, including pieces hiding each other. Pick the fewest viewpoints that see all 9 cells (turning in place is cheap).
2. Read each piece's colour and shape (cube or cylinder) from the camera. Only check with the distance sensors or the front colour sensor when the camera isn't sure.
3. **Optional, no extra cost:** a pan servo (SG90) for the camera, so it can sweep the board without moving the robot.
4. Compare against the current behaviour, which is driving the two inner lines.

## Phase F: Fix, re-test, report

- **Fixes:** apply the improvements to the Python controller and the ESP32-S3 firmware, then re-run the software-in-the-loop test.
- **Re-test:** re-run every world. Keep a change only if it is faster or equal on most worlds and breaks nothing.
- **Report:** `test/TEST_REPORT.md` and the test worlds, openable in Webots.

**Run time:** about 3–8 minutes per run, one Webots at a time, so a full batch takes about 2–3 hours. Results go to you after Phase C, before any code is changed.

# Fix 4 camera study: can the OV7670 read the Section 3 board?

Frames from every test world (45 boards, 2 cylinders + 2 cubes each), robot standing still, then the board detector
(`test/tools/camera_study/detect.py`, the same logic as `board_look()` / `boardLook()`). Lens 0.90 rad, camera 140 mm up.
Reproduce: `test/tools/camera_study/capture.sh <world> <tilt rad>` then `python detect.py <0.50 - tilt>`.

| Camera | Where the robot stands | Boards read completely |
|---|---|---|
| Maze view, 28.6° down (no tilt servo) | exit junction (2.21, 3.05) | 0 / 45 – the board is beyond the view |
| Maze view, no tilt | collection junction (2.01, 3.05) | 0 / 45 (23 of 180 pieces) |
| Maze view, no tilt | two stops on the board lines (1.70, 3.05) + (1.30, 2.90) | 1 / 45 (92 of 180 pieces) |
| **Tilted 14° up (SG90)** | **exit junction** | **45 / 45** |
| Tilted 23° up | exit junction | 45 / 45 |
| Tilted 14° up | collection junction | 21 / 45 – the near corners (6, 8) are outside the view |
| Tilted 14° up, 48 × 32 grid instead of 96 × 64 | exit junction | 10 / 45 – far pieces are 1–2 pixels |

**Errors the look has to survive** (tilted 14°, exit junction, 96 × 64):

| Error | Without the grid-line fit | With the fit (used) |
|---|---|---|
| Pitch ±0.9° (servo, floor) | 10 / 45 | 45 / 45 |
| Pitch ±1.7° … ±3.4° | 0 / 45 | 45 / 45 (fitted to within 0.1°) |
| Heading ±2.9° | 45 / 45 | 45 / 45 |
| Position ±3 cm | 45 / 45 | 45 / 45 |

**Hidden pieces:** 9 of the 45 boards have a piece directly behind another (same bearing, inside its shadow). The rule
"2 + 2 pieces; one colour missing; exactly one empty cell in a shadow" places it correctly on all 9.

**SG90 or MG90S?** The look needs one move to a fixed angle and back. The grid-line fit removes the servo's angle error
and backlash, so the team's own SG90 is enough. An MG90S (metal gears, same size) only adds durability.

**In the full runs (64 runs, Fix 4):** the look ran 60 times and was complete 59 times; 13 times a hidden piece was
placed by the rule. Once (w05_loops, two pieces hidden) it fell back to the ToF inspection, and that run was still right.

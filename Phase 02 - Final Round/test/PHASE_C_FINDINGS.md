# Test plan – Phase A–C results (checkpoint, before any robot code is changed)

**Runs:**
- 32 test worlds, made by `tools/gen.py`:
  - the official arena;
  - 17 full missions, 4 of them "hard";
  - 8 maze starts;
  - 6 Section 3 restarts.
- **Python controller:** run on all 32 worlds.
- **ESP32-S3 brain firmware** (software-in-the-loop): run on 8 of them.
- Every run is timed by a neutral judge and compared with the best possible run on the true map (`tools/reference.py`).
- Full table: `results/phaseC.md`. Pictures: `results/plots/` (real path, best path, stops; red dashed boxes = dead ends the simple camera model counts as not looked at).

## The four things that matter

### 1. The firmware gets lost if the robot is placed slightly crooked (critical, real robot)

**What happened:**
- The firmware failed in Section 1 on all three crooked starts in the batch (16°, 6.7° and 7.9° off straight). Its position drifted about 30–40 cm, and it ended up repeating "no way out on the map" with the robot standing still.
- With the same arenas and a straight start, it finishes: `w02_straight_sil` 166.1 s, `w26_hard_straight_sil` 192.6 s.

**Cause:**
- The brain starts with heading = 90° (`yawOff = HP_F`) and only has the gyro, which measures turning, not direction.
- `whereAmI()` rounds that believed heading to the nearest 90°. So a crooked start is never noticed, and every later position is off by that angle.

**Why the Python runs didn't show it:** the Python controller reads the simulator's absolute compass (`imu.getRollPitchYaw()`), which the real robot doesn't have. **The Python results are therefore optimistic about start placement.**

**Fix idea (Phase F):** at START, measure the heading from the wall on the robot's right (east wall) with the side ToF pair, which is 66 mm apart. The maze wall-alignment already does this. The Python controller should also stop using the absolute compass, so the simulation matches the real robot.

### 2. Tile counting errors in Section 2 (wrong clues)

**What happened:**
- **Python, `w27_hard`:** it missed a green tile in the dead end right next to the maze exit (c8 r4). It drove up the right-hand column and declared the maze explored without looking in, so it counted 1 green instead of 2.
- **Firmware, `w26_hard_straight`:** it counted one green tile twice (c5 r3 and c6 r3, 20 ms apart; the tile is on the boundary between two cells). It got 6 green instead of 5, so cube clue 1 pointed at position 6, and the piece went on a wrong cell (1 instead of 3).

**Why it matters:** a wrong count changes the clues, and can change the clan.

**Fix ideas (Phase D):**
- Never call the maze explored while a possible dead end hasn't been looked into.
- Accept a tile only when the camera sees it at least twice, and from the cell centre.
- Never count the same tile twice.

### 3. Section 1 planner gives up at a legal but tight entrance

**What happened (`w15`):** two cubes sit just left of the maze entrance gap. The planner keeps 96 mm clearance on a 2 cm grid, so the passage it could use is narrower than one grid cell. It decides there's no way out and loops "forget and look again" until the judge stops it. This happens with both controllers.

**Proof the passage is real:** the reference planner gets through it with an 85 mm margin.

**Fix idea:** when no path is found, retry with the hard 86 mm limit and a 1 cm grid near the gap, then centre in the gap using the side ToF sensors.

### 4. Section 3 is where the time goes

Averages over finished Python runs:

| Section | Real time | Gap to best | What the gap is made of |
|---|---|---|---|
| S1 obstacles | 19.9 s | 8.5 s | Mostly turning on the spot (6 s); some short detours into pockets of cubes |
| S2 maze | 45.3 s | 8.2 s | Turning 3.7 s, stops 2.5 s, longer path 2.6 s; already close to the best |
| S3 board | 82.1 s | **61.6 s** | Driving below top speed 39 s, longer path 16 s |

**Why S3 is slow:**
- **Line following at 0.16 m/s and 5 cm/s approaches.** This is the "driving below top speed" part.
- **The inspection drive.** In 27 of 32 runs the clues didn't place every piece, because a clue was 0, above 9 or a repeat. The robot then drove both inner lines of the board. S3 took 88 s with the inspection and 42 s without it.

**Phase E target:** look at the board with the camera from the entrance (plus a turn on the spot) instead of driving the lines. Then drive faster on the straight line legs.

## Smaller notes

- **No wall or cube was ever hit:** the closest the robot came was 5 mm (firmware) and 12 mm (Python).
- **Judge contact flags in S3:** they come from the carried piece touching the gripper. No board piece moved in any run.
- **Simple-explorer baseline** (`s2_fair` in `results/ref/`): the robot's maze strategy beats a plain "nearest unknown first" explorer.
- **All-knowing maze reference:** the robot matched it on the official arena (37.5 s vs 37.5 s).

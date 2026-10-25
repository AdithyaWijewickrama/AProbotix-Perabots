# AProbotix – Phase 1 controller explanation

Team AProbotix, Faculty of Engineering, University of Ruhuna
Files: `controllers/main_controller/main_controller.py`, `protos/TwoWheelRobot.proto` (extension slot)

## 1. What the controller does, in one paragraph

From the start zone the robot localises itself against the arena walls. It
then crosses Section 1 by building a map of the cubes as it sees them and
planning a shortest safe path (A*) to the transition gap, replanning whenever
a new cube appears. It drives through the chamber into the maze and explores
it cell by cell. While exploring, it maps the walls with its distance sensors
and reads the floor colour of the cells ahead with a camera, so most dead ends
are checked without driving into them. Once every possible tile location has
been seen it lights the LED of the majority colour, drives the shortest known
route to the end cell (c9 r4) and stops there. Nothing about the obstacle
layout, maze walls or tile positions is hard-coded. Only the fixed arena
dimensions from the rulebook are used.

## 2. Robot configuration

All devices are in the `extensionSlot`. The chassis, motors, wheel size,
physics and the `led_blue` / `led_green` LEDs are unchanged. The two drive
wheels are moved 2 cm forward (x 0 → 0.02) for balance, as allowed in
rulebook 4.4.

| Device | Name(s) | Used for |
|---|---|---|
| 10 × DistanceSensor (range 1.5 m) | `ds_front`, `ds_fl1/fr1` (±23°), `ds_fl2/fr2` (±46°), `ds_fl3/fr3` (±69°), `ds_left`, `ds_right`, `ds_back` | cube mapping (S1), maze wall mapping (S2), position correction off walls |
| Camera 48×32, tilted 23° down | `cam` | floor colour of the next 1–2 maze cells (blue / green / plain floor) |
| InertialUnit | `imu` | absolute heading |
| Gyro | `gyro` | detecting when a pivot has finished |
| Wheel PositionSensors (built in) | `left/right_wheel_sensor` | distance travelled (odometry) |

No GPS, Emitter, Receiver or Supervisor functions are used.

### Why the robot drives "caster first"

The base robot has a sliding ball caster. We measured its behaviour in
Webots (scripts in `tools/experiments/`):

- **Wheels leading:** a heading error of 0.02 rad grew to 0.3–0.6 rad within
  0.5 m, even with strong steering correction.
- **Caster leading:** errors of up to ~20° from a world axis corrected themselves
  back onto the axis with no steering at all. Headings 25–65° off-axis stayed stable.
- The robot cannot steer while moving without the wheels slipping.
  A pivot in place rotates about a point ≈2.5 cm from the axle.

So all sensors face the caster side and the robot drives in that direction
(negative wheel speed). Motion is always **pivot in place → drive straight →
pivot**. This kept the position estimate within about 1 cm over a whole run.

## 3. Localisation (where the robot thinks it is)

- **Heading** comes straight from the InertialUnit.
- **Position** comes from the wheel encoders, integrated along the IMU heading, for the
  measured pivot point.
- **Corrections** happen whenever a front, back or side sensor hits a wall whose position is
  known (arena boundary in S1, maze grid lines in S2) while the robot points
  along an axis. The estimate is then pulled toward the measured value.

**Start-up check.** The robot turns to the nearest axis and reads its four axis
sensors.
- Walls close on both the east and south sides means it is in the **Section 1
  start zone**. x and y are taken from those two walls.
- Otherwise, a wall close on the west side means it was placed at the **maze
  entrance**, which the evaluators may do for the Section 2 attempts (rule 4.6).
  x comes from the west wall. y comes from where the red chamber tile appears
  in the camera image.

## 4. Section 1 – obstacle navigation

1. **Map.** A 2 cm occupancy grid of Section 1. The outer walls and the gap
   are filled in from the arena dimensions. Every distance-sensor hit inside
   the area marks a cube cell. Each cell stores its distance to the nearest
   obstacle.
2. **Plan.** 8-direction A* from the robot to the gap.
   - Cells closer than 6 cm to an obstacle are blocked.
   - 6–9 cm costs heavily, and up to 15 cm costs a little extra.
   - This keeps the robot centred in the 25 cm passages.
   - Unknown space counts as free, so the robot heads straight for the goal and
     detours only when it actually sees a cube.
3. **Drive.** The robot takes the furthest point on the A* path that lies on
   an exact 0°/45°/90°/… line with clear space along it. It pivots to that
   heading and drives straight there with a smooth braking profile.
4. **Replan.** If a newly seen cube blocks the current line, the robot stops,
   replans and continues.
5. **Safety.** If something is within 3 cm of the front sensors it stops.
   After three failed moves in a row it backs off and replans.
6. When the robot reaches the gap it drives straight north through the
   chamber to the centre of the first maze cell (c1 r1).

## 5. Section 2 – maze exploration and decision

**Wall map.** Each wall segment between two cells has a vote count of
"open" against "wall". Every step, every distance-sensor ray is traced across
the maze grid lines:
- lines it passes through are voted **open**
- the line where it stops is voted **wall**

Only the middle part of each segment is used, to avoid corner ambiguity. With
a 1.5 m range, the robot learns many walls several cells away.

**Tile detection.**
- Every second camera pixel is projected onto the floor, using the known
  camera height and tilt and the robot's position.
- Each pixel is classified as blue, green, floor (black or red) or "other"
  (walls), and votes for the cell it lands in.
- A pixel only counts if the straight line from the camera to that cell
  crosses only edges known to be **open**. Without this, the dark side of a
  wall was mistaken for the floor of the cell behind it.
- After about 12 votes the cell is labelled blue, green or empty.

**What still needs looking at.** Tiles are only placed at dead ends, so a
cell is only "needed" if:
- its colour hasn't been seen yet, and
- it could still be a dead end (fewer than two known openings), and
- it isn't completely walled in.

**Where to go next.** A shortest-path search over (cell, heading) states.
Driving one cell, a 90° pivot and a 180° pivot each have a cost. Good places
to stop are cell/heading pairs from which the camera can see a needed cell,
one or two cells ahead through open edges. Among these, the robot picks the
one with the lowest **travel cost − λ × (distance to the end cell)**, with
λ = 1. This makes it explore far parts of the maze first and pick up
the cells near the end on its way out, which cut up to 33% off some mazes.

**Moving.** The robot replans every simulation step. While the plan keeps
going straight it keeps driving without stopping at each cell. It only brakes
when the plan needs a turn or a stop to look.

**Finish.**
1. When no needed cell is left, the robot counts the blue and green tiles,
   lights `led_blue` or `led_green` (a tie lights blue), and prints the counts.
2. It drives the shortest known route to c9 r4 and stops at the cell centre,
   fully inside the clear area.
3. The motors are held at zero and `done t=…s` is printed with the simulation
   time.

The LED is already on when the robot arrives, which meets the 3 s rule.

## 6. Robustness features

- **No-progress watchdog in the maze:** if the robot hasn't moved for 6 s it
  reverses a little and replans, well inside the 15 s termination rule.
- If a look at a cell somehow gives no usable pixels, the robot marks it and
  moves on instead of waiting forever.
- The camera is ignored while pivoting fast, since the image is smeared.
- The robot always comes to a full stop before switching between pivoting and
  driving, which avoids wheel slip.

## 7. Tested performance

Tested headless in Webots R2025a with a supervisor "judge" that logs the true
position, collisions and times (`tools/harness/`). We used the dev arena plus
21 generated arenas, 34 test worlds in total (`test_worlds/`). The arenas have
random cubes keeping 25 cm passages, random mazes, random tiles at dead ends
and a random start heading. Some worlds start the robot at the maze entrance.

| Result | Value |
|---|---|
| Majority colour correct | every test run |
| End cell reached and stopped | every test run |
| Collisions | 0 |
| Full run (S1 + S2) | ~73–92 s (dev arena 73.4 s) |
| Section 1 alone | ~18–34 s |
| Maze only (start at entrance) | ~55–78 s |

## 8. Main parameters

| Name | Value | Meaning |
|---|---|---|
| `V_MAX` | 0.2 m/s | top speed (= motor limit 10 rad/s) |
| `MAX_DV` | 4.0 rad/s per step | wheel acceleration limit |
| `ACC` | 3.8 m/s² | braking curve |
| `R_HARD`, `R_INF`, `R_SOFT` | 0.06 / 0.09 / 0.15 m | obstacle clearance zones for A* |
| `LAM` | 1.0 | how strongly to explore far-from-the-end cells first |
| `PIV_X` | 0.005 m | measured pivot point (in the caster-forward frame) |

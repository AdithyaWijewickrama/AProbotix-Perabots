# PeraBots 2026 — Phase 1 solution notes

Autonomous controller for the University Category elimination arena (Webots
R2025a). Everything lives in the two files the submission cares about:

- `protos/TwoWheelRobot.proto` — sensors added through the **extension slot**
  only; the locked drivetrain, chassis and the required `led_blue` / `led_green`
  are untouched.
- `controllers/main_controller/main_controller.py` — the full controller.

> **Status: written, not yet run in Webots.** It compiles cleanly and the
> geometry/logic were derived from the supplied `Arena.proto` / `Obstacles.proto`,
> but it has **not** been executed in the simulator from here. Treat the first
> Webots run as a tuning pass (see *Tuning* below).

## Sensors added (extension slot)

| Device | Name | Purpose |
|---|---|---|
| DistanceSensor ×5 | `ds_front`, `ds_front_left`, `ds_front_right`, `ds_left`, `ds_right` | obstacle avoidance (S1) + maze wall detection (S2) |
| Camera | `cam` | blue/green floor-tile detection, black-floor = maze entry |
| InertialUnit | `imu` | absolute world heading (allowed; not a GPS) |
| Gyro | `gyro` | spare, enabled for future use |

Sensors use `type "generic"` (pure geometric distance). The `infra-red` type
would misread the near-black walls and cubes as far away. The `lookupTable`
maps metres → value as `value = distance_m × 1000` over a 0.40 m range, so the
code reads distance as `getValue()/1000`.

Webots conventions used: a **DistanceSensor** shoots its ray along local **+X**;
a **Camera** views along local **−Z**. The camera is pitched ~30° off vertical
so it sees floor a few cm ahead, clear of the chassis.

## Arena geometry (derived from Arena.proto — true for the eval arena too)

- World is Z-up. Section 1 (white) is `x∈[0,2.44], y∈[0,1.22]`; Section 2 maze
  (black) is `y∈[1.22,2.44]`.
- Start marker centre `(2.315, 0.125)` (bottom-right). Transition opening is the
  gap in the top wall at the **west** end; chamber tile at `(0.22, 1.275)`.
- Maze is 9 cols × 4 rows, 25 cm pitch. Cell centre
  `(x,y) = (0.22 + (col−1)·0.25, 1.455 + (row−1)·0.25)`, col 1…9, row 1…4.
- Robot enters the maze at **c1,r1 facing North (+Y)**; end cell is **c9,r4**.
- Tiles are flush floor patches: blue `0 0 0.5`, green `0 0.6 0`; bare floor
  `0.02`. In the dev package: 2 blue, 3 green (→ green majority). **These change
  in evaluation**, so the robot counts them live — nothing is hard-coded.

## How it works

**Section 1** — potential-field navigation: a coarse pose (IMU heading + wheel
odometry, seeded at the nominal start) gives a pull toward the opening; the 5
distance sensors give obstacle repulsion; a stuck-timer triggers a back-off-and-
swing recovery. Arrival is detected when the camera sees the black maze floor.

**Section 2** — micromouse style: depth-first exploration visits every reachable
cell, reading walls from the left/front/right sensors and tile colour from the
camera; 90° turns and one-cell (0.25 m) steps are held straight by the IMU and
centred between side walls. After exploring, a flood-fill gives the shortest
path to c9,r4, where the robot lights the majority-colour LED and stops.

## Testing in Webots

1. Open `worlds/Arena.wbt` in Webots **R2025a**.
2. The robot already references controller `main_controller`; just press Play.
3. Watch the console: Section 1 prints arrival, Section 2 prints
   `blue=… green=… -> majority=…` and the explored-cell count.
4. The robot should finish in the end cell with exactly one LED lit.

## Tuning (first-run checklist)

Constants are grouped at the top of `main_controller.py`:

- **If turns go the wrong way / the maze comes out mirrored** → set
  `YAW_SIGN = -1.0`. This is the single most likely first-run fix (it depends on
  the world's yaw sign convention, which couldn't be confirmed without a run).
- `WALL_M` (0.16) — wall/no-wall threshold. If open passages read as walls,
  raise it; if walls are missed, lower it. Verify by printing the five
  `dist(...)` values in a cell.
- `Kh`, `Kc` — straightness and wall-centring gains. Oscillation → lower; drift
  → raise.
- `Katt`, `Krep`, `DETECT_M` — Section 1 goal pull vs obstacle push. Section 1
  is the least certain part; test it on the dev obstacle layout first.
- `COLOR_MIN`, `BLACK_MAX` and the channel margins in `camera_color()` —
  blue/green classification. Print the averaged `r,g,b` over a known tile to set
  these.
- Speeds `CRUISE`, `TURN_SP`, `MAX_SPEED`.

## Known risks / next steps

- Section 1 relies on coarse odometry for the goal pull; over a long winding
  detour the position estimate drifts (heading stays exact via IMU). The black-
  floor arrival check and the chamber funnel mitigate this, but Section 1 wants
  the most test time.
- Exploration currently visits **all** reachable cells (robust, guarantees every
  tile is found) at the cost of distance travelled, which is scored. A later
  optimisation: stop once all dead-ends are checked, or once it is mathematically
  impossible for the trailing colour to catch up.

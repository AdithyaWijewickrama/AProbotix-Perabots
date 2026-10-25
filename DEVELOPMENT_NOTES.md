# PeraBots 2026 - phase 1 notes

our stuff is in 2 files (rest of the package is untouched):

- `protos/TwoWheelRobot.proto` - only the extensionSlot changed. chassis, wheels,
  motors, caster, physics and the led_blue / led_green are exactly as given
- `controllers/main_controller/main_controller.py` - the whole controller

webots R2025a. open `worlds/Arena.wbt`, press play, watch the console.

## sensors (extension slot)

| what | name | why |
|---|---|---|
| DistanceSensor x10 | `ds_front`, `ds_fl1/fr1`, `ds_fl2/fr2`, `ds_fl3/fr3`, `ds_left`, `ds_right`, `ds_back` | cube map in S1, maze walls in S2, position fixes off walls |
| Camera 48x32 | `cam` | floor colour (blue/green tiles) 1-2 cells ahead |
| InertialUnit | `imu` | heading (its not a GPS so its allowed) |
| Gyro | `gyro` | to know when a pivot has actually stopped |

- all distance sensors are type generic, lookupTable `0 0 0, 1.5 1500 0` so
  value/1000 = metres, range 1.5m
- **everything faces -X (the caster side)**, see below why
- careful: in R2025a the camera looks along +X like the distance sensors, NOT
  -Z. the old version had the cam tilted up at the ceiling because of this

## the big physics thing (read this first)

the caster is just a sliding ball and webots friction is lined up with the
world X/Y axes. what we measured:

- wheels first: any small heading error grows, robot veers off ~13deg from the
  axis. steering cant fix it, tried gains up to 20
- caster first: within ~20deg of an axis it **snaps onto** the axis by itself.
  25-65deg off axis it just stays where it is
- turning on one wheel = that wheel spins, robot doesnt move
- steering while moving = wheel slip = odometry goes to hell
- a pivot in place turns around a point ~2.5cm from the axle (towards the
  caster), not the axle

so the robot drives **caster first** ("backwards"). the controller flips its
frame so "front" = caster side. motion is only ever pivot -> drive straight ->
pivot, and the pose point is that pivot point. with this the position estimate
stays within ~1cm for the whole run (was off by 45cm before lol)

## how it works

**start** - pivot to the nearest N/E/S/W and look at the walls. east AND south
both close = we're in the S1 start corner, x/y from those walls. otherwise
we're at the maze entrance (rulebook 4.6, they might put us there for S2
attempts), then x from the west wall and y from where the red chamber tile is.

**section 1** - 2cm occupancy grid, the known arena walls are pre filled (only
the cubes change in eval). every sensor hit marks a cube cell. A* (8 dirs) to
the gap in the top wall, then drive straight legs at exactly 0/45/90/... deg
(other angles crab sideways). new cube on the line we're driving -> stop and
replan. then straight north through the chamber into c1r1.

**section 2** -
- walls: every sensor ray is walked across the maze grid, lines it passes = open,
  line it hits = wall. hitting a known wall also corrects x/y
- tiles: each camera pixel gets projected onto the floor and votes for a cell.
  only counts if the line from the cam to that cell goes through known OPEN
  edges (otherwise the dark side of walls counted as floor behind them)
- tiles are only in dead ends, so a cell with 2+ known openings doesnt need to
  be looked at. the cam sees 1-2 cells ahead so we usually check a dead end
  from outside without driving in
- next target = dijkstra over (cell, heading). we prefer far-from-the-end
  cells first (`LAM`) so the ones near the end get done on the way out
- nothing left -> majority LED on (tie = blue), drive to c9r4 and stop

## testing

we ran it headless in webots with a judge supervisor (logs true position,
collisions, time) on the dev arena + ~25 random arenas (random cubes keeping
a 25cm passage, random maze, random tiles in dead ends, random start heading,
also starts in the chamber / c1r1). every run: right majority, end cell
reached, 0 collisions. full run ~85-105s, dev arena ~83s. the harness isnt in
the submission folder.

## knobs (top of main_controller.py)

- `MAX_DV`, `ACC`, `V_MAX` - accel / braking / top speed (motor caps at
  10 rad/s = 0.2 m/s anyway)
- `PIV_X` - the measured pivot point, dont touch unless the proto changes
- `LAM` - exploration order bias, tried 0 / 0.5 / 1.0, 1.0 was best
- `classify()` thresholds - blue/green/black, tuned by eye on the dev tiles
- `R_HARD`, `R_INF`, `R_SOFT` - how close to cubes A* is allowed to go

## todo / known stuff

- arc turns instead of stop-pivot-go would be faster but curves kill the
  odometry (see above). maybe for phase 2
- the "no progress for 6s" recovery is just a dumb reverse, never triggered in
  testing but its there because of the 15s rule
- tie in tile count -> blue. rulebook doesnt say what to do

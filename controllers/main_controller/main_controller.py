"""
main_controller.py  -  PeraBots 2026 University Category, Phase 1 solution.

Autonomous controller for the TwoWheelRobot base. It solves both sections of
the elimination arena WITHOUT any hard-coded knowledge of a particular layout:

  Section 1  (white floor, obstacle field)
      Potential-field navigation from the start zone to the Section-2
      transition opening, using an absolute heading reference (InertialUnit)
      plus wheel odometry for a coarse pose, and the 5 distance sensors for
      reactive obstacle avoidance. Entry into Section 2 is detected from the
      camera (the floor turns black).

  Section 2  (black floor, 9 x 4 maze of 25 cm cells)
      Classic "micromouse" style cell-by-cell exploration:
        * depth-first search that visits every reachable cell,
        * walls read from the left / front / right distance sensors,
        * blue / green floor tiles read from the downward camera,
        * 90 deg turns and one-cell steps held straight by the IMU and
          centred between side walls,
      then a flood-fill shortest path to the end cell, where the robot lights
      the LED of the majority tile colour and stops.

Only the extension-slot devices added in TwoWheelRobot.proto are used; the
locked drivetrain, chassis and the required led_blue / led_green are untouched.

------------------------------------------------------------------------------
TUNING NOTES (read before the first Webots run)
------------------------------------------------------------------------------
This controller was written against the geometry in the supplied package but
could not be run here, so the constants below are the first things to check in
simulation:
  * WALL_M            - distance threshold that counts as "a wall is there".
  * Kh / Kc / Katt    - steering gains (straightness, wall-centring, goal pull).
  * CRUISE / TURN_SP  - speeds.
  * COLOR_*           - camera colour thresholds for blue vs green.
  * If every turn goes the wrong way or the maze comes out mirrored, the world
    yaw convention is flipped: set YAW_SIGN = -1.0 (see dir_to_yaw).
All geometry constants (cell size, grid, start/goal) match the provided
Arena.proto and are correct for the evaluation arena too (only the obstacle
layout, maze walls and tile positions change, which this code discovers live).
"""

import math
from controller import Robot, Camera

# ----------------------------------------------------------------------------
# Constants
# ----------------------------------------------------------------------------
TIME_STEP   = 16            # must divide basicTimeStep (16) in the world
WHEEL_R     = 0.02          # wheel radius  [m]  (locked in the PROTO)
MAX_SPEED   = 10.0          # wheel angular speed cap [rad/s]  (~0.20 m/s)
CRUISE      = 8.0           # forward cruise speed    [rad/s]  (~0.16 m/s)
TURN_SP     = 5.0           # in-place turn speed     [rad/s]
MAX_DV      = 0.6           # max wheel-speed change per step [rad/s] (anti-flip
                            # slew: ramps speed instead of snapping it, removing
                            # the torque spikes that pitch the robot over)

CELL        = 0.25          # maze cell pitch [m]
COLS        = 9             # maze columns (x / East)
ROWS        = 4             # maze rows    (y / North)
END_CELL    = (COLS - 1, ROWS - 1)   # c9,r4 in 0-based (col,row)

WALL_M      = 0.16          # distance sensor reading below this => wall present
DETECT_M    = 0.22          # Section-1 obstacle influence distance
FRONT_STOP  = 0.045         # emergency front stop distance in the maze

# Steering gains
Kh   = 2.2                  # heading-hold gain (maze straight driving / turns)
Kc   = 6.0                  # wall-centring gain (maze)
Katt = 1.6                  # goal-attraction gain (Section 1)
Krep = 2.0                  # obstacle-repulsion gain (Section 1)

# Camera colour thresholds (0..255). A tile is called only when one of the
# blue / green channels clearly dominates and is bright enough.
COLOR_MIN   = 70
BLACK_MAX   = 35            # all channels below this => bare black maze floor

# Absolute world headings (standard Webots Z-up, CCW-positive yaw, 0 at +X).
YAW_SIGN = 1.0              # set to -1.0 if the sim's yaw turns out reversed
HALF_PI  = math.pi / 2.0

# Section-1 goal: the centre of the chamber just inside the transition opening
# (top-left of Section 1, from Arena.proto chamber_transition_tile at 0.22,1.275)
GOAL_XY       = (0.22, 1.22)
START_XY      = (2.315, 0.125)   # nominal centre of the red start marker

# ----------------------------------------------------------------------------
# Robot / device setup
# ----------------------------------------------------------------------------
robot = Robot()

left_motor  = robot.getDevice('left_wheel_motor')
right_motor = robot.getDevice('right_wheel_motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_enc  = robot.getDevice('left_wheel_sensor')
right_enc = robot.getDevice('right_wheel_sensor')
left_enc.enable(TIME_STEP)
right_enc.enable(TIME_STEP)

led_blue  = robot.getDevice('led_blue')
led_green = robot.getDevice('led_green')
led_blue.set(0)
led_green.set(0)

ds = {}
for n in ('ds_front', 'ds_front_left', 'ds_front_right', 'ds_left', 'ds_right'):
    ds[n] = robot.getDevice(n)
    ds[n].enable(TIME_STEP)

cam = robot.getDevice('cam')
cam.enable(TIME_STEP)
CAM_W = cam.getWidth()
CAM_H = cam.getHeight()

imu = robot.getDevice('imu')
imu.enable(TIME_STEP)

gyro = robot.getDevice('gyro')          # available for future use
gyro.enable(TIME_STEP)

# ----------------------------------------------------------------------------
# Low-level helpers
# ----------------------------------------------------------------------------
def wrap(a):
    """Wrap angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def step():
    """Advance one simulation step; return False when Webots asks us to quit."""
    return robot.step(TIME_STEP) != -1

def dist(name):
    """Distance sensor reading in metres (value/1000 per the lookupTable)."""
    return ds[name].getValue() / 1000.0

def yaw():
    """Absolute world yaw [rad] from the InertialUnit."""
    return imu.getRollPitchYaw()[2]

def enc_dist():
    """Average wheel travel in metres since power-on."""
    return (left_enc.getValue() + right_enc.getValue()) * 0.5 * WHEEL_R

_last_vl = 0.0
_last_vr = 0.0

def set_wheels(vl, vr):
    """Command wheel speeds with a per-step slew-rate limit. Ramping the speed
    instead of stepping it removes the acceleration/braking torque spikes that
    were tipping the robot onto its nose, and lets it cruise faster safely."""
    global _last_vl, _last_vr
    vl = max(-MAX_SPEED, min(MAX_SPEED, vl))
    vr = max(-MAX_SPEED, min(MAX_SPEED, vr))
    vl = max(_last_vl - MAX_DV, min(_last_vl + MAX_DV, vl))
    vr = max(_last_vr - MAX_DV, min(_last_vr + MAX_DV, vr))
    _last_vl, _last_vr = vl, vr
    left_motor.setVelocity(vl)
    right_motor.setVelocity(vr)

def stop():
    set_wheels(0.0, 0.0)

def settle(n=6):
    """Hold still for a few steps so momentum dies before reading sensors."""
    for _ in range(n):
        if not step():
            return
        set_wheels(0.0, 0.0)

def camera_color():
    """Average the centre of the camera image and classify the floor colour.
    Returns 'blue', 'green' or None."""
    img = cam.getImage()
    if img is None:
        return None
    r = g = b = 0
    x0, x1 = CAM_W // 4, 3 * CAM_W // 4
    y0, y1 = CAM_H // 4, 3 * CAM_H // 4
    n = 0
    for y in range(y0, y1):
        for x in range(x0, x1):
            r += Camera.imageGetRed(img, CAM_W, x, y)
            g += Camera.imageGetGreen(img, CAM_W, x, y)
            b += Camera.imageGetBlue(img, CAM_W, x, y)
            n += 1
    if n == 0:
        return None
    r /= n; g /= n; b /= n
    if b > COLOR_MIN and b > g + 20 and b > r + 20:
        return 'blue'
    if g > COLOR_MIN and g > b + 20 and g > r + 20:
        return 'green'
    return None

def floor_is_black():
    """True when the camera sees the bare (uncoloured) black maze floor."""
    img = cam.getImage()
    if img is None:
        return False
    cx, cy = CAM_W // 2, CAM_H // 2
    r = Camera.imageGetRed(img, CAM_W, cx, cy)
    g = Camera.imageGetGreen(img, CAM_W, cx, cy)
    b = Camera.imageGetBlue(img, CAM_W, cx, cy)
    return r < BLACK_MAX and g < BLACK_MAX and b < BLACK_MAX

# ----------------------------------------------------------------------------
# Direction bookkeeping for the maze
#   dir 0 = North (+Y), 1 = East (+X), 2 = South (-Y), 3 = West (-X)
#   turning right increments dir (N -> E -> S -> W), which is clockwise.
# ----------------------------------------------------------------------------
DELTA = {0: (0, 1), 1: (1, 0), 2: (0, -1), 3: (-1, 0)}   # (dcol, drow)

def dir_to_yaw(d):
    """World yaw target for a maze heading. North = +Y = +pi/2, etc."""
    base = {0: HALF_PI, 1: 0.0, 2: -HALF_PI, 3: math.pi}[d]
    return wrap(base * YAW_SIGN)

def in_grid(c, r):
    return 0 <= c < COLS and 0 <= r < ROWS

# ----------------------------------------------------------------------------
# Maze motion primitives
# ----------------------------------------------------------------------------
def turn_to(target_yaw):
    """Rotate in place to an absolute world yaw using IMU feedback."""
    while step():
        err = wrap(target_yaw - yaw())
        if abs(err) < 0.015:
            break
        w = Kh * err
        # keep enough torque to actually move, cap at TURN_SP
        if w > 0:
            w = max(0.5, min(TURN_SP, w))
        else:
            w = min(-0.5, max(-TURN_SP, w))
        set_wheels(-w, w)          # +w on right, -w on left => CCW (yaw up)
    stop()

def turn_to_dir(d):
    turn_to(dir_to_yaw(d))

def step_forward():
    """Drive straight one cell (CELL metres), held straight by the IMU and
    centred between side walls when present. Returns normally even if a front
    wall forces an early stop (we are then at the cell centre anyway)."""
    start = enc_dist()
    target_yaw = yaw()             # hold whatever heading we currently face
    # snap the hold target to the nearest cardinal so small entry error decays
    cardinals = [HALF_PI, 0.0, -HALF_PI, math.pi, -math.pi]
    target_yaw = min(cardinals, key=lambda c: abs(wrap(c - target_yaw)))
    while step():
        travelled = enc_dist() - start
        if travelled >= CELL - 0.004:
            break
        if dist('ds_front') < FRONT_STOP:
            break
        corr = Kh * wrap(target_yaw - yaw())
        l, rr = dist('ds_left'), dist('ds_right')
        if l < WALL_M and rr < WALL_M:          # both walls -> centre
            corr += Kc * (l - rr)
        elif l < WALL_M:                        # only left wall -> hold offset
            corr += Kc * (l - 0.09)
        elif rr < WALL_M:                       # only right wall
            corr -= Kc * (rr - 0.09)
        # ease off over the last 6 cm so the cell stop is clean (and the slew
        # limiter does not overshoot the 0.25 m target)
        v = CRUISE * max(0.3, min(1.0, (CELL - travelled) / 0.06))
        set_wheels(v - corr, v + corr)
    stop()

# ----------------------------------------------------------------------------
# Maze state
# ----------------------------------------------------------------------------
walls   = {}     # (c,r) -> set of blocked absolute directions {0,1,2,3}
visited = set()
colors  = {}     # (c,r) -> 'blue' / 'green'

def add_wall(c, r, d):
    walls.setdefault((c, r), set()).add(d)
    nc, nr = c + DELTA[d][0], r + DELTA[d][1]
    if in_grid(nc, nr):
        walls.setdefault((nc, nr), set()).add((d + 2) % 4)   # neighbour's side

def sense(c, r, d):
    """Record walls (relative to heading d) and tile colour of cell (c,r)."""
    settle()
    visited.add((c, r))
    if dist('ds_front') < WALL_M:
        add_wall(c, r, d)
    if dist('ds_left') < WALL_M:
        add_wall(c, r, (d + 3) % 4)      # left of heading
    if dist('ds_right') < WALL_M:
        add_wall(c, r, (d + 1) % 4)      # right of heading
    col = camera_color()
    if col is not None:
        colors[(c, r)] = col

def blocked(c, r, d):
    return d in walls.get((c, r), ())

def drive_to_adjacent(cur, nxt, cur_dir):
    """Turn toward and step into an edge-adjacent cell. Returns new heading."""
    dc, dr = nxt[0] - cur[0], nxt[1] - cur[1]
    for d, delta in DELTA.items():
        if delta == (dc, dr):
            turn_to_dir(d)
            step_forward()
            return d
    return cur_dir   # should not happen

# ----------------------------------------------------------------------------
# Section 2 - explore, then go to the end cell and signal
# ----------------------------------------------------------------------------
def explore(entry, entry_dir):
    """Depth-first traversal visiting every reachable cell."""
    cur, cur_dir = entry, entry_dir
    sense(cur[0], cur[1], cur_dir)
    stack = [cur]
    while stack:
        cur = stack[-1]
        nxt = None
        for d in (cur_dir, (cur_dir + 3) % 4, (cur_dir + 1) % 4, (cur_dir + 2) % 4):
            nc, nr = cur[0] + DELTA[d][0], cur[1] + DELTA[d][1]
            if in_grid(nc, nr) and not blocked(cur[0], cur[1], d) and (nc, nr) not in visited:
                nxt = (nc, nr)
                break
        if nxt is not None:
            cur_dir = drive_to_adjacent(cur, nxt, cur_dir)
            sense(nxt[0], nxt[1], cur_dir)
            stack.append(nxt)
        else:
            stack.pop()
            if stack:
                cur_dir = drive_to_adjacent(cur, stack[-1], cur_dir)
    return cur_dir

def flood_distances(goal):
    """BFS distance from goal over discovered open edges between visited cells."""
    dists = {goal: 0}
    frontier = [goal]
    while frontier:
        nxt = []
        for c, r in frontier:
            for d, (dc, dr) in DELTA.items():
                nc, nr = c + dc, r + dr
                if (not in_grid(nc, nr)) or (nc, nr) in dists:
                    continue
                if (nc, nr) not in visited or blocked(c, r, d):
                    continue
                dists[(nc, nr)] = dists[(c, r)] + 1
                nxt.append((nc, nr))
        frontier = nxt
    return dists

def go_to(cur, cur_dir, goal):
    """Follow the flood-fill gradient from cur to goal."""
    dists = flood_distances(goal)
    if goal not in dists or cur not in dists:
        return cur, cur_dir          # unreachable with what we mapped
    while cur != goal:
        best = None
        for d, (dc, dr) in DELTA.items():
            nc, nr = cur[0] + dc, cur[1] + dr
            if blocked(cur[0], cur[1], d) or (nc, nr) not in dists:
                continue
            if dists[(nc, nr)] == dists[cur] - 1:
                best = (nc, nr)
                break
        if best is None:
            break
        cur_dir = drive_to_adjacent(cur, best, cur_dir)
        cur = best
    return cur, cur_dir

def signal_majority():
    n_blue  = sum(1 for v in colors.values() if v == 'blue')
    n_green = sum(1 for v in colors.values() if v == 'green')
    # Tie (incl. 0-0) defaults to blue; a well-formed task will not tie.
    majority = 'blue' if n_blue >= n_green else 'green'
    led_blue.set(1 if majority == 'blue' else 0)
    led_green.set(1 if majority == 'green' else 0)
    print('[Section 2] blue=%d green=%d -> majority=%s' % (n_blue, n_green, majority))

def run_section2(entry=(0, 0), entry_dir=0):
    print('[Section 2] entering maze at %s, exploring...' % (entry,))
    cur_dir = explore(entry, entry_dir)
    # after DFS the robot is back at the entry cell
    cur = entry
    print('[Section 2] explored %d cells; going to end %s' % (len(visited), END_CELL))
    cur, cur_dir = go_to(cur, cur_dir, END_CELL)
    stop()
    signal_majority()          # within the 3 s window - set immediately

# ----------------------------------------------------------------------------
# Section 1 - potential-field navigation to the transition opening
# ----------------------------------------------------------------------------
# sensor mount angles relative to robot forward (for repulsion vectors)
S1_SENSORS = {
    'ds_front':        0.0,
    'ds_front_left':   math.pi / 4,
    'ds_front_right': -math.pi / 4,
    'ds_left':         math.pi / 2,
    'ds_right':       -math.pi / 2,
}

def run_section1():
    """Drive from the start zone to the Section-2 transition, avoiding cubes.
    Uses a coarse odometry pose (seeded at the nominal start) for the goal pull
    and the IMU for heading; the camera detects arrival (floor turns black)."""
    x, y = START_XY
    prev = enc_dist()
    stuck_timer = 0.0
    last_progress = (x, y)

    while step():
        # ---- coarse pose update (IMU heading + wheel odometry) ----
        d = enc_dist()
        ds_travel = d - prev
        prev = d
        h = yaw()
        x += ds_travel * math.cos(h)
        y += ds_travel * math.sin(h)

        # ---- arrival test: we have crossed onto the black maze floor ----
        if y > 1.05 and floor_is_black():
            stop()
            print('[Section 1] reached transition at approx (%.2f, %.2f)' % (x, y))
            return

        # ---- attraction toward the goal ----
        bearing = math.atan2(GOAL_XY[1] - y, GOAL_XY[0] - x)
        turn = Katt * wrap(bearing - h)

        # ---- repulsion from obstacles ----
        front_clear = 1.0
        for name, ang in S1_SENSORS.items():
            m = dist(name)
            if m < DETECT_M:
                strength = (DETECT_M - m) / DETECT_M
                # push heading away from the sensor's side
                turn -= Krep * strength * math.sin(ang) * 2.0
                if abs(ang) < 0.9:                      # forward-ish beam
                    front_clear = min(front_clear, m / DETECT_M)

        # ---- stuck recovery ----
        if math.hypot(x - last_progress[0], y - last_progress[1]) > 0.05:
            last_progress = (x, y)
            stuck_timer = 0.0
        else:
            stuck_timer += TIME_STEP / 1000.0
        if stuck_timer > 2.5:
            # back off and swing to find a new opening
            t0 = robot.getTime()
            while step() and robot.getTime() - t0 < 0.4:
                set_wheels(-CRUISE * 0.6, -CRUISE * 0.6)
            while step() and robot.getTime() - t0 < 1.1:
                set_wheels(-TURN_SP, TURN_SP)
            stuck_timer = 0.0
            prev = enc_dist()
            continue

        # ---- drive ----
        v = CRUISE * max(0.15, front_clear)
        set_wheels(v - turn, v + turn)

# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def main():
    # let sensors fill with valid data before using them
    for _ in range(5):
        if not step():
            return
    run_section1()
    # straighten to North and run the maze
    turn_to_dir(0)
    run_section2(entry=(0, 0), entry_dir=0)
    # hold position with the decision LED lit
    while step():
        stop()

main()

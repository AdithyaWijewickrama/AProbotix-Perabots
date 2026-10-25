# PeraBots 2026 - uni category, phase 1 controller
#
# sec 1: small occupancy grid from the distance sensors while we drive,
#        A* to the gap in the top wall, replan when new cubes show up
# sec 2: map the maze walls with the same sensors. cam looks at the floor
#        of the next 1-2 cells so tiles get spotted without driving into every
#        dead end. go to whatever unseen cell is cheapest, nothing left -> end
#        cell, majority LED, stop.
#
# things we learned the hard way (v1-v6 all crashed into stuff):
#  - the caster is a sliding ball and webots friction is lined up with the
#    world X/Y axes. wheels first = robot veers off axis. CASTER FIRST = it
#    locks onto the axis by itself. so we drive "backwards", the whole robot
#    frame in here is flipped (front = caster side)
#  - it cant really steer while moving, so: pivot -> drive straight -> pivot
#  - pivot happens around a point ~2.5cm from the axle (towards the caster),
#    thats our pose point. heading = IMU, position = encoders + wall fixes

import math
import heapq
from controller import Robot

TS = 16
R_WHEEL = 0.02
HALF_TRACK = 0.032
PIV_X = 0.005          # pivot point, x in OUR frame (caster side = +x, see below)

MAX_DV = 4.0           # rad/s per step wheel slew. was 1.0, 4 is ~10% faster and still flat (10 rocks the robot)
V_MAX = 0.2            # m/s (motor caps at 10 rad/s anyway)
ACC = 3.8              # braking curve, a bit under what the slew allows
W_RUN = 0.15           # max turn rate while driving (more = wheels slip, odometry dies)

# maze
CELL = 0.25
MX0, MY0 = 0.095, 1.33     # bottom left corner of the maze (wall centrelines)
COLS, ROWS = 9, 4
END = (8, 3)               # c9 r4

PI = math.pi
HP = PI / 2
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]   # N E S W
HANG = [HP, 0.0, -HP, PI]

robot = Robot()

lm = robot.getDevice('left_wheel_motor')
rm = robot.getDevice('right_wheel_motor')
for m in (lm, rm):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
W_MAX = min(lm.getMaxVelocity(), 10.0)

le = robot.getDevice('left_wheel_sensor')
re_ = robot.getDevice('right_wheel_sensor')
le.enable(TS)
re_.enable(TS)

led_b = robot.getDevice('led_blue')
led_g = robot.getDevice('led_green')
led_b.set(0)
led_g.set(0)

# name: (x, y, angle) in OUR frame = the proto frame turned 180deg, because we
# drive caster first. so proto (x,y) = (-x,-y) here
SENS = {
    'ds_front': (0.041, 0.0, 0.0),
    'ds_fl1': (0.041, 0.012, 0.4),
    'ds_fr1': (0.041, -0.012, -0.4),
    'ds_fl2': (0.039, 0.022, 0.8),
    'ds_fr2': (0.039, -0.022, -0.8),
    'ds_fl3': (0.03, 0.026, 1.2),
    'ds_fr3': (0.03, -0.026, -1.2),
    'ds_left': (0.0, 0.026, HP),
    'ds_right': (0.0, -0.026, -HP),
    'ds_back': (-0.041, 0.0, PI),
}
DS_MAX = 1.49
ds = {}
for n in SENS:
    ds[n] = robot.getDevice(n)
    ds[n].enable(TS)
D = {n: 9.0 for n in SENS}
AXIS_SENS = ('ds_front', 'ds_left', 'ds_right', 'ds_back')

cam = robot.getDevice('cam')
cam.enable(TS)
CW, CH = cam.getWidth(), cam.getHeight()

imu = robot.getDevice('imu')
imu.enable(TS)
gyro = robot.getDevice('gyro')
gyro.enable(TS)


def wrap(a):
    return (a + PI) % (2 * PI) - PI


# ---------------------------------------------------------------- pose / io
px, py = 2.315, 0.125
yaw = 0.0
_pl = _pr = None
_last_l = _last_r = 0.0
t_sim = 0.0


def step():
    global px, py, yaw, _pl, _pr, t_sim
    if robot.step(TS) == -1:
        raise SystemExit
    t_sim += TS / 1000.0
    l, r = le.getValue(), re_.getValue()
    y2 = wrap(imu.getRollPitchYaw()[2] + PI)     # we face the caster way
    if _pl is not None:
        d = -((l - _pl) + (r - _pr)) * 0.5 * R_WHEEL
        mid = yaw + wrap(y2 - yaw) * 0.5
        px += d * math.cos(mid)
        py += d * math.sin(mid)
    _pl, _pr, yaw = l, r, y2
    for n in SENS:
        D[n] = ds[n].getValue() / 1000.0


def set_vw(v, w):
    # v m/s, w rad/s -> wheels. keeps the ratio if we're over the motor limit
    # turning on one wheel does nothing (wheel just spins) and steering while
    # moving makes the wheels slip -> odometry goes to hell. so w only when
    # pivoting really, W_RUN is basicaly 0 now anyway
    global _last_l, _last_r
    if v != 0:
        w = max(-W_RUN, min(W_RUN, w)) if abs(v) > 0.12 else 0.0
    a = (-v - w * HALF_TRACK) / R_WHEEL     # minus v = caster first
    b = (-v + w * HALF_TRACK) / R_WHEEL
    big = max(abs(a), abs(b))
    if big > W_MAX:
        a *= W_MAX / big
        b *= W_MAX / big
    a = max(_last_l - MAX_DV, min(_last_l + MAX_DV, a))
    b = max(_last_r - MAX_DV, min(_last_r + MAX_DV, b))
    _last_l, _last_r = a, b
    lm.setVelocity(a)
    rm.setVelocity(b)


def speed():
    return -(_last_l + _last_r) * 0.5 * R_WHEEL


def ray(n):
    # where sensor n is in the world + which way it points
    sx, sy, a = SENS[n]
    sx -= PIV_X
    c, s = math.cos(yaw), math.sin(yaw)
    return px + c * sx - s * sy, py + s * sx + c * sy, yaw + a


def near_cardinal(tol):
    k = round(yaw / HP)
    return abs(wrap(yaw - k * HP)) < tol


sense_hook = None


def turn_to(target, tol=0.02):
    full_stop()      # never go straight from driving into a pivot, it skids
    while True:
        step()
        if sense_hook:
            sense_hook()
        e = wrap(target - yaw)
        if abs(e) < tol and abs(gyro.getValues()[2]) < 0.5:
            break
        w = min(6.2, math.sqrt(2 * 80.0 * abs(e)))
        w = max(w, 0.4)
        set_vw(0.0, w if e > 0 else -w)
    full_stop()


def full_stop():
    while abs(_last_l) > 0.05 or abs(_last_r) > 0.05:
        set_vw(0, 0)
        step()
        if sense_hook:
            sense_hook()


def stop_now(n=4):
    for _ in range(n):
        set_vw(0, 0)
        step()


# ---------------------------------------------------------------- camera
# pre compute where each pixel lands on the floor (our frame, relative to the
# pivot point). cam doesnt move so only once. only every 2nd pixel, its enough
CAM_X, CAM_Z, CAM_P = 0.042 - PIV_X, 0.09, 0.4
_f = (CW / 2) / math.tan(0.8 / 2)
PIX = []   # (byte index, fwd, left)
for v in range(1, CH, 2):
    for u in range(1, CW, 2):
        x, y, z = _f, -(u + 0.5 - CW / 2), -(v + 0.5 - CH / 2)
        x2 = x * math.cos(CAM_P) + z * math.sin(CAM_P)
        z2 = -x * math.sin(CAM_P) + z * math.cos(CAM_P)
        if z2 >= -0.05 * math.sqrt(x * x + y * y + z * z):
            continue
        t = -CAM_Z / z2
        gx, gy = CAM_X + t * x2, t * y
        if gx < 0.7:
            PIX.append((4 * (v * CW + u), gx, gy))

BLUE, GREEN, FLOOR, JUNK = 1, 2, 0, -1


def classify(r, g, b):
    m = max(r, g, b)
    if m < 50:
        return FLOOR                     # black maze floor / shadow (thresholds tuned by eye)
    if b > 45 and b > 1.5 * g + 15 and b > 1.5 * r + 15:
        return BLUE
    if g > 45 and g > 1.5 * b + 15 and g > 1.5 * r + 15:
        return GREEN
    if r > 100 and r > 2 * g and r > 2 * b:
        return FLOOR                     # red markers, still floor
    return JUNK                          # walls etc


# ---------------------------------------------------------------- maze map
# edge votes [open, wall]. hz[c][k] = horizontal line y = MY0+k*CELL (south
# edge of row k), vt[k][r] = vertical line x = MX0+k*CELL (west edge of col k)
hz = [[[0, 0] for _ in range(ROWS + 1)] for _ in range(COLS)]
vt = [[[0, 0] for _ in range(ROWS)] for _ in range(COLS + 1)]
for c in range(COLS):
    hz[c][0][1] = hz[c][ROWS][1] = 999       # outer walls (entrance treated as wall too)
for r in range(ROWS):
    vt[0][r][1] = vt[COLS][r][1] = 999

votes = {}     # (c,r) -> [floor, blue, green]
seen = {}      # (c,r) -> FLOOR / BLUE / GREEN
UNK, OPEN, WALL = 0, 1, 2
map_dirty = True    # not used anymore, was for replanning less often


def _st(e):
    if e[0] == 0 and e[1] == 0:
        return UNK
    return WALL if e[1] > e[0] else OPEN


def edge_ref(c, r, h):
    if h == 0:
        return hz[c][r + 1]
    if h == 2:
        return hz[c][r]
    if h == 1:
        return vt[c + 1][r]
    return vt[c][r]


def edge(c, r, h):
    return _st(edge_ref(c, r, h))


def in_grid(c, r):
    return 0 <= c < COLS and 0 <= r < ROWS


def cell_of(x, y):
    return int(math.floor((x - MX0) / CELL)), int(math.floor((y - MY0) / CELL))


def centre(c, r):
    return MX0 + (c + 0.5) * CELL, MY0 + (r + 0.5) * CELL


def _vote(e, w):
    global map_dirty
    before = _st(e)
    e[1 if w else 0] += 1
    if _st(e) != before:
        map_dirty = True


def maze_ray(n, fix):
    # walk the ray over the maze grid lines. any line it goes through (middle
    # part of an edge only, corners are messy) = open, the one it hits = wall
    global px, py
    ox, oy, a = ray(n)
    d = D[n]
    hit = d < DS_MAX
    L = d if hit else DS_MAX
    ux, uy = math.cos(a), math.sin(a)
    if abs(ux) > 1e-3:
        for k in range(COLS + 1):
            t = (MX0 + k * CELL - ox) / ux
            if t <= 0 or t > L + 0.03:
                continue
            yy = oy + uy * t
            r = math.floor((yy - MY0) / CELL)
            if not (0 <= r < ROWS) or abs(yy - MY0 - (r + 0.5) * CELL) > 0.085:
                continue
            if t < L - 0.02:
                _vote(vt[k][r], False)
            elif hit:
                _vote(vt[k][r], True)
                if fix and abs(uy) < 0.06:
                    face = MX0 + k * CELL - math.copysign(0.006, ux)
                    err = face - (ox + ux * d)
                    if abs(err) < 0.03:
                        px += 0.25 * err
    if abs(uy) > 1e-3:
        for k in range(ROWS + 1):
            t = (MY0 + k * CELL - oy) / uy
            if t <= 0 or t > L + 0.03:
                continue
            xx = ox + ux * t
            c = math.floor((xx - MX0) / CELL)
            if not (0 <= c < COLS) or abs(xx - MX0 - (c + 0.5) * CELL) > 0.085:
                continue
            if t < L - 0.02:
                _vote(hz[c][k], False)
            elif hit:
                _vote(hz[c][k], True)
                if fix and abs(ux) < 0.06:
                    face = MY0 + k * CELL - math.copysign(0.006, uy)
                    err = face - (oy + uy * d)
                    if abs(err) < 0.03:
                        py += 0.25 * err


def maze_sense():
    if py < 1.2:
        return
    fix = near_cardinal(0.05)
    for n in SENS:
        maze_ray(n, fix and n in AXIS_SENS)
    look()


def can_see(cx, cy, key):
    # clear line (only OPEN edges) from cam to middle of the cell?
    # without this the dark side of a wall got counted as floor of the cell
    # BEHIND it and we 'saw' empty cells we never saw. took ages to find >:(
    tx, ty = centre(*key)
    n = int(math.hypot(tx - cx, ty - cy) / 0.04) + 1
    prev = None
    for k in range(n + 1):
        f = k / n
        c, r = cell_of(cx + (tx - cx) * f, cy + (ty - cy) * f)
        if not in_grid(c, r):
            if prev is not None:
                return False
            continue
        if prev is None:
            if c != 0 or r != 0:
                if k > 0:
                    return False    # came in from outside somewhere weird
            prev = (c, r)
            continue
        if (c, r) != prev:
            dc, dr = c - prev[0], r - prev[1]
            if abs(dc) + abs(dr) != 1:
                return False        # cut a corner, dont trust it
            h = DIRS.index((dc, dr))
            if edge(prev[0], prev[1], h) != OPEN:
                return False
            prev = (c, r)
    return True


def look():
    global map_dirty
    if abs(gyro.getValues()[2]) > 1.0:
        return              # spinning, dont trust the image
    img = cam.getImage()
    if not img:
        return
    c_, s_ = math.cos(yaw), math.sin(yaw)
    camx, camy = px + c_ * CAM_X, py + s_ * CAM_X
    ok = {}
    hits = {}
    for i, gx, gy in PIX:
        k = classify(img[i + 2], img[i + 1], img[i])
        if k == JUNK:
            continue
        wx = px + c_ * gx - s_ * gy
        wy = py + s_ * gx + c_ * gy
        fx, fy = (wx - MX0) / CELL, (wy - MY0) / CELL
        c, r = int(math.floor(fx)), int(math.floor(fy))
        if not in_grid(c, r) or abs(fx - c - 0.5) > 0.36 or abs(fy - r - 0.5) > 0.36:
            continue
        if (c, r) not in ok:
            ok[(c, r)] = can_see(camx, camy, (c, r))
        if ok[(c, r)]:
            hits.setdefault((c, r), [0, 0, 0])[k] += 1
    for key, h in hits.items():
        v = votes.setdefault(key, [0, 0, 0])
        for j in range(3):
            v[j] += h[j]
        if key not in seen and sum(v) >= 12:
            decide_colour(key)
            map_dirty = True


def decide_colour(key):
    f, b, g = votes.get(key, (0, 0, 0))
    tot = f + b + g
    if b >= 5 and b > g and b >= 0.25 * tot:
        seen[key] = BLUE
    elif g >= 5 and g > b and g >= 0.25 * tot:
        seen[key] = GREEN
    else:
        seen[key] = FLOOR
    if seen[key] != FLOOR:
        print('tile %s at c%d r%d' % ('BLUE' if seen[key] == BLUE else 'GREEN', key[0] + 1, key[1] + 1))


def n_open(c, r):
    return sum(1 for h in range(4) if edge(c, r, h) == OPEN)


def needed(c, r):
    if (c, r) in seen or n_open(c, r) >= 2:
        return False        # seen it, or 2+ openings so it cant be a dead end
    return any(edge(c, r, h) != WALL for h in range(4))


def plan(c0, r0, h0, goal, allow_unk=False, bonus=None):
    # dijkstra over (cell, heading), returns the list of states or None
    # bonus = dont just grab the closest goal, take lowest cost - bonus[cell].
    # used so stuff close to the end cell gets left for last
    start = (c0, r0, h0)
    best = {start: 0.0}
    prev = {}
    pq = [(0.0, start)]
    pick, pick_sc = None, 1e9
    while pq:
        d, s = heapq.heappop(pq)
        if d > best[s]:
            continue
        c, r, h = s
        if goal(c, r, h):
            sc = d - (bonus.get((c, r), 0) if bonus else 0)
            if sc < pick_sc:
                pick, pick_sc = s, sc
            if not bonus:
                break
        if pick is not None and bonus and d > pick_sc + bonus_max:
            break
        nxt = [((c, r, (h + 1) % 4), 0.7), ((c, r, (h + 3) % 4), 0.7), ((c, r, (h + 2) % 4), 1.0)]
        e = edge(c, r, h)
        if e == OPEN or (allow_unk and e == UNK):
            nc, nr = c + DIRS[h][0], r + DIRS[h][1]
            if in_grid(nc, nr):
                nxt.append(((nc, nr, h), 1.0 if e == OPEN else 3.0))
        for s2, k in nxt:
            d2 = d + k
            if d2 < best.get(s2, 1e9):
                best[s2] = d2
                prev[s2] = s
                heapq.heappush(pq, (d2, s2))
    if pick is None:
        return None
    s = pick
    out = [s]
    while s in prev:
        s = prev[s]
        out.append(s)
    return out[::-1]


LAM = 1.0        # how much we prefer far-from-the-end stuff first (tried 0, .5, 1 -> 1 best)
bonus_max = 0.0


def end_bonus():
    # bfs steps from the end cell (unknown edges = open) times LAM
    global bonus_max
    dist = {END: 0}
    q = [END]
    for c, r in q:
        for h in range(4):
            nc, nr = c + DIRS[h][0], r + DIRS[h][1]
            if in_grid(nc, nr) and (nc, nr) not in dist and edge(c, r, h) != WALL:
                dist[(nc, nr)] = dist[(c, r)] + 1
                q.append((nc, nr))
    b = {k: LAM * v for k, v in dist.items()}
    bonus_max = max(b.values())
    return b


def explore_goal(c, r, h):
    nc, nr = c + DIRS[h][0], r + DIRS[h][1]
    if not in_grid(nc, nr) or edge(c, r, h) == WALL:
        return False
    if needed(nc, nr):
        return True
    # cam reaches 2 cells if the way is open
    fc, fr = nc + DIRS[h][0], nr + DIRS[h][1]
    return edge(c, r, h) == OPEN and in_grid(fc, fr) and edge(nc, nr, h) == OPEN and needed(fc, fr)


def end_goal(c, r, h):
    return (c, r) == END


exploring = True


def next_plan(c, r, h):
    global exploring
    if exploring:
        p = plan(c, r, h, explore_goal, bonus=end_bonus() if LAM else None)
        if p is not None:
            return p
        exploring = False
        show_majority()
    p = plan(c, r, h, end_goal)
    if p is None:
        p = plan(c, r, h, end_goal, allow_unk=True)
    return p


def show_majority():
    nb = sum(1 for v in seen.values() if v == BLUE)
    ng = sum(1 for v in seen.values() if v == GREEN)
    blue = nb >= ng     # tie -> blue. rules dont say, shouldnt happen anyway
    led_b.set(1 if blue else 0)
    led_g.set(0 if blue else 1)
    print('explored. blue=%d green=%d -> %s  (t=%.1fs)' % (nb, ng, 'BLUE' if blue else 'GREEN', t_sim))


def fwd_count(p):
    k = 0
    for i in range(1, len(p)):
        if p[i][2] != p[i - 1][2]:
            break
        k += 1
    return k


def aim(h):
    # exact N/E/S/W. caster-first the robot locks onto the axis by itself so
    # no point aiming at the cell centre, it would just snap back anyway
    if abs(wrap(HANG[h] - yaw)) > 0.02:
        turn_to(HANG[h], 0.02)


def enter_maze():
    # straight up through the gap + chamber to the middle of c1r1
    global sense_hook
    sense_hook = maze_sense
    turn_to(HP, 0.008)
    while True:
        step()
        maze_sense()
        rem = 1.455 - py
        if rem < 0.004:
            break
        set_vw(min(V_MAX, math.sqrt(2 * ACC * rem) + 0.01), 0.0)
    full_stop()


def run_maze():
    # all of section 2. we should be sitting in c1r1 here
    global sense_hook
    sense_hook = maze_sense
    h = 0
    mode = 'stopped'
    wait = 0
    last_move, last_pos = t_sim, (px, py)
    while True:
        step()
        maze_sense()
        ax, ay = DIRS[h]
        if math.hypot(px - last_pos[0], py - last_pos[1]) > 0.03:
            last_move, last_pos = t_sim, (px, py)
        if mode == 'run':
            # cell whose centre is next infront of us (or the one we're on)
            cc, cr = cell_of(px, py)
            cc, cr = max(0, min(COLS - 1, cc)), max(0, min(ROWS - 1, cr))
            cx, cy = centre(cc, cr)
            along = (px - cx) * ax + (py - cy) * ay
            if py < MY0:
                dc = (0, 0)
            elif along > 0.02 and edge(cc, cr, h) == OPEN:
                dc = (cc + ax, cr + ay)
            else:
                dc = (cc, cr)
            p = next_plan(dc[0], dc[1], h)
            k = fwd_count(p) if p else 0
            tx, ty = centre(dc[0] + ax * k, dc[1] + ay * k)
            rem = (tx - px) * ax + (ty - py) * ay
            if rem > 0.004:
                set_vw(min(V_MAX, math.sqrt(2 * ACC * rem) + 0.01), 0.0)
            if rem <= 0.004 or min(D['ds_front'], D['ds_fl1'], D['ds_fr1']) < 0.035:
                full_stop()
                mode = 'stopped'
                wait = 0
        else:
            set_vw(0, 0)
            c, r = cell_of(px, py)
            if not in_grid(c, r):
                c, r = 0, 0
            p = next_plan(c, r, h)
            if p is None:
                print('no path?? stuck at c%d r%d' % (c + 1, r + 1))
                return
            if len(p) == 1:
                if (c, r) == END and not exploring:
                    return
                # we're where we wanted to look from, give the cam a few frames
                wait += 1
                if wait > 4:
                    # whatever is visible infront (1 or 2 cells), decide now
                    cc, cr = c, r
                    for _ in range(2):
                        if edge(cc, cr, h) != OPEN:
                            break
                        cc, cr = cc + DIRS[h][0], cr + DIRS[h][1]
                        if in_grid(cc, cr) and (cc, cr) not in seen:
                            decide_colour((cc, cr))
                if wait > 8:
                    # cant see it?? dont get stuck, call it empty
                    cc, cr = c + DIRS[h][0], r + DIRS[h][1]
                    for _ in range(2):
                        if in_grid(cc, cr):
                            seen.setdefault((cc, cr), FLOOR)
                        cc, cr = cc + DIRS[h][0], cr + DIRS[h][1]
                continue
            h = p[1][2]
            aim(h)
            mode = 'run'
        if t_sim - last_move > 6.0:
            # shouldnt happen but the 15s no progress rule would kill the run
            # TODO make this smarter, its just a dumb reverse
            print('wiggle, no progress for 6s')
            for _ in range(20):
                set_vw(-0.08, 0)
                step()
            full_stop()
            last_move = t_sim
            mode = 'stopped'


# ---------------------------------------------------------------- section 1
G = 0.02
NX, NY = 122, 68
R_HARD, R_INF, R_SOFT = 0.06, 0.075, 0.12
occ = bytearray(NX * NY)
clear = [9.0] * (NX * NY)
_OFF = []
for dj in range(-8, 9):
    for di in range(-8, 9):
        dd = math.hypot(di, dj) * G
        if dd <= R_SOFT + 1e-9:
            _OFF.append((di, dj, dd))
new_obs = False


def mark(i, j):
    global new_obs
    if not (0 <= i < NX and 0 <= j < NY) or occ[j * NX + i]:
        return
    occ[j * NX + i] = 1
    new_obs = True
    for di, dj, dd in _OFF:
        a, b = i + di, j + dj
        if 0 <= a < NX and 0 <= b < NY:
            k = b * NX + a
            if dd < clear[k]:
                clear[k] = dd


def mark_rect(x0, y0, x1, y1):
    for j in range(max(0, int(y0 / G)), min(NY, int(y1 / G) + 1)):
        for i in range(max(0, int(x0 / G)), min(NX, int(x1 / G) + 1)):
            mark(i, j)


# known arena walls (fixed by the rules, only cubes/maze inside change)
mark_rect(0, 0, 2.44, 0.006)
mark_rect(0, 0, 0.006, 1.22)
mark_rect(2.434, 0, 2.44, 1.22)
mark_rect(0, 1.214, 0.101, 1.36)
mark_rect(0.339, 1.214, 2.44, 1.36)


def s1_sense():
    # boundary walls give free position fixes, cubes just go in the grid
    global px, py
    fix = near_cardinal(0.05)
    for n in SENS:
        d = D[n]
        if d >= DS_MAX:
            continue
        ox, oy, a = ray(n)
        hx, hy = ox + d * math.cos(a), oy + d * math.sin(a)
        if fix and n in AXIS_SENS:
            for face, isx in ((0.006, 1), (2.434, 1), (0.006, 0), (1.214, 0)):
                if isx and abs(math.sin(a)) < 0.06 and abs(hx - face) < 0.03:
                    px += 0.25 * (face - hx)
                elif not isx and abs(math.cos(a)) < 0.06 and abs(hy - face) < 0.03 \
                        and (face < 1 or hx < 0.09 or hx > 0.35):
                    py += 0.25 * (face - hy)
        if 0.012 < hx < 2.428 and 0.012 < hy < 1.208:
            ex, ey = hx + 0.01 * math.cos(a), hy + 0.01 * math.sin(a)
            mark(int(hx / G), int(hy / G))
            mark(int(ex / G), int(ey / G))


def astar(sx, sy, gx, gy):
    si, sj = int(sx / G), int(sy / G)
    gi, gj = int(gx / G), int(gy / G)
    s, g = sj * NX + si, gj * NX + gi
    best = {s: 0.0}
    prev = {}
    pq = [(0.0, 0.0, s)]
    SQ = 1.41421356
    while pq:
        _, d, k = heapq.heappop(pq)
        if k == g:
            out = []
            while k in prev:
                out.append(((k % NX + 0.5) * G, (k // NX + 0.5) * G))
                k = prev[k]
            return out[::-1]
        if d > best[k]:
            continue
        i, j = k % NX, k // NX
        for di, dj, ln in ((1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
                           (1, 1, SQ), (1, -1, SQ), (-1, 1, SQ), (-1, -1, SQ)):
            a, b = i + di, j + dj
            if not (0 <= a < NX and 0 <= b < NY):
                continue
            k2 = b * NX + a
            cl = clear[k2]
            if cl < R_HARD:
                continue
            if cl < R_INF:
                m = 25.0
            elif cl < R_SOFT:
                m = 1.0 + 2.5 * (R_SOFT - cl) / (R_SOFT - R_INF)
            else:
                m = 1.0
            d2 = d + ln * m
            if d2 < best.get(k2, 1e9):
                best[k2] = d2
                prev[k2] = k
                hh = max(abs(a - gi), abs(b - gj)) + 0.41421356 * min(abs(a - gi), abs(b - gj))
                heapq.heappush(pq, (d2 + hh, d2, k2))
    return None


def line_ok(x0, y0, x1, y1):
    # does the straight line keep away from stuff. first 8cm can be tighter or
    # we never get away from a spot that's already close to a cube
    L = math.hypot(x1 - x0, y1 - y0)
    n = int(L / 0.01) + 1
    for k in range(n + 1):
        f = k / n
        x, y = x0 + (x1 - x0) * f, y0 + (y1 - y0) * f
        i, j = int(x / G), int(y / G)
        if not (0 <= i < NX and 0 <= j < NY):
            return False
        lim = R_HARD if f * L < 0.08 else R_INF
        if clear[j * NX + i] < lim:
            return False
    return True


def pick_seg(path):
    # furthest point on the A* path thats on an exact 0/45/90/.. line from us.
    # only those headings drive straight, at like 10deg off the robot crabs
    # sideways (axis aligned friction again)
    Q = PI / 4
    for k in range(len(path) - 1, 0, -1):
        dx, dy = path[k][0] - px, path[k][1] - py
        L = math.hypot(dx, dy)
        if L < 0.03:
            break
        a = math.atan2(dy, dx)
        q = round(a / Q) * Q
        if abs(L * math.sin(a - q)) > 0.016:
            continue
        d = L * math.cos(a - q)
        if line_ok(px, py, px + d * math.cos(q), py + d * math.sin(q)):
            return wrap(q), d
    # nothing lines up (we're off the grid a bit), just hop to a close point
    k = min(len(path) - 1, 3)
    return math.atan2(path[k][1] - py, path[k][0] - px), math.hypot(path[k][0] - px, path[k][1] - py)


S1_GOAL = (0.22, 1.27)


def s1_done():
    return py > 1.12 and abs(px - 0.22) < 0.03


def back_off():
    # something is right in our face, reverse a few cm and think again
    for _ in range(15):
        set_vw(-0.08, 0)
        step()
        s1_sense()
    full_stop()


def run_s1():
    # pivot, drive one straight leg, repeat. replan as soon as a new cube
    # shows up on the line we're on
    global new_obs, sense_hook
    sense_hook = s1_sense
    stuck = 0
    while not s1_done():
        path = astar(px, py, *S1_GOAL)
        new_obs = False
        if not path:
            turn_to(wrap(yaw + 1.0))      # cant see a way, look around
            continue
        hd, dist = pick_seg(path)
        if abs(wrap(hd - yaw)) > 0.01:
            turn_to(hd, 0.008)
            if new_obs:
                continue          # saw new stuff while turning, plan again
        x0, y0 = px, py
        moved = 0.0
        while True:
            step()
            s1_sense()
            if s1_done():
                return
            moved = (px - x0) * math.cos(hd) + (py - y0) * math.sin(hd)
            rem = dist - moved
            if rem < 0.004:
                break
            if new_obs:
                new_obs = False
                if not line_ok(px, py, x0 + dist * math.cos(hd), y0 + dist * math.sin(hd)):
                    break
            if min(D['ds_front'], D['ds_fl1'], D['ds_fr1']) < 0.03:
                break
            set_vw(min(V_MAX, math.sqrt(2 * ACC * rem) + 0.01), 0.0)
        full_stop()
        stuck = stuck + 1 if moved < 0.01 else 0
        if stuck >= 3:
            back_off()
            stuck = 0


# ---------------------------------------------------------------- start
def where_am_i():
    # did they put us at the S1 start or at the maze entrance? (rulebook 4.6,
    # for the S2 attempts they might place the robot at the entrance)
    global px, py
    k = round(yaw / HP)
    turn_to(k * HP)
    stop_now(3)
    # world E/N/W/S distances from the 4 axis sensors
    wd = {}
    for n in AXIS_SENS:
        ox, oy, a = ray(n)
        q = round(wrap(a) / HP) % 4    # 0=E 1=N 2=W 3=S
        wd[q] = (D[n], ox - px, oy - py)
    e, w, so = wd[0][0], wd[2][0], wd[3][0]
    # start zone = bottom right corner so east AND south are both close.
    # (only checking the floor colour got fooled by a black cube once)
    if not (e < 0.3 and so < 0.3) and w < 0.2:
        # maze side. x from the west wall (chamber + maze share x=0.101)
        px = 0.101 + w - wd[2][1]
        # y: look south at the red chamber tile (y 1.214..1.336). if we're in
        # c1r1 we see it a bit ahead, in the chamber we're sitting on it
        turn_to(-HP, 0.01)
        stop_now(3)
        img = cam.getImage()
        far = 0.0
        for i, gx, gy in PIX:
            r, g, b = img[i + 2], img[i + 1], img[i]
            if abs(gy) < 0.08 and r > 100 and r > 2 * g and r > 2 * b:
                far = max(far, gx)
        py = 1.214 + far if far > 0.15 else 1.275
        print('starting at the maze entrance y~%.3f' % py)
        return 2
    xs = 2.434 - e - wd[0][1]
    ys = 0.006 + wd[3][0] - wd[3][2]
    if 2.12 < xs < 2.44 and 0.0 < ys < 0.32:
        px, py = xs, ys
    else:
        px, py = 2.315, 0.125
    print('start pose %.3f %.3f' % (px, py))
    return 1


# TODO: arc turns instead of stop-pivot-go would be faster but every time we
# tried curves the odometry died. maybe for phase 2
def main():
    for _ in range(3):
        step()
    sec = where_am_i()
    if sec == 1:
        run_s1()
        print('section 1 done t=%.1fs' % t_sim)
    enter_maze()
    run_maze()
    print('done t=%.1fs' % t_sim)
    while True:
        set_vw(0, 0)
        step()


try:
    main()
except SystemExit:
    pass

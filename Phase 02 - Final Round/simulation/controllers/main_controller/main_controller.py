# PeraBots 2026 - University Category Final Round
# The Nine Seals of Pera - Autonomous Controller for GuardianBot
#   v4.0: 8 x VL53L0X ToF (tof_*, 3-120 cm), 2 x HC-SR04 (us_front, us_back),
#         Pololu QTR-8RC line array (gs0..gs7), 2 x TCS3200 (tcs_floor, tcs_front),
#         ESP32-CAM (cam, 48x32), L3G4200D gyro, N20 hall encoders, two servo gripper arms.
#   The real robot runs the same strategy in firmware/esp32cam_brain (C++).
#
# Section 1 & 2: User's optimized A* path planner, sensor ray-casting,
#                occupancy grid mapping, camera-based dead-end tile detection.
# Section 3: Autonomous Phase 3 Guardian Extension:
#            - Solves the 4 maze clues (Tile counts & turn counting)
#            - Identifies loyal clan (Blue Sentinels vs Green Keepers)
#            - Solves Nine-Point Board Magic Square (Sum=15)
#            - Determines winning placement move
#            - Collects corresponding piece from collection station
#            - Places piece on board and navigates to END zone.

import math
import heapq
from controller import Robot

TS = 16
R_WHEEL = 0.0215
HALF_TRACK = 0.062    # 124 mm track
PIV_X = 0.0           # origin = axle centre

MAX_DV = 4.0
V_MAX = 0.30
ACC = 3.8
W_RUN = 0.15

CELL = 0.25
MX0, MY0 = 0.095, 1.33
COLS, ROWS = 9, 4
END = (8, 3)

PI = math.pi
HP = PI / 2
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]
HANG = [HP, 0.0, -HP, PI]

robot = Robot()

lm = robot.getDevice('left_wheel_motor')
rm = robot.getDevice('right_wheel_motor')
for m in (lm, rm):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
W_MAX = min(lm.getMaxVelocity(), 15.0)

le = robot.getDevice('left_wheel_sensor')
re_ = robot.getDevice('right_wheel_sensor')
le.enable(TS)
re_.enable(TS)

led_b = robot.getDevice('led_blue')
led_g = robot.getDevice('led_green')
led_b.set(0)
led_g.set(0)

# name: (x, y, heading, max range)  - origins are the sensor faces (robot frame)
SENS = {
    'us_front': (0.075, 0.0, 0.0, 2.50),
    'us_back': (-0.075, 0.0, PI, 2.50),
    'tof_front': (0.068, 0.0, 0.0, 1.20),
    'tof_fl': (0.050, 0.046, PI / 4, 1.20),
    'tof_fr': (0.050, -0.046, -PI / 4, 1.20),
    'tof_lf': (0.030, 0.064, HP, 1.20),
    'tof_lr': (-0.036, 0.064, HP, 1.20),
    'tof_rf': (0.030, -0.064, -HP, 1.20),
    'tof_rr': (-0.036, -0.064, -HP, 1.20),
    'tof_back': (-0.066, 0.030, PI, 1.20),
}
TOF_MAP_MAX = 0.80      # VL53L0X noise grows with range: map only the first 80 cm
TOF_MIN = 0.031         # nearer than 3 cm reads 30 mm
SIDE_PAIR_DX = 0.066    # tof_lf / tof_lr (and rf / rr) are 66 mm apart: wall-angle fix
DS_MAX = 2.49
US_MAP_MAX = 0.40
ds = {}
for n in SENS:
    ds[n] = robot.getDevice(n)
    ds[n].enable(TS)
D = {n: 9.0 for n in SENS}
AXIS_SENS = ('tof_front', 'tof_lf', 'tof_rf', 'tof_back')
N_GS = 8
GS = [robot.getDevice('gs%d' % i) for i in range(N_GS)]
for g_ in GS:
    g_.enable(TS)
GSV = [0.0] * N_GS
grip_l = robot.getDevice('grip_left')
grip_r = robot.getDevice('grip_right')
grip_l.setPosition(1.05)
grip_r.setPosition(-1.05)
tcs_floor = robot.getDevice('tcs_floor')
tcs_floor.enable(TS)
tcs_front = robot.getDevice('tcs_front')
tcs_front.enable(TS)
TCS_FLOOR_XY = (0.012, 0.030)          # floor colour sensor position (robot frame)
cam = robot.getDevice('cam')            # ESP32-CAM: the real robot receives the same 48x32 grid over UART
cam.enable(TS)
CW, CH = cam.getWidth(), cam.getHeight()


def rng_max(n):
    return SENS[n][3] - 0.005


imu = robot.getDevice('imu')
imu.enable(TS)
gyro = robot.getDevice('gyro')
gyro.enable(TS)


def wrap(a):
    return (a + PI) % (2 * PI) - PI


px, py = 2.315, 0.125
yaw = 0.0
yaw_off = 0.0           # heading correction learned from the side ToF pairs (the real gyro drifts)
_pl = _pr = None
_last_l = _last_r = 0.0
t_sim = 0.0


def step():
    global px, py, yaw, _pl, _pr, t_sim
    if robot.step(TS) == -1:
        raise SystemExit
    t_sim += TS / 1000.0
    l, r = le.getValue(), re_.getValue()
    y2 = wrap(imu.getRollPitchYaw()[2] + yaw_off)
    if _pl is not None:
        d = ((l - _pl) + (r - _pr)) * 0.5 * R_WHEEL
        mid = yaw + wrap(y2 - yaw) * 0.5
        px += d * math.cos(mid)
        py += d * math.sin(mid)
    _pl, _pr, yaw = l, r, y2
    for n in SENS:
        D[n] = ds[n].getValue() / 1000.0
    for i in range(N_GS):
        GSV[i] = GS[i].getValue()
    if LOG:
        log_tick()


def set_vw(v, w):
    global _last_l, _last_r
    if v != 0:
        w = max(-W_RUN, min(W_RUN, w)) if abs(v) > 0.12 else 0.0
    a = (v - w * HALF_TRACK) / R_WHEEL     # left wheel
    b = (v + w * HALF_TRACK) / R_WHEEL     # right wheel
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
    return (_last_l + _last_r) * 0.5 * R_WHEEL


def ray(n):
    sx, sy, a, _ = SENS[n]
    sx -= PIV_X
    c, s = math.cos(yaw), math.sin(yaw)
    return px + c * sx - s * sy, py + s * sx + c * sy, yaw + a


def near_cardinal(tol):
    k = round(yaw / HP)
    return abs(wrap(yaw - k * HP)) < tol


sense_hook = None


def turn_to(target, tol=0.02):
    full_stop()
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


CAM_X, CAM_Z, CAM_P, CAM_FOV = 0.062 - PIV_X, 0.140, 0.50, 0.90
_f = (CW / 2) / math.tan(CAM_FOV / 2)
PIX = []                                # (byte index, ground x, ground y) for every 2nd pixel that sees the floor
for v in range(1, CH, 2):
    for u in range(1, CW, 2):
        x, y, z = _f, -(u + 0.5 - CW / 2), -(v + 0.5 - CH / 2)
        x2 = x * math.cos(CAM_P) + z * math.sin(CAM_P)
        z2 = -x * math.sin(CAM_P) + z * math.cos(CAM_P)
        if z2 >= -0.05 * math.sqrt(x * x + y * y + z * z):
            continue
        t = -CAM_Z / z2
        gx, gy = CAM_X + t * x2, t * y
        if gx < 0.75:
            PIX.append((4 * (v * CW + u), gx, gy))

BLUE, GREEN, FLOOR, JUNK = 1, 2, 0, -1


def tcs_rgb(dev):
    """Average colour seen by a TCS3200 (4x4 camera in the simulation)."""
    img = dev.getImage()
    if not img:
        return 0, 0, 0
    w, h = dev.getWidth(), dev.getHeight()
    r = g = b = 0
    for yy in range(h):
        for xx in range(w):
            r += dev.imageGetRed(img, w, xx, yy)
            g += dev.imageGetGreen(img, w, xx, yy)
            b += dev.imageGetBlue(img, w, xx, yy)
    n = w * h
    return r / n, g / n, b / n


def classify(r, g, b):
    m = max(r, g, b)
    if m < 50:
        return FLOOR
    if b > 45 and b > 1.5 * g + 15 and b > 1.5 * r + 15:
        return BLUE
    if g > 45 and g > 1.5 * b + 15 and g > 1.5 * r + 15:
        return GREEN
    if r > 100 and r > 2 * g and r > 2 * b:
        return FLOOR
    return JUNK


hz = [[[0, 0] for _ in range(ROWS + 1)] for _ in range(COLS)]
vt = [[[0, 0] for _ in range(ROWS)] for _ in range(COLS + 1)]
for c in range(COLS):
    hz[c][0][1] = hz[c][ROWS][1] = 999
for r in range(ROWS):
    vt[0][r][1] = vt[COLS][r][1] = 999

votes = {}
seen = {}
UNK, OPEN, WALL = 0, 1, 2
map_dirty = True


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
    global px, py
    ox, oy, a = ray(n)
    d = D[n]
    mx = rng_max(n)
    hit = d < mx
    if n.startswith('us_') and (not hit or not near_cardinal(0.12)):
        return                          # lost sonar echo != free space; oblique pings are unreliable
    if n.startswith('tof_') and d <= TOF_MIN:
        fix = False                     # closer than 3 cm: still a wall, but no position fix
    L = d if hit else mx
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


def side_align():
    """Two ToF sensors per side see the same straight wall: their difference is the robot's angle to it.
    Corrects the heading slowly (the real L3G4200D drifts); does nothing while turning or without a wall."""
    global yaw_off, yaw
    if not near_cardinal(0.15) or abs(gyro.getValues()[2]) > 0.2:
        return
    for f, r in (('tof_lf', 'tof_lr'), ('tof_rf', 'tof_rr')):
        df, dr = D[f], D[r]
        if TOF_MIN < df < 0.20 and TOF_MIN < dr < 0.20 and abs(df - dr) < 0.03:
            theta = math.atan2(dr - df, SIDE_PAIR_DX) * (1 if f == 'tof_lf' else -1)
            k = round(yaw / HP)
            err = wrap(yaw - k * HP) - theta           # what the gyro says minus what the wall says
            yaw_off -= 0.02 * err
            yaw = wrap(yaw - 0.02 * err)


def maze_sense():
    if py < 1.2:
        return
    fix = near_cardinal(0.05)
    for n in SENS:
        maze_ray(n, fix and n in AXIS_SENS)
    side_align()
    look()


def can_see(cx, cy, key):
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
                    return False
            prev = (c, r)
            continue
        if (c, r) != prev:
            dc, dr = c - prev[0], r - prev[1]
            if abs(dc) + abs(dr) != 1:
                return False
            h = DIRS.index((dc, dr))
            if edge(prev[0], prev[1], h) != OPEN:
                return False
            prev = (c, r)
    return True


def look():
    """Tile colours: camera votes for every cell it can see (through open walls), floor TCS3200 for the cell below."""
    global map_dirty
    if abs(gyro.getValues()[2]) > 1.0:
        return
    c_, s_ = math.cos(yaw), math.sin(yaw)
    hits = {}
    img = cam.getImage()
    if img:
        camx, camy = px + c_ * CAM_X, py + s_ * CAM_X
        ok = {}
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
    k = classify(*tcs_rgb(tcs_floor))
    if k != JUNK:
        tx, ty = TCS_FLOOR_XY
        fx, fy = (px + c_ * tx - s_ * ty - MX0) / CELL, (py + s_ * tx + c_ * ty - MY0) / CELL
        c, r = int(math.floor(fx)), int(math.floor(fy))
        if in_grid(c, r) and abs(fx - c - 0.5) <= 0.35 and abs(fy - r - 0.5) <= 0.35:
            hits.setdefault((c, r), [0, 0, 0])[k] += 3          # the floor sensor is the most trusted vote
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
        return False
    return any(edge(c, r, h) != WALL for h in range(4))


def plan(c0, r0, h0, goal, allow_unk=False, bonus=None):
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


LAM = 1.0
bonus_max = 0.0


def end_bonus():
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
    """Stand where the camera sees an undecided possible dead end 1 or 2 cells ahead."""
    nc, nr = c + DIRS[h][0], r + DIRS[h][1]
    if not in_grid(nc, nr) or edge(c, r, h) == WALL:
        return False
    if needed(nc, nr):
        return True
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
    blue = nb >= ng
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
    if abs(wrap(HANG[h] - yaw)) > 0.02:
        turn_to(HANG[h], 0.02)


def enter_maze():
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
            if rem <= 0.004 or D['us_front'] < 0.0255 or D['tof_front'] < 0.036:
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
                wait += 1
                if wait > 4:                         # looking at the cells ahead: decide on the votes so far
                    cc, cr = c, r
                    for _ in range(2):
                        if edge(cc, cr, h) != OPEN:
                            break
                        cc, cr = cc + DIRS[h][0], cr + DIRS[h][1]
                        if in_grid(cc, cr) and (cc, cr) not in seen:
                            decide_colour((cc, cr))
                if wait > 8:
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
            print('wiggle, no progress for 6s')
            for _ in range(20):
                set_vw(-0.08, 0)
                step()
            full_stop()
            last_move = t_sim
            mode = 'stopped'


G = 0.02
NX, NY = 122, 68
R_HARD, R_INF, R_SOFT = 0.086, 0.096, 0.14   # 75 mm robot radius + 2 cm grid quantisation margin
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


mark_rect(0, 0, 2.44, 0.006)
mark_rect(0, 0, 0.006, 1.22)
mark_rect(2.434, 0, 2.44, 1.22)
mark_rect(0, 1.214, 0.101, 1.36)
mark_rect(0.339, 1.214, 2.44, 1.36)
WALL_CELL = bytes(occ)                  # arena walls: never forgotten


def s1_sense():
    global px, py
    fix = near_cardinal(0.05)
    for n in SENS:
        d = D[n]
        if d >= rng_max(n) or (n.startswith('us_') and d > US_MAP_MAX):
            continue                    # wide sonar cone: only trust close echoes for the map
        if n.startswith('tof_') and d > TOF_MAP_MAX:
            continue                    # ToF: noisy far away
        ox, oy, a = ray(n)
        hx, hy = ox + d * math.cos(a), oy + d * math.sin(a)
        if fix and n in AXIS_SENS and d > TOF_MIN:
            for face, isx in ((0.006, 1), (2.434, 1), (0.006, 0), (1.214, 0)):
                if isx and abs(math.sin(a)) < 0.06 and abs(hx - face) < 0.03:
                    px += 0.25 * (face - hx)
                elif not isx and abs(math.cos(a)) < 0.06 and abs(hy - face) < 0.03                         and (face < 1 or hx < 0.09 or hx > 0.35):
                    py += 0.25 * (face - hy)
        if 0.012 < hx < 2.428 and 0.012 < hy < 1.208:
            mark(int(hx / G), int(hy / G))
            if n.startswith('us_'):
                ex, ey = hx + 0.01 * math.cos(a), hy + 0.01 * math.sin(a)
                mark(int(ex / G), int(ey / G))


def astar(sx, sy, gx, gy, hard=None):
    hard = R_HARD if hard is None else hard
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
            if cl < hard:
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
    k = min(len(path) - 1, 3)
    return math.atan2(path[k][1] - py, path[k][0] - px), math.hypot(path[k][0] - px, path[k][1] - py)


S1_GOAL = (0.22, 1.27)


def s1_done():
    return py > 1.12 and abs(px - 0.22) < 0.03


def forget_near(rad):
    """Drop sensed obstacle cells within rad of the robot (arena walls stay) and rebuild the clearance map.
    Used when the map has closed every way out: noisy marks get re-sensed on the next look around."""
    global new_obs
    for j in range(NY):
        for i in range(NX):
            k = j * NX + i
            if occ[k] and math.hypot((i + 0.5) * G - px, (j + 0.5) * G - py) < rad and not WALL_CELL[k]:
                occ[k] = 0
    for k in range(NX * NY):
        clear[k] = 9.0
    cells = [k for k in range(NX * NY) if occ[k]]
    for k in cells:
        occ[k] = 0
    for k in cells:
        mark(k % NX, k // NX)
    new_obs = True


def back_off():
    for _ in range(15):
        set_vw(-0.08, 0)
        step()
        s1_sense()
    full_stop()


def run_s1():
    global new_obs, sense_hook
    sense_hook = s1_sense
    stuck = 0
    fails = 0
    while not s1_done():
        path = astar(px, py, *S1_GOAL)
        if not path:
            path = astar(px, py, *S1_GOAL, hard=0.078)      # squeeze: body radius + 3 mm
        if not path:
            fails += 1
            if fails >= 2:
                print('S1: no way out on the map - forgetting obstacles within 35 cm and looking again')
                forget_near(0.35)
                fails = 0
            turn_to(wrap(yaw + 1.6))
            new_obs = False
            continue
        fails = 0
        new_obs = False
        hd, dist = pick_seg(path)
        if abs(wrap(hd - yaw)) > 0.01:
            turn_to(hd, 0.008)
            if new_obs:
                continue
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
            if D['us_front'] < 0.025 or D['tof_front'] < 0.036:
                break
            set_vw(min(V_MAX, math.sqrt(2 * ACC * rem) + 0.01), 0.0)
        full_stop()
        stuck = stuck + 1 if moved < 0.01 else 0
        if stuck >= 3:
            back_off()
            stuck = 0


def where_am_i():
    global px, py
    k = round(yaw / HP)
    turn_to(k * HP)
    stop_now(3)
    wd = {}
    for n in AXIS_SENS:
        ox, oy, a = ray(n)
        q = round(wrap(a) / HP) % 4
        wd[q] = (D[n], ox - px, oy - py)
    e, w, so = wd[0][0], wd[2][0], wd[3][0]
    if not (e < 0.3 and so < 0.3) and w < 0.2:
        px = 0.101 + w - wd[2][1]
        turn_to(-HP, 0.01)
        stop_now(3)
        py = 1.214 + D['us_front'] + SENS['us_front'][0] if D['us_front'] < 0.3 else 1.275
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




# ==============================================================================
# LOGGING - every message also goes to run_log.txt next to this controller
# ==============================================================================
import builtins
import os
from collections import deque

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'run_log.txt')
try:
    _logf = open(LOG_PATH, 'w')
except OSError:
    _logf = None
LOG = False


def print(*a, **k):                      # noqa: A001  (module-level override)
    msg = ' '.join(str(x) for x in a)
    builtins.print(msg, flush=True)
    if _logf:
        _logf.write('[%7.2f] %s\n' % (t_sim, msg))
        _logf.flush()


def log_tick():
    pass


# ==============================================================================
# SECTION 3: THE BATTLE OF THE TWO GUARDIANS
# Board geometry (rulebook Figure 1, world frame):
#   board x 0.49..1.71, y 2.59..3.51; robot enters at (1.70, 3.05) heading -x
#   transverse lines x = 0.50 0.90 1.30 1.70 ; long lines y = 2.60 2.90 3.20 3.50
# ==============================================================================
GRIP_OPEN, GRIP_CLOSED = 0.0, 1.05
POCKET_OFF = 0.033                  # piece centre is 33 mm behind the axle
LINE_X = 0.045                      # line array is 45 mm ahead of the axle
GS_Y = [(i - (N_GS - 1) / 2) * 0.009525 for i in range(N_GS)]   # gs0 = right-most (Pololu QTR-8RC)
GS_WHITE, GS_BLACK = 350.0, 0.0     # overwritten by calibrate_line()

XT = (0.50, 0.90, 1.30, 1.70)       # transverse lines (x)
YL = (2.60, 2.90, 3.20, 3.50)       # long lines (y)
ROW_X = (1.50, 1.10, 0.70)          # cell centres, entrance row first
COL_Y = (2.75, 3.05, 3.35)          # robot's left (6) ... right (8) seen from the entrance
MAGIC = ((6, 1, 8), (7, 5, 3), (2, 9, 4))     # Lo Shu square, 1 = middle of first row, 6 on the left
POS_CELL = {MAGIC[i][j]: (ROW_X[i], COL_Y[j]) for i in range(3) for j in range(3)}
CELL_POS = {v: k for k, v in POS_CELL.items()}
EXIT_JUNC = (2.21, 3.05)
COLLECT_JUNC = (2.006, 3.05)
BOARD_IN = (1.70, 3.05)
BOARD_OUT = (0.50, 3.05)
END_PT = (0.13, 3.05)
COLLECT = {'GREEN': (2.006, 2.70), 'BLUE': (2.006, 3.40)}

LINES = [((2.21, 2.44), (2.21, 3.05)), ((1.70, 3.05), (2.21, 3.05)), ((2.006, 2.70), (2.006, 3.40)),
         ((0.13, 3.05), (0.50, 3.05))]
LINES += [((0.50, y), (1.70, y)) for y in YL]
LINES += [((x, 2.60), (x, 3.50)) for x in XT]


def gs_dark():
    span = max(20.0, GS_WHITE - GS_BLACK)
    return [min(1.0, max(0.0, (GS_WHITE - v) / span)) for v in GSV]


def line_reading():
    """(lateral offset of the line in metres, +left; number of dark sensors) or (None, 0)."""
    d = gs_dark()
    s = sum(d)
    if s < 0.6:
        return None, 0
    return sum(di * y for di, y in zip(d, GS_Y)) / s, sum(1 for di in d if di > 0.5)


def calibrate_line():
    """What the CAL button does on the real robot: learn white/black from a short sweep."""
    global GS_WHITE, GS_BLACK
    lo, hi = 1e9, -1e9
    t0 = yaw
    for target in (t0 + 0.35, t0 - 0.35, t0):
        turn_to(wrap(target), 0.03)
        for _ in range(3):
            step()
            lo, hi = min(lo, min(GSV)), max(hi, max(GSV))
    if hi - lo > 60:
        GS_WHITE, GS_BLACK = hi, lo
    print('[S3] line sensors calibrated: white=%.0f black=%.0f' % (GS_WHITE, GS_BLACK))


_ll = _lr = 0.0


def drive_lr(v, w):
    """Wheel command with steering at any speed (accel-limited)."""
    global _ll, _lr, _last_l, _last_r
    a = (v - w * HALF_TRACK) / R_WHEEL
    b = (v + w * HALF_TRACK) / R_WHEEL
    big = max(abs(a), abs(b))
    if big > W_MAX:
        a *= W_MAX / big
        b *= W_MAX / big
    a = max(_last_l - MAX_DV, min(_last_l + MAX_DV, a))
    b = max(_last_r - MAX_DV, min(_last_r + MAX_DV, b))
    _last_l, _last_r = a, b
    lm.setVelocity(a)
    rm.setVelocity(b)


def line_correct(seg):
    """Snap the pose estimate onto the known line we are driving along."""
    global px, py
    (x0, y0), (x1, y1) = seg
    L = math.hypot(x1 - x0, y1 - y0)
    ux, uy = (x1 - x0) / L, (y1 - y0) / L
    nx, ny = -uy, ux
    c, s = math.cos(yaw), math.sin(yaw)
    lx, ly = -s, c
    den = nx * lx + ny * ly
    if abs(den) < 0.7:
        return
    m, nd = line_reading()
    if m is None or nd >= N_GS - 1:
        return
    sx, sy = px + c * LINE_X, py + s * LINE_X
    m_exp = -(nx * (sx - x0) + ny * (sy - y0)) / den
    e = m - m_exp
    if abs(e) < 0.07:
        px -= 0.25 * e * lx
        py -= 0.25 * e * ly


_cross_done = set()


def crossing_correct(ux, uy):
    """When the whole array sees black we are over a perpendicular line: fix the along-track position."""
    global px, py
    m, nd = line_reading()
    if nd < N_GS - 1:
        return
    c, s = math.cos(yaw), math.sin(yaw)
    sx, sy = px + c * LINE_X, py + s * LINE_X
    for (x0, y0), (x1, y1) in LINES:
        vert = abs(x1 - x0) < 1e-6
        if abs(ux) > 0.9 and vert and min(y0, y1) - 0.01 <= sy <= max(y0, y1) + 0.01:
            err = x0 - sx
            key = ('x', x0)
        elif abs(uy) > 0.9 and not vert and min(x0, x1) - 0.01 <= sx <= max(x0, x1) + 0.01:
            err = y0 - sy
            key = ('y', y0)
        else:
            continue
        if abs(err) < 0.10 and key not in _cross_done:      # lines are >= 0.30 m apart: unambiguous
            _cross_done.add(key)
            if abs(ux) > 0.9:
                px += 0.8 * err
            else:
                py += 0.8 * err


def seg_of(p, q):
    for a, b in LINES:
        if on_seg(p, a, b, 0.03) and on_seg(q, a, b, 0.006):
            return (a, b)
    return None


def on_seg(p, a, b, tol=0.006):
    if abs(a[0] - b[0]) < 1e-6:
        return abs(p[0] - a[0]) < tol and min(a[1], b[1]) - tol <= p[1] <= max(a[1], b[1]) + tol
    return abs(p[1] - a[1]) < tol and min(a[0], b[0]) - tol <= p[0] <= max(a[0], b[0]) + tol


def drive_line(tx, ty, v_max=0.16, reverse=False, follow=True, stop_tol=0.004):
    """Drive straight to (tx, ty) along a line (or off-line if follow=False)."""
    global _cross_done
    _cross_done = set()
    x0, y0 = px, py
    L = math.hypot(tx - x0, ty - y0)
    if L < 0.003:
        return
    ux, uy = (tx - x0) / L, (ty - y0) / L
    seg = seg_of((x0, y0), (tx, ty)) if follow else None
    hd = math.atan2(uy, ux) + (PI if reverse else 0.0)
    if abs(wrap(hd - yaw)) > 0.06:
        turn_to(wrap(hd), 0.012)
    while True:
        step()
        if seg:
            line_correct(seg)
            crossing_correct(ux, uy)
        rem = (tx - px) * ux + (ty - py) * uy
        if rem < stop_tol:
            break
        lat = -(tx - px) * uy + (ty - py) * ux          # + = target line is to the left of travel
        e = wrap(hd - yaw)
        v = min(v_max, math.sqrt(2 * 1.2 * rem) + 0.015)
        w = 4.0 * e + 12.0 * lat
        w = max(-1.5, min(1.5, w))
        drive_lr(-v if reverse else v, w)
    stop_drive()


def stop_drive():
    while abs(_last_l) > 0.05 or abs(_last_r) > 0.05:
        drive_lr(0, 0)
        step()
    drive_lr(0, 0)
    step()


def wait(sec):
    for _ in range(int(sec * 1000 / TS)):
        drive_lr(0, 0)
        step()


# ------------------------------------------------------------------ route graph
def graph_nodes(extra=()):
    pts = set()
    for a, b in LINES:
        pts.add(a)
        pts.add(b)
    for x in XT:
        for y in YL + COL_Y:
            pts.add((x, y))
    for x in ROW_X:
        for y in YL:
            pts.add((x, y))
    pts.add(COLLECT_JUNC)
    for p in extra:
        pts.add((round(p[0], 4), round(p[1], 4)))
    return pts


def snap(p):
    """Project a point onto the nearest line of the arena."""
    best, bd = p, 1e9
    for (x0, y0), (x1, y1) in LINES:
        if abs(x1 - x0) < 1e-6:
            q = (x0, min(max(p[1], min(y0, y1)), max(y0, y1)))
        else:
            q = (min(max(p[0], min(x0, x1)), max(x0, x1)), y0)
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        if d < bd:
            best, bd = q, d
    return (round(best[0], 4), round(best[1], 4))


def route(a, b, extra=()):
    a, b = snap(a), snap(b)
    nodes = graph_nodes(list(extra) + [a, b])
    adj = {n: [] for n in nodes}
    for s0, s1 in LINES:
        on = sorted([n for n in nodes if on_seg(n, s0, s1)], key=lambda n: (n[0] - s0[0]) ** 2 + (n[1] - s0[1]) ** 2)
        for p, q in zip(on, on[1:]):
            d = math.hypot(p[0] - q[0], p[1] - q[1])
            adj[p].append((q, d))
            adj[q].append((p, d))
    a = min(nodes, key=lambda n: math.hypot(n[0] - a[0], n[1] - a[1]))
    b = min(nodes, key=lambda n: math.hypot(n[0] - b[0], n[1] - b[1]))
    best = {(a, None): 0.0}
    prev = {}
    pq = [(0.0, a, None)]
    while pq:
        d, n, hd = heapq.heappop(pq)
        if n == b:
            out = [n]
            k = (n, hd)
            while k in prev:
                k = prev[k]
                out.append(k[0])
            return out[::-1]
        if d > best.get((n, hd), 1e9):
            continue
        for m, w in adj[n]:
            h2 = (round(math.copysign(1, m[0] - n[0])) if abs(m[0] - n[0]) > 1e-6 else 0,
                  round(math.copysign(1, m[1] - n[1])) if abs(m[1] - n[1]) > 1e-6 else 0)
            d2 = d + w + (0.08 if hd is not None and h2 != hd else 0.0)
            if d2 < best.get((m, h2), 1e9):
                best[(m, h2)] = d2
                prev[(m, h2)] = (n, hd)
                heapq.heappush(pq, (d2, m, h2))
    return None


def go(target, v_max=0.16, extra=()):
    path = route((px, py), target, extra)
    if path is None:
        print('[S3] no route to', target)
        return False
    for q in path[1:]:
        drive_line(q[0], q[1], v_max)
    return True


# ------------------------------------------------------------------ clues
def maze_clues():
    nb = sum(1 for v in seen.values() if v == BLUE)
    ng = sum(1 for v in seen.values() if v == GREEN)
    # shortest path (fewest cells, then fewest turns) through the mapped maze;
    # the robot enters cell (0,0) heading +y (h=0). Turns in the exit cell are not counted.
    start = (0, 0, 0)
    best = {start: (0, 0, 0, 0)}
    pq = [((0, 0), start, 0, 0)]
    res = None
    while pq:
        (cells, turns), (c, r, h), R, L = heapq.heappop(pq)
        if (c, r) == END:
            res = (cells, R, L)
            break
        for h2 in range(4):
            if edge(c, r, h2) != OPEN:
                continue
            nc, nr = c + DIRS[h2][0], r + DIRS[h2][1]
            if not in_grid(nc, nr) or h2 == (h + 2) % 4:
                continue
            R2 = R + (1 if h2 == (h + 1) % 4 else 0)
            L2 = L + (1 if h2 == (h + 3) % 4 else 0)
            k = (cells + 1, turns + (h2 != h))
            st = (nc, nr, h2)
            if st not in best or k < best[st][:2]:
                best[st] = (k[0], k[1], R2, L2)
                heapq.heappush(pq, (k, st, R2, L2))
    R, L = (res[1], res[2]) if res else (0, 0)
    print('[S3] clues: blue tiles=%d green tiles=%d right turns=%d left turns=%d (path %s cells)'
          % (nb, ng, R, L, res[0] if res else '?'))
    return nb, ng, R, L


def winning_cell(board, clan):
    """board: pos -> 'BLUE'/'GREEN'. Prefer lines along the entry direction (rulebook advice)."""
    along = [(MAGIC[0][j], MAGIC[1][j], MAGIC[2][j]) for j in range(3)]
    across = [MAGIC[i] for i in range(3)]
    diag = [(6, 5, 4), (8, 5, 2)]
    for group in (along, across, diag):
        for line in group:
            mine = [p for p in line if board.get(p) == clan]
            empty = [p for p in line if p not in board]
            if len(mine) == 2 and len(empty) == 1:
                return empty[0], line
    return None, None


# ------------------------------------------------------------------ gripper
def grip_set(pos, sec=0.6):
    grip_l.setPosition(pos)
    grip_r.setPosition(-pos)
    wait(sec)


def piece_colour():
    """Front TCS3200 colour (BLUE / GREEN / None)."""
    k = classify(*tcs_rgb(tcs_front))
    return 'BLUE' if k == BLUE else 'GREEN' if k == GREEN else None


def pick_piece(clan):
    """Back the rear pocket over the piece at the collection point and close the gripper."""
    cx, cy = COLLECT[clan]
    sgn = 1 if cy > COLLECT_JUNC[1] else -1
    go(COLLECT_JUNC)
    stop_y = cy - sgn * 0.135
    drive_line(cx, stop_y, 0.10)                       # face the piece, stop 13.5 cm short
    turn_to(wrap(math.atan2(-sgn, 0)), 0.01)            # spin: pocket now faces the piece
    grip_set(GRIP_OPEN)
    drive_line(cx, cy - sgn * POCKET_OFF, 0.05, reverse=True, follow=False)
    grip_set(GRIP_CLOSED, 0.8)
    ok = True
    print('[S3] picked %s piece' % clan)
    drive_line(COLLECT_JUNC[0], COLLECT_JUNC[1], 0.12)
    return ok


def place_piece(pos):
    cx, cy = POS_CELL[pos]
    ax = cx + 0.20                                        # transverse line on the entrance side
    go((ax, cy), 0.14)
    turn_to(0.0, 0.008)                                   # face +x: pocket points into the cell
    drive_line(cx + POCKET_OFF, cy, 0.05, reverse=True, follow=False)
    grip_set(GRIP_OPEN, 0.6)
    print('[S3] piece released in position %d at (%.2f, %.2f)' % (pos, cx, cy))
    drive_line(ax, cy, 0.08, follow=False)
    grip_set(GRIP_CLOSED, 0.4)


# ------------------------------------------------------------------ inspection
def inspect_board():
    """Drive the two inner long lines and use the side Sharp sensors to find occupied cells."""
    hits = {}

    def sample():
        for n in ('tof_lf', 'tof_lr', 'tof_rf', 'tof_rr'):
            d = D[n]
            if d >= 0.20 or d <= TOF_MIN:
                continue
            ox, oy, a = ray(n)
            hx, hy = ox + d * math.cos(a), oy + d * math.sin(a)
            for p, (cx, cy) in POS_CELL.items():
                if math.hypot(hx - cx, hy - cy) < 0.06:
                    hits[p] = hits.get(p, 0) + 1

    for y in (2.90, 3.20):
        go((1.70, y))
        turn_to(PI, 0.01)
        tx = 0.52
        L = px - tx
        while px > tx + 0.004:
            step()
            line_correct(((0.50, y), (1.70, y)))
            sample()
            e = wrap(PI - yaw)
            lat = (y - py) * -1.0
            drive_lr(min(0.10, math.sqrt(2 * 1.2 * (px - tx)) + 0.015), max(-1.5, min(1.5, 4 * e + 12 * lat)))
        stop_drive()
    occ = sorted(p for p, n in hits.items() if n >= 4)
    print('[S3] inspection: occupied positions', occ, 'hits', hits)
    return occ


def look_colour(pos):
    """Drive up to the piece (without touching it) and read it with the front TCS3200."""
    cx, cy = POS_CELL[pos]
    go((cx + 0.20, cy), 0.12)
    turn_to(PI, 0.01)
    drive_line(cx + 0.108, cy, 0.05, follow=False)       # sensor ~1.2 cm from the piece face
    nb = ng = 0
    for _ in range(15):
        step()
        c = piece_colour()
        nb += c == 'BLUE'
        ng += c == 'GREEN'
    col = 'BLUE' if nb > ng else 'GREEN' if ng > nb else None
    print('[S3] front colour sensor at position %d reads %s (%d blue / %d green samples)' % (pos, col, nb, ng))
    drive_line(cx + 0.20, cy, 0.06, reverse=True, follow=False)
    return col


# ------------------------------------------------------------------ main section 3
CLUE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'clues.json')


def run_phase3(saved=None):
    print('========== SECTION 3: BATTLE OF THE TWO GUARDIANS ==========')
    if saved:
        nb, ng, R, L = saved
        print('[S3] restart in Section 3 - using clues stored after the maze: blue=%d green=%d right=%d left=%d' % saved)
    else:
        nb, ng, R, L = maze_clues()
        try:                      # the real robot keeps these in ESP32 NVS flash for a Section-3 restart
            import json
            json.dump([nb, ng, R, L], open(CLUE_FILE, 'w'))
        except OSError:
            pass
    clan = 'BLUE' if nb >= ng else 'GREEN'          # same rule as show_majority() (tie -> blue)
    led_b.set(1 if clan == 'BLUE' else 0)
    led_g.set(1 if clan == 'GREEN' else 0)
    print('[S3] majority %s -> robot plays for the %s' % (clan, 'Blue Sentinels (cylinder)' if clan == 'BLUE' else 'Green Keepers (cube)'))

    clues = [('BLUE', nb, 'Cylinder 1'), ('GREEN', ng, 'Cube 1'), ('BLUE', R, 'Cylinder 2'), ('GREEN', L, 'Cube 2')]
    board, used, unresolved = {}, {}, []
    for col, v, name in clues:
        if 1 <= v <= 9 and v not in board:
            board[v] = col
            print('[S3] %s -> position %d %s' % (name, v, POS_CELL[v]))
        else:
            unresolved.append(col)
            print('[S3] %s -> value %d is unusable (out of range or already taken)' % (name, v))

    # leave the maze and drive to the collection junction
    turn_to(HP, 0.01)
    drive_line(EXIT_JUNC[0], 2.44, 0.12, follow=False)
    calibrate_line()
    drive_line(EXIT_JUNC[0], EXIT_JUNC[1], 0.16)
    pick_piece(clan)

    if unresolved:
        print('[S3] clues not sufficient (%d piece(s) unknown) -> inspecting the board along the grid lines' % len(unresolved))
        occ = inspect_board()
        unknown = [p for p in occ if p not in board]
        need = {'BLUE': unresolved.count('BLUE'), 'GREEN': unresolved.count('GREEN')}
        for p in unknown:
            if need['BLUE'] and not need['GREEN']:
                board[p] = 'BLUE'
            elif need['GREEN'] and not need['BLUE']:
                board[p] = 'GREEN'
            else:
                col = look_colour(p) or ('BLUE' if need['BLUE'] >= need['GREEN'] else 'GREEN')
                board[p] = col
            need[board[p]] = max(0, need[board[p]] - 1)
        for p in list(board):
            if p not in occ and occ:
                print('[S3] position %d expected a piece but none was seen - dropping it' % p)
                del board[p]

    print('[S3] board: ' + '  '.join('%d:%s' % (p, board.get(p, '.')[0]) for row in MAGIC for p in row))
    for row in MAGIC[::-1]:
        print('        ' + ' '.join({'BLUE': 'B', 'GREEN': 'G'}.get(board.get(p), '.') for p in row))
    print('        (entrance side)')
    win, line = winning_cell(board, clan)
    if win is None:
        free = [p for p in (1, 5, 6, 8, 7, 3, 2, 9, 4) if p not in board]
        win = free[0]
        print('[S3] no winning move available - placing in position %d' % win)
    else:
        print('[S3] winning move: position %d completes line %s' % (win, line))
    place_piece(win)

    go(BOARD_OUT, 0.15)
    drive_line(END_PT[0], END_PT[1], 0.12)
    print('[S3] reached the END zone at (%.2f, %.2f)  t=%.1fs' % (px, py, t_sim))
    print('==================== MISSION COMPLETE ====================')


def main():
    import sys
    global px, py, yaw
    for _ in range(3):
        step()
    if 'section3' in sys.argv[1:]:
        # judges restarted us at the Section 3 start (maze exit cell, facing the exit door)
        import json
        px, py = MX0 + (END[0] + 0.5) * CELL, MY0 + (END[1] + 0.5) * CELL
        saved = tuple(json.load(open(CLUE_FILE)))
        run_phase3(saved)
        while True:
            drive_lr(0, 0)
            step()
    sec = where_am_i()
    if sec == 1:
        run_s1()
        print('section 1 done t=%.1fs' % t_sim)
    enter_maze()
    run_maze()
    print('section 2 maze done t=%.1fs' % t_sim)
    run_phase3()
    while True:
        drive_lr(0, 0)
        step()


try:
    main()
except SystemExit:
    pass

import sys, math
sys.stdout = open('ctrl_log.txt', 'w', buffering=1)
from controller import Robot
robot = Robot()
TS = 16
lm = robot.getDevice('left_wheel_motor'); rm = robot.getDevice('right_wheel_motor')
for m in (lm, rm):
    m.setPosition(float('inf')); m.setVelocity(0)
imu = robot.getDevice('imu'); imu.enable(TS)
le = robot.getDevice('left_wheel_sensor'); re_ = robot.getDevice('right_wheel_sensor')
le.enable(TS); re_.enable(TS)
L = R = 0.0
DV = 0.6

def yaw(): return imu.getRollPitchYaw()[2]
def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi
def step(): robot.step(TS)

def setw(a, b):
    global L, R
    L = max(L - DV, min(L + DV, a)); R = max(R - DV, min(R + DV, b))
    lm.setVelocity(L); rm.setVelocity(R)

def stop():
    while abs(L) > 0.01 or abs(R) > 0.01:
        setw(0, 0); step()
    for _ in range(20): step()

def pivot(target):
    while True:
        step()
        e = wrap(target - yaw())
        if abs(e) < 0.01 and abs(L) < 0.3: break
        w = min(6, math.sqrt(32 * abs(e))); w = max(w, 0.3)
        k = (w if e > 0 else -w) * 1.6
        setw(-k, k)
    stop()

def run(dist, dv, v=10.0, back=False):
    global DV
    DV = dv
    y0 = yaw()
    s0 = (le.getValue() + re_.getValue()) / 2 * 0.02
    sg = -1 if back else 1
    ymin = ymax = 0
    while True:
        step()
        s = abs((le.getValue() + re_.getValue()) / 2 * 0.02 - s0)
        dy = wrap(yaw() - y0)
        ymin, ymax = min(ymin, dy), max(ymax, dy)
        if s > dist: break
        setw(sg * v, sg * v)
    DV = 0.6
    stop()
    return wrap(yaw() - y0), ymin, ymax

def run_hold(dist, hd, k, cap, back=False):
    s0 = (le.getValue() + re_.getValue()) / 2 * 0.02
    sg = -1 if back else 1
    worst = 0
    while True:
        step()
        s = abs((le.getValue() + re_.getValue()) / 2 * 0.02 - s0)
        e = wrap(hd - yaw())
        worst = max(worst, abs(e))
        if s > dist: break
        w = max(-cap, min(cap, k * e))
        d = w * 0.032 / 0.02
        setw(sg * 10 - d, sg * 10 + d)
    stop()
    return worst, wrap(hd - yaw())

def run_v(dist, hd, k, cap, v, back=False):
    s0 = (le.getValue() + re_.getValue()) / 2 * 0.02
    sg = -1 if back else 1
    worst = 0
    while True:
        step()
        s = abs((le.getValue() + re_.getValue()) / 2 * 0.02 - s0)
        e = wrap(hd - yaw())
        worst = max(worst, abs(e))
        if s > dist: break
        w = max(-cap, min(cap, k * e))
        d = w * 0.032 / 0.02
        setw(sg * v - d, sg * v + d)
    stop()
    return worst, wrap(hd - yaw())

for _ in range(10): step()
for deg in (90, 100, 110, 120, 135, 150, 160, 170, 175, 180):
    for err in (0.0,):
        hd = math.radians(deg)
        pivot(hd)
        b = run_v(0.35, hd, 0, 0, 10, back=True)
        pivot(hd)
        f = run_v(0.35, hd, 0, 0, 10)
        print('RES heading=%3d BACK end=%+.3f worst=%.3f | FWD end=%+.3f' % (deg, b[1], b[0], f[1]))
while robot.step(TS) != -1:
    pass

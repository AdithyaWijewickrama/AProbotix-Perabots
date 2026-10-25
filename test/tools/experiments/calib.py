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
t = 0
L = R = 0.0
DV = float(sys.argv[1]) if len(sys.argv) > 1 else 0.6

def step():
    global t
    robot.step(TS); t += TS / 1000

def setw(a, b):
    global L, R
    L = max(L - DV, min(L + DV, a)); R = max(R - DV, min(R + DV, b))
    lm.setVelocity(L); rm.setVelocity(R)

def wrap(a): return (a + math.pi) % (2 * math.pi) - math.pi

def pivot(target, wmax):
    while True:
        step()
        e = wrap(target - imu.getRollPitchYaw()[2])
        if abs(e) < 0.02 and abs(L) < 0.5: break
        w = min(wmax, math.sqrt(2 * 16 * abs(e))); w = max(w, 0.4)
        w = w if e > 0 else -w
        k = w * 0.032 / 0.02
        setw(-k, k)
    for _ in range(30):
        setw(0, 0); step()
    print('MARK t=%.2f yaw=%.3f' % (t, imu.getRollPitchYaw()[2]))

def straight(dist):
    s0 = (le.getValue() + re_.getValue()) / 2 * 0.02
    while True:
        step()
        s = (le.getValue() + re_.getValue()) / 2 * 0.02 - s0
        if s >= dist: break
        v = min(10, math.sqrt(2 * 30 * max(dist - s, 0)) + 0.5)
        setw(v, v)
    for _ in range(30):
        setw(0, 0); step()
    print('MARK t=%.2f straight %.3f' % (t, (le.getValue() + re_.getValue()) / 2 * 0.02 - s0))

def straight_h(dist, hd, dv):
    global DV
    DV = dv
    s0 = (le.getValue() + re_.getValue()) / 2 * 0.02
    while True:
        step()
        s = (le.getValue() + re_.getValue()) / 2 * 0.02 - s0
        if s >= dist: break
        v = min(10, math.sqrt(2 * 30 * dv / 0.6 * max(dist - s, 0)) + 0.3)
        e = wrap(hd - imu.getRollPitchYaw()[2])
        rpy = imu.getRollPitchYaw()
        print('TR t=%.3f s=%.4f L=%.2f R=%.2f pitch=%.4f roll=%.4f' % (t, s, L, R, rpy[1], rpy[0]))
        w = 8 * e * 1.6
        setw(v - w, v + w)
    for _ in range(40):
        setw(0, 0); step()
    print('MARK t=%.2f straight dv=%.2f enc=%.3f' % (t, dv, (le.getValue() + re_.getValue()) / 2 * 0.02 - s0))

for _ in range(10): step()
print('MARK t=%.2f yaw=%.3f' % (t, imu.getRollPitchYaw()[2]))
for dv in (0.6, 0.3):
    straight_h(0.25, math.pi / 2, dv)
    pivot(-math.pi / 2, 6)
    straight_h(0.25, -math.pi / 2, dv)
    pivot(math.pi / 2, 6)
while robot.step(TS) != -1:
    pass

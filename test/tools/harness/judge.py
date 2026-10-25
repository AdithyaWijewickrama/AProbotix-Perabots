import sys, math
sys.stdout = open('judge_log.txt', 'w', buffering=1)
from controller import Supervisor

sup = Supervisor()
TS = 16
node = sup.getFromDef('ROBOT')
node.enableContactPointsTracking(TS, True)
mode = sys.argv[1] if len(sys.argv) > 1 else 's1'
t = 0.0
dist = 0.0
last = None
coll = 0
in_contact = False
t_s1 = None
t_end = None
still = 0.0
maxtilt = 0.0
coll_log = []
LIMIT = float(__import__('os').environ.get('JLIMIT', '300'))

def footprint_in_end(p, yaw):
    # robot body corners (robot frame) incl caster/camera, must be in end cell interior
    pts = [(-0.04, -0.037), (-0.04, 0.037), (0.045, -0.037), (0.045, 0.037)]
    c, s = math.cos(yaw), math.sin(yaw)
    for x, y in pts:
        wx, wy = p[0] + c * x - s * y, p[1] + s * x + c * y
        if not (2.095 + 0.006 < wx < 2.345 - 0.006 and 2.08 + 0.006 < wy < 2.33 - 0.006):
            return False
    return True

while sup.step(TS) != -1:
    t += TS / 1000.0
    p = node.getPosition()
    R = node.getOrientation()
    yaw = math.atan2(R[3], R[0])
    tilt = math.degrees(math.acos(max(-1, min(1, R[8]))))
    maxtilt = max(maxtilt, tilt)
    if last:
        step_d = math.hypot(p[0] - last[0], p[1] - last[1])
        dist += step_d
        still = still + TS / 1000.0 if step_d < 1e-4 else 0.0
    last = p
    if int(t * 1000) % int(__import__('os').environ.get('JEVERY', '500')) < 16:
        print('TRUE t=%.3f axle=(%.3f,%.3f) yaw=%.2f' % (t, p[0]-0.005*math.cos(yaw), p[1]-0.005*math.sin(yaw), yaw))
    cps = node.getContactPoints(True)
    hitting = [c for c in cps if c.getPoint()[2] > 0.012]
    if hitting and not in_contact:
        coll += 1
        cp = hitting[0].getPoint()
        coll_log.append('t=%.1f at (%.3f,%.3f,%.3f)' % (t, cp[0], cp[1], cp[2]))
    in_contact = bool(hitting)
    if t_s1 is None and p[1] > 1.336:
        t_s1 = t
    if footprint_in_end(p, yaw) and still > 1.0:
        t_end = t - 1.0
        break
    if tilt > 45 or t > LIMIT:
        break

# sim_t + why = so a webots that got closed/killed mid run isnt blamed on the robot
why = 'finished' if t_end else ('tipped' if maxtilt > 45 else ('time_limit' if t > LIMIT else 'sim_ended'))
print('JUDGE mode=%s s1_time=%s end_time=%s dist=%.2f collisions=%d maxtilt=%.1f final=(%.3f,%.3f) sim_t=%.1f why=%s' % (
    mode, '%.1f' % t_s1 if t_s1 else None, '%.1f' % t_end if t_end else None, dist, coll, maxtilt, p[0], p[1], t, why))
for c in coll_log[:10]:
    print('JUDGE collision', c)
sys.stdout.flush()
sup.simulationQuit(0)

"""Neutral 'judge' supervisor: times every robot version the same way on the same arena."""
import math
from controller import Supervisor
s = Supervisor()
TS = int(s.getBasicTimeStep())
root = s.getRoot().getField('children')
bot = None
for i in range(root.getCount()):
    n = root.getMFNode(i)
    if n.getTypeName() in ('GuardianBot', 'TwoWheelRobot'):
        bot = n
print('JUDGE robot', bot.getTypeName() if bot else None, flush=True)
t = 0.0
marks = {}
coll, in_coll = 0, False
last_move, last_p = 0.0, None
while s.step(TS) != -1:
    t += TS / 1000.0
    x, y, z = bot.getPosition()
    if 's1' not in marks and y > 1.30:
        marks['s1'] = t; print('JUDGE s1 %.1f' % t, flush=True)
    if 's2' not in marks and y > 2.46:
        marks['s2'] = t; print('JUDGE s2 %.1f' % t, flush=True)
    if 'end' not in marks and math.hypot(x - 0.13, y - 3.05) < 0.10:
        marks['end'] = t; print('JUDGE end %.1f' % t, flush=True)
    hit = any(c.getPoint()[2] > 0.055 for c in bot.getContactPoints(True)) if t > 1 else False
    if hit and not in_coll:
        coll += 1; print('JUDGE collision %d at t=%.1f (%.2f, %.2f)' % (coll, t, x, y), flush=True)
    in_coll = hit
    if last_p is None or math.hypot(x - last_p[0], y - last_p[1]) > 0.02:
        last_p, last_move = (x, y), t
    if int(t * 1000) % 10000 < TS:
        print('JUDGE t=%.0f pos=(%.2f, %.2f) marks=%s collisions=%d' % (t, x, y, marks, coll), flush=True)
    if 'end' in marks and t - marks['end'] > 3 or t - last_move > 60 or t > 600:
        print('JUDGE RESULT %s collisions=%d stopped_at=(%.2f, %.2f) t=%.1f' % (marks, coll, x, y, t), flush=True)
        for _ in range(20):
            s.step(TS)
        s.simulationQuit(0)
        break

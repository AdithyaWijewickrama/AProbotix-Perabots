"""judge.py - neutral supervisor added only to the throwaway test copies (Phase B of docs/TEST_PLAN.md).

Records, without touching the robot:
  judge_path.csv   every 48 ms: t, x, y, yaw, v (m/s), w (rad/s) of the true robot
  judge_result.json section times, collisions (body contact above 5.5 cm), where every game piece ended,
                   where the robot stopped and why the run ended
Section marks: s1 = robot centre past y 1.30 (inside the maze), s2 = past y 2.46 (out of the maze),
end = within 10 cm of the END point (0.13, 3.05). The run ends 3 s after END, after 60 s without moving,
or at 600 s.
"""
import json
import math
from controller import Supervisor

s = Supervisor()
TS = int(s.getBasicTimeStep())
root = s.getRoot().getField("children")
bot, pieces = None, {}
for i in range(root.getCount()):
    n = root.getMFNode(i)
    if n.getTypeName() == "GuardianBot":
        bot = n
    elif n.getTypeName() == "Solid":
        f = n.getField("name")
        nm = f.getSFString() if f else ""
        if nm.startswith(("board_", "collect_")):
            pieces[nm] = n
start_pos = {k: n.getPosition()[:2] for k, n in pieces.items()}
out = open("judge_path.csv", "w")
out.write("t,x,y,yaw,v,w\n")
t, k = 0.0, 0
marks, colls, in_coll = {}, [], False
last_move, last_p = 0.0, None
reason = ""
while s.step(TS) != -1:
    t += TS / 1000.0
    k += 1
    x, y, _ = bot.getPosition()
    R = bot.getOrientation()
    yaw = math.atan2(R[3], R[0])
    vel = bot.getVelocity()
    if k % 3 == 0:
        v = vel[0] * math.cos(yaw) + vel[1] * math.sin(yaw)
        out.write("%.3f,%.4f,%.4f,%.4f,%.3f,%.3f\n" % (t, x, y, yaw, v, vel[5]))
    if "s1" not in marks and y > 1.30:
        marks["s1"] = round(t, 2)
    if "s2" not in marks and y > 2.46:
        marks["s2"] = round(t, 2)
    if "end" not in marks and math.hypot(x - 0.13, y - 3.05) < 0.10:
        marks["end"] = round(t, 2)
    hit = t > 1 and any(c.getPoint()[2] > 0.055 for c in bot.getContactPoints(True))
    if hit and not in_coll:
        colls.append([round(t, 2), round(x, 3), round(y, 3)])
    in_coll = hit
    if last_p is None or math.hypot(x - last_p[0], y - last_p[1]) > 0.02:
        last_p, last_move = (x, y), t
    if "end" in marks and t - marks["end"] > 3:
        reason = "reached END"
    elif t - last_move > 60:
        reason = "no movement for 60 s"
    elif t > 600:
        reason = "time limit 600 s"
    if reason:
        moved = {}
        for nm, n in pieces.items():
            px, py, pz = n.getPosition()
            moved[nm] = {"start": [round(v, 3) for v in start_pos[nm]], "end": [round(px, 3), round(py, 3), round(pz, 3)]}
        res = {"marks": marks, "collisions": colls, "stopped_at": [round(x, 3), round(y, 3)], "t": round(t, 2),
               "reason": reason, "pieces": moved}
        json.dump(res, open("judge_result.json", "w"), indent=1)
        out.close()
        print("JUDGE RESULT", json.dumps(res["marks"]), "collisions=%d" % len(colls), reason, flush=True)
        for _ in range(10):
            s.step(TS)
        s.simulationQuit(0)
        break

# builds test/TEST_REPORT.md and test/results/*.csv from the sim lab database.
#   python test/tools/report.py
# the "final" config = runs whose controller hash matches the current
# controllers/main_controller/main_controller.py exactly.
import csv
import hashlib
import json
import os
import sqlite3
import statistics as st
import sys
import time

H = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.abspath(os.path.join(H, '..'))
ROOT = os.path.abspath(os.path.join(TEST, '..'))
sys.path.insert(0, os.path.join(H, 'lab'))
import lab  # noqa: E402

con = sqlite3.connect(os.path.join(H, 'lab', 'lab.db'))
con.row_factory = sqlite3.Row
rows = [dict(r) for r in con.execute('select * from runs order by id')]
code = open(os.path.join(ROOT, 'controllers', 'main_controller', 'main_controller.py'), encoding='utf-8').read()
final_hash = hashlib.sha1(lab.patch(code, {}).encode()).hexdigest()[:10]

os.makedirs(os.path.join(TEST, 'results'), exist_ok=True)
cols = ['id', 'created', 'label', 'world_key', 'seed', 'mode', 'start', 'gen', 'ctrl_hash', 'params', 'outcome',
        'truth_blue', 'truth_green', 'det_blue', 'det_green', 'det_major', 's1_time', 'explored_time', 'end_time',
        'dist', 'collisions', 'maxtilt', 'tiles_seen', 'wiggles', 'view', 'status', 'note']
with open(os.path.join(TEST, 'results', 'all_runs.csv'), 'w', newline='', encoding='utf-8') as f:
    w = csv.writer(f)
    w.writerow(cols)
    for r in rows:
        w.writerow([r[c] for c in cols])


def latest(pred):
    out = {}
    for r in rows:
        if r['status'] == 'done' and pred(r):
            out[r['world_key']] = r
    return out


final = latest(lambda r: r['ctrl_hash'] == final_hash)
with open(os.path.join(TEST, 'results', 'final_controller.csv'), 'w', newline='', encoding='utf-8') as f:
    w = csv.writer(f)
    w.writerow(cols)
    for k in sorted(final):
        w.writerow([final[k][c] for c in cols])


def group(wk):
    if 'NTILES' in wk:
        return 'hard'
    return 'maze' if wk.startswith('dev_s2') or '_s2' in wk else 's1'


def f1(v):
    return '' if v is None else '%.1f' % v


L = []
P = L.append
fr = list(final.values())
ok = [r for r in fr if r['outcome'] == 'PASS']
P('# Test report – AProbotix Phase 1 controller')
P('')
P('Generated %s from the sim lab database (`test/tools/report.py`).' % time.strftime('%Y-%m-%d %H:%M'))
P('Controller tested: `controllers/main_controller/main_controller.py`, code hash `%s`.' % final_hash)
P('Webots R2025a. Every run was headless at fast mode, judged by a supervisor robot added only in the test copy of the world.')
P('')
P('## 1. Summary')
P('')
if fr:
    P('| | |')
    P('|---|---|')
    P('| Test worlds | %d |' % len(fr))
    P('| Passed (right LED colour, stopped in end cell, 0 collisions) | **%d / %d** |' % (len(ok), len(fr)))
    P('| Collisions in total | %d |' % sum(r['collisions'] or 0 for r in fr))
    P('| Max body tilt | %.1f deg |' % max(r['maxtilt'] or 0 for r in fr))
    P('| Tile counts exactly right | %d / %d |' % (sum(1 for r in fr if r['det_blue'] == r['truth_blue'] and r['det_green'] == r['truth_green']), len(fr)))
    for g, name in (('s1', 'S1 start zone (full run)'), ('hard', 'Hard arenas, S1 start (full run)'), ('maze', 'Maze entrance start')):
        t = [r['end_time'] for r in ok if group(r['world_key']) == g and r['end_time']]
        if t:
            P('| %s | mean %.1f s, min %.1f, max %.1f (%d worlds) |' % (name, st.mean(t), min(t), max(t), len(t)))
    s1 = [r['s1_time'] for r in ok if r['s1_time']]
    if s1:
        P('| Section 1 alone | mean %.1f s, min %.1f, max %.1f |' % (st.mean(s1), min(s1), max(s1)))
    if 'dev' in final:
        P('| Official dev arena | %s s total, Section 1 %s s |' % (f1(final['dev']['end_time']), f1(final['dev']['s1_time'])))
P('')
P('## 2. How we tested')
P('')
P('- **Worlds.** 45 in total, all in `test/worlds/` and openable in Webots:')
P('  - the official dev arena: S1 start, plus starts in the chamber and in c1r1;')
P('  - 21 random arenas with an S1 start and a random start heading;')
P('  - 11 of those again with the robot placed at the maze entrance (rule 4.6, Section 2 attempts);')
P('  - 10 "hard" arenas with 30-40 cubes, 8-9 tile slots and more maze loops.')
P('- **How the random arenas are built** (`test/tools/harness/gen.py`). They keep everything the rulebook fixes: outer walls, gap, chamber, 9x4 maze with 25 cm cells, 10 cm cubes.')
P('  - Cubes are placed so that a 25 cm wide passage from the start to the gap always exists.')
P('  - The maze is random, and tiles sit only at dead ends.')
P('  - The majority colour is never a tie.')
P('- **Judge** (`test/tools/harness/judge.py`). A supervisor robot that exists only in the test copies and is never in the submission. It records:')
P('  - the true position, distance travelled and maximum tilt;')
P('  - any contact with walls or cubes (counted as a collision);')
P('  - the Section 1 time (robot fully past y = 1.336);')
P('  - the finish time (whole body inside the end cell and stopped for 1 s).')
P('- **Pass rule.** A run passes only when all of these hold:')
P('  - the controller lit the majority colour equal to the truth;')
P('  - it finished in the end cell;')
P('  - it had 0 collisions;')
P('  - it tilted less than 10 deg;')
P('  - it did not crash.')
P('- **One run per world.** The simulation is deterministic, so the same world and the same settings always give the same result.')
P('')
P('## 3. Final controller, every world')
P('')
P('| world | truth B/G | seen B/G | result | S1 s | decision s | total s | dist m | coll | tilt |')
P('|---|---|---|---|---|---|---|---|---|---|')
for k in sorted(final, key=lambda k: (group(k), k)):
    r = final[k]
    P('| %s | %s/%s | %s/%s | %s | %s | %s | **%s** | %s | %s | %s |' % (
        k.replace(' LOOPS=0-5,NCUBES=30-40,NTILES=8-9', ' (hard)'), r['truth_blue'], r['truth_green'], r['det_blue'], r['det_green'],
        r['outcome'], f1(r['s1_time']), f1(r['explored_time']), f1(r['end_time']), '%.2f' % (r['dist'] or 0), r['collisions'], f1(r['maxtilt'])))
P('')
P('## 4. Settings we compared')
P('')
P('Each candidate was run on the same worlds as the reference, and only those shared worlds are compared.')
P('The reference is the controller before the last change ("default": R_INF 0.09, R_SOFT 0.15).')
P('')
base = latest(lambda r: r['label'] == 'default')
labs = sorted({r['label'] for r in rows if r['status'] == 'done'} - {'default'})
P('| config | worlds | passed | mean change vs reference | faster on | slower on |')
P('|---|---|---|---|---|---|')
for lb in labs:
    ws = latest(lambda r, lb=lb: r['label'] == lb)
    sh = [w for w in ws if w in base and ws[w]['end_time'] and base[w]['end_time']]
    if not sh:
        continue
    d = [ws[w]['end_time'] - base[w]['end_time'] for w in sh]
    pct = 100 * sum(d) / sum(base[w]['end_time'] for w in sh)
    P('| %s | %d | %d/%d | %+.2f s (%+.1f%%) | %d | %d |' % (
        lb, len(sh), sum(ws[w]['outcome'] == 'PASS' for w in sh), len(sh), st.mean(d), pct,
        sum(v < -0.05 for v in d), sum(v > 0.05 for v in d)))
P('')
P('Adopted: **R_INF 0.075 + R_SOFT 0.12** (now the final controller). Turn costs and the camera vote threshold gave no gain, so they were kept.')
P('')
P('Earlier tuning, done with the command-line harness before the lab existed:')
P('- acceleration MAX_DV 1.0 → 4.0 was about 10% faster; at 10 the robot rocked up to 9 deg;')
P('- exploration bias LAM: 1.0 beat 0.5 and 0 (0 was up to 33% slower on some mazes).')
P('')
P('## 5. Physics findings that shaped the design')
P('')
P('Scripts are in `test/tools/experiments/` (calib.py, calib2.py).')
P('')
P('- **Driving wheels first is unstable.** A heading error of 0.02 rad grew to 0.3-0.6 rad within 0.5 m, even with strong steering correction.')
P('- **Driving caster first locks onto the world axes.**')
P('  - Within about 20 deg of an axis the robot snapped back onto it.')
P('  - Errors of 0.03-0.06 rad decayed to 0.000 with no steering at all.')
P('  - The robot therefore drives caster first, and mostly in straight lines along the axes and diagonals.')
P('- **Odometry.**')
P('  - Straight runs with equal wheel speeds matched the true distance exactly.')
P('  - Steering while moving made the wheels slip: up to 45 cm of position error in Section 1.')
P('  - Pivots turn about a point about 2.5 cm from the axle, and that point is what the controller tracks.')
P('  - With all of this, the position error stayed under about 1 cm for a whole run.')
P('')
P('## 6. Problems found during testing (all fixed)')
P('')
P('- **Camera pointed at the ceiling.** The earlier version had the wrong tilt sign: R2025a cameras look along +X.')
P('- **Tiles "seen" through walls.** Dark wall faces were counted as floor of the cell behind them, which marked unseen cells as empty. Fixed by only counting pixels whose line of sight crosses known-open edges.')
P('- **Wrong start detection.** A cube near the start zone made the robot think it was at the maze entrance. Fixed by requiring both the east and south walls to be close.')
P('- **Webots closed mid-run.** One run (seed 22, hard) ended at sim time about 55 s because Webots closed underneath it. The same world passes when rerun, and the lab now reports such runs as ABORTED.')
P('')
P('## 7. Reproduce')
P('')
P('- **Sim lab** (web UI): double click `test/tools/lab/start_lab.bat`, open http://localhost:8765, pick worlds, run and compare.')
P('- **Command line:** `test/tools/harness/batch.ps1 -seeds 1,2,3` or `-mode s2`.')
P('- **Watch any world:** open `test/worlds/<name>.wbt` in Webots R2025a and press Play. These worlds use `test/controllers/test_runner`, which runs the real `controllers/main_controller/main_controller.py`.')
P('- **Raw data:** `test/results/final_controller.csv` and `test/results/all_runs.csv` (every run in the lab, including the setting sweeps).')
open(os.path.join(TEST, 'TEST_REPORT.md'), 'w', encoding='utf-8').write('\n'.join(L) + '\n')
print('final hash', final_hash, 'worlds', len(fr), 'pass', len(ok), '| runs in db', len(rows))

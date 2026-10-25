# AProbotix sim lab - local web app to generate/pick worlds, run the robot in
# webots with different settings, store everything in sqlite and compare.
#
#   python tools/lab/lab.py        then open http://localhost:8765
#
# only python stdlib, no pip install needed.

import hashlib
import json
import os
import re
import shutil
import sqlite3
import subprocess
import sys
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, '..', '..'))
GEN = os.path.join(ROOT, 'tools', 'harness', 'gen.py')
DB = os.path.join(HERE, 'lab.db')
RUNS = os.path.join(HERE, 'runs')
EXPORTS = os.path.join(HERE, 'exports')
PORT = int(os.environ.get('LAB_PORT') or os.environ.get('PORT') or '8765')

CTRL_SOURCES = {
    'root': os.path.join(ROOT, 'controllers', 'main_controller', 'main_controller.py'),
    'submission': os.path.join(ROOT, 'submission', 'controllers', 'main_controller', 'main_controller.py'),
}

# every tunable number in the controller. regex group 1 = the value, n = how
# many times it must match (so a code change cant silently break a param)
PARAMS = [
    ('V_MAX', 'Motion', r'^V_MAX = ([\d.]+)', 1, 'top speed m/s. motor cap is 0.2, more does nothing'),
    ('MAX_DV', 'Motion', r'^MAX_DV = ([\d.]+)', 1, 'wheel speed change per 16ms step (rad/s). accel/brake hardness'),
    ('ACC', 'Motion', r'^ACC = ([\d.]+)', 1, 'braking curve m/s^2 (when to start slowing down)'),
    ('W_RUN', 'Motion', r'^W_RUN = ([\d.]+)', 1, 'max turn rate while driving rad/s (0.15 = basically none)'),
    ('PIVOT_DECEL', 'Motion', r'math\.sqrt\(2 \* ([\d.]+) \* abs\(e\)\)', 1, 'how late pivots slow down (rad/s^2)'),
    ('PIVOT_WMAX', 'Motion', r'w = min\(([\d.]+), math\.sqrt', 1, 'max pivot speed rad/s (6.25 = wheel cap)'),
    ('R_HARD', 'Section 1', r'^R_HARD, R_INF, R_SOFT = ([\d.]+)', 1, 'A*: never closer than this to a cube (m)'),
    ('R_INF', 'Section 1', r'^R_HARD, R_INF, R_SOFT = [\d.]+, ([\d.]+)', 1, 'A*: heavy penalty below this (m)'),
    ('R_SOFT', 'Section 1', r'^R_HARD, R_INF, R_SOFT = [\d.]+, [\d.]+, ([\d.]+)', 1, 'A*: small penalty below this (m)'),
    ('LAM', 'Section 2', r'^LAM = ([\d.]+)', 1, 'explore far-from-end cells first. 0 = plain nearest'),
    ('TURN90_COST', 'Section 2', r'\(\(c, r, \(h \+ [13]\) % 4\), ([\d.]+)\)', 2, 'planner cost of a 90 deg pivot (1 cell = 1.0)'),
    ('TURN180_COST', 'Section 2', r'\(\(c, r, \(h \+ 2\) % 4\), ([\d.]+)\)', 1, 'planner cost of a 180 deg pivot'),
    ('VOTES_NEEDED', 'Camera', r'sum\(v\) >= (\d+)', 1, 'pixel votes before a cell colour is decided'),
    ('BLACK_MAX', 'Camera', r'if m < (\d+):', 1, 'pixel brighter than this is not black floor'),
    ('WATCHDOG_S', 'Safety', r't_sim - last_move > ([\d.]+)', 1, 'no-progress recovery after this many s'),
]

PRESETS_FILE = os.path.join(ROOT, 'test_worlds', 'truth.json')

db_lock = threading.Lock()


# ------------------------------------------------------------------ db
def db():
    con = sqlite3.connect(DB, timeout=30)
    con.row_factory = sqlite3.Row
    return con


def db_init():
    with db() as con:
        con.executescript('''
        create table if not exists runs (
            id integer primary key autoincrement,
            created text, finished text, status text,
            label text, batch text,
            world_key text, seed integer, mode text, start text, gen text,
            ctrl_src text, params text, ctrl_hash text, git_rev text,
            view text, time_limit real,
            truth_blue integer, truth_green integer,
            det_blue integer, det_green integer, det_major text,
            s1_time real, explored_time real, end_time real,
            dist real, collisions integer, maxtilt real, final text,
            tiles_seen integer, wiggles integer, outcome text, wall_s real, note text
        );
        create table if not exists settings (k text primary key, v text);
        ''')


def setting(k, default=None):
    with db() as con:
        r = con.execute('select v from settings where k=?', (k,)).fetchone()
    return r['v'] if r else default


def set_setting(k, v):
    with db_lock, db() as con:
        con.execute('insert or replace into settings values (?, ?)', (k, v))


def git_rev():
    try:
        return subprocess.run(['git', '-C', ROOT, 'rev-parse', '--short', 'HEAD'],
                              capture_output=True, text=True, timeout=10).stdout.strip()
    except Exception:
        return ''


# ------------------------------------------------------------------ params
def ctrl_path(src):
    return CTRL_SOURCES.get(src) or src


def read_defaults(src):
    text = open(ctrl_path(src), encoding='utf-8').read()
    out = {}
    for name, group, rx, n, desc in PARAMS:
        m = list(re.finditer(rx, text, re.M))
        out[name] = float(m[0].group(1)) if len(m) == n else None
    return out


def patch(text, overrides):
    for name, group, rx, n, desc in PARAMS:
        if name not in overrides or overrides[name] in (None, ''):
            continue
        ms = list(re.finditer(rx, text, re.M))
        if len(ms) != n:
            raise ValueError('param %s: expected %d match(es) in controller, found %d' % (name, n, len(ms)))
        v = overrides[name]
        val = str(int(float(v))) if name in ('VOTES_NEEDED', 'BLACK_MAX') else repr(float(v))
        for m in reversed(ms):
            text = text[:m.start(1)] + val + text[m.end(1):]
    return text


def clean_overrides(src, overrides):
    # only keep the ones that actually differ from the file
    d = read_defaults(src)
    out = {}
    for k, v in (overrides or {}).items():
        if v in (None, '') or k not in d:
            continue
        if d[k] is None or abs(float(v) - d[k]) > 1e-9:
            out[k] = float(v)
    return out


# ------------------------------------------------------------------ worlds
def presets():
    out = [{'key': 'dev', 'seed': 0, 'mode': 's1', 'start': '', 'group': 'dev arena'}]
    try:
        names = sorted(json.load(open(PRESETS_FILE)).keys())
    except Exception:
        names = []
    for n in names:
        if n.startswith('dev'):
            start = '0.22,1.455,1.5708' if 'c1r1' in n else ''
            out.append({'key': n, 'seed': 0, 'mode': 's2', 'start': start, 'group': 'dev arena'})
            continue
        m = re.match(r'seed(\d+)(_s2)?$', n)
        if m:
            out.append({'key': n, 'seed': int(m.group(1)), 'mode': 's2' if m.group(2) else 's1',
                        'start': '', 'group': 'maze-entrance start' if m.group(2) else 'S1 start'})
    return out


def world_key(seed, mode, start, gen):
    k = 'dev' if seed == 0 else 'seed%02d' % seed
    if mode == 's2':
        k += '_s2'
    if start:
        k += '@' + start
    g = {a: b for a, b in (gen or {}).items() if b}
    if g:
        k += ' ' + ','.join('%s=%s' % kv for kv in sorted(g.items()))
    return k


# ------------------------------------------------------------------ runner
queue = []
q_lock = threading.Lock()
current = {'id': None, 'proc': None, 'started': None, 'cancel': False}


def enqueue(spec):
    src = spec.get('ctrl_src') or 'root'
    if not os.path.exists(ctrl_path(src)):
        raise ValueError('controller not found: %s' % ctrl_path(src))
    ov = clean_overrides(src, spec.get('params'))
    code = patch(open(ctrl_path(src), encoding='utf-8').read(), ov)
    h = hashlib.sha1(code.encode()).hexdigest()[:10]
    label = (spec.get('label') or '').strip() or (
        'default' if not ov else ','.join('%s=%g' % kv for kv in sorted(ov.items())))
    if src != 'root' and not spec.get('label'):
        label += ' [%s]' % (src if src in CTRL_SOURCES else 'custom')
    batch = time.strftime('%m%d-%H%M%S')
    gen = {k: v for k, v in (spec.get('gen') or {}).items() if v}
    ids = []
    rev = git_rev()
    with db_lock, db() as con:
        for w in spec['worlds']:
            seed, mode, start = int(w['seed']), w.get('mode', 's1'), w.get('start', '')
            cur = con.execute(
                'insert into runs (created, status, label, batch, world_key, seed, mode, start, gen, ctrl_src, '
                'params, ctrl_hash, git_rev, view, time_limit) values (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)',
                (time.strftime('%Y-%m-%d %H:%M:%S'), 'queued', label, batch, world_key(seed, mode, start, gen),
                 seed, mode, start, json.dumps(gen), src, json.dumps(ov), h, rev,
                 spec.get('view', 'headless'), float(spec.get('time_limit') or 300)))
            ids.append(cur.lastrowid)
    with q_lock:
        queue.extend(ids)
    return ids


def update(rid, **kw):
    with db_lock, db() as con:
        con.execute('update runs set %s where id=?' % ','.join('%s=?' % k for k in kw), (*kw.values(), rid))


def webots_exe():
    home = setting('webots_home', r'D:\Programs\Webots')
    return home, os.path.join(home, 'msys64', 'mingw64', 'bin', 'webots-bin.exe')


def parse_logs(out):
    res = {}
    try:
        c = open(os.path.join(out, 'controllers', 'main_controller', 'ctrl_log.txt'), encoding='utf-8', errors='replace').read()
    except OSError:
        c = ''
    try:
        j = open(os.path.join(out, 'controllers', 'judge', 'judge_log.txt'), encoding='utf-8', errors='replace').read()
    except OSError:
        j = ''
    m = re.search(r'section 1 done t=([\d.]+)s', c)
    res['s1_ctrl'] = float(m.group(1)) if m else None
    m = re.search(r'explored\. blue=(\d+) green=(\d+) -> (\w+)\s+\(t=([\d.]+)s\)', c)
    if m:
        res.update(det_blue=int(m.group(1)), det_green=int(m.group(2)), det_major=m.group(3),
                   explored_time=float(m.group(4)))
    res['tiles_seen'] = len(re.findall(r'^tile ', c, re.M))
    res['wiggles'] = len(re.findall(r'wiggle', c))
    res['error'] = 'Traceback' in c
    m = re.search(r'JUDGE mode=\S+ s1_time=(\S+) end_time=(\S+) dist=([\d.]+) collisions=(\d+) maxtilt=([\d.]+) final=\(([^)]*)\)', j)
    if m:
        f = lambda s: None if s == 'None' else float(s)
        res.update(s1_time=f(m.group(1)), end_time=f(m.group(2)), dist=float(m.group(3)),
                   collisions=int(m.group(4)), maxtilt=float(m.group(5)), final=m.group(6))
    res['judged'] = bool(m)
    return res, c, j


def run_one(rid):
    with db() as con:
        r = dict(con.execute('select * from runs where id=?', (rid,)).fetchone())
    if r['status'] != 'queued':
        return
    out = os.path.join(RUNS, 'run_%d' % rid)
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(RUNS, exist_ok=True)
    t0 = time.time()
    update(rid, status='running')
    try:
        code = patch(open(ctrl_path(r['ctrl_src']), encoding='utf-8').read(), json.loads(r['params']))
        cfile = out + '_ctrl.py'
        open(cfile, 'w', encoding='utf-8').write(code)
        env = dict(os.environ)
        for k in ('START', 'NOOBS', 'NCUBES', 'NTILES', 'LOOPS', 'LAM', 'EST'):
            env.pop(k, None)
        env['CTRL'] = cfile
        if r['start']:
            env['START'] = r['start']
        for k, v in json.loads(r['gen']).items():
            env[k] = str(v)
        args = [sys.executable, GEN, ROOT, out, str(r['seed'])] + (['s2'] if r['mode'] == 's2' else [])
        g = subprocess.run(args, env=env, capture_output=True, text=True, timeout=300)
        if g.returncode:
            raise RuntimeError('world generator failed:\n' + g.stderr[-2000:])
        truth = json.load(open(os.path.join(out, 'truth.json')))
        update(rid, truth_blue=truth.get('blue'), truth_green=truth.get('green'))
        home, exe = webots_exe()
        if not os.path.exists(exe):
            raise RuntimeError('webots not found at %s (set it in Settings)' % exe)
        env['WEBOTS_HOME'] = home
        b = os.path.join(home, 'msys64', 'mingw64', 'bin')
        env['PATH'] = os.pathsep.join([b, os.path.join(b, 'cpp'), os.path.join(home, 'msys64', 'usr', 'bin'), env['PATH']])
        env['JLIMIT'] = str(r['time_limit'])
        env['JEVERY'] = '100000000'
        view = r['view']
        if view == 'headless':
            wargs = ['--batch', '--mode=fast', '--no-rendering', '--minimize']
        elif view == 'visible_fast':
            wargs = ['--batch', '--mode=fast']
        else:
            wargs = ['--batch', '--mode=realtime']
        p = subprocess.Popen([exe] + wargs + [os.path.join(out, 'worlds', 'test.wbt')], env=env)
        current.update(proc=p)
        wall = 900 if view != 'realtime' else r['time_limit'] * 1.5 + 120
        while p.poll() is None:
            if current['cancel'] or time.time() - t0 > wall:
                p.kill()
                break
            time.sleep(0.5)
        res, c, j = parse_logs(out)
        status = 'cancelled' if current['cancel'] else ('done' if res['judged'] else 'failed')
        tb, tg = truth.get('blue', 0), truth.get('green', 0)
        want = 'BLUE' if tb > tg else 'GREEN'
        fails = []
        if res['error']:
            fails.append('CRASH')
        if not res['judged']:
            fails.append('NO RESULT')
        else:
            if res.get('det_major') != want:
                fails.append('WRONG COLOUR' if res.get('det_major') else 'NO DECISION')
            if res.get('end_time') is None:
                fails.append('NOT FINISHED')
            if res.get('collisions'):
                fails.append('COLLISION')
            if (res.get('maxtilt') or 0) > 10:
                fails.append('TILT')
        update(rid, status=status, finished=time.strftime('%Y-%m-%d %H:%M:%S'),
               det_blue=res.get('det_blue'), det_green=res.get('det_green'), det_major=res.get('det_major'),
               s1_time=res.get('s1_time') if r['mode'] == 's1' else None,
               explored_time=res.get('explored_time'), end_time=res.get('end_time'), dist=res.get('dist'),
               collisions=res.get('collisions'), maxtilt=res.get('maxtilt'), final=res.get('final'),
               tiles_seen=res.get('tiles_seen'), wiggles=res.get('wiggles'),
               outcome='PASS' if not fails else ' / '.join(fails), wall_s=round(time.time() - t0, 1))
    except Exception as e:
        update(rid, status='failed', outcome='ERROR', note=str(e)[:2000], wall_s=round(time.time() - t0, 1),
               finished=time.strftime('%Y-%m-%d %H:%M:%S'))
        traceback.print_exc()


def worker():
    while True:
        rid = None
        with q_lock:
            if queue:
                rid = queue.pop(0)
        if rid is None:
            time.sleep(0.5)
            continue
        current.update(id=rid, started=time.time(), cancel=False, proc=None)
        run_one(rid)
        current.update(id=None, proc=None, started=None, cancel=False)


# ------------------------------------------------------------------ analysis
STATIC_TIPS = [
    'Only trust a change after it passes at least 10 S1-start worlds and 5 maze-entrance worlds with 100% PASS. One lucky world means nothing.',
    'Compare configs on the SAME worlds (the Compare tab only uses shared worlds for the averages). Sim time is deterministic, so one run per world is enough.',
    'V_MAX above 0.2 does nothing: the motors are capped at 10 rad/s. Speed comes from MAX_DV/ACC/PIVOT_DECEL and from the planner.',
    'MAX_DV 3-4 was the sweet spot in our tests. At 10 the robot rocked to 9 deg and got slower. Watch the Tilt column: anything over ~3 deg means the robot is bouncing.',
    'Keep W_RUN tiny. Steering while driving makes the wheels slip and the position estimate drifts (we measured 45 cm of error before).',
    'R_HARD below ~0.06 m lets A* plan paths where a pivot can swing the body into a cube. Lower it only if collisions stay at 0 on 20+ worlds.',
    'LAM between 0.5 and 1.5 all worked. 0 (plain nearest-first) was up to 33% slower on some mazes.',
    'VOTES_NEEDED lower = decides tile colours sooner but with fewer pixels. Keep it at 10 or more unless WRONG COLOUR stays at 0 across many worlds.',
    'Stress test with harder worlds: NCUBES 40-45 (dense S1), LOOPS 0 (more dead ends), NTILES 7 (more tiles).',
    'Watch a few runs in "visible realtime" mode now and then. Headless numbers can hide odd behaviour (shaking, scraping walls).',
    'When a config wins: copy the numbers into controllers/main_controller/main_controller.py, re-run the regression, then copy it into submission/ with the comments stripped and rebuild the zip.',
]


def recommend():
    with db() as con:
        rows = [dict(r) for r in con.execute("select * from runs where status='done'")]
    msgs = []
    if not rows:
        return {'data': ['No finished runs yet. Run the "default" config on a set of worlds first, then try variations.'],
                'tips': STATIC_TIPS, 'table': []}
    by = {}
    for r in rows:
        wid = r['world_key']
        by.setdefault(r['label'], {})[wid] = r          # latest run per world wins (ids ascend)
    table = []
    for lab, ws in by.items():
        rs = list(ws.values())
        n = len(rs)
        ok = [r for r in rs if r['outcome'] == 'PASS']
        times = [r['end_time'] for r in ok if r['end_time']]
        table.append({'label': lab, 'worlds': n, 'pass': len(ok),
                      'mean_end': round(sum(times) / len(times), 1) if times else None,
                      'worst_tilt': max((r['maxtilt'] or 0) for r in rs),
                      'collisions': sum((r['collisions'] or 0) for r in rs),
                      'fails': sorted({r['outcome'] for r in rs if r['outcome'] != 'PASS'})})
    base = 'default' if 'default' in by else None
    for t in table:
        if t['pass'] < t['worlds']:
            msgs.append('"%s" FAILED on %d of %d worlds (%s). Do not submit it.' % (
                t['label'], t['worlds'] - t['pass'], t['worlds'], ', '.join(t['fails'])))
        if t['worst_tilt'] > 3:
            msgs.append('"%s" tilts up to %.1f deg: the robot is rocking. Lower MAX_DV / ACC.' % (t['label'], t['worst_tilt']))
        if t['worlds'] < 10:
            msgs.append('"%s" has only %d worlds tested. Run more (10+ S1, 5+ maze-entrance) before trusting it.' % (t['label'], t['worlds']))
    if base:
        best = None
        for lab, ws in by.items():
            if lab == base:
                continue
            shared = [w for w in ws if w in by[base]]
            if len(shared) < 3:
                continue
            all_pass = all(ws[w]['outcome'] == 'PASS' for w in shared)
            pairs = [(ws[w]['end_time'], by[base][w]['end_time']) for w in shared
                     if ws[w]['end_time'] and by[base][w]['end_time']]
            if not pairs:
                continue
            d = sum(a - b for a, b in pairs) / len(pairs)
            msgs.append('"%s" vs default on %d shared worlds: %+.1f s per run (%s).' % (
                lab, len(pairs), d, 'all PASS' if all_pass else 'HAS FAILURES'))
            if all_pass and d < 0 and (best is None or d < best[1]):
                best = (lab, d, len(shared))
        if best:
            msgs.insert(0, 'Best so far: "%s", %.1f s faster than default on %d shared worlds with no failures. %s' % (
                best[0], -best[1], best[2],
                'Test it on more worlds before adopting it.' if best[2] < 15 else 'Good candidate to adopt.'))
        else:
            msgs.insert(0, 'No config beats "default" yet without failures (on at least 3 shared worlds).')
    else:
        msgs.insert(0, 'Run a "default" config (no parameter changes) on your worlds so others can be compared to it.')
    return {'data': msgs, 'tips': STATIC_TIPS, 'table': sorted(table, key=lambda t: (-t['pass'] / max(t['worlds'], 1), t['mean_end'] or 1e9))}


# ------------------------------------------------------------------ http
class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def send(self, obj, code=200, ctype='application/json'):
        body = obj if isinstance(obj, (bytes, bytearray)) else json.dumps(obj).encode()
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def body(self):
        n = int(self.headers.get('Content-Length') or 0)
        return json.loads(self.rfile.read(n) or b'{}')

    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        try:
            if u.path in ('/', '/index.html'):
                return self.send(open(os.path.join(HERE, 'index.html'), 'rb').read(), ctype='text/html; charset=utf-8')
            if u.path == '/api/meta':
                srcs = {k: read_defaults(k) for k in CTRL_SOURCES if os.path.exists(CTRL_SOURCES[k])}
                return self.send({'params': [{'name': p[0], 'group': p[1], 'desc': p[4]} for p in PARAMS],
                                  'defaults': srcs, 'presets': presets(),
                                  'settings': {'webots_home': webots_exe()[0]}, 'root': ROOT})
            if u.path == '/api/defaults':
                return self.send(read_defaults(q.get('src', 'root')))
            if u.path == '/api/runs':
                with db() as con:
                    rows = [dict(r) for r in con.execute('select * from runs order by id desc limit ?', (int(q.get('limit', 2000)),))]
                return self.send(rows)
            if u.path == '/api/status':
                with q_lock:
                    ql = len(queue)
                return self.send({'running': current['id'], 'queued': ql,
                                  'elapsed': round(time.time() - current['started'], 1) if current['started'] else None})
            if u.path == '/api/log':
                out = os.path.join(RUNS, 'run_%d' % int(q['id']))
                res, c, j = parse_logs(out)
                with db() as con:
                    r = dict(con.execute('select * from runs where id=?', (int(q['id']),)).fetchone())
                return self.send({'ctrl': c[-20000:], 'judge': j, 'note': r.get('note'), 'run': r})
            if u.path == '/api/recommend':
                return self.send(recommend())
            return self.send({'error': 'not found'}, 404)
        except Exception as e:
            traceback.print_exc()
            return self.send({'error': str(e)}, 500)

    def do_POST(self):
        u = urlparse(self.path)
        try:
            b = self.body()
            if u.path == '/api/run':
                if not b.get('worlds'):
                    return self.send({'error': 'pick at least one world'}, 400)
                return self.send({'ids': enqueue(b)})
            if u.path == '/api/cancel':
                if b.get('all'):
                    with q_lock:
                        ids = list(queue)
                        queue.clear()
                    for i in ids:
                        update(i, status='cancelled', outcome='CANCELLED')
                current['cancel'] = True
                return self.send({'ok': True})
            if u.path == '/api/delete':
                ids = [int(i) for i in b.get('ids', [])]
                with q_lock:
                    for i in ids:
                        if i in queue:
                            queue.remove(i)
                with db_lock, db() as con:
                    con.executemany('delete from runs where id=? and status != "running"', [(i,) for i in ids])
                for i in ids:
                    shutil.rmtree(os.path.join(RUNS, 'run_%d' % i), ignore_errors=True)
                    try:
                        os.remove(os.path.join(RUNS, 'run_%d_ctrl.py' % i))
                    except OSError:
                        pass
                return self.send({'ok': True})
            if u.path == '/api/relabel':
                with db_lock, db() as con:
                    con.executemany('update runs set label=? where id=?', [(b['label'], int(i)) for i in b['ids']])
                return self.send({'ok': True})
            if u.path == '/api/open':
                # open a finished run's world in the webots window to look at it
                out = os.path.join(RUNS, 'run_%d' % int(b['id']))
                w = os.path.join(out, 'worlds', 'test.wbt')
                if not os.path.exists(w):
                    return self.send({'error': 'run folder is gone'}, 404)
                home, exe = webots_exe()
                env = dict(os.environ, WEBOTS_HOME=home, JLIMIT='100000')
                bp = os.path.join(home, 'msys64', 'mingw64', 'bin')
                env['PATH'] = os.pathsep.join([bp, os.path.join(bp, 'cpp'), os.path.join(home, 'msys64', 'usr', 'bin'), env['PATH']])
                subprocess.Popen([exe, '--mode=pause', w], env=env)
                return self.send({'ok': True})
            if u.path == '/api/export':
                with db() as con:
                    r = dict(con.execute('select * from runs where id=?', (int(b['id']),)).fetchone())
                code = patch(open(ctrl_path(r['ctrl_src']), encoding='utf-8').read(), json.loads(r['params']))
                os.makedirs(EXPORTS, exist_ok=True)
                name = re.sub(r'[^A-Za-z0-9_.=-]+', '_', r['label'])[:80] + '.py'
                p = os.path.join(EXPORTS, name)
                open(p, 'w', encoding='utf-8').write(code)
                return self.send({'path': p})
            if u.path == '/api/settings':
                if b.get('webots_home'):
                    set_setting('webots_home', b['webots_home'])
                return self.send({'ok': True})
            return self.send({'error': 'not found'}, 404)
        except Exception as e:
            traceback.print_exc()
            return self.send({'error': str(e)}, 400)


def main():
    db_init()
    # anything left "running"/"queued" from a previous session gets re-queued
    with db_lock, db() as con:
        con.execute("update runs set status='queued' where status='running'")
        left = [r['id'] for r in con.execute("select id from runs where status='queued' order by id")]
    queue.extend(left)
    threading.Thread(target=worker, daemon=True).start()
    srv = ThreadingHTTPServer(('127.0.0.1', PORT), H)
    print('AProbotix sim lab on http://localhost:%d  (db: %s)' % (PORT, DB), flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

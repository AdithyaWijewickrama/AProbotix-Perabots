"""Build a throwaway Webots test project.
usage: python gen.py <project_src> <out_dir> <seed> [s2]
seed 0 = the dev layout unchanged. Otherwise random maze, tiles, obstacles,
start pose. 's2' starts the robot in the chamber (S2-only attempt)."""
import sys, os, re, random, math, shutil, json
from collections import deque

src, out, seed = sys.argv[1], sys.argv[2], int(sys.argv[3])
s2 = len(sys.argv) > 4 and sys.argv[4] == 's2'
rng = random.Random(seed)
for d in ('worlds', 'protos', 'controllers/main_controller', 'controllers/judge'):
    os.makedirs(os.path.join(out, d), exist_ok=True)

arena = open(os.path.join(src, 'protos/Arena.proto')).read()
obst = open(os.path.join(src, 'protos/Obstacles.proto')).read()
shutil.copy(os.path.join(src, 'protos/TwoWheelRobot.proto'), os.path.join(out, 'protos'))
code = open(os.environ.get('CTRL') or os.path.join(src, 'controllers/main_controller/main_controller.py')).read()
if os.environ.get('LAM'):
    code = re.sub(r'^LAM = [\d.]+', 'LAM = %s' % os.environ['LAM'], code, flags=re.M)
if os.environ.get('EST'):
    # per-step pose log for ana.py / tb.py / route.py (big files)
    code = code.replace(
        "    t_sim += TS / 1000.0\n",
        "    t_sim += TS / 1000.0\n    print('EST t=%.3f axle=(%.3f,%.3f) yaw=%.2f cmd=%.1f,%.1f enc=%.1f,%.1f' % (t_sim, px, py, yaw, _last_l, _last_r, le.getValue(), re_.getValue()))\n")
open(os.path.join(out, 'controllers/main_controller/main_controller.py'), 'w').write(
    "import sys\nsys.stdout = sys.stderr = open('ctrl_log.txt', 'w', buffering=1)\n" + code)
shutil.copy(os.path.join(os.path.dirname(__file__), 'judge.py'), os.path.join(out, 'controllers/judge'))

def solid(name, x, y, z, sx, sy, sz, color=None, bound=True):
    col = 'baseColor %s' % color if color else ''
    b = 'boundingObject Box { size %g %g %g }' % (sx, sy, sz) if bound else ''
    return ('Solid { translation %g %g %g children [ Shape { appearance PBRAppearance { %s roughness 1 metalness 0 } '
            'geometry Box { size %g %g %g } } ] name "%s" %s }\n' % (x, y, z, col, sx, sy, sz, name, b))

def strip_solids(text, pat):
    # remove Solid blocks whose name matches pat (brace matching)
    out_, i = '', 0
    while True:
        j = text.find('Solid {', i)
        if j < 0:
            return out_ + text[i:]
        depth, k = 0, j
        while True:
            if text[k] == '{': depth += 1
            elif text[k] == '}':
                depth -= 1
                if depth == 0: break
            k += 1
        blk = text[j:k + 1]
        m = re.search(r'name "([^"]+)"', blk)
        nm = m.group(1) if m else ''
        out_ += text[i:j]
        if not re.match(pat, nm):
            out_ += blk
        i = k + 1

truth = {}
COLS, ROWS = 9, 4
if seed == 0:
    # dev layout: compute truth from names
    truth['blue'] = len(re.findall(r'maze_tile_blue', arena))
    truth['green'] = len(re.findall(r'maze_tile_green', arena))
    open(os.path.join(out, 'protos/Arena.proto'), 'w').write(arena)
    open(os.path.join(out, 'protos/Obstacles.proto'), 'w').write(obst)
else:
    # ---- maze: random spanning tree + a few extra openings
    walls_v = {(k, r): True for k in range(1, COLS) for r in range(ROWS)}   # line x=k between col k-1,k
    walls_h = {(c, k): True for c in range(COLS) for k in range(1, ROWS)}   # line y=k between row k-1,k
    seen = {(0, 0)}
    st = [(0, 0)]
    while st:
        c, r = st[-1]
        nb = [(c + dc, r + dr) for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1))
              if 0 <= c + dc < COLS and 0 <= r + dr < ROWS and (c + dc, r + dr) not in seen]
        if not nb:
            st.pop(); continue
        n = rng.choice(nb)
        if n[0] != c: walls_v[(max(c, n[0]), r)] = False
        else: walls_h[(c, max(r, n[1]))] = False
        seen.add(n); st.append(n)
    extra = rng.randint(0, 2)
    keys = [k for k, v in list(walls_v.items()) + list(walls_h.items()) if v]
    for _ in range(extra):
        k = rng.choice(keys)
        if rng.random() < 0.5 and k in walls_v: walls_v[k] = False
        elif k in walls_h: walls_h[k] = False
    def opens(c, r):
        n = 0
        if c > 0 and not walls_v[(c, r)]: n += 1
        if c < COLS - 1 and not walls_v[(c + 1, r)]: n += 1
        if r > 0 and not walls_h[(c, r)]: n += 1
        if r < ROWS - 1 and not walls_h[(c, r + 1)]: n += 1
        return n
    deads = [(c, r) for c in range(COLS) for r in range(ROWS) if opens(c, r) == 1 and (c, r) not in ((0, 0), (8, 3))]
    rng.shuffle(deads)
    nt = min(len(deads), rng.randint(3, 7))
    if nt % 2 == 0 and nt > 1: nt -= 1
    tiles = deads[:nt]
    nb_ = rng.randint(0, nt)
    if nb_ * 2 == nt: nb_ += 1
    cols = ['blue'] * nb_ + ['green'] * (nt - nb_)
    rng.shuffle(cols)
    truth = {'blue': cols.count('blue'), 'green': cols.count('green'), 'tiles': {'%d,%d' % t: c for t, c in zip(tiles, cols)}}
    body = ''
    for (k, r), w in walls_v.items():
        if w: body += solid('maze_wall_v_%d_%d' % (k, r), 0.095 + 0.25 * k, 1.33 + 0.25 * (r + 0.5), 0.05, 0.012, 0.262, 0.1)
    for (c, k), w in walls_h.items():
        if w: body += solid('maze_wall_h_%d_%d' % (c, k), 0.095 + 0.25 * (c + 0.5), 1.33 + 0.25 * k, 0.05, 0.262, 0.012, 0.1)
    for (c, r), col in zip(tiles, cols):
        body += solid('maze_tile_%s_%d_%d' % (col, c, r), 0.22 + 0.25 * c, 1.455 + 0.25 * r, 0.001, 0.25, 0.25, 0.001,
                      '0 0 0.5' if col == 'blue' else '0 0.6 0', bound=False)
    a2 = strip_solids(arena, r'maze_(wall|tile)_')
    i = a2.rindex(']')
    a2 = a2[:i] + body + a2[i:]
    open(os.path.join(out, 'protos/Arena.proto'), 'w').write(a2)
    # ---- obstacles: random cubes, keep a 0.25 passage from start to the gap
    G = 0.02
    NX, NY = 122, 61
    def ok(cubes):
        # config space for a disc of radius 0.12
        free = [[True] * NY for _ in range(NX)]
        for i in range(NX):
            for j in range(NY):
                x, y = (i + .5) * G, (j + .5) * G
                if x < 0.126 or x > 2.314 or y < 0.126:
                    free[i][j] = False; continue
                if y > 1.094 and not (0.22 - 0.01 <= x <= 0.22 + 0.01):
                    free[i][j] = False; continue
                for cx, cy in cubes:
                    dx = max(abs(x - cx) - 0.05, 0); dy = max(abs(y - cy) - 0.05, 0)
                    if dx * dx + dy * dy < 0.12 ** 2:
                        free[i][j] = False; break
        s = (int(2.3 / G), int(0.13 / G)); g = (int(0.22 / G), NY - 1)
        if not free[s[0]][s[1]]: return False
        q = deque([s]); vis = {s}
        while q:
            i, j = q.popleft()
            if j >= int(1.1 / G) and abs((i + .5) * G - 0.22) < 0.02: return True
            for a, b in ((i+1,j),(i-1,j),(i,j+1),(i,j-1)):
                if 0 <= a < NX and 0 <= b < NY and (a, b) not in vis and free[a][b]:
                    vis.add((a, b)); q.append((a, b))
        return False
    cubes = []
    tries = 0
    target = rng.randint(25, 45)
    while len(cubes) < target and tries < 400:
        tries += 1
        if cubes and rng.random() < 0.6:
            bx, by = rng.choice(cubes)
            ang = rng.choice([0, math.pi / 2, math.pi, -math.pi / 2])
            nx_, ny_ = bx + 0.1 * math.cos(ang) + rng.uniform(-0.03, 0.03), by + 0.1 * math.sin(ang) + rng.uniform(-0.03, 0.03)
        else:
            nx_, ny_ = rng.uniform(0.06, 2.38), rng.uniform(0.06, 1.16)
        if not (0.055 < nx_ < 2.385 and 0.055 < ny_ < 1.165): continue
        if nx_ > 2.1 and ny_ < 0.34: continue                 # start zone
        if any(abs(nx_ - a) < 0.1 and abs(ny_ - b) < 0.1 for a, b in cubes): continue
        if ok(cubes + [(nx_, ny_)]):
            cubes.append((nx_, ny_))
    ob = 'PROTO Obstacles [\n] {\n  Group {\n    children [\n'
    for n, (x, y) in enumerate(cubes):
        ob += solid('obstacle_%d' % n, round(x, 3), round(y, 3), 0.05, 0.1, 0.1, 0.1, '0.02 0.02 0.02')
    ob += '    ]\n  }\n}\n'
    open(os.path.join(out, 'protos/Obstacles.proto'), 'w').write('#VRML_SIM R2025a utf8\n' + ob)
    truth['cubes'] = len(cubes)

if os.environ.get('NOOBS'):
    open(os.path.join(out, 'protos/Obstacles.proto'), 'w').write('#VRML_SIM R2025a utf8\nPROTO Obstacles [\n] {\n  Group { }\n}\n')
if os.environ.get('START'):
    sx, sy, sa = map(float, os.environ['START'].split(','))
elif s2:
    sx, sy, sa = 0.22 + rng.uniform(-0.01, 0.01), 1.275, math.pi / 2 + rng.uniform(-0.05, 0.05)
elif seed == 0:
    sx, sy, sa = 2.315, 0.125, math.pi / 2
else:
    sx, sy, sa = 2.315 + rng.uniform(-0.04, 0.04), 0.125 + rng.uniform(-0.04, 0.04), rng.uniform(-math.pi, math.pi)
# robot origin is 2cm behind the axle
ox, oy = sx - 0.02 * math.cos(sa), sy - 0.02 * math.sin(sa)
world = '''#VRML_SIM R2025a utf8
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "../protos/Arena.proto"
EXTERNPROTO "../protos/Obstacles.proto"
EXTERNPROTO "../protos/TwoWheelRobot.proto"
WorldInfo { basicTimeStep 16 }
Viewpoint { orientation 0.32 0.13 -0.94 2.5 position 4.9 3.8 3.3 }
TexturedBackground { texture "empty_office" }
TexturedBackgroundLight { texture "empty_office" }
DirectionalLight { direction 0.3 0.4 -1 intensity 2.2 castShadows TRUE }
Arena { }
Obstacles { }
DEF ROBOT TwoWheelRobot { translation %f %f 0.001 rotation 0 0 1 %f name "team_robot_template" controller "main_controller" bodyColor 1 0.55 0 }
Robot { name "judge" supervisor TRUE controller "judge" controllerArgs [ "%s" ] }
''' % (ox, oy, sa, 's2' if s2 else 's1')
open(os.path.join(out, 'worlds/test.wbt'), 'w').write(world)
json.dump(truth, open(os.path.join(out, 'truth.json'), 'w'))
print('truth', json.dumps(truth))

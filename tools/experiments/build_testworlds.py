import os, re, json, shutil, subprocess, sys
H = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(H, '..', '..'))
OUT = os.path.join(REPO, 'test_worlds')
TMP = os.path.join(REPO, 'tools', 'harness', 'runs', 'tw_tmp')
shutil.rmtree(OUT, ignore_errors=True)
shutil.rmtree(TMP, ignore_errors=True)
for d in ('worlds', 'protos', 'controllers/main_controller'):
    os.makedirs(os.path.join(OUT, d))
shutil.copy(os.path.join(REPO, 'protos/TwoWheelRobot.proto'), os.path.join(OUT, 'protos'))
shutil.copy(os.path.join(REPO, 'controllers/main_controller/main_controller.py'),
            os.path.join(OUT, 'controllers/main_controller'))

jobs = [(s, '', None) for s in range(1, 22)]
jobs += [(s, 's2', None) for s in (1, 2, 3, 4, 5, 7, 9, 10, 13, 16, 19)]
jobs += [(0, 's2', None), (0, 's2c1r1', '0.22,1.455,1.5708')]
index = {}
env0 = dict(os.environ)
for env_k in ('CTRL', 'START', 'NOOBS', 'LAM'):
    env0.pop(env_k, None)
for seed, mode, start in jobs:
    tmp = os.path.join(TMP, 'r%d%s' % (seed, mode))
    env = dict(env0)
    if start:
        env['START'] = start
    args = [sys.executable, os.path.join(REPO, 'tools', 'harness', 'gen.py'), REPO, tmp, str(seed)]
    if mode.startswith('s2'):
        args.append('s2')
    subprocess.run(args, check=True, env=env, capture_output=True)
    tag = 'seed%02d' % seed if seed else 'dev'
    name = tag + ('' if not mode else '_' + mode)
    # per-layout protos get their own names so they can live in one project
    A, O = 'Arena_' + tag, 'Obstacles_' + tag
    if seed and not os.path.exists(os.path.join(OUT, 'protos', A + '.proto')):
        a = open(os.path.join(tmp, 'protos/Arena.proto')).read().replace('PROTO Arena [', 'PROTO %s [' % A)
        o = open(os.path.join(tmp, 'protos/Obstacles.proto')).read().replace('PROTO Obstacles [', 'PROTO %s [' % O)
        open(os.path.join(OUT, 'protos', A + '.proto'), 'w').write(a)
        open(os.path.join(OUT, 'protos', O + '.proto'), 'w').write(o)
    if not seed:
        A, O = 'Arena', 'Obstacles'
        for f in ('Arena.proto', 'Obstacles.proto'):
            shutil.copy(os.path.join(REPO, 'protos', f), os.path.join(OUT, 'protos', f))
    w = open(os.path.join(tmp, 'worlds/test.wbt')).read()
    w = re.sub(r'Robot \{ name "judge".*\n', '', w)          # no judge in the viewable worlds
    w = w.replace('"../protos/Arena.proto"', '"../protos/%s.proto"' % A)
    w = w.replace('"../protos/Obstacles.proto"', '"../protos/%s.proto"' % O)
    w = w.replace('\nArena { }', '\n%s { }' % A).replace('\nObstacles { }', '\n%s { }' % O)
    w = w.replace('WorldInfo { basicTimeStep 16 }', 'WorldInfo { title "%s" basicTimeStep 16 }' % name)
    open(os.path.join(OUT, 'worlds', name + '.wbt'), 'w').write(w)
    index[name] = json.load(open(os.path.join(tmp, 'truth.json')))
json.dump(index, open(os.path.join(OUT, 'truth.json'), 'w'), indent=1, sort_keys=True)
shutil.rmtree(TMP, ignore_errors=True)
print(len(index), 'worlds')

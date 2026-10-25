# rebuilds the test worlds in test/worlds + test/protos from their seeds.
# the worlds use the project's real protos/TwoWheelRobot.proto and the
# test_runner controller (which just runs controllers/main_controller), so the
# project still has exactly ONE TwoWheelRobot.proto and ONE main_controller.
import os, re, json, shutil, subprocess, sys
H = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.abspath(os.path.join(H, '..', '..'))
ROOT = os.path.abspath(os.path.join(TEST, '..'))
TMP = os.path.join(TEST, 'tools', 'harness', 'runs', 'tw_tmp')
for d in ('worlds', 'protos'):
    shutil.rmtree(os.path.join(TEST, d), ignore_errors=True)
    os.makedirs(os.path.join(TEST, d))
shutil.rmtree(TMP, ignore_errors=True)

jobs = [(s, '', None) for s in range(1, 22)]
jobs += [(s, 's2', None) for s in (1, 2, 3, 4, 5, 7, 9, 10, 13, 16, 19)]
jobs += [(0, 's2', None), (0, 's2c1r1', '0.22,1.455,1.5708')]
HARD = {'LOOPS': '0-5', 'NCUBES': '30-40', 'NTILES': '8-9'}
jobs += [(s, 'hard', None) for s in range(22, 32)]
index = {}
env0 = {k: v for k, v in os.environ.items() if k not in ('CTRL', 'START', 'NOOBS', 'LAM', 'EST', 'LOOPS', 'NCUBES', 'NTILES')}
for seed, mode, start in jobs:
    tmp = os.path.join(TMP, 'r%d%s' % (seed, mode))
    env = dict(env0)
    if start:
        env['START'] = start
    if mode == 'hard':
        env.update(HARD)
    args = [sys.executable, os.path.join(TEST, 'tools', 'harness', 'gen.py'), ROOT, tmp, str(seed)]
    if mode.startswith('s2'):
        args.append('s2')
    subprocess.run(args, check=True, env=env, capture_output=True)
    tag = ('seed%02d' % seed if seed else 'dev') + ('_hard' if mode == 'hard' else '')
    name = tag + ('_' + mode if mode.startswith('s2') else '')
    A, O = 'Arena_' + tag, 'Obstacles_' + tag
    if not os.path.exists(os.path.join(TEST, 'protos', A + '.proto')):
        a = open(os.path.join(tmp, 'protos/Arena.proto')).read().replace('PROTO Arena [', 'PROTO %s [' % A)
        o = open(os.path.join(tmp, 'protos/Obstacles.proto')).read().replace('PROTO Obstacles [', 'PROTO %s [' % O)
        open(os.path.join(TEST, 'protos', A + '.proto'), 'w').write(a)
        open(os.path.join(TEST, 'protos', O + '.proto'), 'w').write(o)
    w = open(os.path.join(tmp, 'worlds/test.wbt')).read()
    w = re.sub(r'Robot \{ name "judge".*\n', '', w)          # no judge in the viewable worlds
    w = w.replace('"../protos/Arena.proto"', '"../protos/%s.proto"' % A)
    w = w.replace('"../protos/Obstacles.proto"', '"../protos/%s.proto"' % O)
    w = w.replace('"../protos/TwoWheelRobot.proto"', '"../../protos/TwoWheelRobot.proto"')
    w = w.replace('\nArena { }', '\n%s { }' % A).replace('\nObstacles { }', '\n%s { }' % O)
    w = w.replace('controller "main_controller"', 'controller "test_runner"')
    w = w.replace('WorldInfo { basicTimeStep 16 }', 'WorldInfo { title "%s" basicTimeStep 16 }' % name)
    open(os.path.join(TEST, 'worlds', name + '.wbt'), 'w').write(w)
    index[name] = json.load(open(os.path.join(tmp, 'truth.json')))
json.dump(index, open(os.path.join(TEST, 'truth.json'), 'w'), indent=1, sort_keys=True)
shutil.rmtree(TMP, ignore_errors=True)
print(len(index), 'worlds')

# builds dist/AProbotix_Submission.zip
#   python make_submission_zip.py
#
# 1. makes submission/ from the root working copy (comments stripped, checks
#    the code itself didnt change)
# 2. refreshes test/TEST_REPORT.md from the lab database (if there is one)
# 3. zips PeraBots_UC/{protos, controllers/main_controller, worlds, docs, test}
import ast
import io
import os
import re
import shutil
import subprocess
import sys
import tokenize
import zipfile

ROOT = os.path.dirname(os.path.abspath(__file__))
SUB = os.path.join(ROOT, 'submission')
TEAM = 'AProbotix'
TOP = 'PeraBots_UC'
OUT = os.path.join(ROOT, 'dist', '%s_Submission.zip' % TEAM)


def strip_py(src):
    toks = [t for t in tokenize.generate_tokens(io.StringIO(src).readline) if t.type != tokenize.COMMENT]
    lines = tokenize.untokenize(toks).split('\n')
    orig = src.split('\n')
    keep = [l.rstrip() for i, l in enumerate(lines) if not (l.strip() == '' and i < len(orig) and orig[i].strip().startswith('#'))]
    txt = re.sub(r'\n{4,}', '\n\n\n', '\n'.join(keep)).lstrip('\n')
    assert ast.dump(ast.parse(src)) == ast.dump(ast.parse(txt)), 'stripping comments changed the code!'
    return txt


def strip_proto(src):
    # only our comments inside the extension slot, the organisers header stays
    a = src.index('  field SFColor    bodyColor')
    b = src.index('] {\n  Robot {')
    seg = '\n'.join(l for l in src[a:b].split('\n') if not l.strip().startswith('#'))
    return src[:a] + seg + src[b:]


def sync_submission():
    for d in ('protos', 'controllers/main_controller', 'worlds'):
        os.makedirs(os.path.join(SUB, d), exist_ok=True)
    c = open(os.path.join(ROOT, 'controllers/main_controller/main_controller.py'), encoding='utf-8').read()
    open(os.path.join(SUB, 'controllers/main_controller/main_controller.py'), 'w', encoding='utf-8', newline='\n').write(strip_py(c))
    p = open(os.path.join(ROOT, 'protos/TwoWheelRobot.proto'), encoding='utf-8').read()
    open(os.path.join(SUB, 'protos/TwoWheelRobot.proto'), 'w', encoding='utf-8', newline='\n').write(strip_proto(p))
    # (.Arena.wbproj is only webots window layout, keep the one in submission/)
    for f in ('protos/Arena.proto', 'protos/Obstacles.proto', 'worlds/Arena.wbt', 'worlds/.Arena.jpg'):
        shutil.copyfile(os.path.join(ROOT, f), os.path.join(SUB, f))


def check():
    sub_c = open(os.path.join(SUB, 'controllers/main_controller/main_controller.py'), encoding='utf-8').read()
    assert '#' not in sub_c, 'comment left in submission controller'
    extra = [f for f in os.listdir(os.path.join(SUB, 'controllers/main_controller')) if f != 'main_controller.py']
    assert not extra, 'unexpected files in controllers/main_controller: %s' % extra
    p = open(os.path.join(SUB, 'protos/TwoWheelRobot.proto'), encoding='utf-8').read()
    assert 'name "led_blue"' in p and 'name "led_green"' in p and 'supervisor FALSE' in p
    assert not re.search(r'\bGPS\b|\bEmitter\b|\bReceiver\b', p.split('extensionSlot')[1].split('] {')[0]), 'forbidden device in extension slot'


README = '''AProbotix - PeraBots 2026 University Category, Phase 1 submission

  protos/TwoWheelRobot.proto              our robot (sensors in the extensionSlot)
  protos/Arena.proto, Obstacles.proto     unmodified
  controllers/main_controller/            our controller (main_controller.py)
  worlds/Arena.wbt                        dev world, open in Webots R2025a and press Play
  CONTROLLER_EXPLANATION.md               what the controller does
  DEVELOPMENT_NOTES.md                    design notes
  test/                                   test worlds, test report, test tools (not needed to run the robot)

The robot needs no extra files: everything is in main_controller.py.
'''


def build_zip():
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    if os.path.exists(OUT):
        os.remove(OUT)
    skip_dirs = {'__pycache__', 'runs', 'exports'}
    skip_files = {'lab.db'}
    with zipfile.ZipFile(OUT, 'w', zipfile.ZIP_DEFLATED) as z:
        for d, dirs, files in os.walk(SUB):
            dirs[:] = [x for x in dirs if x not in skip_dirs]
            for f in files:
                p = os.path.join(d, f)
                z.write(p, TOP + '/' + os.path.relpath(p, SUB).replace(os.sep, '/'))
        for f in ('CONTROLLER_EXPLANATION.md', 'DEVELOPMENT_NOTES.md'):
            z.write(os.path.join(ROOT, f), TOP + '/' + f)
        z.writestr(TOP + '/README.txt', README)
        for d, dirs, files in os.walk(os.path.join(ROOT, 'test')):
            dirs[:] = [x for x in dirs if x not in skip_dirs]
            for f in files:
                if f in skip_files or f.endswith('.pyc'):
                    continue
                p = os.path.join(d, f)
                z.write(p, TOP + '/' + os.path.relpath(p, ROOT).replace(os.sep, '/'))
    with zipfile.ZipFile(OUT) as z:
        names = z.namelist()
    req = [TOP + '/protos/TwoWheelRobot.proto', TOP + '/protos/Arena.proto', TOP + '/protos/Obstacles.proto',
           TOP + '/controllers/main_controller/main_controller.py', TOP + '/worlds/Arena.wbt']
    for r in req:
        assert r in names, 'missing ' + r
    assert sum(n.endswith('/TwoWheelRobot.proto') for n in names) == 1, 'more than one TwoWheelRobot.proto in zip'
    assert sum(n.endswith('/main_controller.py') for n in names) == 1, 'more than one main_controller.py in zip'
    return names


if __name__ == '__main__':
    sync_submission()
    check()
    rep = os.path.join(ROOT, 'test', 'tools', 'report.py')
    if os.path.exists(os.path.join(ROOT, 'test', 'tools', 'lab', 'lab.db')):
        subprocess.run([sys.executable, rep], check=True)
    names = build_zip()
    print('%s  (%d files, %.1f MB)' % (OUT, len(names), os.path.getsize(OUT) / 1e6))

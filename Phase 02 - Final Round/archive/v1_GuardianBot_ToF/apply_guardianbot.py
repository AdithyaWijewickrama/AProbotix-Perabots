"""
apply_guardianbot.py - switch the Webots project from TwoWheelRobot to GuardianBot.

  * worlds/*.wbt          -> use GuardianBot.proto, add caster/tyre contact materials
  * main_controller.py    -> geometry constants + clean sign conventions for GuardianBot
Originals are copied to *.bak_twowheel before anything is changed.
Run from the "Final Round" folder:  python webots_project/apply_guardianbot.py
"""
import os
import re
import shutil
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
WORLDS = [os.path.join(ROOT, "worlds", w) for w in ("perabots_arena.wbt", "Arena.wbt")]
CTRL = os.path.join(ROOT, "controllers", "main_controller", "main_controller.py")


def backup(p):
    b = p + ".bak_twowheel"
    if not os.path.exists(b):
        shutil.copy2(p, b)


def sub1(text, old, new, label):
    if old not in text:
        if new in text:
            return text            # already applied
        sys.exit("pattern not found (%s) - controller differs from expected version" % label)
    return text.replace(old, new, 1)


ROBOT_NODE = """GuardianBot {
  translation 2.315 0.125 0.0
  rotation 0 0 1 1.5708
  name "team_robot"
  controller "main_controller"
}"""

CONTACTS = """contactProperties [
    ContactProperties {
      material1 "default"
      material2 "default"
      coulombFriction [ 0.8 ]
    }
    ContactProperties {
      material1 "default"
      material2 "tire"
      coulombFriction [ 1.0 ]
    }
    ContactProperties {
      material1 "default"
      material2 "caster"
      coulombFriction [ 0.05 ]
    }
  ]"""


def patch_world(p):
    if not os.path.exists(p):
        return
    backup(p)
    t = open(p, encoding="utf-8").read()
    t = t.replace('EXTERNPROTO "../protos/TwoWheelRobot.proto"', 'EXTERNPROTO "../protos/GuardianBot.proto"')
    t = re.sub(r"TwoWheelRobot \{.*?\n\}", ROBOT_NODE, t, flags=re.S)
    t = t.replace("# ROBOT: TwoWheelRobot with 3D visible sensors at START Position",
                  "# ROBOT: GuardianBot (buildable 130 mm robot, see GuardianBot_BOM.md) at START")
    if '"caster"' not in t:
        t = re.sub(r"contactProperties \[.*?\n  \]", CONTACTS, t, count=1, flags=re.S)
    open(p, "w", encoding="utf-8", newline="\n").write(t)
    print("patched", p)


SENS_NEW = """SENS = {   # GuardianBot ToF layout (x, y from axle centre, heading) - from make_guardianbot.py
    'ds_front': (0.0570, 0.0000, 0.0),
    'ds_fl1': (0.0525, 0.0222, 0.4),
    'ds_fr1': (0.0525, -0.0222, -0.4),
    'ds_fl2': (0.0397, 0.0409, 0.8),
    'ds_fr2': (0.0397, -0.0409, -0.8),
    'ds_fl3': (0.0207, 0.0531, 1.2),
    'ds_fr3': (0.0207, -0.0531, -1.2),
    'ds_left': (0.0, 0.0570, HP),
    'ds_right': (0.0, -0.0570, -HP),
    'ds_back': (-0.0085, 0.0, PI),
}
DS_MAX = 1.29   # VL53L4CD returns 1300 mm when nothing is in range"""


def patch_ctrl(p):
    backup(p)
    t = open(p, encoding="utf-8").read()
    t = sub1(t, "HALF_TRACK = 0.032\nPIV_X = 0.005", "HALF_TRACK = 0.0555   # GuardianBot: 111 mm track\nPIV_X = 0.0           # origin = axle centre", "geometry")
    t = sub1(t, "y2 = wrap(imu.getRollPitchYaw()[2] + PI)", "y2 = wrap(imu.getRollPitchYaw()[2])", "yaw")
    t = sub1(t, "d = -((l - _pl) + (r - _pr)) * 0.5 * R_WHEEL", "d = ((l - _pl) + (r - _pr)) * 0.5 * R_WHEEL", "odom")
    t = sub1(t, "a = (-v - w * HALF_TRACK) / R_WHEEL\n    b = (-v + w * HALF_TRACK) / R_WHEEL",
             "a = (v - w * HALF_TRACK) / R_WHEEL     # left wheel\n    b = (v + w * HALF_TRACK) / R_WHEEL     # right wheel", "set_vw")
    t = sub1(t, "return -(_last_l + _last_r) * 0.5 * R_WHEEL", "return (_last_l + _last_r) * 0.5 * R_WHEEL", "speed")
    t = re.sub(r"SENS = \{.*?\n\}\nDS_MAX = 1\.49", SENS_NEW, t, count=1, flags=re.S)
    t = sub1(t, "CAM_X, CAM_Z, CAM_P = 0.042 - PIV_X, 0.09, 0.4", "CAM_X, CAM_Z, CAM_P = 0.048 - PIV_X, 0.092, 0.4", "camera")
    t = sub1(t, "R_HARD, R_INF, R_SOFT = 0.06, 0.075, 0.12", "R_HARD, R_INF, R_SOFT = 0.072, 0.085, 0.125   # 65 mm body radius + margin", "clearance")
    open(p, "w", encoding="utf-8", newline="\n").write(t)
    print("patched", p)


if __name__ == "__main__":
    for w in WORLDS:
        patch_world(w)
    patch_ctrl(CTRL)

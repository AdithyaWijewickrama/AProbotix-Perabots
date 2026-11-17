"""
make_printed_parts.py - GuardianBot v3 3-D printed parts (CadQuery). Units: mm.
Outputs STL (slicer) and STEP (CAD) into ../3d_prints/. Hole positions match make_acrylic_dxf.py.

Print: PETG or PLA+, 0.2 mm layers, 4 walls, 40 % infill. Pads: TPU 95A (or glue on rubber strip).
Parts (quantity):
  n20_clamp            (2)  saddle over the N20 gearbox, 2 x M2.5 into the base
  sharp_side_tower     (1)  5 mm centre plate carrying ir_left + ir_right back to back (lens at z 46)
  sharp_front_hanger   (1)  hangs ir_front, ir_fl (+45 deg), ir_fr (-45 deg) under the mid deck (lens at z 58)
  hcsr04_hanger        (4)  holds an HC-SR04 under the top deck (transducers at z 92)
  servo_cradle         (2)  clamps a Kpower M0170 on the mid deck, shaft pointing down
  gripper_arm_left/right (1+1) arm on the servo spline hub, reaches down to piece mid-height
  pocket               (1)  rear capture pocket, 55 mm throat, flared mouth (z 6 -> 70)
  tcs_front_bracket    (1)  GY-31 facing forward at z 28 (piece colour)
  tcs_floor_spacer     (2)  8 mm spacers: floor GY-31 lens 8 mm above the floor
  caster_spacer        (1)  sets the ball-caster height (sand to level the robot)
  deck_spacer_54p5     (4)  base -> mid deck spacer tube on M3 threaded rod
  deck_spacer_32       (4)  mid -> top deck spacer tube on M3 threaded rod
Run:  python make_printed_parts.py
"""
import math
import os

import cadquery as cq

from guardianbot_geometry import (ARM_L, BASE_Z1, HINGE_X, HINGE_Y, MID_Z0, POCKET_FLARE_X, POCKET_IN_HW,
                                  POCKET_MOUTH_HW, POCKET_WALL_T, POCKET_X, SERVO_BODY, SHARP, TCS_FRONT,
                                  TOP_Z0, US)

MM = 1000.0
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "3d_prints")
M25, M3 = 2.7, 3.2
INS_M3 = 4.0                       # M3 heat-set insert hole (or tap M3 directly into 2.5 mm)
SHARP_TAB = 37.0                   # GP2Y0A21 mounting-hole spacing
BASE_TOP = BASE_Z1 * MM            # 15.5
MID_BOT = MID_Z0 * MM              # 70.0
TOP_BOT = TOP_Z0 * MM              # 105.0


def box(x, y, z, at=(0, 0, 0), centered=(True, True, False)):
    return cq.Workplane("XY").box(x, y, z, centered=centered).translate(at)


def zhole(d, h, at):
    return cq.Workplane("XY").circle(d / 2).extrude(h).translate(at)


def save(shape, name):
    os.makedirs(OUT, exist_ok=True)
    cq.exporters.export(shape, os.path.join(OUT, name + ".stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(shape, os.path.join(OUT, name + ".step"))


def n20_clamp():
    """Saddle over a 12 x 10 mm N20 gearbox; screws 22 mm apart."""
    b = box(30, 10, 12.5)
    b = b.cut(box(12.4, 12, 10.3, at=(0, 0, -0.1)))                     # gearbox channel
    for x in (-11, 11):
        b = b.cut(zhole(M25, 20, (x, 0, -1)))
    return b


def sharp_side_tower():
    """Centre plate 5 mm thick, sensors screw on both faces (M3 through), lens centre 46 mm above floor."""
    zc = SHARP["ir_left"][2] * MM - BASE_TOP                             # 30.5 above the base top
    L = 54.0
    plate = box(L, 5, zc + 12, at=(0, 0, 0))
    foot = box(L + 10, 14, 4)
    b = plate.union(foot)
    for dx in (-SHARP_TAB / 2, SHARP_TAB / 2):
        b = b.cut(cq.Workplane("XZ").circle(M3 / 2).extrude(10, both=True).translate((dx, 0, zc)))
    for dx in (-22.0, 22.0):                                             # base screws (base holes -26/+18 from sensor x)
        b = b.cut(zhole(M3, 10, (dx, 0, -1)))
    # cable notch under each sensor connector
    b = b.cut(box(10, 7, 6, at=(0, 0, zc - 13)))
    return b


def sharp_front_hanger():
    """Hangs three GP2Y0A21 under the mid deck: front (0 deg) and +-45 deg, lens centre z 58."""
    drop = MID_BOT - SHARP["ir_front"][2] * MM                           # 12 mm below the deck to lens centre
    top = box(30, 46, 3, at=(52 - 50, 0, 0))
    b = cq.Workplane("XY").box(1, 1, 1).translate((0, 0, -100))
    plate = top
    for name in ("ir_front", "ir_fl", "ir_fr"):
        x, y, z, a = SHARP[name]
        r = math.hypot(x, y) * MM
        # vertical plate behind the sensor body (sensor back face at r - 13.5)
        p = box(3, 48, drop + 10, at=(0, 0, -(drop + 10) + 3))
        for dy in (-SHARP_TAB / 2, SHARP_TAB / 2):
            p = p.cut(cq.Workplane("YZ").circle(M3 / 2).extrude(10, both=True).translate((0, dy, -drop)))
        p = p.translate((r - 13.5 - 1.5 - 50, 0, 0)).rotate((-50, 0, 0), (-50, 0, 1), math.degrees(a))
        plate = plate.union(p)
    # deck screws (mid deck holes at (46, +-18) and (58, 0)); part origin x = 50 in robot frame
    for (hx, hy) in ((46, 18), (46, -18), (58, 0)):
        plate = plate.cut(zhole(M3, 6, (hx - 50, hy, -1)))
    del b
    return plate.mirror("XY")          # print flat side down


def hcsr04_hanger():
    """Flange under the top deck; vertical plate holds the HC-SR04 (16 mm transducers 26 mm apart)."""
    zc = TOP_BOT - US["us_front"][2] * MM                                # 13 mm below the deck
    flange = box(16, 34, 3, at=(-8, 0, 0), centered=(False, True, False))
    plate = box(3, 47, zc + 12, at=(5, 0, -(zc + 12) + 3), centered=(False, True, False))
    b = flange.union(plate)
    for k in (-12.0, 12.0):
        b = b.cut(zhole(M3, 6, (-2.0, k, -1)))
    for k in (-13.0, 13.0):
        b = b.cut(cq.Workplane("YZ").circle(8.3).extrude(5).translate((4.0, k, 3 - zc)))
    return b


def servo_cradle():
    """Clamps the M0170 body (28.5 x 13 x 27): open frame with two M3 feet 24 mm apart."""
    L, W, H = SERVO_BODY[0] * MM, SERVO_BODY[1] * MM, 20.0
    b = box(L + 6, W + 6, H)
    b = b.cut(box(L + 0.4, W + 0.4, H + 2, at=(0, 0, -1)))
    feet = box(10, W + 30, 3)
    b = b.union(feet.translate((4.0 + 7.0 - L / 2 + L / 2 - 7.0, 0, 0)))
    for dy in (-12.0, 12.0):
        b = b.cut(zhole(M3, 6, (4.0, dy * 1.0 if abs(dy) > (W + 6) / 2 else dy, -1)))
    b = b.cut(box(6, W + 8, 5, at=(-L / 2, 0, -0.01)))                  # cable exit
    return b


def gripper_arm(side):
    """Hub on the servo spline (horn screwed on top) down to the arm; arm length = ARM_L."""
    s = 1 if side == "left" else -1
    hub_h = MID_BOT - 8.0                                                # from z 8 up to the deck underside
    hub = cq.Workplane("XY").circle(4.5).extrude(hub_h)
    hub = hub.cut(cq.Workplane("XY").circle(1.3).extrude(hub_h + 2).translate((0, 0, -1)))   # horn screw
    arm_h = 34.0
    arm = box(ARM_L * MM, 3.5, arm_h, centered=(False, True, False)).translate((-ARM_L * MM, 0, 0))
    b = hub.union(arm)
    # small cross-plate on top to screw the servo's single-arm horn (2 x M2 self-tappers)
    horn = box(18, 6, 2.5, at=(-4, 0, hub_h - 2.5))
    b = b.union(horn)
    for x in (-10.0, 2.0):
        b = b.cut(zhole(1.6, 6, (x, 0, hub_h - 4)))
    # pad groove on the inner face
    b = b.cut(box(ARM_L * MM - 8, 1.2, arm_h - 8, at=(-ARM_L * MM + 3, -s * 1.3, 4), centered=(False, True, False)))
    return b


def pocket():
    t = POCKET_WALL_T * MM
    px, ihw, fx, mhw = POCKET_X * MM, POCKET_IN_HW * MM, POCKET_FLARE_X * MM, POCKET_MOUTH_HW * MM
    hx = HINGE_X * MM + 6.0
    z0, z1 = 6.0, BASE_TOP + 20.0                                        # walls reach 20 mm above the base
    h = z1 - z0
    b = box(t, 2 * (ihw + t), h, at=(px, 0, z0), centered=(False, True, False))
    for s in (1, -1):
        b = b.union(box(px - fx, t, h, at=(fx, s * (ihw + t / 2), z0), centered=(False, True, False)))
        pts = [(fx, s * ihw), (hx, s * mhw), (hx, s * (mhw + t)), (fx, s * (ihw + t))]
        b = b.union(cq.Workplane("XY").polyline(pts).close().extrude(h).translate((0, 0, z0)))
        # tabs that sit on the base top, M2.5 holes at x -20 / -48, y +-40
        tab = box(36, 12, 3, at=(-34, s * 40, BASE_TOP))
        bridge = box(36, 40 - ihw, 3, at=(-34, s * (ihw + (40 - ihw) / 2), BASE_TOP))
        b = b.union(tab).union(bridge)
        for x in (-20.0, -48.0):
            b = b.cut(zhole(M25, 6, (x, s * 40, BASE_TOP - 1)))
    return b


def tcs_front_bracket():
    """GY-31 (33 x 33 mm) standing upright at the nose, lens at z 28 facing forward."""
    zc = TCS_FRONT[2] * MM - BASE_TOP
    foot = box(12, 40, 3, at=(-6, 0, 0))
    wall = box(3, 36, zc + 17, at=(0, 0, 0), centered=(False, True, False))
    b = foot.union(wall)
    for s in (1, -1):
        b = b.cut(zhole(M3, 6, (-6, s * 15, -1)))
        b = b.cut(cq.Workplane("YZ").circle(M3 / 2).extrude(6).translate((-1, s * 12, zc)))
    b = b.cut(cq.Workplane("YZ").circle(6).extrude(6).translate((-1, 0, zc)))   # LED / lens window
    return b


def tcs_floor_spacer():
    return cq.Workplane("XY").circle(3.5).extrude(4.5).cut(zhole(M3, 6, (0, 0, -1)))


def deck_spacer(length):
    """Spacer tube between decks, clamped by M3 threaded rod (base-mid 54.5 mm, mid-top 32 mm)."""
    return cq.Workplane("XY").circle(3.75).extrude(length).cut(zhole(3.5, length + 2, (0, 0, -1)))


def caster_spacer(thickness=2.0):
    b = box(10, 30, thickness)
    for k in (-9.5, 9.5):
        b = b.cut(cq.Workplane("XY").slot2D(7, M25, 90).extrude(5).translate((0, k, -1)))
    return b


PARTS = {
    "n20_clamp": n20_clamp,
    "sharp_side_tower": sharp_side_tower,
    "sharp_front_hanger": sharp_front_hanger,
    "hcsr04_hanger": hcsr04_hanger,
    "servo_cradle": servo_cradle,
    "gripper_arm_left": lambda: gripper_arm("left"),
    "gripper_arm_right": lambda: gripper_arm("right"),
    "pocket": pocket,
    "tcs_front_bracket": tcs_front_bracket,
    "tcs_floor_spacer": tcs_floor_spacer,
    "caster_spacer": caster_spacer,
    "deck_spacer_54p5": lambda: deck_spacer(54.5),
    "deck_spacer_32": lambda: deck_spacer(32.0),
}

if __name__ == "__main__":
    for name, fn in PARTS.items():
        shp = fn()
        bb = shp.val().BoundingBox()
        save(shp, name)
        print("%-20s %5.1f x %5.1f x %5.1f mm   %5.1f cm3" % (name, bb.xlen, bb.ylen, bb.zlen, shp.val().Volume() / 1000))

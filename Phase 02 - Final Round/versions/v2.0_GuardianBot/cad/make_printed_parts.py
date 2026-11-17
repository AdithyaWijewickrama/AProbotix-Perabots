"""
make_printed_parts.py - 3-D printed parts for GuardianBot (CadQuery).  Units: mm.
Outputs STL (for slicing) and STEP (for Fusion/SolidWorks/FreeCAD) into ../3d_prints/.

Print settings: PETG, 0.2 mm layers, 4 walls, 40 % gyroid.  Gripper pads: TPU 95A.
Heat-set inserts: M2.5 x 4 mm (pocket), M2 x 3 mm (brackets).

Parts (quantity):
  pocket.stl            (1)  rear capture pocket: 55 mm throat, flared mouth, top flange, base tabs
  gripper_arm_left.stl  (1)  arm + hub on a 3 mm steel hinge pin
  gripper_arm_right.stl (1)
  gripper_pad.stl       (2)  TPU pad, glue into the arm groove
  hinge_lever.stl       (2)  sits on the pin above the PCB, push-rod eye
  servo_cradle.stl      (1)  MG90S holder on the PCB
  sharp_bracket.stl     (4)  vertical GP2Y0A41SK0F holder on the base plate
  hcsr04_hanger.stl     (4)  hangs an HC-SR04 under the crown
  camera_mast.stl       (1)  XIAO ESP32S3 Sense cradle, 23 deg down
  caster_spacer.stl     (1)  sets the ball-caster height
"""
import math
import os

import cadquery as cq

from guardianbot_geometry import (BASE_Z1, HINGE_X, HINGE_Y, PCB_Z0, POCKET_FLARE_X, POCKET_IN_HW,
                                  POCKET_MOUTH_HW, POCKET_WALL_T, POCKET_X)

MM = 1000.0
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "3d_prints")
M2, M25, M3 = 2.2, 2.7, 3.2
INS_M2, INS_M25 = 3.2, 3.6          # heat-set insert holes


def save(shape, name):
    os.makedirs(OUT, exist_ok=True)
    cq.exporters.export(shape, os.path.join(OUT, name + ".stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(shape, os.path.join(OUT, name + ".step"))
    return name


# ------------------------------------------------------------------ pocket
def pocket():
    t = POCKET_WALL_T * MM
    px = POCKET_X * MM
    ihw = POCKET_IN_HW * MM
    fx = POCKET_FLARE_X * MM
    mhw = POCKET_MOUTH_HW * MM
    hx = HINGE_X * MM + 4.0
    z0, z1 = 6.0, PCB_Z0 * MM            # 6 mm above the floor up to the PCB underside
    h = z1 - z0
    # back wall
    body = cq.Workplane("XY").box(t, 2 * (ihw + t), h, centered=(False, True, False)).translate((px - t, 0, z0))
    for s in (1, -1):
        # straight side wall
        sw = cq.Workplane("XY").box(px - fx, t, h, centered=(False, False, False)).translate(
            (fx, ihw if s > 0 else -ihw - t, z0))
        body = body.union(sw)
        # flared lead-in wall (polygon extrusion)
        y_in0, y_in1 = s * ihw, s * mhw
        pts = [(fx, y_in0), (hx, y_in1), (hx, y_in1 + s * t), (fx, y_in0 + s * t)]
        fl = cq.Workplane("XY").polyline(pts).close().extrude(h).translate((0, 0, z0))
        body = body.union(fl)
        # lower hinge bushing (3 mm pin) at the mouth
        bush = cq.Workplane("XY").circle(4.0).extrude(3.0).translate((HINGE_X * MM, s * HINGE_Y * MM, z0 - 1.0))
        bridge = cq.Workplane("XY").box(6.0, 4.0, 3.0, centered=(True, True, False)).translate(
            ((HINGE_X * MM + hx) / 2 + 1.0, s * (mhw + 1.0), z0 - 1.0))
        body = body.union(bush).union(bridge)
        body = body.cut(cq.Workplane("XY").circle(1.6).extrude(10).translate((HINGE_X * MM, s * HINGE_Y * MM, z0 - 3)))
        # base-plate tab (sits on top of the 3 mm base, M2.5 insert from below)
        tab = cq.Workplane("XY").box(10.0, 12.0, 3.0, centered=(True, False, False)).translate(
            (-20.0, s * (ihw + t) if s > 0 else -(ihw + t) - 12.0, BASE_Z1 * MM))
        body = body.union(tab)
        body = body.cut(cq.Workplane("XY").circle(INS_M25 / 2).extrude(4).translate((-20.0, s * 39.0, BASE_Z1 * MM - 0.5)))
    # top flange under the PCB with two M2.5 inserts
    fl_w = 2 * (ihw + t) + 14.0
    top = cq.Workplane("XY").box(px - fx + t + 4.0, fl_w, 3.0, centered=(False, True, False)).translate(
        (fx - 4.0, 0, z1 - 3.0))
    body = body.union(top)
    for s in (1, -1):
        body = body.cut(cq.Workplane("XY").circle(INS_M25 / 2).extrude(5).translate((-15.0, s * 36.0, z1 - 4.5)))
    # window for the TCS34725 colour sensor (sensor sits behind the back wall)
    win = cq.Workplane("XY").box(t + 2, 10.0, 10.0).translate((px - t / 2, 0, 20.0))
    body = body.cut(win)
    # piece clearance check volume is the open space between the walls (nothing to cut)
    return body


# ------------------------------------------------------------------ gripper
def gripper_arm(side):
    s = 1 if side == "left" else -1
    L, T, H = 32.0, 3.0, 30.0
    hub = cq.Workplane("XY").circle(4.0).extrude(H)
    arm = cq.Workplane("XY").box(L, T, H, centered=(False, True, False)).translate((-L, 0, 0))
    body = hub.union(arm)
    body = body.cut(cq.Workplane("XY").circle(1.55).extrude(H + 2).translate((0, 0, -1)))       # 3 mm pin press-fit
    # grub-screw hole (M2) across the hub
    body = body.cut(cq.Workplane("YZ").circle(0.9).extrude(10).translate((0, 0, H / 2)).translate((0, 0, 0)))
    # groove for the TPU pad on the inner face (inner face points to -y for the left arm)
    groove = cq.Workplane("XY").box(L - 8.0, 1.2, H - 6.0, centered=(False, True, False)).translate(
        (-L + 2.0, -s * (T / 2 - 0.5), 3.0))
    body = body.cut(groove)
    return body


def gripper_pad():
    return cq.Workplane("XY").box(23.6, 1.8, 23.6).edges("|Z").fillet(0.6)


def hinge_lever():
    hub = cq.Workplane("XY").circle(4.5).extrude(4.0)
    arm = cq.Workplane("XY").box(12.0, 5.0, 2.5, centered=(False, True, False))
    eye = cq.Workplane("XY").circle(2.5).extrude(2.5).translate((12.0, 0, 0))
    body = hub.union(arm).union(eye)
    body = body.cut(cq.Workplane("XY").circle(1.55).extrude(6).translate((0, 0, -1)))
    body = body.cut(cq.Workplane("XY").circle(0.65).extrude(6).translate((12.0, 0, -1)))   # 1 mm push-rod
    body = body.cut(cq.Workplane("YZ").circle(0.9).extrude(10).translate((0, 0, 2.0)))       # M2 grub screw
    return body


def servo_cradle():
    """MG90S: body 22.8 x 12.2, mounting tabs 32.3 long, tab underside 15.9 mm above the body base."""
    W, D, H = 36.0, 18.0, 15.9
    body = cq.Workplane("XY").box(W, D, H, centered=(True, True, False))
    body = body.cut(cq.Workplane("XY").box(23.2, 12.6, H + 2, centered=(True, True, False)).translate((0, 0, -1)))
    for x in (-13.75, 13.75):                               # tab screws (M2 inserts)
        body = body.cut(cq.Workplane("XY").circle(INS_M2 / 2).extrude(5).translate((x, 0, H - 4.5)))
    for x in (-15.5, 15.5):                                  # M2 screws down into the PCB
        for y in (-6.5, 6.5):
            body = body.cut(cq.Workplane("XY").circle(M2 / 2).extrude(H + 2).translate((x, y, -1)))
    body = body.cut(cq.Workplane("XY").box(8.0, D + 2, 4.0, centered=(True, True, False)).translate((0, 0, -0.01)))  # cable exit
    return body


# ------------------------------------------------------------------ sensor mounts
def sharp_bracket():
    """Upright for a GP2Y0A41SK0F mounted vertically (holes 37 mm apart), foot on the base plate."""
    up_w, up_h, t = 14.0, 44.0, 2.5
    foot = cq.Workplane("XY").box(10.0, 24.0, 3.0, centered=(False, True, False)).translate((-10.0, 0, 0))
    upright = cq.Workplane("XY").box(t, up_w, up_h, centered=(False, True, False))
    body = foot.union(upright)
    for k in (-6.0, 6.0):
        body = body.cut(cq.Workplane("XY").circle(M2 / 2).extrude(5).translate((-5.0, k, -1)))
    zc = 22.0
    for dz in (-18.5, 18.5):
        body = body.cut(cq.Workplane("YZ").circle(M25 / 2).extrude(t + 2).translate((-1.0, 0, zc + dz)))
    # gusset
    g = cq.Workplane("XZ").polyline([(-8.0, 3.0), (0.0, 3.0), (0.0, 14.0)]).close().extrude(2.0).translate((0, 1.0, 0))
    body = body.union(g)
    return body


def hcsr04_hanger():
    """Flange screws under the crown; plate holds the two 16 mm transducers (26 mm apart)."""
    flange = cq.Workplane("XY").box(16.0, 32.0, 3.0, centered=(False, True, False)).translate((-8.0, 0, 0))
    plate = cq.Workplane("XY").box(3.0, 47.0, 27.0, centered=(False, True, False)).translate((8.0, 0, -24.0))
    body = flange.union(plate)
    for k in (-12.0, 12.0):
        body = body.cut(cq.Workplane("XY").circle(INS_M2 / 2).extrude(4).translate((0.0, k, -0.5)))
    for k in (-13.0, 13.0):
        body = body.cut(cq.Workplane("YZ").circle(8.3).extrude(5).translate((7.0, k, -14.5)))
    # small ledge the HC-SR04 PCB rests on
    body = body.union(cq.Workplane("XY").box(3.0, 40.0, 2.0, centered=(False, True, False)).translate((5.0, 0, -24.0)))
    return body


def camera_mast():
    base = cq.Workplane("XY").box(14.0, 22.0, 3.0, centered=(True, True, False))
    for k in (-6.0, 6.0):
        base = base.cut(cq.Workplane("XY").circle(M2 / 2).extrude(5).translate((0, k, -1)))
    post = cq.Workplane("XY").box(4.0, 18.0, 9.0, centered=(True, True, False)).translate((-4.0, 0, 3.0))
    cradle = (cq.Workplane("XY").box(24.0, 21.0, 4.0)
              .cut(cq.Workplane("XY").box(21.6, 18.0, 4.0).translate((0.6, 0, 1.2)))   # XIAO pocket, open on top
              .cut(cq.Workplane("XY").box(6.0, 9.5, 6.0).translate((-10.5, 0, 0)))      # USB-C opening
              .rotate((0, 0, 0), (0, 1, 0), 23).translate((2.0, 0, 13.5)))
    return base.union(post).union(cradle)


def caster_spacer(thickness=1.6):
    body = cq.Workplane("XY").box(10.0, 22.0, thickness, centered=(True, True, False))
    for k in (-6.35, 6.35):
        body = body.cut(cq.Workplane("XY").circle(M2 / 2).extrude(5).translate((0, k, -1)))
    return body


PARTS = {
    "pocket": pocket,
    "gripper_arm_left": lambda: gripper_arm("left"),
    "gripper_arm_right": lambda: gripper_arm("right"),
    "gripper_pad": gripper_pad,
    "hinge_lever": hinge_lever,
    "servo_cradle": servo_cradle,
    "sharp_bracket": sharp_bracket,
    "hcsr04_hanger": hcsr04_hanger,
    "camera_mast": camera_mast,
    "caster_spacer": caster_spacer,
}

if __name__ == "__main__":
    for name, fn in PARTS.items():
        shp = fn()
        bb = shp.val().BoundingBox()
        save(shp, name)
        print("%-18s %5.1f x %5.1f x %5.1f mm   %.1f cm3" % (name, bb.xlen, bb.ylen, bb.zlen, shp.val().Volume() / 1000))

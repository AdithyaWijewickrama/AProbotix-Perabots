"""
make_printed_parts.py - GuardianBot v4.2 3-D printed parts (CadQuery). Units: mm.
Outputs STL (slicer) and STEP (CAD) into ../3d_prints/. Hole positions match make_acrylic_dxf.py.

Print: PETG or PLA+, 0.2 mm layers, 4 walls, 40 % infill. Pads: TPU 95A (or glue on rubber strip).
Parts (quantity):
  n20_clamp            (2)  saddle over the N20 gearbox, 2 x M2.5 into the base
  tof_post_40          (4)  side VL53L0X posts on the base (sensor centre z 40, two per side)
  tof_post_60          (3)  front / front-left / front-right VL53L0X posts on the base (sensor centre z 60)
  tof_back_hanger      (1)  back VL53L0X under the top deck (sensor centre z 90, sees over a held piece)
  hcsr04_hanger        (2)  holds an HC-SR04 under the top deck (transducers at z 92)
  servo_cradle         (2)  M0170 on the PCB deck, shaft down through the hub hole; the cap screws
                            up into the top deck (no screw holes in the PCB)
  gripper_arm_left/right (1+1) arm on the servo spline hub, reaches down to piece mid-height
  pocket               (1)  rear capture pocket, 55 mm throat, flared mouth (z 6 -> 70)
  tcs_front_bracket    (1)  GY-31 facing forward at z 28 (piece colour)
  tcs_floor_spacer     (2)  8 mm spacers: floor GY-31 lens 8 mm above the floor
  caster_spacer        (1)  sets the ball-caster height (sand to level the robot)
  camera_mast          (1)  v4.2: mast plate holding an SG90 tilt servo (window + 2 tab screws) and two stop pins;
                            same 2 x M3 foot holes in the top deck as v4.0 / v4.1
  camera_tilt_bracket  (1)  v4.2: OV7670 plate (lens 140 mm high, 29 deg down in the maze view) on an arm that
                            screws to the SG90 horn; its tail meets the stops at -2 and +29 deg
  deck_spacer_54p5     (4)  base -> mid deck spacer tube on M3 threaded rod
  deck_spacer_33p4     (4)  PCB deck -> top deck spacer tube on M3 threaded rod
Run:  python make_printed_parts.py
"""
import math
import os

import cadquery as cq

from guardianbot_geometry import (CAM_PITCH, CAM_X, CAM_Z, OV7670_BOARD, TILT_PIVOT, SG90_BODY, SG90_TABS, SG90_SHAFT_END, SG90_Y0, TILT_STOPS_DEG, TOP_Z1, ARM_L, BASE_Z1, HINGE_X, HINGE_Y, MID_Z0, POCKET_FLARE_X, POCKET_IN_HW,
                                  POCKET_MOUTH_HW, POCKET_WALL_T, POCKET_X, SERVO_BODY, TCS_FRONT, TOF,
                                  TOP_Z0, US, MID_Z1)

MM = 1000.0
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "3d_prints")
M25, M3 = 2.7, 3.2
INS_M3 = 4.0                       # M3 heat-set insert hole (or tap M3 directly into 2.5 mm)
TOF_W, TOF_H = 10.7, 25.0          # GY-VL53L0XV2 board (upright, sensor ~4 mm from the top edge)
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


def tof_post(zc):
    """Upright plate for a GY-VL53L0XV2. Foot: 2 x M3 on the viewing axis, 6.5 and 14 mm behind the
    sensor face (matches make_acrylic_dxf.tof_post_holes). The board sits in a 1.8 mm pocket on the
    front face (sensor centre at height zc above the floor) and is held by one M2 screw or hot glue."""
    h = zc - BASE_TOP + 6.0                                              # sensor ~6 mm below the board top
    foot = box(18, 12, 3, at=(-10.25, 0, 0))
    plate = box(3, TOF_W + 4, h, at=(-2.5 - 1.5, 0, 0))
    rib = box(10, 2.4, h * 0.45, at=(-9, 0, 0))
    b = foot.union(plate).union(rib)
    for k in (6.5, 14.0):
        b = b.cut(zhole(M3, 6, (-k, 0, -1)))
    b = b.cut(box(1.8, TOF_W + 0.4, TOF_H, at=(-2.5 + 0.9 - 1.8 - 0.01 + 0.9, 0, h - TOF_H + 0.01)))   # board pocket
    b = b.cut(cq.Workplane("YZ").circle(1.1).extrude(8).translate((-8, 0, h - TOF_H + 3.0)))       # M2 at the board's lower hole
    b = b.cut(box(4, 6, 4, at=(-4, 0, h - TOF_H - 2)))                                            # header / cable clearance
    return b


def tof_back_hanger():
    """Back VL53L0X under the top deck at TOF['tof_back'] (sensor centre z 90). 2 x M3 into the top deck
    at (x + 10, y +- 6)."""
    drop = TOP_BOT - TOF["tof_back"][2] * MM                             # 15 mm below the top deck
    flange = box(16, 18, 3, at=(4, 0, 0))
    h = drop + 7.0
    plate = box(3, TOF_W + 4, h, at=(-1.5 + 0.0, 0, -h + 3))
    b = flange.union(plate)
    for k in (-6.0, 6.0):
        b = b.cut(zhole(M3, 6, (10.0, k, -1)))
    b = b.cut(box(1.8, TOF_W + 0.4, TOF_H, at=(-2.1, 0, 3 - h - 0.01)))
    return b


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
    """Sleeve around the M0170 body (28.5 x 13 x 27) standing on the PCB deck, with a cap that reaches
    the top deck underside (105 - 71.6 = 33.4 mm) and one M3 screw up into the top deck at the
    servo centre-ish (SERVO_TOP_HOLES). The output shaft goes down through the PCB hub hole."""
    L, W, H = SERVO_BODY[0] * MM, SERVO_BODY[1] * MM, TOP_BOT - MID_Z1 * MM - 0.4
    b = box(L + 6, W + 6, H)
    b = b.cut(box(L + 0.4, W + 0.4, SERVO_BODY[2] * MM + 0.5, at=(0, 0, -0.01)))     # servo pocket (open bottom)
    b = b.cut(box(L + 12, W - 4, 6, at=(0, 0, 8)))                                    # mounting-ear slots
    b = b.cut(box(6, W + 8, 5, at=(-L / 2, 0, -0.01)))                               # cable exit
    # M3 hole at SERVO_TOP_HOLES = (-46, +-34.5); servo centre is (HINGE_X + L/2 - 7, HINGE_Y)
    hx = -46.0 - (HINGE_X * MM + L / 2 - 7.0)
    b = b.cut(zhole(2.6, 10, (hx, 0, H - 9)))                                         # tap M3 / self-tapper
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


def _tilt_frame():
    """Mast frame: origin = lens x on the top-deck surface (x forward, z up), in mm."""
    px, pz = TILT_PIVOT[0] * MM - CAM_X * MM, TILT_PIVOT[1] * MM - TOP_Z1 * MM
    return (px, pz), CAM_Z * MM - TOP_Z1 * MM


TAIL_DEG, TAIL_R, PIN_R = 150.0, 12.0, 1.75       # bracket tail direction in the maze view, stop radius, pin radius


def _stop_pins():
    """Centres (x, z) of the two stop pins: the tail (4 mm wide) touches them at TILT_STOPS_DEG."""
    (px, pz), _ = _tilt_frame()
    out = []
    for k, ang in enumerate(TILT_STOPS_DEG):
        f = math.radians(TAIL_DEG + ang)
        t = (math.cos(f), math.sin(f))
        n = (math.sin(f), -math.cos(f)) if k == 0 else (-math.sin(f), math.cos(f))   # CW side at the low stop, CCW at the high
        out.append((px + TAIL_R * t[0] + (2 + PIN_R) * n[0], pz + TAIL_R * t[1] + (2 + PIN_R) * n[1]))
    return out


def camera_mast():
    """v4.2: mast plate (xz plane) with a window for an SG90 tilt servo. The servo's shaft (+y) is the tilt axis,
    26 mm behind and 11 mm above the lens; its tabs screw to the plate's +y face (2 x M2 self-tappers, 27.8 mm apart).
    Two 3.5 mm stop pins limit the camera bracket's tail to -2 deg (maze view, no stall) and +29 deg.
    The plate is cut away under the servo's front so the camera plate can swing past it.
    Foot: 2 x M3 into the top deck at (CAM_X - 18, +-10), as in v4.0 / v4.1."""
    (px, pz), zl = _tilt_frame()
    L, W, H = (v * MM for v in SG90_BODY)
    tab_len, tab_t, tab_z, hole_pitch = (v * MM for v in SG90_TABS)
    y_tab = SG90_Y0 * MM + tab_z                                # plane of the tabs (= plate's +y face)
    z_top = pz + SG90_SHAFT_END * MM                            # case end next to the shaft
    z_bot = z_top - L
    pins = _stop_pins()
    foot = box(24, 36, 3, at=(-19, 0, 0))
    for s_ in (1, -1):
        foot = foot.cut(zhole(M3, 6, (-18, s_ * 10, -1)))
    x0 = min(c[0] for c in pins) - PIN_R - 1
    x1 = px + W / 2 + 3
    plate_top = max(z_top + (tab_len - L) / 2, max(c[1] for c in pins) + PIN_R) + 1.5
    plate = box(x1 - x0, 3, plate_top, at=((x0 + x1) / 2, y_tab - 1.5, 0))
    plate = plate.cut(box(W + 0.6, 10, L + 0.6, at=(px, y_tab - 1.5, z_bot - 0.3)))           # servo window
    plate = plate.cut(box(x1 - px + 4, 10, z_bot - 3.3, at=((px + x1) / 2 + 2, y_tab - 1.5, 3)))   # notch for the camera plate
    zc = (z_top + z_bot) / 2
    for dz in (-hole_pitch / 2, hole_pitch / 2):                                                # tab screw holes
        plate = plate.cut(cq.Workplane("XZ").circle(0.9).extrude(-10).translate((px, y_tab - 5, zc + dz)))
    rib = box(3, 16, plate_top - 4, at=(x0 + 1.5, y_tab - 1.5, 0))                             # rear rib (yz)
    mast = foot.union(plate).union(rib)
    for cx, cz in pins:                                                                         # pins: plate face -> y 19
        mast = mast.union(cq.Workplane("XZ").circle(PIN_R).extrude(-(19 - y_tab + 1)).translate((cx, y_tab - 1, cz)))
    return mast


def camera_tilt_bracket(angle_deg=0.0):
    """v4.2: the moving part. OV7670 plate (3 mm, square to the lens axis, 4 x M2 holes 28 mm apart, 16 mm lens
    window) joined by a side arm to the SG90 horn (the single-arm horn screws on with 2 x M2 x 5 at 7 mm from the axis),
    plus a short tail that meets the mast's stop pins. Drawn in the maze view (angle 0); angle_deg rotates it for checks."""
    (px, pz), zl = _tilt_frame()
    bw, bh, bt = (v * MM for v in OV7670_BOARD)
    y_horn = SG90_Y0 * MM + SG90_BODY[2] * MM + 4.0 + 1.5           # shaft (4 mm) + horn (1.5 mm) above the case
    plate = box(3, bw + 4, bh + 2, at=(0, 0, -(bh + 2) / 2))
    plate = plate.cut(cq.Workplane("YZ").rect(16, 16).extrude(10, both=True))
    for dy in (-14, 14):
        for dz in (-14, 14):
            plate = plate.cut(cq.Workplane("YZ").circle(1.1).extrude(10, both=True).translate((0, dy, dz)))
    plate = plate.translate((-1.5, 0, 0)).rotate((0, 0, 0), (0, 1, 0), math.degrees(CAM_PITCH)).translate((-8.5, 0, zl))
    # side arm (xz plane, y from the horn face to the plate's +y edge): pivot hub -> plate centre, and the tail
    yw = (bw + 4) / 2 - y_horn
    pcx = -8.5 - 1.5 * math.cos(CAM_PITCH)
    pcz = zl + 1.5 * math.sin(CAM_PITCH)
    hub = cq.Workplane("XZ").circle(9).extrude(-yw).translate((px, y_horn, pz))
    ang = math.atan2(pcz - pz, pcx - px)
    ln = math.hypot(pcx - px, pcz - pz)
    bar = (cq.Workplane("XZ").rect(ln, 12).extrude(-yw)
           .rotate((0, 0, 0), (0, 1, 0), -math.degrees(ang)).translate(((px + pcx) / 2, y_horn, (pz + pcz) / 2)))
    tail = (cq.Workplane("XZ").rect(TAIL_R + 3, 4).extrude(-yw).translate(((TAIL_R + 3) / 2, 0, 0))
            .rotate((0, 0, 0), (0, 1, 0), -TAIL_DEG).translate((px, y_horn, pz)))
    arm = hub.union(bar).union(tail)
    arm = arm.cut(cq.Workplane("XZ").circle(3.2).extrude(-20).translate((px, y_horn - 5, pz)))   # horn screw / boss
    for dx in (-7, 7):
        arm = arm.cut(cq.Workplane("XZ").circle(0.9).extrude(-20).translate((px + dx, y_horn - 5, pz)))
    b = plate.union(arm)
    if angle_deg:
        b = b.rotate((px, 0, pz), (px, 1, pz), -angle_deg)
    return b


def deck_spacer(length):
    """Spacer tube between decks, clamped by M3 threaded rod (base-PCB 54.5 mm, PCB-top 33.4 mm)."""
    return cq.Workplane("XY").circle(3.75).extrude(length).cut(zhole(3.5, length + 2, (0, 0, -1)))


def caster_spacer(thickness=2.0):
    b = box(10, 30, thickness)
    for k in (-9.5, 9.5):
        b = b.cut(cq.Workplane("XY").slot2D(7, M25, 90).extrude(5).translate((0, k, -1)))
    return b


PARTS = {
    "n20_clamp": n20_clamp,
    "tof_post_40": lambda: tof_post(TOF["tof_lf"][2] * MM),
    "tof_post_60": lambda: tof_post(TOF["tof_front"][2] * MM),
    "tof_back_hanger": tof_back_hanger,
    "hcsr04_hanger": hcsr04_hanger,
    "servo_cradle": servo_cradle,
    "gripper_arm_left": lambda: gripper_arm("left"),
    "gripper_arm_right": lambda: gripper_arm("right"),
    "pocket": pocket,
    "tcs_front_bracket": tcs_front_bracket,
    "tcs_floor_spacer": tcs_floor_spacer,
    "caster_spacer": caster_spacer,
    "camera_mast": camera_mast,
    "camera_tilt_bracket": camera_tilt_bracket,
    "deck_spacer_54p5": lambda: deck_spacer(54.5),
    "deck_spacer_33p4": lambda: deck_spacer(33.4),
}

if __name__ == "__main__":
    for name, fn in PARTS.items():
        shp = fn()
        bb = shp.val().BoundingBox()
        save(shp, name)
        print("%-20s %5.1f x %5.1f x %5.1f mm   %5.1f cm3" % (name, bb.xlen, bb.ylen, bb.zlen, shp.val().Volume() / 1000))

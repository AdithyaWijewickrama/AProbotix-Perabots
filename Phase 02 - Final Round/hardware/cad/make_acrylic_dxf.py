"""
make_acrylic_dxf.py - GuardianBot v3 laser-cut decks (3 mm acrylic or 3 mm plywood). Units: mm.

Top view, +X = robot forward, +Y = robot left, origin = robot centre (wheel-axle centre).
Writes into ../acrylic/:
  deck_base_3mm.dxf    base: wheel notches, N20 clamps, caster, line array, floor TCS3200,
                       Sharp side tower, front TCS bracket, pocket, battery strap slots
  deck_mid_3mm.dxf     mid: L298N, 2 x LM2596, MPU6050, front Sharp hanger, servo cradles + hub notches
  deck_top_3mm.dxf     top: Arduino Mega 2560, ESP32-S3 holder, 4 x HC-SR04 hangers, START/CAL, 2 LEDs
  sheet_layout.dxf     all three decks on one 480 x 160 mm sheet
  deck_*.stl           3 mm plates (for the assembly preview)
Every hole position comes from guardianbot_geometry.py (same numbers as the Webots model).
Run:  python make_acrylic_dxf.py      (needs: pip install cadquery ezdxf)
"""
import math
import os

import cadquery as cq
import ezdxf

from guardianbot_geometry import (BASE_STANDOFFS, BATTERY, CASTER_X, HINGE_X, HINGE_Y, LINE_X, MID_R,
                                  MODULES, POCKET_HW, POCKET_X, R_BODY, SERVO_BODY, SHARP, TCS_FLOOR,
                                  TCS_FRONT, TOP_BUTTONS, TOP_LEDS, TOP_R, TOP_STANDOFFS, TRACK_HALF, US,
                                  WHEEL_R, WHEEL_W)

MM = 1000.0
T = 3.0
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "acrylic")
M2, M25, M3 = 2.2, 2.7, 3.2

# Arduino Mega 2560 mounting holes in board coordinates (mm, origin at the USB/power-jack corner)
MEGA_HOLES = [(13.97, 2.54), (15.24, 50.8), (66.04, 7.62), (66.04, 35.56), (90.17, 50.8), (96.52, 2.54)]
MEGA_L, MEGA_W = 101.6, 53.3


def m(v):
    return v * MM


class Plate:
    """Deck as a 3 mm solid; the DXF is its mid-plane section."""

    def __init__(self, r):
        self.solid = cq.Workplane("XY").circle(r).extrude(T)

    def cut_box(self, x, y, w, h):
        self.solid = self.solid.cut(cq.Workplane("XY").box(w, h, T + 2).translate((x, y, T / 2)))

    def hole(self, x, y, d):
        self.solid = self.solid.cut(cq.Workplane("XY").circle(d / 2).extrude(T + 2).translate((x, y, -1)))

    def slot(self, x, y, length, width, ang=0.0):
        s = cq.Workplane("XY").slot2D(length, width, ang).extrude(T + 2).translate((x, y, -1))
        self.solid = self.solid.cut(s)


def standoffs(p, pts, d=M3):
    for x, y in pts:
        p.hole(m(x), m(y), d)


# ------------------------------------------------------------------ base deck
def base():
    p = Plate(m(R_BODY))
    px, phw = m(POCKET_X), m(POCKET_HW)
    p.cut_box((px - 80) / 2, 0, 80 + px, 2 * phw)                      # rear pocket notch
    wx, wy0 = m(WHEEL_R) + 2.0, m(TRACK_HALF - WHEEL_W / 2) - 2.0
    for s in (1, -1):
        p.cut_box(0, s * (wy0 + 15), 2 * wx, 30)                       # wheel notches
    standoffs(p, BASE_STANDOFFS)
    for s in (1, -1):
        for x in (-11.0, 11.0):
            p.hole(x, s * 41.0, M25)                                    # N20 clamp screws
        p.slot(0, s * 26.0, 8.0, 4.0)                                   # encoder cable
    for s in (1, -1):
        p.slot(m(CASTER_X), s * 9.5, 7.0, M25, 90)                      # ball caster (14-24 mm spacing)
    for s in (1, -1):
        p.slot(m(LINE_X), s * 27.0, 8.0, M3, 90)                        # line array end holes
    p.slot(m(LINE_X) - 9.0, 0, 12.0, 5.0, 90)
    tx, ty = m(TCS_FLOOR[0]), m(TCS_FLOOR[1])
    for dx, dy in ((-11.0, -11.0), (11.0, 11.0)):
        p.hole(tx + dx, ty + dy, M3)                                    # floor TCS3200
    p.cut_box(tx, ty, 10.0, 6.0)
    sx = m(SHARP["ir_left"][0])
    for dx in (-26.0, 18.0):
        p.hole(sx + dx, 0, M3)                                          # Sharp side tower
    fx = m(TCS_FRONT[0]) - 8.0
    for s in (1, -1):
        p.hole(fx, s * 15.0, M3)                                        # front TCS bracket
    for s in (1, -1):
        for x in (-20.0, -48.0):
            p.hole(x, s * 40.0, M25)                                    # pocket tabs
    (bx, by), (bl, bw, bh) = BATTERY
    for s in (1, -1):
        p.slot(m(bx), s * (m(bw) / 2 + 4.0), 22.0, 3.2)                 # LiPo strap
    for s in (1, -1):
        p.slot(-36.0, s * 46.0, 12.0, 6.0)                              # cables
    return p


# ------------------------------------------------------------------ mid deck
def mid():
    p = Plate(m(MID_R))
    px, phw = m(POCKET_X), m(POCKET_HW)
    p.cut_box((px - 80) / 2, 0, 80 + px, 2 * phw - 4.0)
    standoffs(p, BASE_STANDOFFS)
    standoffs(p, TOP_STANDOFFS)
    for s in (1, -1):
        hx, hy = m(HINGE_X), s * m(HINGE_Y)
        p.hole(hx, hy, 10.0)                                            # servo output hub
        p.cut_box(hx - 8.0, hy, 16.0, 10.0)                             # open to the edge
        cx = hx + m(SERVO_BODY[0]) / 2 - 7.0
        for dy in (-12.0, 12.0):
            p.hole(cx + 4.0, hy + dy, M3)                               # servo cradle
    (lx, ly), _, _ = MODULES["l298n"]
    for dx in (-18.5, 18.5):
        for dy in (-18.5, 18.5):
            p.hole(m(lx) + dx, m(ly) + dy, M3)
    for name in ("buck5", "buck6"):
        (bx, by), _, _ = MODULES[name]
        p.slot(m(bx) - 15.0, m(by) - 7.5, 6.0, M3)
        p.slot(m(bx) + 15.0, m(by) + 7.5, 6.0, M3)
    (ix, iy), _, _ = MODULES["mpu6050"]
    for dx in (-7.6, 7.6):
        p.hole(m(ix) + dx, m(iy) + 7.5, M3)
    (vx, vy), _, _ = MODULES["level"]
    for dx in (-8.0, 8.0):
        p.slot(m(vx) + dx, m(vy), 5.0, 2.2, 90)
    for s in (1, -1):
        p.hole(46.0, s * 18.0, M3)                                      # front Sharp hanger
    p.hole(58.0, 0, M3)
    p.slot(12.0, 0, 16.0, 8.0)
    for s in (1, -1):
        p.slot(-36.0, s * 46.0, 12.0, 6.0)
    return p


# ------------------------------------------------------------------ top deck
def top():
    p = Plate(m(TOP_R))
    standoffs(p, TOP_STANDOFFS)
    (mx, my), _, _ = MODULES["mega"]
    for bx, by in MEGA_HOLES:
        p.hole(m(mx) + (by - MEGA_W / 2), m(my) - (bx - MEGA_L / 2), M3)
    (ex, ey), (el, ew, eh), _ = MODULES["esp32s3"]
    for s in (1, -1):
        p.slot(m(ex) + 16.0, s * 30.0, 8.0, 3.2, 0)                     # ESP32-S3 strap
    for name, (x, y, z, a) in US.items():
        r = math.hypot(x, y) * MM - 14.0
        for k in (-12.0, 12.0):
            p.hole(r * math.cos(a) - k * math.sin(a), r * math.sin(a) + k * math.cos(a), M3)
    for x, y in TOP_BUTTONS.values():
        p.hole(m(x), m(y), 7.0)
    for x, y in TOP_LEDS.values():
        p.hole(m(x), m(y), 5.1)
    return p


def export(p, name):
    os.makedirs(OUT, exist_ok=True)
    cq.exporters.export(p.solid, os.path.join(OUT, name + ".stl"), tolerance=0.05)
    cq.exporters.exportDXF(p.solid.section(T / 2), os.path.join(OUT, name + ".dxf"))
    return os.path.join(OUT, name + ".dxf")


def layout(files):
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    doc.layers.add("CUT", color=1)
    doc.layers.add("INFO", color=3)
    msp = doc.modelspace()
    for i, f in enumerate(files):
        ox = 80.0 + 160.0 * i
        for e in ezdxf.readfile(f).modelspace():
            c = e.copy()
            c.translate(ox, 80.0, 0)
            c.dxf.layer = "CUT"
            msp.add_entity(c)
        msp.add_text(os.path.basename(f)[:-4], height=4, dxfattribs={"layer": "INFO", "insert": (ox - 30, 2)})
    msp.add_lwpolyline([(0, 0), (480, 0), (480, 160), (0, 160)], close=True, dxfattribs={"layer": "INFO"})
    doc.saveas(os.path.join(OUT, "sheet_layout.dxf"))


if __name__ == "__main__":
    files = []
    for name, fn in (("deck_base_3mm", base), ("deck_mid_3mm", mid), ("deck_top_3mm", top)):
        p = fn()
        files.append(export(p, name))
        bb = p.solid.val().BoundingBox()
        print("%-15s %.0f x %.0f mm" % (name, bb.xlen, bb.ylen))
    layout(files)
    print("written to", os.path.normpath(OUT))

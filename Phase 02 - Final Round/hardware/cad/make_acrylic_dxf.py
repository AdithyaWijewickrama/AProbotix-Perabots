"""
make_acrylic_dxf.py - GuardianBot v4 laser-cut decks (3 mm acrylic or 3 mm plywood). Units: mm.

Top view, +X = robot forward, +Y = robot left, origin = robot centre (wheel-axle centre).
The MIDDLE floor is the PCB deck (hardware/pcb_v4). Writes into ../acrylic/:
  deck_base_3mm.dxf    base: wheel notches, N20 clamps, caster, QTR-8RC, floor TCS3200, 7 ToF posts,
                       front TCS bracket, pocket, 2S LiPo strap slots, cable slots
  deck_mid_3mm_vero_fallback.dxf
                       ONLY if the PCB is not made: the same outline and holes as the PCB deck, plus
                       slots for the wires; a vero board with the same circuit is screwed on top
  deck_top_3mm.dxf     top: camera mast, ESP32-S3 carrier, 2 x HC-SR04 hangers, back ToF hanger, OLED, keypad, 2 LEDs,
                       servo cradle caps, cable slots
  sheet_layout.dxf     all three decks on one 480 x 160 mm sheet
  deck_*.stl           3 mm plates (for the assembly preview)
Every hole position comes from guardianbot_geometry.py (same numbers as the Webots model).
Run:  python make_acrylic_dxf.py      (needs: pip install cadquery ezdxf)
"""
import math
import os

import cadquery as cq
import ezdxf

import sys

from guardianbot_geometry import (CAM_X, S3_CARRIER_HOLES, BASE_STANDOFFS, BATTERY, CASTER_X, LINE_X,
                                  MODULES, POCKET_HW, POCKET_X, R_BODY, SERVO_TOP_HOLES, TCS_FLOOR,
                                  TCS_FRONT, TOF, TOP_BUTTONS, TOP_LEDS, TOP_R, TOP_STANDOFFS, TRACK_HALF, US,
                                  WHEEL_R, WHEEL_W)

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "deck_v4"))
import deck_design as DD  # noqa: E402

MM = 1000.0
T = 3.0
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "acrylic")
M2, M25, M3 = 2.2, 2.7, 3.2

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


TOF_POST_HOLES = (6.5, 14.0)   # the two M3 feet of a printed ToF post: distance behind the sensor face


def tof_post_holes(p, name):
    """Foot of a printed ToF post: 2 x M3 on the viewing axis, behind the sensor face."""
    x, y, z, a = TOF[name]
    for k in TOF_POST_HOLES:
        p.hole(m(x) - k * math.cos(a), m(y) - k * math.sin(a), M3)


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
        p.slot(m(LINE_X), s * 34.6, 6.0, 2.4, 90)                       # QTR-8RC end holes (#2 / M2)
    p.slot(m(LINE_X) - 9.5, 0, 14.0, 5.0, 90)                          # QTR cable (11-pin header)
    tx, ty = m(TCS_FLOOR[0]), m(TCS_FLOOR[1])
    for dx, dy in ((-11.0, -11.0), (11.0, 11.0)):
        p.hole(tx + dx, ty + dy, M3)                                    # floor TCS3200
    p.cut_box(tx, ty, 10.0, 6.0)
    for name in ("tof_front", "tof_fl", "tof_fr", "tof_lf", "tof_lr", "tof_rf", "tof_rr"):
        tof_post_holes(p, name)                                         # 7 ToF posts stand on the base
    fx = m(TCS_FRONT[0]) - 8.0
    for s in (1, -1):
        p.hole(fx, s * 15.0, M3)                                        # front TCS bracket
    for s in (1, -1):
        for x in (-20.0, -48.0):
            p.hole(x, s * 40.0, M25)                                    # pocket tabs
    (bx, by), (bl, bw, bh) = BATTERY
    for s in (1, -1):
        p.slot(m(bx) - 6.0, s * (m(bw) / 2 + 4.0), 12.0, 3.2)           # LiPo strap (velcro)
    for s in (1, -1):
        p.slot(-46.0, s * 45.0, 10.0, 6.0)                              # cables up to the PCB deck
    return p


# ------------------------------------------------------------------ mid deck (vero fallback only)
def mid():
    """Acrylic copy of the PCB deck (same outline, cut-outs and holes) for the vero-board fallback."""
    p = Plate(DD.R_DECK)
    for x0, y0, x1, y1 in DD.CUTOUTS:
        p.cut_box((x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0)
    for hx, hy in DD.HUBS:
        p.hole(hx, hy, DD.HUB_D)
    for hx, hy, d in DD.HOLES:
        p.hole(hx, hy, d)
    for x, y in ((-6.0, 58.0), (-6.0, -58.0), (30.0, 0.0), (-40.0, 0.0), (55.0, 40.0), (55.0, -40.0)):
        p.slot(x, y, 14.0, 6.0, 90)                                     # wire slots
    return p


# ------------------------------------------------------------------ top deck
def top():
    p = Plate(m(TOP_R))
    standoffs(p, TOP_STANDOFFS)
    for s in (1, -1):
        p.hole(m(CAM_X) - 18.0, s * 10.0, M3)                           # camera mast foot
    for name, (x, y, z, a) in US.items():
        r = math.hypot(x, y) * MM - 14.0
        for k in (-12.0, 12.0):
            p.hole(r * math.cos(a) - k * math.sin(a), r * math.sin(a) + k * math.cos(a), M3)
    x, y, z, a = TOF["tof_back"]
    for k in (-6.0, 6.0):
        p.hole(m(x) + 10.0, m(y) + k, M3)                               # back ToF hanger
    (ox, oy), (ol, ow, oh), _ = MODULES["oled"]
    for dx in (-ol * MM / 2 + 2.0, ol * MM / 2 - 2.0):
        for dy in (-ow * MM / 2 + 2.0, ow * MM / 2 - 2.0):
            p.hole(m(ox) + dx, m(oy) + dy, 2.2)                         # OLED M2 corner holes (check your module)
    p.slot(m(ox) + 4.0, m(oy) - ow * MM / 2 - 4.0, 10.0, 4.0)          # OLED cable
    (kx, ky), _, _ = MODULES["keypad"]
    p.slot(m(kx) + 12.0, m(ky), 10.0, 3.0, 90)                          # keypad ribbon (keypad is self-adhesive)
    for x, y in SERVO_TOP_HOLES:
        p.hole(m(x), m(y), M3)                                          # servo cradle caps
    for x, y in TOP_BUTTONS.values():
        p.hole(m(x), m(y), 7.0)
    for x, y in TOP_LEDS.values():
        p.hole(m(x), m(y), 5.1)
    for x, y in S3_CARRIER_HOLES:
        p.hole(m(x), m(y), M3)                                          # ESP32-S3 carrier (flat, behind the mast)
    p.slot(30.0, 0, 16.0, 6.0)                                          # S3 link cable / sonar cables
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
    for name, fn in (("deck_base_3mm", base), ("deck_mid_3mm_vero_fallback", mid), ("deck_top_3mm", top)):
        p = fn()
        files.append(export(p, name))
        bb = p.solid.val().BoundingBox()
        print("%-15s %.0f x %.0f mm" % (name, bb.xlen, bb.ylen))
    layout(files)
    print("written to", os.path.normpath(OUT))

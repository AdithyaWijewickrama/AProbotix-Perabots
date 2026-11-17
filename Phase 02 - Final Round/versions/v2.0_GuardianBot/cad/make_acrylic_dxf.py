"""
make_acrylic_dxf.py - laser-cut files for GuardianBot (units: millimetres, top view, +X = robot forward,
+Y = robot left; origin = robot centre = wheel-axle centre).

  ../acrylic/acrylic_base_3mm_black.dxf   base plate (motors, caster, line array, battery, Sharp brackets)
  ../acrylic/acrylic_crown_3mm_clear.dxf  crown disc (4 x HC-SR04 hangers, camera mast)
  ../acrylic/acrylic_sheet_layout.dxf     both parts side by side (260 x 144 mm)

Layers: CUT (red, through cut), ENGRAVE (blue, raster/vector engrave), INFO (green, not cut).
Run with:  python make_acrylic_dxf.py
"""
import math
import os

import ezdxf

from guardianbot_geometry import (BASE_STANDOFFS, BATTERY, CASTER_X, CROWN_R, CROWN_STANDOFFS, LINE_X,
                                  POCKET_HW, POCKET_X, R_BODY, SHARP, TRACK_HALF, US, WHEEL_R, WHEEL_W)

MM = 1000.0
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "acrylic")
M2, M25, M3 = 2.2, 2.7, 3.2           # clearance hole diameters


def new_doc():
    doc = ezdxf.new("R2010", setup=True)
    doc.units = ezdxf.units.MM
    doc.layers.add("CUT", color=1)
    doc.layers.add("ENGRAVE", color=5)
    doc.layers.add("INFO", color=3)
    return doc


def hole(msp, x, y, d, ox=0, oy=0):
    msp.add_circle((ox + x, oy + y), d / 2, dxfattribs={"layer": "CUT"})


def slot(msp, x, y, length, width, ang=0.0, ox=0, oy=0):
    """Obround slot centred at (x, y)."""
    r = width / 2
    c, s = math.cos(ang), math.sin(ang)
    h = (length - width) / 2
    p1 = (ox + x - c * h, oy + y - s * h)
    p2 = (ox + x + c * h, oy + y + s * h)
    nx, ny = -s * r, c * r
    a = math.degrees(ang)
    msp.add_line((p1[0] + nx, p1[1] + ny), (p2[0] + nx, p2[1] + ny), dxfattribs={"layer": "CUT"})
    msp.add_line((p1[0] - nx, p1[1] - ny), (p2[0] - nx, p2[1] - ny), dxfattribs={"layer": "CUT"})
    msp.add_arc(p2, r, a - 90, a + 90, dxfattribs={"layer": "CUT"})
    msp.add_arc(p1, r, a + 90, a + 270, dxfattribs={"layer": "CUT"})


def rect(msp, x, y, w, h, ox=0, oy=0, layer="CUT"):
    pts = [(ox + x - w / 2, oy + y - h / 2), (ox + x + w / 2, oy + y - h / 2),
           (ox + x + w / 2, oy + y + h / 2), (ox + x - w / 2, oy + y + h / 2)]
    msp.add_lwpolyline(pts, close=True, dxfattribs={"layer": layer})


# ------------------------------------------------------------------ base plate
def base_outline_mm():
    """Circle R65 with the rear pocket notch (62 mm wide) and two wheel wells."""
    R = R_BODY * MM
    wx = WHEEL_R * MM + 2.5                      # wheel well half-length along x (22.5 mm)
    wy = (TRACK_HALF - WHEEL_W / 2) * MM - 1.5   # inner edge of the wheel well (50.5 mm)
    px, phw = POCKET_X * MM, POCKET_HW * MM
    xc = -math.sqrt(R * R - phw * phw)
    yc = math.sqrt(R * R - wx * wx)
    pts = []

    def arc(a0, a1, n):
        for i in range(n + 1):
            a = a0 + (a1 - a0) * i / n
            pts.append((R * math.cos(a), R * math.sin(a)))

    pts += [(px, -phw), (xc, -phw)]
    arc(math.atan2(-phw, xc), math.atan2(-yc, -wx), 12)
    pts += [(-wx, -wy), (wx, -wy)]
    arc(math.atan2(-yc, wx), math.atan2(yc, wx), 60)
    pts += [(wx, wy), (-wx, wy)]
    arc(math.atan2(yc, -wx), math.atan2(phw, xc), 12)
    pts += [(px, phw)]
    return pts


def draw_base(msp, ox=0.0, oy=0.0):
    pts = [(ox + x, oy + y) for x, y in base_outline_mm()]
    msp.add_lwpolyline(pts, close=True, dxfattribs={"layer": "CUT"})
    # standoffs base <-> PCB (M3)
    for x, y in BASE_STANDOFFS:
        hole(msp, x * MM, y * MM, M3, ox, oy)
    # motor brackets (Pololu #989 style): 2 x M2 per bracket, 18 mm apart along x
    for s in (1, -1):
        by = s * 45.5
        for dx in (-9.0, 9.0):
            hole(msp, dx, by, M2, ox, oy)
    # ball caster (Pololu 3/8" metal ball caster): 2 x M2, 12.7 mm apart across
    for dy in (-6.35, 6.35):
        hole(msp, CASTER_X * MM, dy, M2, ox, oy)
    # line array QTRX-MD-08A: 2 x M2 near the board ends + wire slot behind it
    for dy in (-26.5, 26.5):
        hole(msp, LINE_X * MM, dy, M2, ox, oy)
    slot(msp, LINE_X * MM - 15.0, 0, 14, 4, 0, ox, oy)
    # battery strap slots (hook-and-loop strap around the LiPo)
    (bx, by), (bsx, bsy, _) = BATTERY
    for s in (1, -1):
        slot(msp, bx * MM, s * (bsy * MM / 2 + 4.0), 14, 3.2, 0, ox, oy)
    # Sharp IR brackets: 2 x M2 each, on the bracket foot 9 mm inside the lens face
    for n, (x, y, z, a) in SHARP.items():
        cx, cy = (x * MM - 9.0 * math.cos(a)), (y * MM - 9.0 * math.sin(a))
        tx, ty = -math.sin(a), math.cos(a)
        for k in (-6.0, 6.0):
            hole(msp, cx + k * tx, cy + k * ty, M2, ox, oy)
    # PTFE anti-tip skids (M3 screw-in domes)
    for s in (1, -1):
        hole(msp, -40.0, s * 45.0, M3, ox, oy)
    # pocket part fixing (M2.5 into heat-set inserts of the printed pocket)
    for s in (1, -1):
        hole(msp, -20.0, s * 39.0, M25, ox, oy)
    # motor / encoder wire pass-throughs
    for s in (1, -1):
        slot(msp, 10.0, s * 11.0, 8, 4, math.pi / 2, ox, oy)
    # engraving
    msp.add_text("GUARDIANBOT  BASE  3mm", dxfattribs={"layer": "ENGRAVE", "height": 3.0}).set_placement(
        (ox - 2.0, oy - 2.0), align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    msp.add_text("FRONT >", dxfattribs={"layer": "ENGRAVE", "height": 3.0}).set_placement(
        (ox + 28.0, oy + 14.0), align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)


# ------------------------------------------------------------------ crown
def draw_crown(msp, ox=0.0, oy=0.0):
    R = CROWN_R * MM
    msp.add_circle((ox, oy), R, dxfattribs={"layer": "CUT"})
    for x, y in CROWN_STANDOFFS:
        hole(msp, x * MM, y * MM, M3, ox, oy)
    # HC-SR04 hangers: 2 x M2 per sensor, tangential 24 mm apart, 47 mm from centre
    for n, (x, y, z, a) in US.items():
        cx, cy = 47.0 * math.cos(a), 47.0 * math.sin(a)
        tx, ty = -math.sin(a), math.cos(a)
        for k in (-12.0, 12.0):
            hole(msp, cx + k * tx, cy + k * ty, M2, ox, oy)
    # camera mast (2 x M2) on the front edge
    for dy in (-6.0, 6.0):
        hole(msp, 50.0, dy, M2, ox, oy)
    # cable pass-through + finger hole for the USB-C access
    msp.add_circle((ox - 8.0, oy), 9.0, dxfattribs={"layer": "CUT"})
    slot(msp, 6.0, -31.0, 18, 7, 0, ox, oy)
    msp.add_text("PERABOTS 2026", dxfattribs={"layer": "ENGRAVE", "height": 4.0}).set_placement(
        (ox + 18.0, oy + 22.0), align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)
    msp.add_text("GUARDIANBOT", dxfattribs={"layer": "ENGRAVE", "height": 4.0}).set_placement(
        (ox + 18.0, oy + 16.0), align=ezdxf.enums.TextEntityAlignment.MIDDLE_CENTER)


def save(doc, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name)
    doc.saveas(p)
    return p


if __name__ == "__main__":
    d = new_doc()
    draw_base(d.modelspace())
    print(save(d, "acrylic_base_3mm_black.dxf"))
    d = new_doc()
    draw_crown(d.modelspace())
    print(save(d, "acrylic_crown_3mm_clear.dxf"))
    d = new_doc()
    m = d.modelspace()
    rect(m, 130, 72, 260, 144, layer="INFO")
    draw_base(m, 70, 72)
    draw_crown(m, 196, 72)
    m.add_text("260 x 144 mm. LEFT part: 3 mm BLACK acrylic (base).  RIGHT part: 3 mm CLEAR acrylic (crown).",
               dxfattribs={"layer": "INFO", "height": 2.5}).set_placement((3, 2))
    print(save(d, "acrylic_sheet_layout.dxf"))

"""
export_models.py - makes the 3-D / 2-D models that the versions website shows (viewer.html).

For every version listed in VERSIONS below it writes versions/<folder>/models/:
  pcb.glb          the PCB in 3-D: board outline with holes and cut-outs, both sides textured with the
                   real copper / solder-mask openings / silkscreen / drill holes, a simple body for every
                   part (made from the footprint name and courtyard, so no 3-D model library is needed),
                   and the Arduino Mega / ESP32 boards where they sit
  deck_*.svg       2-D views of the laser-cut DXF files
  manifest.json    what the viewer lists: PCB, printed parts (STL), acrylic decks (STL + SVG), downloads
The printed-part / deck STL files and the downloads are NOT copied here: build_site.py copies them
straight from where they live in the repo (paths in the manifest are relative to "Phase 02 - Final Round").

Run it on a PC with KiCad 7 (pcbnew python, needs to load the boards), whenever a board or a CAD file changes:
    python export_models.py            (all versions)      python export_models.py v4.0
Needs: KiCad 7, rsvg-convert, pip install trimesh mapbox_earcut shapely pillow ezdxf matplotlib
"""
import glob
import io
import json
import math
import os
import re
import subprocess
import sys
import tempfile

import numpy as np
import pcbnew
import trimesh
from PIL import Image, ImageDraw
from shapely.geometry import Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)                         # "Phase 02 - Final Round"
PXMM = 16                                            # texture resolution, pixels per mm

# ------------------------------------------------------------------ what to export per version
VERSIONS = {
    "v2.0": dict(folder="v2.0_GuardianBot",
                 pcb="versions/v2.0_GuardianBot/hardware/pcb/guardianbot_main.kicad_pcb",
                 prints="versions/v2.0_GuardianBot/hardware/3d_prints",
                 decks="versions/v2.0_GuardianBot/hardware/acrylic",
                 extras="esp_on_sockets",
                 files=["versions/v2.0_GuardianBot/hardware/pcb/guardianbot_main_gerbers_JLCPCB.zip",
                        "versions/v2.0_GuardianBot/hardware/pcb/guardianbot_main_schematic.pdf",
                        "versions/v2.0_GuardianBot/hardware/pcb/guardianbot_main_bom.csv",
                        "versions/v2.0_GuardianBot/hardware/pcb/guardianbot_main.step",
                        "versions/v2.0_GuardianBot/docs/GuardianBot_build_guide_and_BOM.md"]),
    "v3.3": dict(folder="v3.3_shield_pcb",
                 pcb="versions/v3.3_shield_pcb/pcb_v3/guardianbot_v3_mega_shield.kicad_pcb",
                 prints="versions/v3.3_shield_pcb/3d_prints",
                 decks="versions/v3.3_shield_pcb/acrylic",
                 extras="mega_below_esp_on_top",
                 files=["versions/v3.3_shield_pcb/pcb_v3/guardianbot_v3_mega_shield_gerbers.zip",
                        "versions/v3.3_shield_pcb/pcb_v3/guardianbot_v3_mega_shield_schematic.pdf",
                        "versions/v3.3_shield_pcb/pcb_v3/guardianbot_v3_mega_shield_BOM.csv",
                        "versions/v3.3_shield_pcb/pcb_v3/guardianbot_v3_mega_shield-assembly_top_silkscreen.pdf",
                        "versions/v3.3_shield_pcb/GuardianBot_v3_build_guide.md"]),
    "v4.0": dict(folder="v4.0_esp32cam_pcbdeck",
                 pcb="hardware/pcb_v4/guardianbot_v4_pcb_deck.kicad_pcb",
                 prints="hardware/3d_prints",
                 decks="hardware/acrylic",
                 extras="mega_flipped_above",
                 files=["hardware/pcb_v4/guardianbot_v4_pcb_deck_gerbers.zip",
                        "hardware/pcb_v4/guardianbot_v4_pcb_deck_schematic.pdf",
                        "hardware/pcb_v4/guardianbot_v4_pcb_deck_BOM.csv",
                        "hardware/pcb_v4/guardianbot_v4_pcb_deck-assembly_top_silkscreen.pdf",
                        "hardware/pcb_v4/guardianbot_v4_pcb_deck-assembly_bottom_silkscreen_mirrored.pdf",
                        "docs/GuardianBot_v4_build_guide.md"]),
}

# colours (sRGB 0-255)
C = dict(mask=(14, 92, 52), cu=(46, 138, 82), pad=(214, 168, 66), fr4=(190, 172, 120), silk=(245, 245, 240),
         hole=(18, 18, 18), edge=(30, 100, 60), xh=(239, 232, 216), black=(28, 28, 30), gold=(212, 175, 55),
         cap=(31, 60, 122), capTop=(190, 190, 195), res=(201, 180, 138), disc=(217, 130, 43),
         term=(47, 143, 79), mega=(0, 110, 160), silver=(185, 188, 194), esp=(25, 25, 28), chip=(20, 20, 22),
         yellow=(230, 190, 40), grey=(120, 120, 120))


def mm(v):
    return pcbnew.ToMM(v)


# ------------------------------------------------------------------ board outline + textures
def outline(board):
    ps = pcbnew.SHAPE_POLY_SET()
    board.GetBoardPolygonOutlines(ps)
    polys = []
    for i in range(ps.OutlineCount()):
        o = ps.COutline(i)
        ext = [(mm(o.CPoint(k).x), mm(o.CPoint(k).y)) for k in range(o.PointCount())]
        holes = []
        for j in range(ps.HoleCount(i)):
            h = ps.CHole(i, j)
            holes.append([(mm(h.CPoint(k).x), mm(h.CPoint(k).y)) for k in range(h.PointCount())])
        polys.append(Polygon(ext, holes).buffer(0))
    return polys


def plot_layers(board, layers):
    """Plot layers to black-on-white SVG at 1:1 page scale and rasterise them -> {layer: bool array (page px)}."""
    tmp = tempfile.mkdtemp()
    pc = pcbnew.PLOT_CONTROLLER(board)
    po = pc.GetPlotOptions()
    po.SetOutputDirectory(tmp)
    po.SetPlotFrameRef(False)
    po.SetBlackAndWhite(True)
    po.SetMirror(False)
    po.SetScale(1.0)
    po.SetAutoScale(False)
    po.SetDrillMarksType(pcbnew.DRILL_MARKS_NO_DRILL_SHAPE)
    po.SetSubtractMaskFromSilk(True)
    out = {}
    for name, lid in layers:
        pc.SetLayer(lid)
        pc.OpenPlotfile(name, pcbnew.PLOT_FORMAT_SVG, name)
        pc.PlotLayer()
        pc.ClosePlot()
        png = os.path.join(tmp, name + ".png")
        dpi = PXMM * 25.4
        subprocess.run(["rsvg-convert", "-d", str(dpi), "-p", str(dpi), "-b", "white", pc.GetPlotFileName(), "-o", png],
                       check=True)
        out[name] = np.array(Image.open(png).convert("L")) < 128
    return out


def side_texture(board, bbox, side, holes):
    """One side of the board as an RGB image covering bbox (KiCad mm), top-view orientation."""
    cu, mask, silk = ((pcbnew.F_Cu, pcbnew.F_Mask, pcbnew.F_SilkS) if side == "top"
                      else (pcbnew.B_Cu, pcbnew.B_Mask, pcbnew.B_SilkS))
    L = plot_layers(board, [("cu", cu), ("mask", mask), ("silk", silk)])
    x0, y0, x1, y1 = bbox
    i0, j0 = int(round(y0 * PXMM)), int(round(x0 * PXMM))
    h, w = int(round((y1 - y0) * PXMM)), int(round((x1 - x0) * PXMM))

    def crop(a):
        out = np.zeros((h, w), bool)
        sub = a[max(i0, 0):i0 + h, max(j0, 0):j0 + w]
        out[:sub.shape[0], :sub.shape[1]] = sub
        return out
    cuA, maskA, silkA = crop(L["cu"]), crop(L["mask"]), crop(L["silk"])
    img = np.empty((h, w, 3), np.uint8)
    img[:] = C["mask"]
    img[cuA] = C["cu"]
    img[maskA & ~cuA] = C["fr4"]
    img[maskA & cuA] = C["pad"]
    img[silkA & ~maskA] = C["silk"]
    im = Image.fromarray(img)
    d = ImageDraw.Draw(im)
    for x, y, r in holes:
        px, py = (x - x0) * PXMM, (y - y0) * PXMM
        d.ellipse([px - r * PXMM, py - r * PXMM, px + r * PXMM, py + r * PXMM], fill=C["hole"])
    return im.quantize(colors=16).convert("RGB")


def textured_face(poly, z, img, bbox, flip):
    """Flat face of the board outline at height z, UV-mapped onto img (bbox in KiCad mm)."""
    v2, f = trimesh.creation.triangulate_polygon(poly, engine="earcut")
    x0, y0, x1, y1 = bbox
    uv = np.c_[(v2[:, 0] - x0) / (x1 - x0), 1.0 - (v2[:, 1] - y0) / (y1 - y0)]
    v3 = np.c_[v2[:, 0], -v2[:, 1], np.full(len(v2), z)]
    if flip:
        f = f[:, ::-1]
    else:
        f = f.copy()
    m = trimesh.Trimesh(v3, f, process=False)
    m.visual = trimesh.visual.TextureVisuals(uv=uv, material=trimesh.visual.material.PBRMaterial(
        baseColorTexture=img, metallicFactor=0.0, roughnessFactor=0.8))
    return m


# ------------------------------------------------------------------ simple part bodies
def color_mesh(m, rgb):
    m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(list(rgb) + [255], (len(m.faces), 1)))
    return m


def box(cx, cy, z0, z1, sx, sy, rgb):
    """Box centred at KiCad (cx, cy), z0..z1 (mm, up)."""
    m = trimesh.creation.box(extents=(sx, sy, abs(z1 - z0)))
    m.apply_translation((cx, -cy, (z0 + z1) / 2))
    return color_mesh(m, rgb)


def cyl(cx, cy, z0, z1, r, rgb, axis=None, length=None):
    if axis is None:
        m = trimesh.creation.cylinder(radius=r, height=abs(z1 - z0), sections=24)
        m.apply_translation((cx, -cy, (z0 + z1) / 2))
    else:                                    # lying cylinder along axis (dx, dy) in KiCad coordinates
        m = trimesh.creation.cylinder(radius=r, height=length, sections=20)
        ang = math.atan2(-axis[1], axis[0])
        m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, (0, 1, 0)))
        m.apply_transform(trimesh.transformations.rotation_matrix(ang, (0, 0, 1)))
        m.apply_translation((cx, -cy, (z0 + z1) / 2))
    return color_mesh(m, rgb)


def courtyard(fp):
    for lay in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
        c = fp.GetCourtyard(lay)
        if c.OutlineCount():
            bb = c.BBox()
            break
    else:
        bb = fp.GetBoundingBox(False, False)
    return mm(bb.GetLeft()), mm(bb.GetTop()), mm(bb.GetRight()), mm(bb.GetBottom())


def part_bodies(fp, T):
    """Meshes for one footprint. T = board thickness. Bottom-side parts hang below z = 0."""
    name = str(fp.GetFPID().GetLibItemName())
    bottom = fp.IsFlipped()
    pads = list(fp.Pads())
    if not pads or name.startswith("MountingHole"):
        return []
    xa, ya, xb, yb = courtyard(fp)
    cx, cy, sx, sy = (xa + xb) / 2, (ya + yb) / 2, xb - xa, yb - ya
    px = [mm(p.GetPosition().x) for p in pads]
    py = [mm(p.GetPosition().y) for p in pads]
    mx, my = (min(px) + max(px)) / 2, (min(py) + max(py)) / 2
    along = (max(px) - min(px), max(py) - min(py))

    def Z(h0, h1):                            # height range above the surface the part sits on
        return (-h1, -h0) if bottom else (T + h0, T + h1)
    out = []
    if name.startswith("JST_XH"):
        out.append(box(cx, cy, *Z(0, 7.0), sx - 0.6, sy - 0.6, C["xh"]))
        out.append(box(mx, my, *Z(4.0, 7.05), max(along[0], 0.6) + 1.4 if along[0] > along[1] else 1.4,
                       max(along[1], 0.6) + 1.4 if along[1] >= along[0] else 1.4, (200, 192, 176)))
    elif name.startswith("JST_SH"):
        out.append(box(cx, cy, *Z(0, 2.9), sx - 0.8, sy - 0.8, C["xh"]))
    elif name.startswith("PinHeader"):
        out.append(box(mx, my, *Z(0, 2.5), (along[0] or 0) + 2.5, (along[1] or 0) + 2.5, C["black"]))
        for x, y in zip(px, py):
            out.append(box(x, y, *Z(0, 8.5), 0.64, 0.64, C["gold"]))
    elif name.startswith("PinSocket"):
        out.append(box(mx, my, *Z(0, 8.5), (along[0] or 0) + 2.5, (along[1] or 0) + 2.5, C["black"]))
    elif name.startswith("CP_Radial") or name.startswith("CP_Elec"):
        dm = re.search(r"_D([\d.]+)mm", name) or re.search(r"CP_Elec_([\d.]+)x([\d.]+)", name)
        D = float(dm.group(1))
        H = float(dm.group(2)) if name.startswith("CP_Elec") else {5.0: 11.0, 6.3: 11.0, 8.0: 11.5, 10.0: 12.5}.get(D, 11.0)
        z0, z1 = Z(0.5, H)
        out.append(cyl(mx, my, z0, z1, D / 2, C["cap"]))
        zt = z1 if not bottom else z0
        out.append(cyl(mx, my, zt - 0.05, zt + 0.05, D / 2 - 0.4, C["capTop"]))
    elif name.startswith("C_Disc"):
        horiz = along[0] >= along[1]
        out.append(box(mx, my, *Z(1.0, 6.0), 3.0 if horiz else 1.6, 1.6 if horiz else 3.0, C["disc"]))
    elif name.startswith("R_Axial"):
        L = float(re.search(r"_L([\d.]+)mm", name).group(1))
        Dd = float(re.search(r"_D([\d.]+)mm", name).group(1))
        ax = (1, 0) if along[0] >= along[1] else (0, 1)
        z0, z1 = Z(0, Dd)
        out.append(cyl(mx, my, z0, z1, Dd / 2, C["res"], axis=ax, length=L))
        span = max(along) + 0.6
        out.append(box(mx, my, *Z(Dd / 2 - 0.3, Dd / 2 + 0.3), span if ax[0] else 0.6, 0.6 if ax[0] else span, C["silver"]))
    elif name.startswith("TO-92"):
        horiz = along[0] >= along[1]
        out.append(box(mx, my, *Z(2.0, 6.8), 4.8 if horiz else 3.8, 3.8 if horiz else 4.8, C["black"]))
    elif name.startswith("TerminalBlock"):
        out.append(box(cx, cy, *Z(0, 10.0), sx - 0.6, sy - 0.6, C["term"]))
    elif name.startswith("SW_PUSH"):
        out.append(box(mx, my, *Z(0, 3.5), 6.0, 6.0, C["black"]))
        out.append(cyl(mx, my, *Z(3.5, 5.0), 1.75, C["grey"]))
    elif name.startswith("AMASS_XT"):
        out.append(box(cx, cy, *Z(0, 8.0), sx - 0.8, sy - 0.8, C["yellow"]))
    elif re.match(r"(SOIC|SSOP|SOT|QFN|TSSOP)", name):
        out.append(box(cx, cy, *Z(0, 1.6 if name.startswith("SOT") else 2.0), sx - 1.6, sy - 1.6, C["chip"]))
    elif re.match(r"(C|R|LED)_\d{4}", name):
        rgb = {"C": (180, 140, 90), "R": C["chip"], "L": (240, 240, 230)}[name[0]]
        out.append(box(mx, my, *Z(0, 0.8), max(along[0] + 0.8, 1.0), max(along[1] + 0.8, 1.0), rgb))
    elif name:
        out.append(box(cx, cy, *Z(0, 3.0), max(sx - 0.6, 1), max(sy - 0.6, 1), C["grey"]))
    return out


def merge(meshes):
    """Merge meshes that only carry face colours into one mesh (keeps colours)."""
    meshes = [m for m in meshes if m is not None and len(m.faces)]
    return trimesh.util.concatenate(meshes) if meshes else None


# ------------------------------------------------------------------ boards that sit on / under the PCB
def mega_from_pads(board):
    fp = next((f for f in board.GetFootprints() if f.GetReference() == "MEGA1"), None)
    if fp is None:
        return None, None
    return fp, {p.GetNumber(): (mm(p.GetPosition().x), mm(p.GetPosition().y)) for p in fp.Pads()}


def arduino_frame(P):
    """Map Arduino board coords (u from the USB end, v from the power/analog edge) to KiCad mm using
    three known header pins (Arduino Mega 2560 R3 header geometry)."""
    ref = {"A0": (50.8, 2.54), "D7": (45.72, 50.8), "D53": (96.52, 10.16), "D22": (93.98, 48.26)}
    names = [n for n in ref if n in P]
    A = np.array([[ref[n][0], ref[n][1], 1.0] for n in names])
    B = np.array([P[n] for n in names])
    M, *_ = np.linalg.lstsq(A, B, rcond=None)           # affine (u, v, 1) -> (x, y)
    return lambda u, v: tuple(np.array([u, v, 1.0]) @ M)


def quad_box(f, u0, v0, u1, v1, z0, z1, rgb):
    """Box over the board-coordinate rectangle (u0..u1, v0..v1), axis-aligned in KiCad after mapping."""
    pts = [f(u, v) for u, v in ((u0, v0), (u1, v0), (u0, v1), (u1, v1))]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return box((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, z0, z1, max(xs) - min(xs), max(ys) - min(ys), rgb)


def mega_meshes(board, T, mode):
    fp, P = mega_from_pads(board)
    if fp is None:
        return []
    f = arduino_frame(P)
    out = []
    if mode == "mega_flipped_above":            # v4: Mega face-down, PCB 11 mm above the deck
        zb = T + 11.0
        out.append(quad_box(f, 0, 0, 101.6, 53.34, zb, zb + 1.6, C["mega"]))
        for (x, y) in P.values():
            out.append(box(x, y, zb - 8.5, zb, 2.54, 2.54, C["black"]))           # female headers (face down)
            out.append(box(x, y, T, zb - 8.5, 0.64, 0.64, C["gold"]))              # long male pins
        out.append(quad_box(f, -6.2, 31.7, 10.0, 43.8, zb - 11.0, zb, C["silver"]))    # USB-B (hangs down)
        out.append(quad_box(f, -1.8, 3.3, 12.0, 12.3, zb - 11.0, zb, C["black"]))      # DC jack
        out.append(quad_box(f, 30.0, 18.0, 44.0, 32.0, zb - 1.2, zb, C["chip"]))       # ATmega2560
    elif mode == "mega_below_esp_on_top":       # v3.3: shield on top of the Mega, ESP32-S3 on the shield
        zt = -3.0 - 8.5                         # Mega top surface
        out.append(quad_box(f, 0, 0, 101.6, 53.34, zt - 1.6, zt, C["mega"]))
        for (x, y) in P.values():
            out.append(box(x, y, zt, zt + 8.5, 2.54, 2.54, C["black"]))
            out.append(box(x, y, zt + 8.5, 0, 0.64, 0.64, C["gold"]))
        out.append(quad_box(f, -6.2, 31.7, 10.0, 43.8, zt, zt + 10.9, C["silver"]))
        out.append(quad_box(f, -1.8, 3.3, 12.0, 12.3, zt, zt + 10.9, C["black"]))
        out.append(quad_box(f, 30.0, 18.0, 44.0, 32.0, zt, zt + 1.2, C["chip"]))
    return out


def esp_meshes(board, T):
    """ESP32 dev board on two 1x22 socket footprints (U1 / U2, or any PinSocket_1x22 pair)."""
    socks = [fp for fp in board.GetFootprints()
             if fp.GetReference() in ("U1", "U2") or "PinSocket_1x22" in str(fp.GetFPID().GetLibItemName())]
    if len(socks) < 2:
        return []
    out, rows = [], []
    for fp in socks[:2]:
        name = str(fp.GetFPID().GetLibItemName())
        pads = list(fp.Pads())
        xs = [mm(p.GetPosition().x) for p in pads]
        ys = [mm(p.GetPosition().y) for p in pads]
        if not name:                               # v3.3 custom sockets: only some pads exist, draw the full 22-pin row
            horiz = max(xs) - min(xs) >= max(ys) - min(ys)
            if horiz:
                x0 = min(xs)
                xs = [x0 + 2.54 * k for k in range(22)] if x0 + 2.54 * 21 >= max(xs) - 0.1 else xs
                ys = [ys[0]] * len(xs)
        rows.append((min(xs), min(ys), max(xs), max(ys)))
        if not name:
            out.append(box((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, T, T + 8.5,
                           max(xs) - min(xs) + 2.54, max(ys) - min(ys) + 2.54, C["black"]))
    x0 = min(r[0] for r in rows) - 1.3
    y0 = min(r[1] for r in rows) - 1.3
    x1 = max(r[2] for r in rows) + 1.3
    y1 = max(r[3] for r in rows) + 1.3
    zb = T + 8.5 + 2.5
    out.append(box((x0 + x1) / 2, (y0 + y1) / 2, zb, zb + 1.6, x1 - x0 + 1.0, y1 - y0 + 1.0, C["esp"]))
    horiz = x1 - x0 > y1 - y0
    if horiz:                                       # RF module can at the antenna end
        out.append(box(x1 - 9.5, (y0 + y1) / 2, zb + 1.6, zb + 4.8, 15.0, 17.5, C["silver"]))
    else:
        out.append(box((x0 + x1) / 2, y0 + 9.5, zb + 1.6, zb + 4.8, 17.5, 15.0, C["silver"]))
    for (rx0, ry0, rx1, ry1) in rows:
        out.append(box((rx0 + rx1) / 2, (ry0 + ry1) / 2, T + 8.5, zb, rx1 - rx0 + 0.64, ry1 - ry0 + 0.64, C["gold"]))
    return out


# ------------------------------------------------------------------ the PCB scene
def pcb_glb(pcb_path, out_path, extras):
    board = pcbnew.LoadBoard(pcb_path)
    T = mm(board.GetDesignSettings().GetBoardThickness())
    polys = outline(board)
    bb = board.GetBoardEdgesBoundingBox()
    bbox = (mm(bb.GetLeft()) - 0.5, mm(bb.GetTop()) - 0.5, mm(bb.GetRight()) + 0.5, mm(bb.GetBottom()) + 0.5)
    holes = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            ds = p.GetDrillSize()
            if ds.x > 0:
                holes.append((mm(p.GetPosition().x), mm(p.GetPosition().y), mm(min(ds.x, ds.y)) / 2))
    for t in board.GetTracks():
        if isinstance(t, pcbnew.PCB_VIA):
            holes.append((mm(t.GetPosition().x), mm(t.GetPosition().y), mm(t.GetDrillValue()) / 2))
    top_img = side_texture(board, bbox, "top", holes)
    bot_img = side_texture(board, bbox, "bottom", holes)

    scene = trimesh.Scene()
    for k, poly in enumerate(polys):
        core = trimesh.creation.extrude_polygon(Polygon([(x, -y) for x, y in poly.exterior.coords],
                                                        [[(x, -y) for x, y in r.coords] for r in poly.interiors]),
                                                T - 0.04, engine="earcut")
        core.apply_translation((0, 0, 0.02))
        scene.add_geometry(color_mesh(core, C["edge"]), node_name="board_core_%d" % k, geom_name="board_core_%d" % k)
        scene.add_geometry(textured_face(poly, T, top_img, bbox, True), node_name="board_top_%d" % k,
                           geom_name="board_top_%d" % k)
        scene.add_geometry(textured_face(poly, 0.0, bot_img, bbox, False), node_name="board_bottom_%d" % k,
                           geom_name="board_bottom_%d" % k)
    parts = []
    for fp in board.GetFootprints():
        if fp.GetReference() == "MEGA1" or (fp.GetReference() in ("U1", "U2") and not str(fp.GetFPID().GetLibItemName())):
            continue
        parts += part_bodies(fp, T)
    m = merge(parts)
    if m is not None:
        scene.add_geometry(m, node_name="parts", geom_name="parts")
    mods = []
    if extras in ("mega_flipped_above", "mega_below_esp_on_top"):
        mods += mega_meshes(board, T, extras)
    m = merge(mods)
    if m is not None:
        scene.add_geometry(m, node_name="arduino_mega", geom_name="arduino_mega")
    if extras in ("mega_below_esp_on_top", "esp_on_sockets"):
        m = merge(esp_meshes(board, T))
        if m is not None:
            scene.add_geometry(m, node_name="esp32_board", geom_name="esp32_board")
    # centre on the board and make Y up (glTF convention): (x, y, z) -> (x, z, -y)
    cx, cy = (bbox[0] + bbox[2]) / 2, -(bbox[1] + bbox[3]) / 2
    Tm = trimesh.transformations.translation_matrix((-cx, -cy, 0))
    R = trimesh.transformations.rotation_matrix(-math.pi / 2, (1, 0, 0))
    scene.apply_transform(R @ Tm)
    data = scene.export(file_type="glb")
    open(out_path, "wb").write(data)
    size = (bbox[2] - bbox[0], bbox[3] - bbox[1])
    print("  pcb.glb %.0f kB, board %.1f x %.1f mm" % (len(data) / 1024, size[0] - 1, size[1] - 1))
    return {"w": round(size[0] - 1, 1), "h": round(size[1] - 1, 1), "t": T}


# ------------------------------------------------------------------ DXF -> SVG
def dxf_svg(dxf, svg):
    import ezdxf
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from ezdxf.addons.drawing import Frontend, RenderContext
    from ezdxf.addons.drawing.matplotlib import MatplotlibBackend
    from ezdxf.addons.drawing.config import Configuration, BackgroundPolicy, ColorPolicy
    doc = ezdxf.readfile(dxf)
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_axes([0, 0, 1, 1])
    cfg = Configuration(background_policy=BackgroundPolicy.WHITE, color_policy=ColorPolicy.BLACK)
    Frontend(RenderContext(doc), MatplotlibBackend(ax), config=cfg).draw_layout(doc.modelspace())
    ax.set_aspect("equal")
    ax.margins(0.02)
    fig.savefig(svg, format="svg", bbox_inches="tight", pad_inches=0.05, transparent=False)
    plt.close(fig)


def nice(name):
    return name.replace("_", " ")


def export(vid):
    cfg = VERSIONS[vid]
    out = os.path.join(HERE, cfg["folder"], "models")
    os.makedirs(out, exist_ok=True)
    print(vid)
    man = {"id": vid, "items": [], "files": []}
    if cfg.get("pcb"):
        info = pcb_glb(os.path.join(ROOT, cfg["pcb"]), os.path.join(out, "pcb.glb"), cfg.get("extras"))
        man["items"].append({"group": "PCB", "name": "PCB with parts", "kind": "glb",
                             "src": "versions/%s/models/pcb.glb" % cfg["folder"], "info": info})
    for stl in sorted(glob.glob(os.path.join(ROOT, cfg["prints"], "*.stl"))):
        man["items"].append({"group": "3-D printed parts", "name": nice(os.path.basename(stl)[:-4]), "kind": "stl",
                             "src": os.path.relpath(stl, ROOT).replace(os.sep, "/")})
    for dxf in sorted(glob.glob(os.path.join(ROOT, cfg["decks"], "*.dxf"))):
        base = os.path.basename(dxf)[:-4]
        svg = os.path.join(out, base + ".svg")
        dxf_svg(dxf, svg)
        stl = dxf[:-4] + ".stl"
        it = {"group": "Laser-cut decks", "name": nice(base), "kind": "stl" if os.path.exists(stl) else "svg",
              "svg": "versions/%s/models/%s.svg" % (cfg["folder"], base),
              "dxf": os.path.relpath(dxf, ROOT).replace(os.sep, "/")}
        if os.path.exists(stl):
            it["src"] = os.path.relpath(stl, ROOT).replace(os.sep, "/")
        man["items"].append(it)
    for f in cfg.get("files", []):
        p = os.path.join(ROOT, f)
        if os.path.exists(p):
            man["files"].append({"name": os.path.basename(f), "src": f, "kb": round(os.path.getsize(p) / 1024)})
    json.dump(man, open(os.path.join(out, "manifest.json"), "w"), indent=1)
    print("  %d items, %d downloads" % (len(man["items"]), len(man["files"])))


if __name__ == "__main__":
    for v in (sys.argv[1:] or list(VERSIONS)):
        export(v)

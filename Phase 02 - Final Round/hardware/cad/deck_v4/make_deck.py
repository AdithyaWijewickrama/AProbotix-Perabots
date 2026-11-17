"""
make_deck.py - builds the GuardianBot v4 PCB mid deck in KiCad 7 from deck_design.py.

  python3 make_deck.py place    placed board + overlap check
  python3 make_deck.py route    autoroute (../autoroute.py engine, local-fab rules) + GND pours + DRC
  python3 make_deck.py fab      Gerbers, drill, toner-transfer PDFs, BOM, positions
  python3 make_deck.py all      everything
Output: ../../pcb_v4/
Needs: KiCad 7 (pcbnew python module), numpy, scipy, shapely.
KiCad view = robot seen from the top, FRONT UP: KiCad x = OX - Y, KiCad y = OY - X.
"""
import csv
import math
import os
import sys
import zipfile

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
import deck_design as D  # noqa: E402

OUT = os.path.normpath(os.path.join(HERE, "..", "..", "pcb_v4"))
NAME = "guardianbot_v4_pcb_deck"
PCB = os.path.join(OUT, NAME + ".kicad_pcb")
OX, OY = 100.0, 100.0                  # KiCad position of the robot centre (wheel axle)


def kc(X, Y):
    """robot (X forward, Y left) -> KiCad (front up)."""
    return pcbnew.VECTOR2I(pcbnew.FromMM(OX - Y), pcbnew.FromMM(OY - X))


def rb(x, y):
    """KiCad mm -> robot (X, Y)."""
    return OY - y, OX - x


def load(fpid):
    lib, name = fpid.split(":")
    fp = pcbnew.FootprintLoad(os.path.join(D.FP, lib + ".pretty"), name)
    if fp is None:
        raise SystemExit("footprint not found: " + fpid)
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def tht_pad(fp, name, u, v, size=1.8, drill=1.0, shape=pcbnew.PAD_SHAPE_CIRCLE):
    p = pcbnew.PAD(fp)
    p.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
    p.SetShape(shape)
    p.SetSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
    p.SetDrillSize(pcbnew.VECTOR2I(pcbnew.FromMM(drill), pcbnew.FromMM(drill)))
    p.SetLayerSet(p.PTHMask())
    p.SetNumber(name)
    o = fp.GetPosition()
    p.SetPos0(pcbnew.VECTOR2I(kc(u, v).x - o.x, kc(u, v).y - o.y))     # KiCad 7 stores pads relative to the footprint
    p.SetPosition(kc(u, v))
    fp.Add(p)
    return p


def courtyard(fp, du0, dv0, du1, dv1):
    for (a, b) in (((du0, dv0), (du1, dv0)), ((du1, dv0), (du1, dv1)), ((du1, dv1), (du0, dv1)), ((du0, dv1), (du0, dv0))):
        s = pcbnew.FP_SHAPE(fp)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetLayer(pcbnew.F_CrtYd)
        s.SetWidth(pcbnew.FromMM(0.05))
        fp.Add(s)
        s.SetStart(kc(*a))
        s.SetEnd(kc(*b))


def orient(fp, direction):
    """Rotate so that pad 1 -> pad 2 points along the robot direction ('+X', '-X', '+Y', '-Y')."""
    want = {"+X": (0, -1), "-X": (0, 1), "+Y": (-1, 0), "-Y": (1, 0)}[direction]      # KiCad y points down
    for ang in (0, 90, 180, 270):
        fp.SetOrientationDegrees(ang)
        pads = {p.GetNumber(): p.GetPosition() for p in fp.Pads()}
        if "1" not in pads or "2" not in pads:
            return
        dx, dy = pads["2"].x - pads["1"].x, pads["2"].y - pads["1"].y
        L = math.hypot(dx, dy)
        if L and abs(dx / L - want[0]) < 1e-3 and abs(dy / L - want[1]) < 1e-3:
            return
    raise SystemExit("cannot orient %s" % fp.GetReference())


def edge_ring(board, coords):
    pts = list(coords)
    for p, q in zip(pts, pts[1:]):
        sh = pcbnew.PCB_SHAPE(board)
        sh.SetShape(pcbnew.SHAPE_T_SEGMENT)
        sh.SetLayer(pcbnew.Edge_Cuts)
        sh.SetWidth(pcbnew.FromMM(0.1))
        sh.SetStart(kc(*p))
        sh.SetEnd(kc(*q))
        board.Add(sh)


def add_text(board, txt, X, Y, size, bottom=False):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(txt)
    t.SetLayer(pcbnew.B_SilkS if bottom else pcbnew.F_SilkS)
    t.SetMirrored(bottom)
    t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
    t.SetTextThickness(pcbnew.FromMM(0.15 if size < 1.2 else 0.2))
    t.SetPosition(kc(X, Y))
    board.Add(t)


def place():
    os.makedirs(OUT, exist_ok=True)
    board = pcbnew.BOARD()
    ds = board.GetDesignSettings()
    ds.SetCopperLayerCount(2)
    ds.m_TrackMinWidth = pcbnew.FromMM(0.25)
    ds.m_MinClearance = pcbnew.FromMM(D.CLEARANCE)
    ds.m_ViasMinSize = pcbnew.FromMM(D.VIA[0])
    ds.m_MinThroughDrill = pcbnew.FromMM(0.5)
    ds.m_HoleClearance = pcbnew.FromMM(0.3)
    ds.m_CopperEdgeClearance = pcbnew.FromMM(0.5)
    ds.SetBoardThickness(pcbnew.FromMM(1.6))
    nc = ds.m_NetSettings.m_DefaultNetClass
    nc.SetTrackWidth(pcbnew.FromMM(D.SIGNAL_W))
    nc.SetClearance(pcbnew.FromMM(D.CLEARANCE))
    nc.SetViaDiameter(pcbnew.FromMM(D.VIA[0]))
    nc.SetViaDrill(pcbnew.FromMM(D.VIA[1]))

    poly = D.outline()
    edge_ring(board, poly.exterior.coords)
    for ring in poly.interiors:
        edge_ring(board, ring.coords)

    nets = {}

    def net(n):
        if n not in nets:
            ni = pcbnew.NETINFO_ITEM(board, n)
            board.Add(ni)
            nets[n] = ni
        return nets[n]

    # ---- Mega headers (long male pins soldered in the deck, the Mega plugs on upside-down)
    mega = pcbnew.FOOTPRINT(board)
    mega.SetReference("MEGA1")
    mega.SetValue("Arduino Mega 2560 R3 (upside-down)")
    board.Add(mega)
    mega.SetPosition(kc(0, 0))
    for name, (X, Y) in D.MEGA.items():
        p = tht_pad(mega, name, X, Y, 1.7, 1.0)
        n = D.MEGA_NET.get(name, "")
        if n:
            p.SetNet(net(n))
    mega.Reference().SetVisible(False)
    mega.Value().SetVisible(False)

    # ---- everything else from the library
    for ref, val, fpid, X, Y, direction, side, pinmap, desc in D.PARTS:
        fp = load(fpid)
        fp.SetReference(ref)
        fp.SetValue(val)
        board.Add(fp)
        if side == "B":
            fp.Flip(fp.GetPosition(), False)
        orient(fp, direction)
        fp.SetPosition(kc(0, 0))
        p1 = {p.GetNumber(): p.GetPosition() for p in fp.Pads()}["1"]
        fp.SetPosition(pcbnew.VECTOR2I(kc(X, Y).x - (p1.x - kc(0, 0).x), kc(X, Y).y - (p1.y - kc(0, 0).y)))
        for pad in fp.Pads():
            n = pinmap.get(pad.GetNumber(), "")
            if n:
                pad.SetNet(net(n))
        fp.Value().SetVisible(False)
        if ref.startswith("J"):
            fp.Reference().SetVisible(False)
            # connector name centred along the connector, beside it, on its own side
            pads = [rb(pcbnew.ToMM(p.GetPosition().x), pcbnew.ToMM(p.GetPosition().y)) for p in fp.Pads()]
            xs = [q[0] for q in pads]
            ys = [q[1] for q in pads]
            bb = courtyard_box(fp)
            if direction in ("+Y", "-Y"):
                tx = bb[0] - 0.9 if (side == "B" and X >= 0) or X < -32 else bb[2] + 0.9
                add_text(board, val, tx, sum(ys) / len(ys), 0.9, side == "B")
            else:
                add_text(board, val, min(xs) - 2.0, Y, 0.9, side == "B")

    for txt, X, Y, size, bottom in (("GuardianBot v4 PCB deck rev A", -64.0, 0.0, 1.3, False),
                                    ("PeraBots 2026  team AProbotix", -66.5, 0.0, 1.0, False),
                                    ("FRONT", 66.0, 0.0, 1.5, False), ("FRONT", 66.0, 0.0, 1.5, True)):
        if size < 0.5:
            continue
        add_text(board, txt, X, Y, size, bottom)
    board.Save(PCB)
    write_project()
    check_overlaps(board)
    print("placed ->", PCB)


def courtyard_box(fp):
    """courtyard bounding box in robot mm (X0, Y0, X1, Y1), either side."""
    for lay in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
        c = fp.GetCourtyard(lay)
        if c.OutlineCount():
            bb = c.BBox()
            break
    else:
        bb = fp.GetBoundingBox(False, False)
    xa, ya = rb(pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()))
    xb, yb = rb(pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))
    return min(xa, xb), min(ya, yb), max(xa, xb), max(ya, yb)


def check_overlaps(board):
    """Courtyard overlaps (both sides together: the THT pins go through), parts outside the deck or over a
    cut-out / hole, and parts on top of a Mega pin."""
    from shapely.geometry import box, Point
    poly = D.outline()
    boxes = [(fp.GetReference(), courtyard_box(fp)) for fp in board.GetFootprints() if fp.GetReference() != "MEGA1"]
    bad = 0
    for i, (r, b1) in enumerate(boxes):
        if not poly.buffer(-0.3).contains(box(*b1)):
            print("  OFF-BOARD / HOLE  %s  X %.1f..%.1f Y %.1f..%.1f" % (r, b1[0], b1[2], b1[1], b1[3]))
            bad += 1
        for s, b2 in boxes[i + 1:]:
            if b1[0] < b2[2] and b2[0] < b1[2] and b1[1] < b2[3] and b2[1] < b1[3]:
                print("  OVERLAP  %s / %s  (%.2f x %.2f mm)" % (r, s, min(b1[2], b2[2]) - max(b1[0], b2[0]),
                                                             min(b1[3], b2[3]) - max(b1[1], b2[1])))
                bad += 1
        for name, (X, Y) in D.MEGA.items():
            if b1[0] - 0.85 < X < b1[2] + 0.85 and b1[1] - 0.85 < Y < b1[3] + 0.85:
                print("  MEGA %s at (%.2f, %.2f) under %s" % (name, X, Y, r))
                bad += 1
    print("overlap check: %d problem(s)" % bad)
    return bad


def write_project():
    import json
    pro = {
        "board": {"design_settings": {
            "defaults": {"board_outline_line_width": 0.1, "copper_line_width": D.SIGNAL_W},
            "rules": {"min_clearance": D.CLEARANCE, "min_track_width": 0.2, "min_via_diameter": D.VIA[0],
                      "min_through_hole_diameter": 0.5, "min_hole_to_hole": 0.5, "min_copper_edge_clearance": 0.5,
                      "min_hole_clearance": 0.3, "solder_mask_min_width": 0.0},
            "track_widths": [0.0, D.SIGNAL_W, 0.6, 1.0],
            "via_dimensions": [{"diameter": 0.0, "drill": 0.0}, {"diameter": D.VIA[0], "drill": D.VIA[1]}]}},
        "meta": {"filename": NAME + ".kicad_pro", "version": 1},
        "net_settings": {"classes": [{"name": "Default", "clearance": D.CLEARANCE, "track_width": D.SIGNAL_W,
                                      "via_diameter": D.VIA[0], "via_drill": D.VIA[1], "microvia_diameter": 0.3,
                                      "microvia_drill": 0.1, "diff_pair_width": 0.2, "diff_pair_gap": 0.25,
                                      "wire_width": 6, "bus_width": 12, "line_style": 0,
                                      "pcb_color": "rgba(0, 0, 0, 0.000)", "schematic_color": "rgba(0, 0, 0, 0.000)",
                                      "priority": 2147483647}], "meta": {"version": 3}},
        "sheets": [], "text_variables": {}}
    json.dump(pro, open(os.path.join(OUT, NAME + ".kicad_pro"), "w"), indent=2)


# ------------------------------------------------------------------ routing
def pour(board):
    gnd = board.GetNetsByName()["GND"]
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(gnd)
        z.SetLocalClearance(pcbnew.FromMM(0.5))
        z.SetMinThickness(pcbnew.FromMM(0.4))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(pcbnew.FromMM(0.5))
        z.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.6))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        ol = z.Outline()
        ol.NewOutline()
        for k in range(180):
            t = 2 * math.pi * k / 180
            p = kc((D.R_DECK - 0.6) * math.cos(t), (D.R_DECK - 0.6) * math.sin(t))
            ol.Append(p.x, p.y)
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())


def route():
    import numpy as np
    import autoroute as ar
    ar.G = 0.15
    ar.CLR = D.CLEARANCE
    ar.VIA_D, ar.VIA_DRILL = D.VIA
    ar.POWER = {}                     # route signals first, power after, GND last (+ pours)
    ar.VIA_COST = 10.0
    ar.width_of = lambda n: D.POWER_W.get(n, D.SIGNAL_W)
    _astar = ar.astar
    ar.astar = lambda *a, **k: _astar(*a, **dict(k, limit=3000000))

    import shapely
    inner = D.outline().buffer(-0.9)

    class RectGrid(ar.Grid):
        def __init__(self, board):
            super().__init__(board)
            X, Y = OY - self.Y, OX - self.X
            self.outside = ~shapely.contains_xy(inner, X, Y)
    ar.Grid = RectGrid
    # GND is routed last (it cannot block signals any more) and the GND pours on both layers do the rest

    place()                                  # start from a clean placed board (no tracks, no zones)
    board = pcbnew.LoadBoard(PCB)
    bias, best = [b for b in os.environ.get("BIAS", "").split(",") if b] + ["SDA", "SCL", "+3V3", "ESP_TX", "ESP_RX", "MEGA_TX2", "MEGA_RX2"], None
    for attempt in range(int(os.environ.get("ROUTE_TRIES", "4"))):
        res = ar.route_all(board, bias)
        failed = res[2]
        print("attempt %d: %d connection(s) failed %s" % (attempt + 1, len(failed), failed[:8]), flush=True)
        if best is None or len(failed) < len(best[2]):
            best = res
        if not failed:
            break
        bias = list(dict.fromkeys([f[0] for f in failed] + bias))
    grid, results, failed, net_id, nets, pads, cp = best
    ar.cell_pad.clear()
    ar.cell_pad.update(cp)
    failed = repair(ar, grid, results, failed, net_id, nets)
    thin = {n: [p for p in ps if id(p) in THIN] for n, ps in results.items()}
    thick = {n: [p for p in ps if id(p) not in THIN] for n, ps in results.items()}
    nseg, nvia = ar.write_tracks(board, grid, thick)
    wo = ar.width_of
    ar.width_of = lambda n: 0.25
    a2, b2 = ar.write_tracks(board, grid, {n: ps for n, ps in thin.items() if ps})
    ar.width_of = wo
    nseg, nvia = nseg + a2, nvia + b2
    pour(board)
    board.Save(PCB)
    print("tracks %d, vias %d, unrouted %d" % (nseg, nvia, len(failed)))
    drc(board)
    return failed


def repair(ar, grid, results, failed, net_id, nets):
    """Second chance for the connections the main pass could not make: route them one by one on the
    finished board with a thinner track (0.25 mm) and 0.2 mm clearance - still easy for a local shop."""
    w_thin, clr0 = 0.25, ar.CLR
    still = []
    for n, padname in failed:
        ar.CLR = 0.2
        nid = net_id[n]
        blk = grid.blocked(nid, w_thin)
        vblk = grid.via_blocked(nid)
        ar.CLR = clr0
        pl = nets[n]
        esc = {id(p): ar.escape_cells(grid, p) for p in pl}
        for p in pl:
            for c in esc[id(p)]:
                ar.cell_pad.setdefault((n,) + c, p)
        tgt_pad = next(p for p in pl if p["ref"] + "." + p["num"] == padname)
        tree = set()
        for path in results.get(n, []):
            tree.update(path)
        for p in pl:
            if p is not tgt_pad and (tree & esc[id(p)] or not tree):
                tree.update(esc[id(p)])
        if not tree:
            still.append((n, padname))
            continue
        free = set().union(*esc.values())
        path = ar.astar(grid, blk, vblk, list(tree), esc[id(tgt_pad)], free, (tgt_pad["x"], tgt_pad["y"]), NET=n)
        if path is None:
            still.append((n, padname))
            continue
        ar.commit(grid, path, nid, w_thin)
        results.setdefault(n, []).append(path)
        THIN.add(id(path))
    print("repair pass: %d of %d fixed" % (len(failed) - len(still), len(failed)))
    return still


THIN = set()


def drc(board=None):
    board = board or pcbnew.LoadBoard(PCB)
    rpt = os.path.join(OUT, NAME + "_drc.txt")
    ok = pcbnew.WriteDRCReport(board, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
    txt = open(rpt).read() if ok else ""
    import re
    m = re.findall(r"\*\* Found (\d+) (DRC violations|unconnected pads)", txt)
    print("DRC:", m or "report written", "->", rpt)


# ------------------------------------------------------------------ fabrication outputs
def fab():
    board = pcbnew.LoadBoard(PCB)
    fabdir = os.path.join(OUT, "fab")
    os.makedirs(fabdir, exist_ok=True)
    pc = pcbnew.PLOT_CONTROLLER(board)
    po = pc.GetPlotOptions()
    po.SetOutputDirectory(fabdir)
    po.SetPlotFrameRef(False)
    po.SetUseGerberProtelExtensions(True)
    po.SetCreateGerberJobFile(False)
    po.SetSubtractMaskFromSilk(True)
    po.SetUseAuxOrigin(False)
    layers = [("F_Cu", pcbnew.F_Cu), ("B_Cu", pcbnew.B_Cu), ("F_Mask", pcbnew.F_Mask), ("B_Mask", pcbnew.B_Mask),
              ("F_Silkscreen", pcbnew.F_SilkS), ("B_Silkscreen", pcbnew.B_SilkS), ("Edge_Cuts", pcbnew.Edge_Cuts)]
    files = []
    for name, lid in layers:
        pc.SetLayer(lid)
        pc.OpenPlotfile(name, pcbnew.PLOT_FORMAT_GERBER, name)
        pc.PlotLayer()
        files.append(pc.GetPlotFileName())
    pc.ClosePlot()
    dw = pcbnew.EXCELLON_WRITER(board)
    dw.SetOptions(False, False, board.GetDesignSettings().GetAuxOrigin(), False)
    dw.SetFormat(True)
    dw.CreateDrillandMapFilesSet(fabdir, True, False)
    for f in os.listdir(fabdir):
        if f.endswith(".drl"):
            files.append(os.path.join(fabdir, f))
    zp = os.path.join(OUT, NAME + "_gerbers.zip")
    with zipfile.ZipFile(zp, "w", zipfile.ZIP_DEFLATED) as z:
        for f in files:
            z.write(f, os.path.basename(f))
    print("gerbers ->", zp)

    # toner-transfer / photo-mask PDFs at 1:1 (bottom copper as seen from the top = mirrored)
    po.SetOutputDirectory(OUT)
    po.SetDrillMarksType(pcbnew.DRILL_MARKS_FULL_DRILL_SHAPE)
    po.SetPlotFrameRef(False)
    po.SetScale(1.0)
    po.SetAutoScale(False)
    for name, lid, mirror in (("toner_TOP_copper_mirrored", pcbnew.F_Cu, True),
                              ("toner_BOTTOM_copper", pcbnew.B_Cu, False),
                              ("assembly_top_silkscreen", pcbnew.F_SilkS, False),
                              ("assembly_bottom_silkscreen_mirrored", pcbnew.B_SilkS, True)):
        po.SetMirror(mirror)
        po.SetBlackAndWhite(True)
        pc.SetLayer(lid)
        pc.OpenPlotfile(name, pcbnew.PLOT_FORMAT_PDF, name)
        pc.PlotLayer()
        if lid in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            pc.SetLayer(pcbnew.Edge_Cuts)
            pc.PlotLayer()
    pc.ClosePlot()

    # BOM + placement
    rows = {}
    for ref, val, fpid, u, v, d, side, pm, desc in D.PARTS:
        k = (val, fpid.split(":")[1])
        rows.setdefault(k, [[], desc])[0].append(ref)
    with open(os.path.join(OUT, NAME + "_BOM.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Qty", "Value", "Footprint", "References", "Note"])
        w.writerow([1, "long male headers (pin 20 mm+): 8,8,8,8,10,8,8 pin + 2x18", "-", "MEGA1",
                    "solder in the deck, the Mega plugs on upside-down 11 mm above (4x M3 x 11 mm spacers)"])
        for (val, fp), (refs, desc) in sorted(rows.items(), key=lambda kv: kv[1][0][0]):
            w.writerow([len(refs), val, fp, " ".join(refs), desc])
    with open(os.path.join(OUT, NAME + "_positions.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Ref", "Value", "robot_X_mm", "robot_Y_mm", "Rot", "Side"])
        for fp in board.GetFootprints():
            p = fp.GetPosition()
            X, Y = rb(pcbnew.ToMM(p.x), pcbnew.ToMM(p.y))
            w.writerow([fp.GetReference(), fp.GetValue(), "%.3f" % X, "%.3f" % Y,
                        "%.0f" % fp.GetOrientationDegrees(), "bottom" if fp.IsFlipped() else "top"])
    print("BOM, positions, toner PDFs ->", OUT)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "all"
    if cmd == "place":
        place()
    if cmd in ("route", "all"):
        route()
    if cmd == "drc":
        drc()
    if cmd == "pour":
        b = pcbnew.LoadBoard(PCB)
        pour(b)
        b.Save(PCB)
        drc(b)
    if cmd in ("fab", "all"):
        fab()

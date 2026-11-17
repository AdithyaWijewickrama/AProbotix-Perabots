"""
make_shield.py - builds the GuardianBot v3 Mega shield in KiCad 7 from shield_design.py.

  python3 make_shield.py place    placed board + overlap check
  python3 make_shield.py route    autoroute (../autoroute.py engine, local-fab rules) + GND pours + DRC
  python3 make_shield.py fab      Gerbers, drill, toner-transfer PDFs, BOM, positions, 3D STEP
  python3 make_shield.py all      everything
Output: ../../pcb_v3/ (KiCad project, fab/, docs/)
Needs: KiCad 7 (pcbnew python module), numpy, scipy.
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
import shield_design as D  # noqa: E402

OUT = os.path.normpath(os.path.join(HERE, "..", "..", "pcb_v3"))
NAME = "guardianbot_v3_mega_shield"
PCB = os.path.join(OUT, NAME + ".kicad_pcb")
OX, OY = 50.0, 100.0                    # KiCad position of shield (u, v) = (0, 0)


def kc(u, v):
    """shield (u, v) -> KiCad. v grows from the POWER/ANALOG header towards the digital header,
    which is UP in KiCad (top view, USB end on the left, exactly as an Arduino is drawn)."""
    return pcbnew.VECTOR2I(pcbnew.FromMM(OX + u), pcbnew.FromMM(OY - v))


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
    """Rotate so that pad 1 -> pad 2 points along direction ('+u', '-u', '+v', '-v')."""
    want = {"+u": (1, 0), "-u": (-1, 0), "+v": (0, -1), "-v": (0, 1)}[direction]      # KiCad y points down
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

    # outline: rectangle with 2 mm corner chamfers
    c = 2.0
    k = D.REAR_RIGHT_CHAMFER
    pts = [(D.U_MIN + c, D.V_MIN), (D.U_MAX - k, D.V_MIN), (D.U_MAX, D.V_MIN + k), (D.U_MAX, D.V_MAX - c),
           (D.U_MAX - c, D.V_MAX), (D.U_MIN + c, D.V_MAX), (D.U_MIN, D.V_MAX - c), (D.U_MIN, D.V_MIN + c)]
    for a, b in zip(pts, pts[1:] + pts[:1]):
        s = pcbnew.PCB_SHAPE(board)
        s.SetShape(pcbnew.SHAPE_T_SEGMENT)
        s.SetLayer(pcbnew.Edge_Cuts)
        s.SetWidth(pcbnew.FromMM(0.1))
        s.SetStart(kc(*a))
        s.SetEnd(kc(*b))
        board.Add(s)

    nets = {}

    def net(n):
        if n not in nets:
            ni = pcbnew.NETINFO_ITEM(board, n)
            board.Add(ni)
            nets[n] = ni
        return nets[n]

    # ---- Mega headers (male pins under the shield) as one footprint
    mega = pcbnew.FOOTPRINT(board)
    mega.SetReference("MEGA1")
    mega.SetValue("Arduino Mega 2560 R3 headers")
    board.Add(mega)
    mega.SetPosition(kc(0, 0))
    for name, (u, v) in D.MEGA.items():
        p = tht_pad(mega, name, u, v, 1.7, 1.0)
        n = D.MEGA_NET.get(name, "")
        if n:
            p.SetNet(net(n))
    mega.Reference().SetVisible(False)
    mega.Value().SetVisible(False)
    # No mounting holes on the shield: it is held by the Mega headers. Screw the Mega down first.

    # ---- ESP32-S3-DevKitC-1 sockets
    for ref, v0, mp in (("U1", D.ESP_V1, D.ESP_J1), ("U2", D.ESP_V3, D.ESP_J3)):
        fp = pcbnew.FOOTPRINT(board)
        fp.SetReference(ref)
        fp.SetValue("ESP32-S3 DevKitC-1 " + ("J1" if ref == "U1" else "J3") + " (1x22 socket)")
        board.Add(fp)
        fp.SetPosition(kc(D.ESP_U0, v0))
        # only the socket pins we use + end pins for support: the gaps let tracks pass between the rows
        for k in range(22):
            if (k + 1) not in D.ESP_SOCKET_PINS[ref]:
                continue
            p = tht_pad(fp, str(k + 1), D.ESP_U0 + 2.54 * k, v0, 1.7, 1.0,
                        pcbnew.PAD_SHAPE_RECT if k == 0 else pcbnew.PAD_SHAPE_CIRCLE)
            if mp.get(k + 1):
                p.SetNet(net(mp[k + 1]))
        courtyard(fp, D.ESP_U0 - 1.5, v0 - 1.5, D.ESP_U0 + 21 * 2.54 + 1.5, v0 + 1.5)
        fp.Value().SetVisible(False)

    # ---- everything else from the library
    for ref, val, fpid, u, v, direction, pinmap, desc in D.PARTS:
        fp = load(fpid)
        fp.SetReference(ref)
        fp.SetValue(val)
        board.Add(fp)
        orient(fp, direction)
        fp.SetPosition(kc(0, 0))
        p1 = {p.GetNumber(): p.GetPosition() for p in fp.Pads()}["1"]
        fp.SetPosition(pcbnew.VECTOR2I(kc(u, v).x - (p1.x - kc(0, 0).x), kc(u, v).y - (p1.y - kc(0, 0).y)))
        for pad in fp.Pads():
            n = pinmap.get(pad.GetNumber(), "")
            if n:
                pad.SetNet(net(n))
        fp.Value().SetVisible(False)
        if ref.startswith("J"):
            fp.Reference().SetVisible(False)

    # ---- silkscreen: connector names next to each connector, title
    for ref, val, fpid, u, v, direction, pinmap, desc in D.PARTS:
        if not ref.startswith("J"):
            continue
        t = pcbnew.PCB_TEXT(board)
        t.SetText(val)
        t.SetLayer(pcbnew.F_SilkS)
        t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(0.9), pcbnew.FromMM(0.9)))
        t.SetTextThickness(pcbnew.FromMM(0.15))
        if ref in ("J23", "J24"):                     # two servo headers side by side: short names
            t.SetText("SV_L" if ref == "J23" else "SV_R")
            t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(0.8), pcbnew.FromMM(0.8)))
            t.SetPosition(kc(u, v - 2.3))
        elif direction == "+u":
            t.SetPosition(kc(u + 2.0, v - 4.0 if v < 0 or 30 < v else v + 4.6))
        else:
            t.SetPosition(kc(u + 6.5, v - 2.5))
        board.Add(t)
    for txt, u, v, size in (("GuardianBot v3 Mega shield rev A", 53.5, 26.0, 1.6), ("PeraBots 2026  team AProbotix", 53.5, 30.0, 1.2),
                            ("FRONT ->", 50.0, 52.0, 1.0)):
        t = pcbnew.PCB_TEXT(board)
        t.SetText(txt)
        t.SetLayer(pcbnew.B_SilkS if txt != "FRONT ->" else pcbnew.F_SilkS)     # title on the back side
        if txt != "FRONT ->":
            t.SetMirrored(True)
        t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
        t.SetTextThickness(pcbnew.FromMM(0.18))
        t.SetPosition(kc(u, v))
        board.Add(t)
    board.Save(PCB)
    write_project()
    check_overlaps(board)
    print("placed ->", PCB)


def check_overlaps(board):
    """Report courtyard overlaps and parts outside the outline."""
    boxes = []
    for fp in board.GetFootprints():
        if fp.GetReference().startswith("H"):
            continue
        bb = fp.GetCourtyard(pcbnew.F_CrtYd).BBox() if fp.GetCourtyard(pcbnew.F_CrtYd).OutlineCount() else fp.GetBoundingBox(False, False)
        if fp.GetReference() == "MEGA1":
            continue
        boxes.append((fp.GetReference(), pcbnew.ToMM(bb.GetLeft()) - OX, OY - pcbnew.ToMM(bb.GetBottom()),
                      pcbnew.ToMM(bb.GetRight()) - OX, OY - pcbnew.ToMM(bb.GetTop())))
    bad = 0
    for i in range(len(boxes)):
        r, a0, b0, a1, b1 = boxes[i]
        if a0 < D.U_MIN or b0 < D.V_MIN or a1 > D.U_MAX or b1 > D.V_MAX:
            print("  OUTSIDE  %s  u %.1f..%.1f v %.1f..%.1f" % (r, a0, a1, b0, b1))
            bad += 1
        for j in range(i + 1, len(boxes)):
            s, c0, d0, c1, d1 = boxes[j]
            if a0 < c1 and c0 < a1 and b0 < d1 and d0 < b1:
                ov = (min(a1, c1) - max(a0, c0), min(b1, d1) - max(b0, d0))
                print("  OVERLAP  %s / %s  (%.2f x %.2f mm)" % (r, s, ov[0], ov[1]))
                bad += 1
    # Mega pads / holes under courtyards
    for name, (u, v) in list(D.MEGA.items()) + [("hole", h) for h in D.MEGA_HOLES]:
        rr = 0.85 if name != "hole" else 1.9
        for r, a0, b0, a1, b1 in boxes:
            if a0 - rr < u < a1 + rr and b0 - rr < v < b1 + rr:
                print("  MEGA %s at (%.2f, %.2f) under %s" % (name, u, v, r))
                bad += 1
    print("overlap check: %d problem(s)" % bad)
    return bad


def write_project():
    import json
    pro = {
        "board": {"design_settings": {
            "defaults": {"board_outline_line_width": 0.1, "copper_line_width": D.SIGNAL_W},
            "rules": {"min_clearance": D.CLEARANCE, "min_track_width": 0.25, "min_via_diameter": D.VIA[0],
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
        m, k = 0.6, D.REAR_RIGHT_CHAMFER
        for u, v in ((D.U_MIN + m, D.V_MIN + m), (D.U_MAX - k, D.V_MIN + m), (D.U_MAX - m, D.V_MIN + k),
                     (D.U_MAX - m, D.V_MAX - m), (D.U_MIN + m, D.V_MAX - m)):
            p = kc(u, v)
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

    class RectGrid(ar.Grid):
        def __init__(self, board):
            super().__init__(board)
            bb = board.GetBoardEdgesBoundingBox()
            x0, y0 = pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop())
            x1, y1 = pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom())
            e = 0.5
            self.outside = (self.X < x0 + e) | (self.X > x1 - e) | (self.Y < y0 + e) | (self.Y > y1 - e)
    ar.Grid = RectGrid
    # GND is routed last (it cannot block signals any more) and the GND pours on both layers do the rest

    place()                                  # start from a clean placed board (no tracks, no zones)
    board = pcbnew.LoadBoard(PCB)
    bias, best = ["MEGA_TX2", "MEGA_RX2", "ESP_TX", "ESP_RX", "CAM_TX", "CAM_RX", "+3V3"], None
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
                              ("assembly_top_silkscreen", pcbnew.F_SilkS, False)):
        po.SetMirror(mirror)
        po.SetBlackAndWhite(True)
        pc.SetLayer(lid)
        pc.OpenPlotfile(name, pcbnew.PLOT_FORMAT_PDF, name)
        pc.PlotLayer()
        if lid == pcbnew.F_SilkS:
            pc.SetLayer(pcbnew.Edge_Cuts)
            pc.PlotLayer()
    pc.ClosePlot()

    # BOM + placement
    rows = {}
    for ref, val, fpid, u, v, d, pm, desc in D.PARTS:
        k = (val, fpid.split(":")[1])
        rows.setdefault(k, [[], desc])[0].append(ref)
    with open(os.path.join(OUT, NAME + "_BOM.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Qty", "Value", "Footprint", "References", "Note"])
        w.writerow([2, "1x22 female header 2.54 mm", "PinSocket_1x22", "U1 U2", "ESP32-S3-DevKitC-1 sockets"])
        w.writerow([1, "Arduino stacking/long male headers: 8,8,8,8,10,8,8 pin + 2x18", "-", "MEGA1",
                    "solder under the shield, plug into the Mega"])
        for (val, fp), (refs, desc) in sorted(rows.items(), key=lambda kv: kv[1][0][0]):
            w.writerow([len(refs), val, fp, " ".join(refs), desc])
    with open(os.path.join(OUT, NAME + "_positions.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Ref", "Value", "X_mm", "Y_mm", "Rot", "Side"])
        for fp in board.GetFootprints():
            p = fp.GetPosition()
            w.writerow([fp.GetReference(), fp.GetValue(), "%.3f" % (pcbnew.ToMM(p.x) - OX), "%.3f" % (OY - pcbnew.ToMM(p.y)),
                        "%.0f" % fp.GetOrientationDegrees(), "top"])
    print("BOM, positions, toner PDFs ->", OUT)


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "all"
    if cmd == "place":
        place()
    if cmd in ("route", "all"):
        route()
    if cmd == "drc":
        drc()
    if cmd == "finish":                    # fix rules / hide refs on an already routed board, refill, DRC
        b = pcbnew.LoadBoard(PCB)
        b.GetDesignSettings().m_TrackMinWidth = pcbnew.FromMM(0.25)
        for fp in b.GetFootprints():
            if fp.GetReference().startswith("J"):
                fp.Reference().SetVisible(False)
        for d in b.GetDrawings():
            if isinstance(d, pcbnew.PCB_TEXT):
                txt = d.GetText()
                if txt in ("SERVO_L", "SERVO_R"):
                    d.SetText("SV_L" if txt == "SERVO_L" else "SV_R")
                    d.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(0.8), pcbnew.FromMM(0.8)))
                    ref = "J23" if txt == "SERVO_L" else "J24"
                    pu = [p for p in D.PARTS if p[0] == ref][0]
                    d.SetPosition(kc(pu[3], pu[4] - 2.3))
                if txt.startswith("GuardianBot v3") or txt.startswith("PeraBots 2026"):
                    title = txt.startswith("GuardianBot")
                    d.SetText("GuardianBot v3 Mega shield rev A" if title else "PeraBots 2026  team AProbotix")
                    d.SetLayer(pcbnew.B_SilkS)
                    d.SetMirrored(True)
                    sz = 1.6 if title else 1.2
                    d.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(sz), pcbnew.FromMM(sz)))
                    d.SetPosition(kc(53.5, 26.0 if title else 30.0))
        holes = [fp for fp in b.GetFootprints() if fp.GetReference().startswith("H")]
        for fp in holes:
            b.Delete(fp)
        pcbnew.ZONE_FILLER(b).Fill(b.Zones())
        b.Save(PCB)
        write_project()
        drc(b)
    if cmd == "pour":
        b = pcbnew.LoadBoard(PCB)
        pour(b)
        b.Save(PCB)
        drc(b)
    if cmd in ("fab", "all"):
        fab()

"""
make_pcb.py - builds the GuardianBot main board in KiCad 7, autoroutes it with Freerouting,
pours GND planes, and saves the board.  Fabrication outputs come from export_pcb.sh.

Needs: KiCad 7 (pcbnew python module), Java 17+, freerouting-1.9.0.jar
Run:   /usr/bin/python3 make_pcb.py place     -> placed, unrouted board
       /usr/bin/python3 make_pcb.py dsn       -> Specctra DSN for Freerouting
       java -jar freerouting-1.9.0.jar -de ../pcb/guardianbot_main.dsn -do ../pcb/guardianbot_main.ses
       /usr/bin/python3 make_pcb.py finish    -> import routes + GND pour on both layers
Output: ../pcb/guardianbot_main.kicad_pcb (+ .kicad_pro)
"""
import math
import os
import subprocess
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pcb_design import BOARD_R, FP, HOLES, PARTS  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pcb")
NAME = "guardianbot_main"
PCB = os.path.join(OUT, NAME + ".kicad_pcb")
CX, CY = 100.0, 100.0          # board centre on the KiCad page (mm)


def kc(x, y):
    """robot mm -> KiCad position (front of robot = up)."""
    return pcbnew.VECTOR2I(pcbnew.FromMM(CX - y), pcbnew.FromMM(CY - x))


def load(fpid):
    lib, name = fpid.split(":")
    fp = pcbnew.FootprintLoad(os.path.join(FP, lib + ".pretty"), name)
    if fp is None:
        raise SystemExit("footprint not found: " + fpid)
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def place():
    os.makedirs(OUT, exist_ok=True)
    board = pcbnew.BOARD()
    ds = board.GetDesignSettings()
    ds.SetCopperLayerCount(2)
    ds.m_TrackMinWidth = pcbnew.FromMM(0.2)
    ds.m_MinClearance = pcbnew.FromMM(0.15)
    ds.m_ViasMinSize = pcbnew.FromMM(0.6)
    ds.m_MinThroughDrill = pcbnew.FromMM(0.3)
    ds.SetBoardThickness(pcbnew.FromMM(1.6))
    nc = ds.m_NetSettings.m_DefaultNetClass
    nc.SetTrackWidth(pcbnew.FromMM(0.2))
    nc.SetClearance(pcbnew.FromMM(0.15))
    nc.SetViaDiameter(pcbnew.FromMM(0.6))
    nc.SetViaDrill(pcbnew.FromMM(0.3))

    c = pcbnew.PCB_SHAPE(board)
    c.SetShape(pcbnew.SHAPE_T_CIRCLE)
    c.SetLayer(pcbnew.Edge_Cuts)
    c.SetWidth(pcbnew.FromMM(0.1))
    c.SetCenter(kc(0, 0))
    c.SetEnd(kc(0, -BOARD_R))
    board.Add(c)

    nets = {}

    def net(n):
        if n not in nets:
            ni = pcbnew.NETINFO_ITEM(board, n)
            board.Add(ni)
            nets[n] = ni
        return nets[n]

    for ref, val, fpid, x, y, rot, side, pinmap, desc in PARTS:
        fp = load(fpid)
        fp.SetReference(ref)
        fp.SetValue(val)
        fp.SetPosition(kc(x, y))
        fp.SetOrientationDegrees(rot)
        board.Add(fp)
        if side == "B":
            fp.Flip(fp.GetPosition(), False)
        for pad in fp.Pads():
            n = pinmap.get(pad.GetNumber(), "")
            if n:
                pad.SetNet(net(n))
    for ref, fpid, x, y in HOLES:
        fp = load(fpid)
        fp.SetReference(ref)
        fp.SetPosition(kc(x, y))
        fp.Reference().SetVisible(False)
        board.Add(fp)
    for txt, x, y, size in (("GUARDIANBOT main board v1", -22, 0, 1.3), ("PeraBots 2026", -25, 0, 1.1),
                            ("FRONT", 61, 0, 1.2)):
        t = pcbnew.PCB_TEXT(board)
        t.SetText(txt)
        t.SetPosition(kc(x, y))
        t.SetLayer(pcbnew.F_SilkS)
        t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
        board.Add(t)

    # sanity: DevKit pin 1 at robot y=+26.67, pin 22 at -26.67
    for ref in ("J1", "J3"):
        fp = board.FindFootprintByReference(ref)
        pp = {p.GetNumber(): p.GetPosition() for p in fp.Pads()}
        y1, y22 = CX - pcbnew.ToMM(pp["1"].x), CX - pcbnew.ToMM(pp["22"].x)
        assert abs(y1 - 26.67) < 0.05 and abs(y22 + 26.67) < 0.05, (ref, y1, y22)
    board.Save(PCB)
    write_project()
    print("placed", len(PARTS), "parts,", len(HOLES), "holes ->", PCB)


def write_project():
    """KiCad 7 keeps net classes / design rules in the .kicad_pro file."""
    import json
    pro = {
        "board": {"design_settings": {
            "defaults": {"board_outline_line_width": 0.1, "copper_line_width": 0.2},
            "rules": {"min_clearance": 0.15, "min_track_width": 0.2, "min_via_diameter": 0.6,
                      "min_through_hole_diameter": 0.3, "min_hole_to_hole": 0.25,
                      "min_copper_edge_clearance": 0.4, "min_hole_clearance": 0.25,
                      "solder_mask_min_width": 0.0},
            "track_widths": [0.0, 0.2, 0.4, 0.5, 0.6, 0.8],
            "via_dimensions": [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.6, "drill": 0.3}]}},
        "meta": {"filename": NAME + ".kicad_pro", "version": 1},
        "net_settings": {"classes": [{"name": "Default", "clearance": 0.15, "track_width": 0.2,
                                      "via_diameter": 0.6, "via_drill": 0.3, "microvia_diameter": 0.3,
                                      "microvia_drill": 0.1, "diff_pair_width": 0.2, "diff_pair_gap": 0.25,
                                      "wire_width": 6, "bus_width": 12, "line_style": 0, "pcb_color": "rgba(0, 0, 0, 0.000)",
                                      "schematic_color": "rgba(0, 0, 0, 0.000)", "priority": 2147483647}],
                         "meta": {"version": 3}},
        "sheets": [], "text_variables": {}}
    with open(os.path.join(OUT, NAME + ".kicad_pro"), "w") as f:
        json.dump(pro, f, indent=2)


def pour(board):
    gnd = board.GetNetsByName()["GND"]
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(gnd)
        z.SetLocalClearance(pcbnew.FromMM(0.3))
        z.SetMinThickness(pcbnew.FromMM(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        ol = z.Outline()
        ol.NewOutline()
        R = BOARD_R - 0.6
        for i in range(72):
            a = 2 * math.pi * i / 72
            p = kc(R * math.cos(a), R * math.sin(a))
            ol.Append(p.x, p.y)
        board.Add(z)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())


def _sexpr(text):
    """Tiny s-expression parser for Specctra .ses files."""
    import re
    toks = re.findall(r'\(|\)|"[^"]*"|[^\s()]+', text)
    stack, cur = [], []
    for t in toks:
        if t == "(":
            stack.append(cur)
            cur = []
        elif t == ")":
            done, cur = cur, stack.pop()
            cur.append(done)
        else:
            cur.append(t.strip('"'))
    return cur[0]


def import_ses(board, ses):
    """Add the routed wires and vias from a Freerouting session file to the board."""
    tree = _sexpr(open(ses).read())
    routes = next(x for x in tree if isinstance(x, list) and x and x[0] == "routes")
    res = next(x for x in routes if isinstance(x, list) and x[0] == "resolution")
    scale = 1.0 / float(res[2]) / 1000.0           # file units -> mm
    nets = board.GetNetsByName()
    layer = {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu}
    n_tr = n_via = 0
    netout = next(x for x in routes if isinstance(x, list) and x[0] == "network_out")
    for net in netout[1:]:
        name = net[1]
        ni = nets[name] if nets.has_key(name) else None
        for item in net[2:]:
            if item[0] == "wire":
                path = item[1]
                lay, width = layer[path[1]], float(path[2]) * scale
                pts = [(float(path[i]) * scale, -float(path[i + 1]) * scale) for i in range(3, len(path) - 1, 2)]
                for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
                    t = pcbnew.PCB_TRACK(board)
                    t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(x0), pcbnew.FromMM(y0)))
                    t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(x1), pcbnew.FromMM(y1)))
                    t.SetWidth(pcbnew.FromMM(width))
                    t.SetLayer(lay)
                    if ni:
                        t.SetNet(ni)
                    board.Add(t)
                    n_tr += 1
            elif item[0] == "via":
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(float(item[2]) * scale),
                                              pcbnew.FromMM(-float(item[3]) * scale)))
                v.SetWidth(pcbnew.FromMM(0.6))
                v.SetDrill(pcbnew.FromMM(0.3))
                v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
                if ni:
                    v.SetNet(ni)
                board.Add(v)
                n_via += 1
    print("imported %d track segments, %d vias" % (n_tr, n_via))


def export_dsn():
    board = pcbnew.LoadBoard(PCB)
    dsn = os.path.join(OUT, NAME + ".dsn")
    pcbnew.ExportSpecctraDSN(board, dsn)
    print("wrote", dsn, "- now run:  java -jar freerouting-1.9.0.jar -de", dsn, "-do", dsn[:-4] + ".ses")


def finish():
    board = pcbnew.LoadBoard(PCB)
    import_ses(board, os.path.join(OUT, NAME + ".ses"))
    pour(board)
    board.Save(PCB)
    print("saved", PCB)


if __name__ == "__main__":
    {"place": place, "dsn": export_dsn, "finish": finish}[sys.argv[1]]()

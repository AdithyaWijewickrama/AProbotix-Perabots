"""
fix_unrouted.py - finish a routed deck: route the connections DRC still reports as unconnected,
on top of the existing copper (thin 0.25 mm track, 0.2 mm clearance), optionally after ripping up the
GND tracks (the GND pours on both layers carry GND anyway). Then refill the pours and rerun DRC.
    python3 fix_unrouted.py [--rip-gnd] [--width 0.25] [--clr 0.2]
"""
import math
import os
import re
import sys

import numpy as np
import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
import autoroute as ar  # noqa: E402
import make_deck as M  # noqa: E402
import deck_design as D  # noqa: E402


def unconnected(board):
    rpt = "/tmp/_fix_drc.txt"
    pcbnew.WriteDRCReport(board, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
    txt = open(rpt).read()
    out = []
    for blk in txt.split("[unconnected_items]")[1:]:
        pads = re.findall(r"PTH pad (\S+) \[([^\]]+)\] of (\S+)", blk.split("\n[")[0])
        if len(pads) >= 2:
            out.append((pads[0][1], (pads[0][2], pads[0][0]), (pads[1][2], pads[1][0])))
    return out


def main():
    rip = "--rip-gnd" in sys.argv
    w = float(sys.argv[sys.argv.index("--width") + 1]) if "--width" in sys.argv else 0.25
    clr = float(sys.argv[sys.argv.index("--clr") + 1]) if "--clr" in sys.argv else 0.2
    board = pcbnew.LoadBoard(M.PCB)
    ds = board.GetDesignSettings()                 # the fixes use 0.2 mm clearance (8 mil, fine for local shops)
    ds.m_MinClearance = pcbnew.FromMM(0.2)
    ds.m_NetSettings.m_DefaultNetClass.SetClearance(pcbnew.FromMM(0.2))
    if rip:
        for t in list(board.GetTracks()):
            if t.GetNetname() == "GND":
                board.Delete(t)
    todo = unconnected(board)
    print("unconnected:", todo)
    import shapely
    ar.G, ar.CLR = 0.1, clr
    ar.VIA_D, ar.VIA_DRILL = D.VIA
    inner = D.outline().buffer(-0.9)
    grid = ar.Grid(board)
    X, Y = M.OY - grid.Y, M.OX - grid.X
    grid.outside = ~shapely.contains_xy(inner, X, Y)
    nets, pads = ar.collect(board)
    net_id = {n: k + 1 for k, n in enumerate(sorted(nets))}
    ar.paint_pads(grid, pads, net_id)
    for t in board.GetTracks():
        nid = net_id.get(t.GetNetname(), -1)
        if isinstance(t, pcbnew.PCB_VIA):
            x, y = pcbnew.ToMM(t.GetPosition().x), pcbnew.ToMM(t.GetPosition().y)
            for l in (0, 1):
                grid.paint_disc(l, x, y, pcbnew.ToMM(t.GetWidth()) / 2, nid)
            i, j = grid.ij(x, y)
            grid.holes[i, j] = True
            continue
        l = 0 if t.GetLayer() == pcbnew.F_Cu else 1
        a, b = t.GetStart(), t.GetEnd()
        ax, ay, bx, by = (pcbnew.ToMM(v) for v in (a.x, a.y, b.x, b.y))
        n = max(1, int(math.hypot(bx - ax, by - ay) / (ar.G / 2)))
        for k in range(n + 1):
            grid.paint_disc(l, ax + (bx - ax) * k / n, ay + (by - ay) * k / n, pcbnew.ToMM(t.GetWidth()) / 2, nid)
    done = 0
    results = {}
    for net, pa, pb in todo:
        nid = net_id[net]
        pl = nets[net]
        A = next(p for p in pl if (p["ref"], p["num"]) == pa)
        B = next(p for p in pl if (p["ref"], p["num"]) == pb)
        ea, eb = ar.escape_cells(grid, A), ar.escape_cells(grid, B)
        src = set(ea)                      # straight from pad A to pad B
        ar.cell_pad = {}
        for c in ea:
            ar.cell_pad[(net,) + c] = A
        for c in eb:
            ar.cell_pad[(net,) + c] = B
        blk = grid.blocked(nid, w)
        vblk = grid.via_blocked(nid)
        path = ar.astar(grid, blk, vblk, list(src), eb, set(ea) | set(eb), (B["x"], B["y"]), limit=8000000, NET=net)
        if path is None:
            print("  still failed:", net, pa, pb)
            continue
        ar.commit(grid, path, nid, w)
        results.setdefault(net, []).append(path)
        done += 1
        print("  routed", net)
    wo = ar.width_of
    ar.width_of = lambda n: w
    ar.write_tracks(board, grid, results)
    if rip:                                  # GND again, after the signals: a tree over every GND pad
        gres = []
        nid = net_id["GND"]
        gw = 0.4
        blk = grid.blocked(nid, gw)
        vblk = grid.via_blocked(nid)
        pl = list(nets["GND"])
        esc = {id(p): ar.escape_cells(grid, p) for p in pl}
        ar.cell_pad = {}
        for p in pl:
            for c in esc[id(p)]:
                ar.cell_pad.setdefault(("GND",) + c, p)
        tree = set(esc[id(pl[0])])
        free = set().union(*esc.values())
        miss = 0
        for p in sorted(pl[1:], key=lambda p: math.hypot(p["x"] - pl[0]["x"], p["y"] - pl[0]["y"])):
            if tree & esc[id(p)]:
                continue
            path = ar.astar(grid, blk, vblk, list(tree), esc[id(p)], free, (p["x"], p["y"]), limit=3000000, NET="GND")
            if path is None:
                miss += 1
                continue
            gres.append(path)
            tree.update(path)
            tree.update(esc[id(p)])
        ar.width_of = lambda n: gw
        ar.write_tracks(board, grid, {"GND": gres})
        print("GND re-routed, %d pad(s) left to the pours" % miss)
    ar.width_of = wo
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(M.PCB)
    print("fixed %d of %d" % (done, len(todo)))
    left = unconnected(board)
    print("remaining unconnected:", left)
    M.D.CLEARANCE = 0.2
    M.write_project()
    M.drc(board)


if __name__ == "__main__":
    main()

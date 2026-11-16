"""
autoroute.py - a small 2-layer grid autorouter for the GuardianBot main board (no external tools needed).

Approach: 0.2 mm routing grid on F.Cu and B.Cu, A* search with 45-degree moves and vias,
net by net with rip-up-and-retry ordering.  Before each net, the foreign copper is dilated by
(track/2 + clearance) so the search is a simple O(1) lookup per cell.  SMD pads get a short
straight "escape stub" so fine-pitch ICs (SSOP-24, SOIC-28) can be reached.
GND is not routed: GND pours on both layers + stitching vias beside every SMD GND pad.

Rules (JLCPCB standard 2-layer): signal 0.2 mm, power 0.4-0.8 mm, clearance 0.15 mm, via 0.6/0.3 mm.
Run:  /usr/bin/python3 autoroute.py      (after make_pcb.py place)
"""
import heapq
import math
import os
import sys
import time

import numpy as np
import pcbnew
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_pcb import PCB, pour  # noqa: E402

G = 0.15                       # grid pitch, mm
CLR = 0.15
VIA_D, VIA_DRILL = 0.6, 0.3
POWER = {"GND": 0.4, "VBAT": 0.6, "+5V": 0.5, "+3V3": 0.4, "BAT_RAW": 0.8, "BAT_SW": 0.8,
         "MOT_L1": 0.5, "MOT_L2": 0.5, "MOT_R1": 0.5, "MOT_R2": 0.5}
VIA_COST = 14.0
LAYERS = (pcbnew.F_Cu, pcbnew.B_Cu)
cell_pad = {}
EDGE = 0.5


def disc(r):
    k = int(math.ceil(r / G))
    a = np.arange(-k, k + 1) * G
    return (a[:, None] ** 2 + a[None, :] ** 2) <= r * r + 1e-9


class Grid:
    def __init__(self, board):
        bb = board.GetBoardEdgesBoundingBox()
        self.x0 = pcbnew.ToMM(bb.GetX()) - 1
        self.y0 = pcbnew.ToMM(bb.GetY()) - 1
        self.nx = int((pcbnew.ToMM(bb.GetWidth()) + 2) / G) + 1
        self.ny = int((pcbnew.ToMM(bb.GetHeight()) + 2) / G) + 1
        cx, cy = pcbnew.ToMM(bb.GetCenter().x), pcbnew.ToMM(bb.GetCenter().y)
        R = pcbnew.ToMM(bb.GetWidth()) / 2
        self.owner = np.zeros((2, self.nx, self.ny), dtype=np.int32)    # copper owner (net id), -1 = keep-out
        ii, jj = np.meshgrid(np.arange(self.nx), np.arange(self.ny), indexing="ij")
        self.X, self.Y = self.x0 + ii * G, self.y0 + jj * G
        self.outside = np.hypot(self.X - cx, self.Y - cy) > R - EDGE
        self.holes = np.zeros((self.nx, self.ny), dtype=bool)

    def ij(self, x, y):
        return int(round((x - self.x0) / G)), int(round((y - self.y0) / G))

    def xy(self, i, j):
        return self.x0 + i * G, self.y0 + j * G

    def paint_rect(self, l, xa, ya, xb, yb, net):
        m = (self.X >= xa - 1e-6) & (self.X <= xb + 1e-6) & (self.Y >= ya - 1e-6) & (self.Y <= yb + 1e-6)
        self._paint(l, m, net)

    def paint_disc(self, l, x, y, r, net):
        i0, j0 = self.ij(x, y)
        d = disc(r)
        k = d.shape[0] // 2
        i_a, i_b = max(0, i0 - k), min(self.nx, i0 + k + 1)
        j_a, j_b = max(0, j0 - k), min(self.ny, j0 + k + 1)
        sub = d[i_a - (i0 - k):d.shape[0] - ((i0 + k + 1) - i_b), j_a - (j0 - k):d.shape[1] - ((j0 + k + 1) - j_b)]
        view = self.owner[l, i_a:i_b, j_a:j_b]
        clash = sub & (view != 0) & (view != net)
        view[sub & (view == 0)] = net
        view[clash] = -1

    def _paint(self, l, m, net):
        v = self.owner[l]
        clash = m & (v != 0) & (v != net)
        v[m & (v == 0)] = net
        v[clash] = -1

    def blocked(self, nid, width):
        """Cells where a track centre of net nid / width would violate clearance (per layer)."""
        r = width / 2 + CLR + G * 0.5
        st = disc(r)
        out = np.empty((2, self.nx, self.ny), dtype=bool)
        for l in (0, 1):
            foreign = (self.owner[l] != 0) & (self.owner[l] != nid)
            out[l] = ndimage.binary_dilation(foreign, structure=st) | self.outside
        # edge clearance for the track itself
        rr = int(math.ceil((width / 2) / G))
        out |= ndimage.binary_dilation(self.outside, iterations=max(1, rr))[None]
        return out

    def via_blocked(self, nid):
        r = VIA_D / 2 + CLR + G * 0.5
        st = disc(r)
        f = ((self.owner[0] != 0) & (self.owner[0] != nid)) | ((self.owner[1] != 0) & (self.owner[1] != nid))
        hole_keep = ndimage.binary_dilation(self.holes, structure=disc(VIA_DRILL + 0.3 + G))
        return ndimage.binary_dilation(f, structure=st) | ndimage.binary_dilation(self.outside, iterations=3) | hole_keep


def collect(board):
    nets, pads = {}, []
    for fp in board.GetFootprints():
        fpc = fp.GetPosition()
        for p in fp.Pads():
            name = p.GetNetname()
            bb = p.GetBoundingBox()
            lay = [k for k, L in enumerate(LAYERS) if p.IsOnLayer(L)]
            rec = dict(net=name, x=pcbnew.ToMM(p.GetPosition().x), y=pcbnew.ToMM(p.GetPosition().y),
                       xa=pcbnew.ToMM(bb.GetLeft()), ya=pcbnew.ToMM(bb.GetTop()),
                       xb=pcbnew.ToMM(bb.GetRight()), yb=pcbnew.ToMM(bb.GetBottom()),
                       layers=lay, ref=fp.GetReference(), num=p.GetNumber(),
                       fx=pcbnew.ToMM(fpc.x), fy=pcbnew.ToMM(fpc.y),
                       smd=p.GetAttribute() == pcbnew.PAD_ATTRIB_SMD,
                       npth=p.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH)
            pads.append(rec)
            if name:
                nets.setdefault(name, []).append(rec)
    return nets, pads


def width_of(net):
    return POWER.get(net, 0.2)


def paint_pads(grid, pads, net_id):
    for p in pads:
        nid = net_id.get(p["net"], -1) if p["net"] else -1
        if p["npth"] or not p["layers"]:
            for l in (0, 1):
                grid.paint_rect(l, p["xa"] - 0.3, p["ya"] - 0.3, p["xb"] + 0.3, p["yb"] + 0.3, -1)
            continue
        for l in p["layers"]:
            grid.paint_rect(l, p["xa"] - G / 2, p["ya"] - G / 2, p["xb"] + G / 2, p["yb"] + G / 2, nid)
        if p["smd"] and nid > 0:
            for (l, i, j) in escape_cells(grid, p):
                x, y = grid.xy(i, j)
                grid.paint_disc(l, x, y, 0.1 + G / 2, nid)


def escape_cells(grid, p):
    """Cells where a route may end on this pad: the pad's inner area (THT) or, for SMD pads,
    the pad centre line extended 1 mm outward away from the part centre (escape stub)."""
    cells = set()
    hw, hh = (p["xb"] - p["xa"]) / 2, (p["yb"] - p["ya"]) / 2
    if not p["smd"]:
        r = 0.45 * min(hw, hh)
        k = int(r / G)
        i0, j0 = grid.ij(p["x"], p["y"])
        for l in p["layers"]:
            for di in range(-k, k + 1):
                for dj in range(-k, k + 1):
                    if (di * G) ** 2 + (dj * G) ** 2 <= r * r + 1e-9:
                        cells.add((l, i0 + di, j0 + dj))
            cells.add((l, i0, j0))
        return cells
    dx, dy = p["x"] - p["fx"], p["y"] - p["fy"]
    if hw >= hh:
        ux, uy, half = (1 if dx >= 0 else -1), 0, hw
    else:
        ux, uy, half = 0, (1 if dy >= 0 else -1), hh
    l = p["layers"][0]
    for k in range(int((half + 1.0) / G) + 1):
        cells.add((l,) + grid.ij(p["x"] + ux * k * G, p["y"] + uy * k * G))
    return cells


DIRS = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
        (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]


def astar(grid, blk, vblk, sources, targets, free, tgt_xy, limit=600000, NET=None):
    tx, ty = tgt_xy
    best, prev, pq = {}, {}, []
    for s in sources:
        best[s] = 0.0
        heapq.heappush(pq, (0.0, 0.0, s, -1))
    n = 0
    while pq:
        f, d, s, lastdir = heapq.heappop(pq)
        if d > best.get(s, 1e18):
            continue
        if s in targets:
            path = [s]
            while s in prev:
                s = prev[s]
                path.append(s)
            return path[::-1]
        n += 1
        if n > limit:
            return None
        l, i, j = s
        for k, (di, dj, c) in enumerate(DIRS):
            s2 = (l, i + di, j + dj)
            if not (0 <= s2[1] < grid.nx and 0 <= s2[2] < grid.ny):
                continue
            if s2 not in free and blk[l, s2[1], s2[2]]:
                continue
            c2 = c + (0.15 if l == 1 else 0.0) + (0.4 if (lastdir >= 0 and k != lastdir) else 0.0)
            d2 = d + c2
            if d2 < best.get(s2, 1e18):
                best[s2] = d2
                prev[s2] = s
                x, y = grid.xy(s2[1], s2[2])
                heapq.heappush(pq, (d2 + math.hypot(x - tx, y - ty) / G, d2, s2, k))
        s2 = (1 - l, i, j)
        pa = cell_pad.get((NET,) + s)
        in_tht = pa is not None and not pa["smd"] and s2 in free
        if in_tht or (not blk[1 - l, i, j] and not vblk[i, j]):
            d2 = d + VIA_COST
            if d2 < best.get(s2, 1e18):
                best[s2] = d2
                prev[s2] = s
                x, y = grid.xy(i, j)
                heapq.heappush(pq, (d2 + math.hypot(x - tx, y - ty) / G, d2, s2, -1))
    return None


def commit(grid, path, nid, width):
    for a, b in zip(path, path[1:]):
        if a[0] != b[0]:
            x, y = grid.xy(a[1], a[2])
            grid.holes[a[1], a[2]] = True
            for l in (0, 1):
                grid.paint_disc(l, x, y, VIA_D / 2 + G / 2, nid)
    for l, i, j in path:
        x, y = grid.xy(i, j)
        grid.paint_disc(l, x, y, width / 2 + G / 2, nid)


def simplify(path):
    out, vias, run = [], [], [path[0]]
    for p in path[1:]:
        if p[0] != run[-1][0]:
            if len(run) > 1:
                out.append(run)
            vias.append((p[1], p[2]))
            run = [p]
            continue
        if len(run) >= 2:
            d0 = (run[-1][1] - run[-2][1], run[-1][2] - run[-2][2])
            d1 = (p[1] - run[-1][1], p[2] - run[-1][2])
            if d0 != d1:
                out.append(run)
                run = [run[-1]]
        run.append(p)
    if len(run) > 1:
        out.append(run)
    return [(r[0][0], (r[0][1], r[0][2]), (r[-1][1], r[-1][2])) for r in out], vias


def route_all(board, order_bias=()):
    nets, pads = collect(board)
    names = [n for n in nets if len(nets[n]) > 1]
    net_id = {n: k + 1 for k, n in enumerate(sorted(nets))}
    grid = Grid(board)
    paint_pads(grid, pads, net_id)

    def span(n):
        xs = [p["x"] for p in nets[n]]
        ys = [p["y"] for p in nets[n]]
        return (max(xs) - min(xs)) + (max(ys) - min(ys))

    order = [n for n in order_bias if n in names] + sorted(
        [n for n in names if n not in order_bias], key=lambda n: (n == "GND", n not in POWER, span(n)))
    results, failed = {}, []
    global cell_pad
    cell_pad = {}
    for n in order:
        nid, w = net_id[n], width_of(n)
        blk = grid.blocked(nid, w)
        vblk = grid.via_blocked(nid)
        pl = list(nets[n])
        esc = {id(p): escape_cells(grid, p) for p in pl}
        for p in pl:
            for c in esc[id(p)]:
                cell_pad.setdefault((n,) + c, p)
        tree = set(esc[id(pl[0])])
        free = set().union(*esc.values())
        todo = pl[1:]
        net_paths = []
        while todo:
            tx = np.mean([grid.xy(c[1], c[2])[0] for c in tree])
            ty = np.mean([grid.xy(c[1], c[2])[1] for c in tree])
            todo.sort(key=lambda p: math.hypot(p["x"] - tx, p["y"] - ty))
            p = todo.pop(0)
            tgt = esc[id(p)]
            path = astar(grid, blk, vblk, list(tree), tgt, free, (p["x"], p["y"]), NET=n)
            if path is None:
                failed.append((n, p["ref"] + "." + p["num"]))
                continue
            net_paths.append(path)
            tree.update(path)
            tree.update(tgt)
        for path in net_paths:
            commit(grid, path, nid, w)
        results[n] = net_paths
    return grid, results, failed, net_id, nets, pads, dict(cell_pad)


def add_track(board, a, b, w, layer, ni):
    t = pcbnew.PCB_TRACK(board)
    t.SetStart(pcbnew.VECTOR2I(pcbnew.FromMM(a[0]), pcbnew.FromMM(a[1])))
    t.SetEnd(pcbnew.VECTOR2I(pcbnew.FromMM(b[0]), pcbnew.FromMM(b[1])))
    t.SetWidth(pcbnew.FromMM(w))
    t.SetLayer(layer)
    t.SetNet(ni)
    board.Add(t)


def add_via(board, xy, ni):
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(pcbnew.VECTOR2I(pcbnew.FromMM(xy[0]), pcbnew.FromMM(xy[1])))
    v.SetWidth(pcbnew.FromMM(VIA_D))
    v.SetDrill(pcbnew.FromMM(VIA_DRILL))
    v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetNet(ni)
    board.Add(v)


def write_tracks(board, grid, results):
    nets = board.GetNetsByName()
    nseg = nvia = 0
    for n, paths in results.items():
        ni, w = nets[n], width_of(n)
        for path in paths:
            segs, vias = simplify(path)
            for l, a, b in segs:
                add_track(board, grid.xy(*a), grid.xy(*b), w, LAYERS[l], ni)
                nseg += 1
            for i, j in vias:
                pa = cell_pad.get((n, 0, i, j)) or cell_pad.get((n, 1, i, j))
                if pa is not None and not pa["smd"]:
                    continue                      # layer change inside a plated through-hole pad
                add_via(board, grid.xy(i, j), ni)
                nvia += 1
            # tie both path ends exactly to their pad centres
            for end in (path[0], path[-1]):
                pa = cell_pad.get((n,) + end)
                if pa is not None:
                    xy = grid.xy(end[1], end[2])
                    if math.hypot(xy[0] - pa["x"], xy[1] - pa["y"]) > 1e-3:
                        add_track(board, xy, (pa["x"], pa["y"]), min(w, 0.25) if pa["smd"] else w, LAYERS[end[0]], ni)
                        nseg += 1
    return nseg, nvia


def gnd_stitch(board, grid, pads, net_id):
    """A via beside every SMD GND pad so it reaches the bottom-layer GND pour."""
    gid = net_id["GND"]
    ni = board.GetNetsByName()["GND"]
    vblk = grid.via_blocked(gid)
    lblk = grid.blocked(gid, 0.3)
    n = 0
    for p in pads:
        if p["net"] != "GND" or not p["smd"]:
            continue
        l = p["layers"][0]
        hw, hh = (p["xb"] - p["xa"]) / 2, (p["yb"] - p["ya"]) / 2
        done = False
        for r in np.arange(0.5, 3.5, 0.2):
            for k in range(24):
                a = 2 * math.pi * k / 24
                x = p["x"] + (hw + r) * math.cos(a)
                y = p["y"] + (hh + r) * math.sin(a)
                i, j = grid.ij(x, y)
                if vblk[i, j]:
                    continue
                xx, yy = grid.xy(i, j)
                # the short link from pad to via must not cross foreign copper
                steps = int(math.hypot(xx - p["x"], yy - p["y"]) / (G / 2)) + 1
                okl = True
                for s in range(steps + 1):
                    qi, qj = grid.ij(p["x"] + (xx - p["x"]) * s / steps, p["y"] + (yy - p["y"]) * s / steps)
                    if lblk[l, qi, qj] and grid.owner[l, qi, qj] != gid:
                        okl = False
                        break
                if not okl:
                    continue
                add_via(board, (xx, yy), ni)
                add_track(board, (p["x"], p["y"]), (xx, yy), 0.3, LAYERS[l], ni)
                for L in (0, 1):
                    grid.paint_disc(L, xx, yy, VIA_D / 2 + G / 2, gid)
                grid.holes[i, j] = True
                vblk = grid.via_blocked(gid)
                lblk = grid.blocked(gid, 0.3)
                n += 1
                done = True
                break
            if done:
                break
    return n


if __name__ == "__main__":
    t0 = time.time()
    board = pcbnew.LoadBoard(PCB)
    for t in list(board.GetTracks()):
        board.Remove(t)
    for z in list(board.Zones()):
        board.Remove(z)
    bias, best = ["AIN1", "AIN2", "BIN1", "BIN2", "STBY", "PWMA", "PWMB"], None
    for attempt in range(int(os.environ.get("ROUTE_TRIES", "5"))):
        res = route_all(board, bias)
        failed = res[2]
        print("attempt %d: %d connections failed %s  (%.0fs)" % (attempt + 1, len(failed), failed[:6], time.time() - t0),
              flush=True)
        if best is None or len(failed) < len(best[2]):
            best = res
        if not failed:
            break
        bias = list(dict.fromkeys([f[0] for f in failed] + bias))
    grid, results, failed, net_id, nets, pads, cp = best
    cell_pad.clear()
    cell_pad.update(cp)
    nseg, nvia = write_tracks(board, grid, results)
    ns = gnd_stitch(board, grid, pads, net_id)
    pour(board)
    board.Save(PCB)
    print("tracks %d, vias %d, GND stitch vias %d, unrouted connections %d -> %s" % (nseg, nvia, ns, len(failed), PCB))

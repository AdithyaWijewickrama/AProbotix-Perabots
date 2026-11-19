"""reference.py - the best possible run on a test world, computed from the true map (Phase B).

usage: python test/tools/reference.py <world name>     -> test/results/ref/<name>.json

The reference robot has GuardianBot's real limits (simulation/controllers/main_controller):
top speed 0.30 m/s, acceleration / braking 3.8 m/s^2, turn on the spot up to 5.2 rad/s
(wheel speed cap) with 80 rad/s^2, and it can only curve gently while driving fast (W_RUN 0.15 rad/s),
so every corner is a stop + turn on the spot. It knows the whole map. "Best" = least time.

Section 1: START -> inside the maze entrance (y 1.30). Shortest path on a 1 cm grid around the true cubes,
           robot radius 75 mm + 10 mm margin, then straightened (any-angle) and timed.
Section 2: maze entrance -> out of the maze (y 2.46). The tour that looks into every dead end (the only
           cells that can hold a clue tile) and leaves: a dead end counts as checked when the robot is in it,
           or faces it from the next cell or from two cells away down a straight open corridor (camera).
           Exact search over (cell, heading, dead ends checked).
Section 3: out of the maze -> END. Collect the clan's piece, place it on a winning position (or the first
           free one in the controller's order if none), drive to END. Along the board lines, with the
           gripper times the controller uses (open 0.6 s, close 0.8 s, release 0.6 s).
"""
import heapq
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import gen  # noqa: E402

V, A = 0.30, 3.8
WMAX, ALPHA = 5.2, 80.0
ROBOT_R, MARGIN = 0.075, 0.010
CELL = gen.CELL


def t_straight(d, v=V, a=A):
    """Rest-to-rest time over distance d with a trapezoid speed profile."""
    if d <= 0:
        return 0.0
    if d >= v * v / a:
        return d / v + v / a
    return 2 * math.sqrt(d / a)


def t_turn(th):
    th = abs(th)
    if th < 1e-3:
        return 0.0
    if th >= WMAX * WMAX / ALPHA:
        return th / WMAX + WMAX / ALPHA + 0.05
    return 2 * math.sqrt(th / ALPHA) + 0.05


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


TURN_T = [0.0]                                               # turn time accumulated by polyline_time


def polyline_time(pts, yaw0):
    t, turns, d_all, yaw = 0.0, 0, 0.0, yaw0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        if d < 1e-4:
            continue
        h = math.atan2(y1 - y0, x1 - x0)
        dth = wrap(h - yaw)
        if abs(dth) > math.radians(3):
            t += t_turn(dth)
            TURN_T[0] += t_turn(dth)
            turns += 1
        yaw = h
        t += t_straight(d)
        d_all += d
    return t, d_all, turns


# ------------------------------------------------------------------ section 1
def section1(truth):
    obs = truth["obstacles"]
    G = 0.01
    infl = ROBOT_R + MARGIN
    nx, ny = int(2.44 / G), int(1.34 / G)

    def free(i, j):
        x, y = (i + 0.5) * G, (j + 0.5) * G
        if x < infl or x > 2.44 - infl or y < infl:
            return False
        if y > 1.22 - infl:                                  # only through the chamber gap (x 0.095 .. 0.345)
            if not (0.095 + infl <= x <= 0.345 - infl):
                return False
        for ox, oy in obs:
            if math.hypot(max(abs(x - ox) - 0.05, 0), max(abs(y - oy) - 0.05, 0)) < infl:
                return False
        return True

    F = [[free(i, j) for j in range(ny)] for i in range(nx)]
    x0, y0, a0 = truth["start_pose"]
    s = (int(x0 / G), int(y0 / G))
    goal_y = int(1.31 / G)
    dist = {s: 0.0}
    prev = {}
    pq = [(0.0, s)]
    end = None
    while pq:
        d, (i, j) = heapq.heappop(pq)
        if d > dist.get((i, j), 9e9):
            continue
        if j >= goal_y:
            end = (i, j)
            break
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if not di and not dj:
                    continue
                a, b = i + di, j + dj
                if 0 <= a < nx and 0 <= b < ny and F[a][b]:
                    nd = d + math.hypot(di, dj) * G
                    if nd < dist.get((a, b), 9e9):
                        dist[(a, b)] = nd
                        prev[(a, b)] = (i, j)
                        heapq.heappush(pq, (nd, (a, b)))
    if end is None:
        return None
    cells = [end]
    while cells[-1] in prev:
        cells.append(prev[cells[-1]])
    cells.reverse()

    def los(p, q):
        n = int(max(abs(q[0] - p[0]), abs(q[1] - p[1]))) * 2 + 1
        for k in range(n + 1):
            i = round(p[0] + (q[0] - p[0]) * k / n)
            j = round(p[1] + (q[1] - p[1]) * k / n)
            if not F[i][j]:
                return False
        return True

    pts, k = [cells[0]], 0                                   # string pulling (any-angle)
    while k < len(cells) - 1:
        m = len(cells) - 1
        while m > k + 1 and not los(cells[k], cells[m]):
            m -= 1
        pts.append(cells[m])
        k = m
    path = [(x0, y0)] + [((i + 0.5) * G, (j + 0.5) * G) for i, j in pts[1:]]
    TURN_T[0] = 0.0
    t, d, turns = polyline_time(path, a0)
    return {"time": round(t, 2), "distance": round(d, 3), "turns": turns, "turn_time": round(TURN_T[0], 2), "fixed_time": 0.0, "path": [[round(a, 3), round(b, 3)] for a, b in path]}


# ------------------------------------------------------------------ section 2
def section2(truth):
    vw = {tuple(w) for w in truth["maze"]["v_walls"]}
    hw = {tuple(w) for w in truth["maze"]["h_walls"]}
    deads = [tuple(d) for d in truth["maze"]["dead_ends"]]
    idx = {d: i for i, d in enumerate(deads)}
    full = (1 << len(deads)) - 1
    op = lambda c, r, h: gen.is_open(vw, hw, c, r, h)
    cell_t = CELL / V                                        # cruising one cell
    stop_start = V / A                                       # extra time of one stop + restart
    t90, t180 = t_turn(math.pi / 2) + stop_start, t_turn(math.pi) + stop_start

    def seen(c, r, h):
        m = 0
        if (c, r) in idx:
            m |= 1 << idx[(c, r)]
        cc, rr = c, r
        for _ in range(2):                                   # camera: 1-2 cells ahead down an open corridor
            if not op(cc, rr, h):
                break
            cc, rr = cc + gen.DIRS[h][0], rr + gen.DIRS[h][1]
            if (cc, rr) in idx:
                m |= 1 << idx[(cc, rr)]
        return m

    s0 = (0, 0, 0, seen(0, 0, 0))
    best = {s0: 0.0}
    prev = {}
    pq = [(0.0, s0)]
    goal = None
    while pq:
        t, st = heapq.heappop(pq)
        if t > best.get(st, 9e9):
            continue
        c, r, h, m = st
        if (c, r) == gen.END and h == 0 and m == full:
            goal = st
            break
        nxt = []
        for dh, cost in ((1, t90), (3, t90), (2, t180)):     # turn on the spot
            h2 = (h + dh) % 4
            nxt.append(((c, r, h2, m | seen(c, r, h2)), cost))
        if op(c, r, h):
            c2, r2 = c + gen.DIRS[h][0], r + gen.DIRS[h][1]
            nxt.append(((c2, r2, h, m | seen(c2, r2, h)), cell_t))
        for s2, cost in nxt:
            if t + cost < best.get(s2, 9e9):
                best[s2] = t + cost
                prev[s2] = st
                heapq.heappush(pq, (t + cost, s2))
    if goal is None:
        return None
    seq = [goal]
    while seq[-1] in prev:
        seq.append(prev[seq[-1]])
    seq.reverse()
    cells = []
    for c, r, h, m in seq:
        if not cells or cells[-1] != (c, r):
            cells.append((c, r))
    turns = sum(1 for a, b in zip(seq, seq[1:]) if a[2] != b[2])
    turn_time = sum(t_turn(math.pi / 2 * (1 if (b[2] - a[2]) % 2 else 2)) for a, b in zip(seq, seq[1:]) if a[2] != b[2])
    t = best[goal] + stop_start + (0.155 + 0.255) / V        # entry from y 1.30 to the first cell, exit to y 2.46
    pts = [[round(gen.MX0 + (c + 0.5) * CELL, 3), round(gen.MY0 + (r + 0.5) * CELL, 3)] for c, r in cells]
    pts = [[0.22, 1.30]] + pts + [[pts[-1][0], 2.46]]
    d = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    return {"time": round(t, 2), "distance": round(d, 3), "turns": turns, "turn_time": round(turn_time, 2), "fixed_time": 0.0, "cells": len(cells),
            "cells_unique": len(set(cells)), "dead_ends": len(deads), "path": pts}


def section2_fair(truth):
    """Simple explorer baseline (reported as s2_fair): a robot that does NOT know the maze.
    It is not a lower bound - GuardianBot's own strategy (needed cells, END bonus) usually beats it -
    but it shows how much the robot gains over plain "nearest unknown first" exploration. Ideal sensing: in a cell it knows that cell's 4 walls;
    facing down an open corridor the camera shows the next 2 cells and their walls. Strategy: always go to the
    nearest place that reveals something about a cell it can't yet rule out (unknown walls assumed open), until
    every dead end is found and looked at, then leave. Same time model as section2()."""
    vw = {tuple(w) for w in truth["maze"]["v_walls"]}
    hw = {tuple(w) for w in truth["maze"]["h_walls"]}
    op_true = lambda c, r, h: gen.is_open(vw, hw, c, r, h)
    known = {}                                               # (c, r, h) -> True open / False wall

    def learn(c, r, h):
        """ToF: walls along each of the 4 directions, up to 3 cells (0.8 m); camera: tiles up to 2 cells ahead."""
        for hh in range(4):
            cc, rr = c, r
            for _ in range(3):
                dc, dr = gen.DIRS[hh]
                if not (0 <= cc + dc < gen.COLS and 0 <= rr + dr < gen.ROWS):
                    break
                o = op_true(cc, rr, hh)
                known[(cc, rr, hh)] = o
                known[(cc + dc, rr + dr, (hh + 2) % 4)] = o
                if not o:
                    break
                cc, rr = cc + dc, rr + dr
        cc, rr = c, r
        looked.add((c, r))
        for _ in range(2):
            if not op_true(cc, rr, h):
                break
            cc, rr = cc + gen.DIRS[h][0], rr + gen.DIRS[h][1]
            looked.add((cc, rr))

    def op_k(c, r, h):
        dc, dr = gen.DIRS[h]
        if not (0 <= c + dc < gen.COLS and 0 <= r + dr < gen.ROWS):
            return False
        return known.get((c, r, h), True)

    def resolved(c, r):
        if sum(1 for h in range(4) if known.get((c, r, h)) is True) >= 2:
            return True                                      # two known openings: cannot be a dead end
        if not all((c, r, h) in known or not (0 <= c + gen.DIRS[h][0] < gen.COLS and 0 <= r + gen.DIRS[h][1] < gen.ROWS)
                   for h in range(4)):
            return False
        n = sum(op_k(c, r, h) for h in range(4))
        return n != 1 or (c, r) in looked or (c, r) in ((0, 0), gen.END)

    cell_t, stop_start = CELL / V, V / A
    t90, t180 = t_turn(math.pi / 2) + stop_start, t_turn(math.pi) + stop_start
    looked = set()
    st = (0, 0, 0)
    learn(*st)
    total, turn_time, turns, cells, guard = 0.0, 0.0, 0, [(0, 0)], 0

    def plan(goal_fn):
        best, prev, pq = {st: 0.0}, {}, [(0.0, st)]
        while pq:
            t, s_ = heapq.heappop(pq)
            if t > best.get(s_, 9e9):
                continue
            if goal_fn(s_) and s_ != st:
                seq = [s_]
                while seq[-1] in prev:
                    seq.append(prev[seq[-1]])
                return seq[::-1]
            c, r, h = s_
            nxt = [((c, r, (h + 1) % 4), t90), ((c, r, (h + 3) % 4), t90), ((c, r, (h + 2) % 4), t180)]
            if op_k(c, r, h):
                nxt.append(((c + gen.DIRS[h][0], r + gen.DIRS[h][1], h), cell_t))
            for s2, cost in nxt:
                if t + cost < best.get(s2, 9e9):
                    best[s2] = t + cost
                    prev[s2] = s_
                    heapq.heappush(pq, (t + cost, s2))
        return None

    def reveals(s_):
        """Would the sensors learn something new here? (unknown walls on a ToF ray, or an unlooked dead-end
        candidate in the camera's view). Uses only what is known; unknown walls count as open."""
        c, r, h = s_
        for hh in range(4):
            cc, rr = c, r
            for _ in range(3):
                dc, dr = gen.DIRS[hh]
                if not (0 <= cc + dc < gen.COLS and 0 <= rr + dr < gen.ROWS):
                    break
                if (cc, rr, hh) not in known:
                    return True
                if not known[(cc, rr, hh)]:
                    break
                cc, rr = cc + dc, rr + dr
        cc, rr = c, r
        for k in range(3):
            if k and (cc, rr) not in looked and not resolved(cc, rr):
                return True
            if k == 0 and (c, r) not in looked:
                return True
            if k == 2 or not op_k(cc, rr, h):
                break
            cc, rr = cc + gen.DIRS[h][0], rr + gen.DIRS[h][1]
        return False

    while guard < 400:
        guard += 1
        todo = [(c, r) for c in range(gen.COLS) for r in range(gen.ROWS) if not resolved(c, r)]
        goal = (lambda s_: s_[:2] == gen.END and s_[2] == 0) if not todo else reveals
        seq = plan(goal)
        if seq is None:
            break
        for a, b in zip(seq, seq[1:]):                       # execute until something new is learnt
            if a[2] != b[2]:
                tt = t90 if (b[2] - a[2]) % 2 else t180
                total += tt
                turn_time += tt - stop_start
                turns += 1
            else:
                if not op_true(*a):
                    break                                    # wall discovered on the way: replan
                total += cell_t
                cells.append(b[:2])
            st = b
            before = len(known), len(looked)
            learn(*st)
            if todo and (len(known), len(looked)) != before:
                break
        if not todo and st[:2] == gen.END and st[2] == 0:
            break
    total += stop_start + (0.155 + 0.255) / V
    pts = [[round(gen.MX0 + (c + 0.5) * CELL, 3), round(gen.MY0 + (r + 0.5) * CELL, 3)] for c, r in cells]
    pts = [[0.22, 1.30]] + pts + [[pts[-1][0], 2.46]]
    d = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    return {"time": round(total, 2), "distance": round(d, 3), "turns": turns, "turn_time": round(turn_time, 2),
            "fixed_time": 0.0, "cells": len(cells), "path": pts}


# ------------------------------------------------------------------ section 3
def section3(truth):
    board = {int(k): v for k, v in truth["board"].items()}
    clan = truth["clan"]
    win = truth["winning_cells"]
    if win:
        target = win[0]
    else:
        target = [p for p in (1, 5, 6, 8, 7, 3, 2, 9, 4) if p not in board][0]
    cx, cy = gen.POS_CELL[target]
    coll = (2.006, 2.70) if clan == "GREEN" else (2.006, 3.40)
    sgn = 1 if coll[1] > 3.05 else -1
    # along the lines: exit door -> exit junction -> collection junction -> stop 13.5 cm short, turn, back in, grip
    legs = [[(2.21, 2.46), (2.21, 3.05), (2.006, 3.05), (2.006, coll[1] - sgn * 0.135)]]
    TURN_T[0] = t_turn(math.pi) + t_turn(math.pi / 2)
    fixed = t_turn(math.pi) + 0.6 + 0.8                      # spin so the pocket faces the piece, open, close
    back_in = 0.135 - 0.033
    legs.append([(2.006, coll[1] - sgn * 0.033), (2.006, 3.05)])
    # to the transverse line 20 cm before the target cell, turn to face +x, back in, release, pull out
    ax = cx + 0.20
    leg3 = [(2.006, 3.05), (1.70, 3.05)]
    if abs(cy - 3.05) > 1e-3:
        leg3 += [(1.70, cy)]
    leg3 += [(ax, cy)]
    legs.append(leg3)
    fixed += t_turn(math.pi / 2) + 0.6 + 2 * t_straight(0.20 - 0.033)
    legs.append([(ax, cy), (ax, 3.05) if abs(cy - 3.05) > 1e-3 else (ax, cy), (0.50, 3.05), (0.13, 3.05)])
    t, d, turns = fixed + t_straight(back_in), back_in, 2
    yaw = math.pi / 2
    for leg in legs:
        lt, ld, lturn = polyline_time(leg, yaw)
        t, d, turns = t + lt, d + ld, turns + lturn
        if len(leg) > 1:
            yaw = math.atan2(leg[-1][1] - leg[-2][1], leg[-1][0] - leg[-2][0])
    pts = [list(p) for leg in legs for p in leg]
    return {"time": round(t, 2), "distance": round(d, 3), "turns": turns, "turn_time": round(TURN_T[0], 2),
            "fixed_time": 2.0, "target": target, "path": pts}


def reference(name):
    truth = json.load(open(os.path.join(TEST, "truth", name + ".json")))
    out = {"name": name, "start": truth["start"]}
    if truth["start"] == "full":
        out["s1"] = section1(truth)
    if truth["start"] in ("full", "maze"):
        out["s2"] = section2(truth)
        out["s2_fair"] = section2_fair(truth)
    out["s3"] = section3(truth)
    os.makedirs(os.path.join(TEST, "results", "ref"), exist_ok=True)
    json.dump(out, open(os.path.join(TEST, "results", "ref", name + ".json"), "w"), indent=1)
    return out


if __name__ == "__main__":
    for nm in sys.argv[1:]:
        r = reference(nm)
        print(nm, "  ".join("%s %.1f s %.2f m %d turns" % (k, r[k]["time"], r[k]["distance"], r[k]["turns"])
                            for k in ("s1", "s2", "s2_fair", "s3") if r.get(k)))

"""
gen.py - random but legal Final Round test arenas (Phase A of docs/TEST_PLAN.md).

usage:  python test/tools/gen.py <name> <seed> [start] [options]
        start = full (START area, default) | maze (robot at the maze entrance) | s3 (Section 3 restart)
        options (env vars): NOBS=a-b  (Section 1 cubes, default 30-43)
                            LOOPS=a-b (extra maze openings, default 0-2)
                            NTILES=a-b (clue tiles, default 3-7, always odd)
                            DECEPTIVE=1 (dead ends near the exit, long side branches)
                            STRAIGHT=1 (no random start angle), CROOK=deg (start this far off straight)
        seed 0 = the official arena (rulebook Figure 1), unchanged.

writes  test/protos/Arena_<name>.proto, test/protos/Obstacles_<name>.proto,
        test/worlds/<name>.wbt  (opens in Webots R2025a, uses the real GuardianBot.proto and main_controller)
        test/truth/<name>.json  (obstacles, maze walls, tiles, clues, board, correct answer)

The arena keeps everything the rulebook fixes (outer walls, section dividers, chamber, 9 x 4 maze frame,
25 cm cells, Section 3 board and lines). Only the Section 1 cubes, the maze walls, the clue tiles and the
board pieces change. The board pieces follow the clues: cylinders at (blue tiles) and (right turns), cubes at
(green tiles) and (left turns); a clue outside 1-9 or on a taken position gets a random free position, which
is what makes the robot inspect the board.
"""
import heapq
import json
import math
import os
import random
import re
import sys
from collections import deque

HERE = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.dirname(HERE)
ROOT = os.path.dirname(TEST)                               # "Phase 02 - Final Round"
SIM = os.path.join(ROOT, "simulation")

COLS, ROWS, CELL = 9, 4, 0.25
MX0, MY0 = 0.095, 1.33
END = (8, 3)
DIRS = [(0, 1), (1, 0), (0, -1), (-1, 0)]                  # h = 0 +y, 1 +x, 2 -y, 3 -x (controller convention)
MAGIC = ((6, 1, 8), (7, 5, 3), (2, 9, 4))
ROW_X = (1.50, 1.10, 0.70)
COL_Y = (2.75, 3.05, 3.35)
POS_CELL = {MAGIC[i][j]: (ROW_X[i], COL_Y[j]) for i in range(3) for j in range(3)}
START_POSE = (2.315, 0.125, math.pi / 2)
MAZE_POSE = (0.22, 1.275, math.pi / 2)                      # on the chamber tile, facing into the maze
S3_POSE = (2.220, 2.205, math.pi / 2)                       # maze exit cell, facing the exit door
ROBOT_R = 0.075


def env_range(name, lo, hi):
    v = os.environ.get(name)
    if not v:
        return lo, hi
    a, _, b = v.partition("-")
    return int(a), int(b or a)


def solid(name, x, y, z, sx, sy, sz, color="0.02 0.02 0.02", bound=True):
    b = " boundingObject Box { size %g %g %g }" % (sx, sy, sz) if bound else ""
    return ('      Solid { translation %.4f %.4f %.4f children [ Shape { appearance PBRAppearance { baseColor %s roughness 1 '
            'metalness 0 } geometry Box { size %g %g %g } } ] name "%s"%s }\n' % (x, y, z, color, sx, sy, sz, name, b))


# ------------------------------------------------------------------ maze
def maze_edges_from_official(arena):
    """Parse the official maze walls into the open/closed edge sets used below."""
    vw, hw = set(), set()                                  # vw: (k, r) wall on line x=k between col k-1,k ; hw: (c, k)
    for m in re.finditer(r'translation ([\d.]+) ([\d.]+) [\d.]+ children .*?size ([\d.]+) ([\d.]+) [\d.]+ .*?name "maze_wall_', arena):
        x, y, sx, sy = map(float, m.groups())
        if sx < 0.05:                                       # vertical wall
            k = round((x - MX0) / CELL)
            n = round(sy / CELL)
            r0 = (y - sy / 2 - MY0) / CELL
            for r in range(round(r0), round(r0) + n):
                vw.add((k, r))
        else:
            k = round((y - MY0) / CELL)
            n = round(sx / CELL)
            c0 = (x - sx / 2 - MX0) / CELL
            for c in range(round(c0), round(c0) + n):
                hw.add((c, k))
    return vw, hw


def random_maze(rng, deceptive):
    vw = {(k, r) for k in range(1, COLS) for r in range(ROWS)}
    hw = {(c, k) for c in range(COLS) for k in range(1, ROWS)}
    seen, st = {(0, 0)}, [(0, 0)]
    while st:                                               # depth-first spanning tree (long corridors)
        c, r = st[-1]
        nb = [(c + dc, r + dr) for dc, dr in DIRS if 0 <= c + dc < COLS and 0 <= r + dr < ROWS and (c + dc, r + dr) not in seen]
        if deceptive and nb:                                # prefer going away from the exit first: long side branches
            nb.sort(key=lambda p: -(abs(p[0] - END[0]) + abs(p[1] - END[1])))
            nb = nb[:1] if rng.random() < 0.6 else nb
        if not nb:
            st.pop()
            continue
        n = rng.choice(nb)
        if n[0] != c:
            vw.discard((max(c, n[0]), r))
        else:
            hw.discard((c, max(r, n[1])))
        seen.add(n)
        st.append(n)
    walls = sorted(vw | hw, key=str)
    for _ in range(rng.randint(*env_range("LOOPS", 0, 2))):
        w = rng.choice(sorted(vw | hw))
        vw.discard(w)
        hw.discard(w)
    return vw, hw


def is_open(vw, hw, c, r, h):
    dc, dr = DIRS[h]
    nc, nr = c + dc, r + dr
    if not (0 <= nc < COLS and 0 <= nr < ROWS):
        return False
    if dc:
        return (max(c, nc), r) not in vw
    return (c, max(r, nr)) not in hw


def n_open(vw, hw, c, r):
    return sum(is_open(vw, hw, c, r, h) for h in range(4))


def clue_turns(vw, hw):
    """Same as main_controller.maze_clues(): fewest cells, then fewest turns, from (0,0) heading +y to END."""
    start = (0, 0, 0)
    best = {start: (0, 0)}
    pq = [((0, 0), start, 0, 0, [(0, 0)])]
    while pq:
        (cells, turns), (c, r, h), R, L, path = heapq.heappop(pq)
        if (c, r) == END:
            return R, L, path
        for h2 in range(4):
            if not is_open(vw, hw, c, r, h2) or h2 == (h + 2) % 4:
                continue
            nc, nr = c + DIRS[h2][0], r + DIRS[h2][1]
            R2 = R + (h2 == (h + 1) % 4)
            L2 = L + (h2 == (h + 3) % 4)
            k = (cells + 1, turns + (h2 != h))
            st = (nc, nr, h2)
            if st not in best or k < best[st]:
                best[st] = k
                heapq.heappush(pq, (k, st, R2, L2, path + [(nc, nr)]))
    return 0, 0, []


# ------------------------------------------------------------------ section 1
def s1_free(obs, inflate):
    """Grid (2 cm) of free space for a disc of radius `inflate`; True = free."""
    G = 0.02
    nx, ny = int(2.44 / G), int(1.22 / G)
    free = [[True] * ny for _ in range(nx)]
    for i in range(nx):
        x = (i + 0.5) * G
        for j in range(ny):
            y = (j + 0.5) * G
            if x < inflate or x > 2.44 - inflate or y < inflate:
                free[i][j] = False
                continue
            if y > 1.22 - inflate and not (0.095 + inflate - 0.03 <= x <= 0.345 - inflate + 0.03):
                free[i][j] = False
                continue
            for ox, oy in obs:
                if abs(x - ox) < 0.05 + inflate and abs(y - oy) < 0.05 + inflate:
                    if math.hypot(max(abs(x - ox) - 0.05, 0), max(abs(y - oy) - 0.05, 0)) < inflate:
                        free[i][j] = False
                        break
    return free, G


def s1_reachable(obs, inflate):
    free, G = s1_free(obs, inflate)
    nx, ny = len(free), len(free[0])
    g = (int(0.22 / G), ny - 1)
    s0 = [(i, j) for i in range(int(2.20 / G), nx) for j in range(0, int(0.25 / G)) if free[i][j]]
    if not s0:
        return False
    q, seen = deque(s0), set(s0)
    while q:
        i, j = q.popleft()
        if j >= ny - 2 and abs(i - g[0]) <= 2:
            return True
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if 0 <= a < nx and 0 <= b < ny and free[a][b] and (a, b) not in seen:
                seen.add((a, b))
                q.append((a, b))
    return False


def random_obstacles(rng):
    lo, hi = env_range("NOBS", 30, 43)
    n = rng.randint(lo, hi)
    for _ in range(200):
        obs = []
        while len(obs) < n:                                 # short rows of 1-5 touching cubes, like the official field
            k = rng.randint(1, 5)
            x, y = rng.uniform(0.06, 2.38), rng.uniform(0.06, 1.16)
            dx, dy = rng.choice(((0.1, 0), (0, 0.1), (0.1, rng.uniform(-0.06, 0.06)), (rng.uniform(-0.06, 0.06), 0.1)))
            for m in range(k):
                ox, oy = x + m * dx, y + m * dy
                if not (0.056 <= ox <= 2.384 and 0.056 <= oy <= 1.164):
                    break
                if ox > 2.12 and oy < 0.33:                 # keep the START area clear
                    break
                if any(abs(ox - a) < 0.1 and abs(oy - b) < 0.1 for a, b in obs):
                    break
                obs.append((round(ox, 3), round(oy, 3)))
                if len(obs) >= n:
                    break
        if s1_reachable(obs, 0.105):                        # rulebook: a passage wide enough for the robot (>= 21 cm)
            return obs
    raise RuntimeError("no legal Section 1 layout found")


# ------------------------------------------------------------------ board
def winning_cells(board, clan):
    lines = [tuple(MAGIC[i][j] for i in range(3)) for j in range(3)] + [MAGIC[i] for i in range(3)] + [(6, 5, 4), (8, 5, 2)]
    out = set()
    for line in lines:
        mine = [p for p in line if board.get(p) == clan]
        empty = [p for p in line if p not in board]
        if len(mine) == 2 and len(empty) == 1:
            out.add(empty[0])
    return sorted(out)


def make_board(rng, nb, ng, R, L):
    """Clue pieces first; pieces whose clue is unusable go to random free positions, chosen (when possible)
    so that the robot's clan has a winning move - the game assumes one exists."""
    import itertools
    board, note, loose = {}, [], []
    for col, v, nm in (("BLUE", nb, "cyl1"), ("GREEN", ng, "cube1"), ("BLUE", R, "cyl2"), ("GREEN", L, "cube2")):
        if 1 <= v <= 9 and v not in board:
            board[v] = col
        else:
            loose.append((col, v, nm))
    clan = "BLUE" if nb >= ng else "GREEN"
    free = [q for q in range(1, 10) if q not in board]
    combos = list(itertools.permutations(free, len(loose)))
    rng.shuffle(combos)
    good = [c for c in combos if winning_cells({**board, **{p: col for p, (col, _, _) in zip(c, loose)}}, clan)]
    pick = (good or combos or [()])[0]
    for p, (col, v, nm) in zip(pick, loose):
        board[p] = col
        note.append("%s clue %d unusable -> placed at random position %d" % (nm, v, p))
    if not good and loose:
        note.append("no placement gives a winning move")
    return board, note


# ------------------------------------------------------------------ build
def piece(name, x, y, col):
    if col == "BLUE":
        geo = "Cylinder { height 0.05 radius 0.025 }"
        rgb = "0.05 0.2 0.9"
    else:
        geo = "Box { size 0.05 0.05 0.05 }"
        rgb = "0.05 0.75 0.15"
    return ('Solid {\n  translation %.3f %.3f 0.025\n  children [ Shape { appearance PBRAppearance { baseColor %s roughness 0.4 metalness 0 } '
            'geometry %s } ]\n  name "%s"\n  boundingObject %s\n  physics Physics { density -1 mass 0.03 }\n}\n' % (x, y, rgb, geo, name, geo))


def build(name, seed, start="full"):
    rng = random.Random(seed)
    arena = open(os.path.join(SIM, "protos", "Arena.proto")).read()
    obst = open(os.path.join(SIM, "protos", "Obstacles.proto")).read()
    if seed == 0:
        vw, hw = maze_edges_from_official(arena)
        tiles = {}
        for m in re.finditer(r'translation ([\d.]+) ([\d.]+) [\d.]+ children .*?name "maze_tile_(blue|green)_', arena):
            x, y, col = float(m.group(1)), float(m.group(2)), m.group(3).upper()
            tiles[(int((x - MX0) // CELL), int((y - MY0) // CELL))] = col
        obs = [tuple(map(float, m.groups())) for m in re.finditer(r"translation ([\d.]+) ([\d.]+) 0\.05", obst)]
    else:
        deceptive = os.environ.get("DECEPTIVE") == "1"
        lo_t = env_range("NTILES", 3, 7)[0]
        for _ in range(500):                                 # need enough dead ends for the clue tiles
            vw, hw = random_maze(rng, deceptive)
            deads = [(c, r) for c in range(COLS) for r in range(ROWS) if n_open(vw, hw, c, r) == 1 and (c, r) not in ((0, 0), END)]
            if len(deads) >= lo_t:
                break
        if deceptive:                                        # tiles far from the straight route first
            deads.sort(key=lambda p: -(abs(p[0] - END[0]) + abs(p[1] - END[1])) + rng.random())
        else:
            rng.shuffle(deads)
        nt = min(len(deads), rng.randint(*env_range("NTILES", 3, 7)))
        if nt % 2 == 0:
            nt -= 1
        nblue = rng.randint(0, nt)
        if nblue * 2 == nt:
            nblue += 1
        cols = ["BLUE"] * nblue + ["GREEN"] * (nt - nblue)
        rng.shuffle(cols)
        tiles = dict(zip(deads[:nt], cols))
        obs = random_obstacles(rng)

    nb = sum(1 for v in tiles.values() if v == "BLUE")
    ng = len(tiles) - nb
    R, L, path = clue_turns(vw, hw)
    if seed == 0:
        board = {2: "GREEN", 4: "GREEN", 5: "BLUE", 3: "BLUE"}  # rulebook Figure 1 (what perabots_arena.wbt uses)
        note = ["official Figure 1 board"]
    else:
        board, note = make_board(rng, nb, ng, R, L)
    clan = "BLUE" if nb >= ng else "GREEN"
    deads = [(c, r) for c in range(COLS) for r in range(ROWS) if n_open(vw, hw, c, r) == 1 and (c, r) not in ((0, 0), END)]

    # ---- protos
    a2 = arena
    if seed != 0:
        a2 = re.sub(r'^      Solid \{ translation [^\n]*name "maze_(wall|tile)_[^\n]*\n', "", a2, flags=re.M)
        body = ""
        for (k, r) in sorted(vw):
            body += solid("maze_wall_v_%d_%d" % (k, r), MX0 + k * CELL, MY0 + (r + 0.5) * CELL, 0.05, 0.012, 0.262, 0.1, "1 1 1")
        for (c, k) in sorted(hw):
            body += solid("maze_wall_h_%d_%d" % (c, k), MX0 + (c + 0.5) * CELL, MY0 + k * CELL, 0.05, 0.262, 0.012, 0.1, "1 1 1")
        for (c, r), col in sorted(tiles.items()):
            body += solid("maze_tile_%s_%d_%d" % (col.lower(), c, r), MX0 + (c + 0.5) * CELL, MY0 + (r + 0.5) * CELL, 0.001,
                          0.25, 0.25, 0.001, "0 0 1" if col == "BLUE" else "0 1 0", bound=False)
        a2 = a2.replace("      # ========================================================================\n      # SECTION 3",
                        "      # random maze %s (test/tools/gen.py seed %d)\n%s      # ========================================================================\n      # SECTION 3" % (name, seed, body), 1)
        assert "random maze" in a2
    a2 = a2.replace("PROTO Arena [", "PROTO Arena_%s [" % name)
    o2 = "#VRML_SIM R2025a utf8\n# Section 1 cubes for test world %s (seed %d)\nPROTO Obstacles_%s [\n] {\n  Group {\n    children [\n" % (name, seed, name)
    for i, (x, y) in enumerate(obs):
        o2 += solid("obstacle_%d" % (i + 1), x, y, 0.05, 0.1, 0.1, 0.1)
    o2 += "    ]\n  }\n}\n"
    os.makedirs(os.path.join(TEST, "protos"), exist_ok=True)
    open(os.path.join(TEST, "protos", "Arena_%s.proto" % name), "w").write(a2)
    open(os.path.join(TEST, "protos", "Obstacles_%s.proto" % name), "w").write(o2)

    # ---- world
    w = open(os.path.join(SIM, "worlds", "perabots_arena.wbt")).read()
    w = w.replace('EXTERNPROTO "../protos/Arena.proto"', 'EXTERNPROTO "../protos/Arena_%s.proto"' % name)
    w = w.replace('EXTERNPROTO "../protos/Obstacles.proto"', 'EXTERNPROTO "../protos/Obstacles_%s.proto"' % name)
    w = w.replace('EXTERNPROTO "../protos/GuardianBot.proto"', 'EXTERNPROTO "../../simulation/protos/GuardianBot.proto"')
    w = re.sub(r"\nArena \{\n\}", "\nArena_%s {\n}" % name, w)
    w = re.sub(r"\nObstacles \{\n\}", "\nObstacles_%s {\n}" % name, w)
    head, _, _ = w.partition("# ---- movable game pieces")
    pieces = "# ---- movable game pieces (from the clues, see test/truth/%s.json) ----\n" % name
    for p, col in sorted(board.items()):
        x, y = POS_CELL[p]
        pieces += piece("board_%s_pos%d" % ("blue_cyl" if col == "BLUE" else "green_cube", p), x, y, col)
    pieces += piece("collect_green_cube", 2.006, 2.700, "GREEN") + piece("collect_blue_cyl", 2.006, 3.400, "BLUE")
    x0, y0, a0 = {"full": START_POSE, "maze": MAZE_POSE, "s3": S3_POSE}[start]
    if start == "full" and seed != 0:
        crook = rng.uniform(-0.35, 0.35)                   # placed by hand: not perfectly straight
        if os.environ.get("STRAIGHT") != "1":              # STRAIGHT=1: same arena, robot placed exactly straight
            a0 += crook
    if os.environ.get("CROOK"):                            # CROOK=deg: place the robot this many degrees off straight
        a0 += math.radians(float(os.environ["CROOK"]))
    args = ('\n  controllerArgs [ "section3" "clues=%d,%d,%d,%d" ]' % (nb, ng, R, L)) if start == "s3" else ""
    robot = ('# ---- robot ----\nGuardianBot {\n  translation %.3f %.3f 0\n  rotation 0 0 1 %.4f\n  name "team_robot"\n'
             '  controller "test_runner"%s\n}\n' % (x0, y0, a0, args))
    w = head + pieces + robot
    w = w.replace('title "PeraBots 2026 - The Nine Seals of Pera"', 'title "GuardianBot test world %s (seed %d, start %s)"' % (name, seed, start))
    os.makedirs(os.path.join(TEST, "worlds"), exist_ok=True)
    open(os.path.join(TEST, "worlds", "%s.wbt" % name), "w").write(w)

    truth = {
        "name": name, "seed": seed, "start": start, "start_pose": [x0, y0, a0],
        "obstacles": obs,
        "maze": {"v_walls": sorted(vw), "h_walls": sorted(hw), "dead_ends": deads,
                 "tiles": {"%d,%d" % k: v for k, v in sorted(tiles.items())}},
        "clues": {"blue": nb, "green": ng, "right": R, "left": L, "clue_path": path},
        "board": {str(k): v for k, v in sorted(board.items())}, "board_note": note,
        "clan": clan, "winning_cells": winning_cells(board, clan),
    }
    os.makedirs(os.path.join(TEST, "truth"), exist_ok=True)
    json.dump(truth, open(os.path.join(TEST, "truth", "%s.json" % name), "w"), indent=1)
    return truth


if __name__ == "__main__":
    nm, sd = sys.argv[1], int(sys.argv[2])
    st = sys.argv[3] if len(sys.argv) > 3 else "full"
    t = build(nm, sd, st)
    print("%s: %d cubes, tiles B%d G%d, clues R%d L%d, board %s, clan %s, win %s %s" % (
        nm, len(t["obstacles"]), t["clues"]["blue"], t["clues"]["green"], t["clues"]["right"], t["clues"]["left"],
        t["board"], t["clan"], t["winning_cells"], "; ".join(t["board_note"])))

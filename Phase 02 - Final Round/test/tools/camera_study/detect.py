"""Fix 4 camera study: the board detector (same logic as board_look() in main_controller.py), run offline
on frames captured by capture.sh.   usage: python detect.py <pitch> [pose index] [downsample]
pose 0 = exit junction (2.21, 3.05) facing the board, 1 = collection junction, 2..4 = on the board lines."""
import math, json, sys, os
HERE = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.dirname(os.path.dirname(HERE))
OUT = os.environ.get("LOOK_DIR", "/tmp/look")

BLUE, GREEN, FLOOR, JUNK = 1, 2, 0, -1
ROW_X = (1.50, 1.10, 0.70)
COL_Y = (2.75, 3.05, 3.35)
MAGIC = ((6, 1, 8), (7, 5, 3), (2, 9, 4))
POS_CELL = {MAGIC[i][j]: (ROW_X[i], COL_Y[j]) for i in range(3) for j in range(3)}
CAM_X, CAM_Z, FOV = 0.062, 0.140, 0.90
PIECE_H, PIECE_R = 0.05, 0.025


def classify(r, g, b):
    m = max(r, g, b)
    if m < 50:
        return FLOOR
    if b > 45 and b > 1.5 * g + 15 and b > 1.5 * r + 15:
        return BLUE
    if g > 45 and g > 1.5 * b + 15 and g > 1.5 * r + 15:
        return GREEN
    return FLOOR


def ray(u, v, W, H, pitch):
    f = (W / 2) / math.tan(FOV / 2)
    x, y, z = f, -(u + 0.5 - W / 2), -(v + 0.5 - H / 2)
    x2 = x * math.cos(pitch) + z * math.sin(pitch)
    z2 = -x * math.sin(pitch) + z * math.cos(pitch)
    return x2, y, z2


def hit(u, v, W, H, pitch, zp, pose):
    """Where the pixel's ray meets the plane z = zp (world x, y), or None."""
    x2, y, z2 = ray(u, v, W, H, pitch)
    if z2 >= -1e-6:
        return None
    t = (zp - CAM_Z) / z2
    gx, gy = CAM_X + t * x2, t * y
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return px + c * gx - s * gy, py + s * gx + c * gy, math.hypot(gx - CAM_X, gy)


def project(wx, wy, wz, W, H, pitch, pose):
    px, py, yaw = pose
    c, s_ = math.cos(yaw), math.sin(yaw)
    dx, dy = wx - px, wy - py
    X, Y, Z = c * dx + s_ * dy - CAM_X, -s_ * dx + c * dy, wz - CAM_Z
    x = X * math.cos(pitch) - Z * math.sin(pitch)
    z = X * math.sin(pitch) + Z * math.cos(pitch)
    if x <= 0.01:
        return None
    f = (W / 2) / math.tan(FOV / 2)
    return W / 2 - f * Y / x - 0.5, H / 2 - f * z / x - 0.5


XT = (0.50, 0.90, 1.30, 1.70)
YL = (2.60, 2.90, 3.20, 3.50)


def calibrate(img, W, H, pose, pitch, span=0.08):
    """Fit the camera pitch (and heading) to the board's black grid lines: the servo and the floor are never exact."""
    dark = [[max(img[4 * (v * W + u)], img[4 * (v * W + u) + 1], img[4 * (v * W + u) + 2]) < 90 for u in range(W)] for v in range(H)]
    pts = [(x, 2.62 + 0.04 * k, 0.08, 0.0) for x in XT for k in range(22)] + [(1.68 - 0.04 * k, y, 0.0, 0.06) for y in YL for k in range(30)]
    best = (-1, pitch, 0.0)
    for dyaw in (-0.04, -0.02, 0.0, 0.02, 0.04):
        pz = (pose[0], pose[1], pose[2] + dyaw)
        for i in range(int(2 * span / 0.002) + 1):
            p = pitch - span + 0.002 * i
            sc = 0
            for wx, wy, ox, oy in pts:
                q = project(wx, wy, 0.0, W, H, p, pz)
                n = [project(wx + k * ox, wy + k * oy, 0.0, W, H, p, pz) for k in (-1, 1)]
                if q and all(n):
                    vals = []
                    for (uu, vv) in [q] + n:
                        u, v = int(round(uu)), int(round(vv))
                        vals.append(dark[v][u] if 0 <= u < W and 0 <= v < H else None)
                    if None not in vals:
                        sc += vals[0] - 0.5 * (vals[1] + vals[2])     # dark on the line, light on both sides
            if sc > best[0]:
                best = (sc, p, dyaw)
    return best


def hidden_cells(found, pose):
    """Empty cells that sit in the shadow of a seen piece (same bearing, farther, inside its shadow)."""
    cx0, cy0 = pose[0] + CAM_X * math.cos(pose[2]), pose[1] + CAM_X * math.sin(pose[2])
    out = []
    for q, (qx, qy) in POS_CELL.items():
        if q in found:
            continue
        dq, aq = math.hypot(qx - cx0, qy - cy0), math.atan2(qy - cy0, qx - cx0)
        for p in found:
            px_, py_ = POS_CELL[p]
            dp, ap = math.hypot(px_ - cx0, py_ - cy0), math.atan2(py_ - cy0, px_ - cx0)
            lat = abs(math.sin(aq - ap)) * dq
            # shadow of a 5 cm piece seen from 14 cm: reaches (dp + r) * 14 / 9
            if dp < dq and lat < 0.04 and (dq - PIECE_R) < (dp + PIECE_R) * CAM_Z / (CAM_Z - PIECE_H):
                out.append(q)
                break
    return out


def fill_hidden(found, pose, n_each=2):
    """The board always has 2 cylinders + 2 cubes: put the missing ones in the hidden cells if that is unambiguous."""
    miss = [c for c in ('BLUE', 'GREEN') for _ in range(n_each - sum(1 for v in found.values() if v == c))]
    if not miss or any(n_each - sum(1 for v in found.values() if v == c) < 0 for c in ('BLUE', 'GREEN')):
        return found
    hid = hidden_cells(found, pose)
    if len(hid) == len(miss) and len(set(miss)) == 1:
        found = dict(found)
        for q in hid:
            found[q] = miss[0]
    return found


def detect(img, W, H, pose, pitch, debug=False, calib=True):
    if calib:
        sc, pitch, dyaw = calibrate(img, W, H, pose, pitch)
        pose = (pose[0], pose[1], pose[2] + dyaw)
    lab = [[classify(img[4 * (v * W + u) + 2], img[4 * (v * W + u) + 1], img[4 * (v * W + u)]) for u in range(W)] for v in range(H)]
    for v in range(1, H - 1):                       # close 1-pixel gaps left by sensor noise (vertical and horizontal)
        for u in range(W):
            if lab[v][u] not in (BLUE, GREEN) and lab[v - 1][u] == lab[v + 1][u] and lab[v - 1][u] in (BLUE, GREEN):
                lab[v][u] = lab[v - 1][u]
    for v in range(H):
        for u in range(1, W - 1):
            if lab[v][u] not in (BLUE, GREEN) and lab[v][u - 1] == lab[v][u + 1] and lab[v][u - 1] in (BLUE, GREEN):
                lab[v][u] = lab[v][u - 1]
    seen = [[False] * W for _ in range(H)]
    votes = {}
    blobs = []
    for v0 in range(H):
        for u0 in range(W):
            k = lab[v0][u0]
            if k not in (BLUE, GREEN) or seen[v0][u0]:
                continue
            st, pts = [(u0, v0)], []
            seen[v0][u0] = True
            while st:
                u, v = st.pop()
                pts.append((u, v))
                for du, dv in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, -1), (1, -1), (-1, 1)):
                    a, b = u + du, v + dv
                    if 0 <= a < W and 0 <= b < H and not seen[b][a] and lab[b][a] == k:
                        seen[b][a] = True
                        st.append((a, b))
            if len(pts) < 2:
                continue
            # split the blob into image columns; each column: bottom pixel -> piece base, top pixel -> piece top
            cols = {}
            for u, v in pts:
                lo, hi = cols.get(u, (v, v))
                cols[u] = (min(lo, v), max(hi, v))
            for u, (top, bot) in cols.items():
                b = hit(u, bot + 0.5, W, H, pitch, 0.0, pose)          # lower edge of the bottom pixel = base
                if b is None:
                    continue
                cands = [b]
                t = hit(u, top - 0.5, W, H, pitch, PIECE_H, pose)      # upper edge of the top pixel = top face
                t2 = hit(u, top + 1.5, W, H, pitch, PIECE_H, pose)     # 2 pixels lower: still far beyond -> 2nd piece
                if t is not None and t2 is not None and t2[2] > b[2] + 2 * PIECE_R + 0.10:
                    cands.append(t)                                    # a second piece hidden behind in this column
                for (wx, wy, d), back in zip(cands, (1, -1)):
                    # move from the front (or back) face to the centre, along the view direction
                    ux, uy = wx - (pose[0] + CAM_X * math.cos(pose[2])), wy - (pose[1] + CAM_X * math.sin(pose[2]))
                    n = math.hypot(ux, uy) or 1
                    cx, cy = wx + back * PIECE_R * ux / n, wy + back * PIECE_R * uy / n
                    best = min(POS_CELL, key=lambda p: math.hypot(POS_CELL[p][0] - cx, POS_CELL[p][1] - cy))
                    e = math.hypot(POS_CELL[best][0] - cx, POS_CELL[best][1] - cy)
                    if e < 0.13:
                        votes.setdefault(best, [0, 0])[k - 1] += 1
            blobs.append((k, len(pts)))
    out = {}
    for p, (nb, ng) in votes.items():
        if nb + ng >= 2:
            out[p] = 'BLUE' if nb >= ng else 'GREEN'
    out = fill_hidden(out, pose)
    if debug:
        return out, votes, blobs
    return out


if __name__ == '__main__':
    pitch = sys.argv[1]
    poses = [[2.21, 3.05, math.pi], [2.006, 3.05, math.pi], [1.70, 3.05, math.pi], [1.70, 2.90, math.pi], [1.30, 2.90, math.pi]]
    K = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    ds = int(sys.argv[3]) if len(sys.argv) > 3 else 1
    ok = tot = 0
    for w in sorted(f[:-5] for f in os.listdir(os.path.join(TEST, 'truth'))):
        d = os.path.join(OUT, 'run_%s_%s' % (w, pitch), 'out', 'img')
        if not os.path.exists(d + '_meta.json'):
            continue
        m = json.load(open(d + '_meta.json'))
        img = open('%s_%d.raw' % (d, K), 'rb').read()
        W, H = m['w'], m['h']
        if ds > 1:                      # average down like the firmware grid
            W2, H2 = W // ds, H // ds
            nb = bytearray(4 * W2 * H2)
            for v in range(H2):
                for u in range(W2):
                    for c in range(3):
                        s = sum(img[4 * ((v * ds + j) * W + u * ds + i) + c] for i in range(ds) for j in range(ds))
                        nb[4 * (v * W2 + u) + c] = s // (ds * ds)
            img, W, H = bytes(nb), W2, H2
        truth = {int(k): v for k, v in json.load(open(os.path.join(TEST, 'truth', w + '.json')))['board'].items()}
        got, votes, blobs = detect(img, W, H, poses[K], float(pitch), True)
        tot += 1
        good = got == truth
        ok += good
        if not good:
            print(w, 'truth', sorted(truth.items()), 'got', sorted(got.items()), 'votes', votes)
    print('pitch', pitch, 'pose', K, 'ds', ds, ': %d / %d exact' % (ok, tot))

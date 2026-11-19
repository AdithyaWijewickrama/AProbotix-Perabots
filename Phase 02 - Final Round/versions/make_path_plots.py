"""
make_path_plots.py - draws each version's path on the official arena for the versions website.

    python make_path_plots.py      -> versions/<folder>/images/path.png for every version that has a path CSV

The path CSV is the neutral judge's judge_path.csv (t, x, y every 0.1 s; judge/judge.py writes it).
Each version keeps its own copy at versions/<folder>/path/judge_path.csv, the section colours use the judge's own
lines (y > 1.30 m, y > 2.46 m) and the title uses the END time and collisions from spec.yaml.
The latest version uses test/results/runs/w00_official_python/judge_path.csv (same arena, latest code).
Needs: pip install pyyaml matplotlib
"""
import csv
import glob
import json
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "test", "tools"))
import gen  # noqa: E402  arena constants (maze origin, cell size, board cells)

TRUTH = json.load(open(os.path.join(ROOT, "test", "truth", "w00_official.json"), encoding="utf-8"))
LATEST_CSV = os.path.join(ROOT, "test", "results", "runs", "w00_official_python", "judge_path.csv")


def wall_rects():
    m = TRUTH["maze"]
    R = [(gen.MX0 + k * gen.CELL, gen.MY0 + (r + 0.5) * gen.CELL, 0.006, 0.131) for k, r in m["v_walls"]]
    R += [(gen.MX0 + (c + 0.5) * gen.CELL, gen.MY0 + k * gen.CELL, 0.131, 0.006) for c, k in m["h_walls"]]
    R += [(x, y, 0.05, 0.05) for x, y in TRUTH["obstacles"]]
    return R


def plot(spec, P, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    sim = spec.get("simulation") or {}
    fig, ax = plt.subplots(figsize=(5.6, 8.2))
    ax.set_aspect("equal")
    ax.set_xlim(-0.03, 2.47)
    ax.set_ylim(-0.03, 3.69)
    ax.add_patch(Rectangle((0, 1.22), 2.44, 1.22, color="#e4e4e4", zorder=0))
    for x, y, hx, hy in wall_rects():
        ax.add_patch(Rectangle((x - hx, y - hy), 2 * hx, 2 * hy, color="#333", zorder=2))
    ax.plot([0, 2.44, 2.44, 0, 0], [0, 0, 3.66, 3.66, 0], color="#333", lw=2)
    for k, v in TRUTH["maze"]["tiles"].items():
        c, r = map(int, k.split(","))
        ax.add_patch(Rectangle((gen.MX0 + c * gen.CELL + 0.01, gen.MY0 + r * gen.CELL + 0.01), 0.23, 0.23,
                               color="#2a5bd7" if v == "BLUE" else "#2bb24c", alpha=0.4, zorder=1))
    for x in (0.50, 0.90, 1.30, 1.70):
        ax.plot([x, x], [2.59, 3.51], color="#aaa", lw=0.8)
    for y in (2.60, 2.90, 3.20, 3.50):
        ax.plot([0.49, 1.71], [y, y], color="#aaa", lw=0.8)
    for k, v in TRUTH["board"].items():
        cx, cy = gen.POS_CELL[int(k)]
        pc = Circle((cx, cy), 0.03) if v == "BLUE" else Rectangle((cx - 0.03, cy - 0.03), 0.06, 0.06)
        pc.set_color("#2a5bd7" if v == "BLUE" else "#2bb24c")
        ax.add_patch(pc)
    ax.add_patch(Rectangle((0.006, 2.925), 0.25, 0.25, color="red", alpha=0.3))
    ax.text(0.13, 3.05, "END", ha="center", va="center", fontsize=7, color="#900")
    # section marks from the path itself, with the judge's own lines (y > 1.30 m, y > 2.46 m)
    s1 = next((p["t"] for p in P if p["y"] > 1.30), None)
    s2 = next((p["t"] for p in P if p["y"] > 2.46), None)
    t_end = P[-1]["t"]
    parts = [("Section 1", 0, s1 or t_end, "#e07020")]
    if s1:
        parts.append(("Section 2 (maze)", s1, s2 or t_end, "#8a2be2"))
    if s2:
        parts.append(("Section 3", s2, t_end, "#0a8a0a"))
    for nm, t0, t1, col in parts:
        seg = [p for p in P if t0 <= p["t"] <= t1]
        ax.plot([p["x"] for p in seg], [p["y"] for p in seg], color=col, lw=1.8, zorder=6, label=nm)
    ax.add_patch(Circle((P[0]["x"], P[0]["y"]), 0.04, fill=False, ec="#000", lw=1.5, zorder=8))
    ax.text(P[0]["x"], P[0]["y"] - 0.08, "start", ha="center", fontsize=7)
    hits = [p for i, p in enumerate(P) if i and p.get("collisions", 0) > P[i - 1].get("collisions", 0)]
    for p in hits:
        ax.plot(p["x"], p["y"], "x", color="red", ms=9, mew=2.5, zorder=9)
    if hits:
        ax.plot([], [], "x", color="red", ms=8, mew=2, label="collision")
    if sim.get("end_s") is None:
        ax.plot(P[-1]["x"], P[-1]["y"], "o", color="red", ms=9, zorder=9, label="stopped here")
    res = "END %.1f s" % sim["end_s"] if sim.get("end_s") is not None else "did not finish"
    ax.set_title("%s - official arena\n%s, %s collisions" % (spec["id"], res, sim.get("collisions", "?")), fontsize=10)
    ax.legend(loc="lower left", fontsize=7, framealpha=0.9)
    ax.set_xticks([])
    ax.set_yticks([])
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    plt.close(fig)


def main():
    specs = sorted(glob.glob(os.path.join(HERE, "v*", "spec.yaml")))
    latest = specs[-1]
    for f in specs:
        spec = yaml.safe_load(open(f, encoding="utf-8"))
        d = os.path.dirname(f)
        src = os.path.join(d, "path", "judge_path.csv")
        if not os.path.exists(src) and f == latest and os.path.exists(LATEST_CSV):
            src = LATEST_CSV
        if not os.path.exists(src):
            print("%-6s no path yet" % spec["id"])
            continue
        P = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(open(src))]
        os.makedirs(os.path.join(d, "images"), exist_ok=True)
        plot(spec, P, os.path.join(d, "images", "path.png"))
        print("%-6s path.png from %s (%d points)" % (spec["id"], os.path.relpath(src, ROOT), len(P)))


if __name__ == "__main__":
    main()

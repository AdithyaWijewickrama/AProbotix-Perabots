"""analyze.py - why the robot's run differs from the best one (Phase C of docs/TEST_PLAN.md).

usage: python test/tools/analyze.py <run> [<run> ...]      run = <world>_<ctrl>, e.g. w01_python
       python test/tools/analyze.py --all                   every run in test/results/runs/

For each run it reads the judge's true path, the truth file and the reference (reference.py) and writes
  test/results/analysis/<run>.json   per-section numbers and the gap split into causes
  test/results/plots/<run>.png       real path over the reference path, stops and turns marked, maze coverage
and test/results/phaseC.md / phaseC.csv for all analysed runs.

Gap causes (seconds, per section; they add up to the gap up to rounding):
  longer path   (real distance - reference distance) / top speed: detours, backtracking, safety margins
  turning       time turning on the spot beyond what the reference needed
  stops         time standing still (replanning, camera / colour looks, gripper) beyond the reference's gripper time
  below top speed  driving time beyond distance / top speed: acceleration, slow approaches, line-following limits
Energy is a rough motor model (rolling + turning + acceleration), useful only to compare runs.
"""
import csv
import glob
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import gen  # noqa: E402
import reference as ref  # noqa: E402

V = ref.V
MASS = 0.58


def load_path(fn):
    rows = list(csv.DictReader(open(fn)))
    return [{k: float(v) for k, v in r.items()} for r in rows]


def classify(p):
    v, w = abs(p["v"]), abs(p["w"])
    if v < 0.01 and w < 0.15:
        return "stopped"
    if v < 0.05 and w >= 0.15:
        return "turning"
    if v >= 0.25:
        return "cruising"
    return "slow"


def section_stats(P, t0, t1):
    seg = [p for p in P if t0 <= p["t"] < t1]
    if len(seg) < 2:
        return None
    dt = seg[1]["t"] - seg[0]["t"]
    cat = {"stopped": 0.0, "turning": 0.0, "cruising": 0.0, "slow": 0.0, "reversing": 0.0}
    dist, energy, turns, stops = 0.0, 0.0, 0, []
    run_k, run_t0, turn_acc = None, 0.0, 0.0
    for a, b in zip(seg, seg[1:]):
        k = classify(a)
        cat[k] += dt
        if a["v"] < -0.01:
            cat["reversing"] += dt
        dist += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        acc = (b["v"] - a["v"]) / dt
        energy += (1.5 * abs(a["v"]) + 0.35 * abs(a["w"]) + MASS * max(0.0, acc * a["v"]) / 0.5) * dt
        if k != run_k:
            if run_k == "stopped" and a["t"] - run_t0 >= 0.3:
                stops.append([round(run_t0, 2), round(a["t"] - run_t0, 2), round(a["x"], 3), round(a["y"], 3)])
            if run_k == "turning" and abs(turn_acc) > math.radians(20):
                turns += 1
            run_k, run_t0, turn_acc = k, a["t"], 0.0
        if k == "turning":
            turn_acc += a["w"] * dt
    return {"time": round(t1 - t0, 2), "distance": round(dist, 3), "turns": turns, "stops": len(stops),
            "stop_list": stops, "energy_J": round(energy, 1), **{k: round(v, 2) for k, v in cat.items()}}


def gap_split(act, r, _unused=0.0):
    """Split real - best into causes; the parts add up to the gap exactly."""
    if not act or not r:
        return None
    gap = act["time"] - r["time"]
    longer = (act["distance"] - r["distance"]) / V
    turning = act["turning"] - r["turn_time"]
    stops = act["stopped"] - r["fixed_time"]
    ref_drive = r["time"] - r["turn_time"] - r["fixed_time"]
    below = (act["cruising"] + act["slow"] - act["distance"] / V) - (ref_drive - r["distance"] / V)
    rest = gap - longer - turning - stops - below
    return {"gap": round(gap, 2), "longer path": round(longer, 2), "turning": round(turning, 2),
            "stops": round(stops, 2), "below top speed": round(below, 2), "other": round(rest, 2)}


def maze_coverage(P, truth, t0, t1):
    vw = {tuple(w) for w in truth["maze"]["v_walls"]}
    hw = {tuple(w) for w in truth["maze"]["h_walls"]}
    deads = [tuple(d) for d in truth["maze"]["dead_ends"]]
    checked, visits, seq = set(), {}, []
    for p in P:
        if not (t0 <= p["t"] <= t1):
            continue
        c = int((p["x"] - gen.MX0) // gen.CELL)
        r = int((p["y"] - gen.MY0) // gen.CELL)
        if not (0 <= c < gen.COLS and 0 <= r < gen.ROWS):
            continue
        if not seq or seq[-1] != (c, r):
            seq.append((c, r))
            visits[(c, r)] = visits.get((c, r), 0) + 1
        if (c, r) in deads:
            checked.add((c, r))
        yaw = p["yaw"]
        h = round(((math.pi / 2 - yaw) % (2 * math.pi)) / (math.pi / 2)) % 4      # 0 +y, 1 +x, 2 -y, 3 -x
        if abs(((math.pi / 2 - yaw) - h * math.pi / 2 + math.pi) % (2 * math.pi) - math.pi) > 0.35:
            continue
        cc, rr = c, r
        for _ in range(2):
            if not gen.is_open(vw, hw, cc, rr, h):
                break
            cc, rr = cc + gen.DIRS[h][0], rr + gen.DIRS[h][1]
            if (cc, rr) in deads:
                checked.add((cc, rr))
    missed = [d for d in deads if d not in checked]
    revisits = sum(v - 1 for v in visits.values())
    return {"dead_ends": len(deads), "checked": len(deads) - len(missed), "missed": missed,
            "cells_entered": len(seq), "cells_unique": len(visits), "revisits": revisits, "never_entered": 36 - len(visits)}


def wall_rects(truth):
    R = [(gen.MX0 + k * gen.CELL, gen.MY0 + (r + 0.5) * gen.CELL, 0.006, 0.131) for k, r in truth["maze"]["v_walls"]]
    R += [(gen.MX0 + (c + 0.5) * gen.CELL, gen.MY0 + k * gen.CELL, 0.131, 0.006) for c, k in truth["maze"]["h_walls"]]
    R += [(x, y, 0.05, 0.05) for x, y in truth["obstacles"]]
    R += [(1.34, 1.33, 1.0, 0.006), (0.095, 1.83, 0.006, 0.506), (2.335, 1.83, 0.006, 0.506), (1.04, 2.33, 0.95, 0.006),
          (1.3925, 1.22, 1.0535, 0.006), (0.0475, 1.22, 0.0535, 0.006), (1.04, 2.44, 1.04, 0.006), (2.39, 2.44, 0.05, 0.006)]
    return R


def clearance(P, truth):
    R = wall_rects(truth)
    worst, hits, in_hit = 9.0, 0, False
    for p in P:
        d = min(math.hypot(max(abs(p["x"] - x) - hx, 0), max(abs(p["y"] - y) - hy, 0)) for x, y, hx, hy in R) - 0.075
        worst = min(worst, d)
        h = d < -0.003
        if h and not in_hit:
            hits += 1
        in_hit = h
    return round(worst * 1000, 1), hits


def s3_result(res, truth):
    clan = truth["clan"]
    pc = res.get("pieces", {})
    mine = pc.get("collect_blue_cyl" if clan == "BLUE" else "collect_green_cube")
    out = {"clan": clan, "winning_cells": truth["winning_cells"], "placed_at": None, "correct": False, "board_disturbed": []}
    if mine:
        x, y = mine["end"][:2]
        for p, (cx, cy) in gen.POS_CELL.items():
            if math.hypot(x - cx, y - cy) < 0.08:
                out["placed_at"] = p
        if out["placed_at"] is not None:
            board = {int(k) for k in truth["board"]}
            ok_cells = truth["winning_cells"] or [q for q in (1, 5, 6, 8, 7, 3, 2, 9, 4) if q not in board]
            out["correct"] = out["placed_at"] in ok_cells
    for nm, d in pc.items():
        if nm.startswith("board_") and math.hypot(d["end"][0] - d["start"][0], d["end"][1] - d["start"][1]) > 0.02:
            out["board_disturbed"].append(nm)
    return out


def robot_counts(log):
    m = re.findall(r"explored\.? blue=(\d+) green=(\d+)", log)
    c = re.findall(r"\[S3\] clues: blue tiles=(\d+) green tiles=(\d+) right turns=(\d+) left turns=(\d+)", log)
    insp = "inspecting the board" in log
    found = {}
    for col, c_, r_ in re.findall(r"tile (BLUE|GREEN) at c(\d) r(\d)", log):
        found["%d,%d" % (int(c_) - 1, int(r_) - 1)] = col
    return {"maze_count": [int(v) for v in m[-1]] if m else None, "clues": [int(v) for v in c[-1]] if c else None,
            "inspected_board": insp, "tiles_found": found}


def plot(run, truth, P, rf, A, res):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    fig, ax = plt.subplots(figsize=(7.2, 10.4))
    ax.set_aspect("equal")
    ax.set_xlim(-0.03, 2.47)
    ax.set_ylim(-0.03, 3.69)
    ax.add_patch(Rectangle((0, 1.22), 2.44, 1.22, color="#d9d9d9", zorder=0))
    for x, y, hx, hy in wall_rects(truth):
        ax.add_patch(Rectangle((x - hx, y - hy), 2 * hx, 2 * hy, color="#333", zorder=2))
    ax.plot([0, 2.44, 2.44, 0, 0], [0, 0, 3.66, 3.66, 0], color="#333", lw=2)
    for k, v in truth["maze"]["tiles"].items():
        c, r = map(int, k.split(","))
        ax.add_patch(Rectangle((gen.MX0 + c * gen.CELL + 0.01, gen.MY0 + r * gen.CELL + 0.01), 0.23, 0.23,
                               color="#2a5bd7" if v == "BLUE" else "#2bb24c", alpha=0.45, zorder=1))
    cov = A.get("s2_coverage")
    if cov:
        for c, r in cov["missed"]:
            ax.add_patch(Rectangle((gen.MX0 + c * gen.CELL + 0.02, gen.MY0 + r * gen.CELL + 0.02), 0.21, 0.21,
                                   fill=False, ec="red", lw=2, ls="--", zorder=3))
    for x in (0.50, 0.90, 1.30, 1.70):
        ax.plot([x, x], [2.59, 3.51], color="#999", lw=0.8)
    for y in (2.60, 2.90, 3.20, 3.50):
        ax.plot([0.49, 1.71], [y, y], color="#999", lw=0.8)
    for p, (cx, cy) in gen.POS_CELL.items():
        ax.text(cx + 0.09, cy + 0.09, str(p), fontsize=7, color="#777")
    for k, v in truth["board"].items():
        cx, cy = gen.POS_CELL[int(k)]
        ax.add_patch((Circle((cx, cy), 0.03) if v == "BLUE" else Rectangle((cx - 0.03, cy - 0.03), 0.06, 0.06))
                     if True else None)
        ax.patches[-1].set_color("#2a5bd7" if v == "BLUE" else "#2bb24c")
    for w in truth["winning_cells"]:
        cx, cy = gen.POS_CELL[w]
        ax.add_patch(Circle((cx, cy), 0.05, fill=False, ec="gold", lw=2, zorder=4))
    ax.add_patch(Rectangle((0.006, 2.925), 0.25, 0.25, color="red", alpha=0.3))
    for key, col in (("s1", "#555"), ("s2", "#555"), ("s3", "#555")):
        if rf.get(key) and rf[key].get("path"):
            q = rf[key]["path"]
            ax.plot([a[0] for a in q], [a[1] for a in q], ls="--", color=col, lw=1.4, zorder=5)
    cols = {"S1": "#e07020", "S2": "#8a2be2", "S3": "#0a8a0a"}
    for nm, (t0, t1) in A["intervals"].items():
        seg = [p for p in P if t0 <= p["t"] <= t1]
        ax.plot([p["x"] for p in seg], [p["y"] for p in seg], color=cols[nm], lw=1.8, zorder=6, label=nm + " real path")
        st = A["sections"].get(nm) or {}
        for t, d, x, y in st.get("stop_list", []):
            ax.add_patch(Circle((x, y), 0.012 + 0.006 * min(d, 6), color="red", alpha=0.45, zorder=7))
    ax.plot([], [], ls="--", color="#555", label="best path (reference)")
    ax.add_patch(Circle((-1, -1), 0.01, color="red", alpha=0.45, label="stop (size = length)"))
    s = A["summary"]
    ax.set_title("%s   real %s   best %s\n%s" % (run, s["real"], s["best"], s["note"]), fontsize=9)
    ax.legend(loc="lower left", fontsize=7, framealpha=0.9)
    ax.set_xticks([])
    ax.set_yticks([])
    os.makedirs(os.path.join(TEST, "results", "plots"), exist_ok=True)
    fig.tight_layout()
    fig.savefig(os.path.join(TEST, "results", "plots", run + ".png"), dpi=90)
    plt.close(fig)


def analyze(run):
    d = os.path.join(TEST, "results", "runs", run)
    world = run.rsplit("_", 1)[0]
    truth = json.load(open(os.path.join(TEST, "truth", world + ".json")))
    rf = ref.reference(world)
    res = json.load(open(os.path.join(d, "judge_result.json")))
    P = load_path(os.path.join(d, "judge_path.csv"))
    log = open(os.path.join(d, "webots_log.txt"), errors="ignore").read()
    if os.path.exists(os.path.join(d, "sil_log.txt")):          # firmware runs log to sil_log.txt
        log += "\n" + open(os.path.join(d, "sil_log.txt"), errors="ignore").read()
    m = res["marks"]
    t_end = m.get("end", res["t"])
    iv = {}
    if truth["start"] == "full":
        iv["S1"] = (0.0, m.get("s1", t_end))
    if truth["start"] in ("full", "maze") and "s1" in m or truth["start"] == "maze":
        iv["S2"] = (m.get("s1", 0.0) if truth["start"] == "full" else 0.0, m.get("s2", t_end))
    if "s2" in m or truth["start"] == "s3":
        iv["S3"] = (m.get("s2", 0.0), t_end)
    A = {"run": run, "world": world, "start": truth["start"], "marks": m, "reason": res["reason"], "intervals": iv,
         "sections": {}, "gaps": {}}
    for nm, (t0, t1) in iv.items():
        A["sections"][nm] = section_stats(P, t0, t1)
        fixed = 2.0 if nm == "S3" else 0.0
        A["gaps"][nm] = gap_split(A["sections"][nm], rf.get(nm.lower()), fixed)
    if "S2" in iv:
        A["s2_coverage"] = maze_coverage(P, truth, *iv["S2"])
    A["clearance_mm"], A["wall_overlaps"] = clearance(P, truth)
    A["judge_collisions"] = len(res["collisions"])
    A["s3"] = s3_result(res, truth)
    A["robot"] = robot_counts(log)
    tc = truth["clues"]
    A["maze_count_ok"] = A["robot"]["maze_count"] == [tc["blue"], tc["green"]] if A["robot"]["maze_count"] else None
    tt, tf = truth["maze"]["tiles"], A["robot"]["tiles_found"]
    A["tiles"] = {"true": len(tt), "found_right": sum(1 for k, v in tf.items() if tt.get(k) == v),
                  "false_or_wrong_colour": [k for k, v in tf.items() if tt.get(k) != v], "missed": [k for k in tt if k not in tf]}
    moved = A["s3"]["board_disturbed"]
    A["collisions"] = A["wall_overlaps"] + len(moved)
    total_real = round(t_end - (0 if truth["start"] == "full" else 0), 2)
    best = sum(rf[k]["time"] for k in ("s1", "s2", "s3") if rf.get(k))
    note = []
    if "end" not in m:
        note.append("DID NOT FINISH (%s)" % res["reason"])
    if A["maze_count_ok"] is False:
        note.append("WRONG TILE COUNT")
    if A["collisions"]:
        note.append("COLLISION")
    if not A["s3"]["correct"]:
        note.append("piece not on a correct cell")
    A["summary"] = {"real": "%.1f s" % total_real if "end" in m else "-", "best": "%.1f s" % best,
                    "note": ", ".join(note) or "mission complete, piece correct"}
    os.makedirs(os.path.join(TEST, "results", "analysis"), exist_ok=True)
    json.dump(A, open(os.path.join(TEST, "results", "analysis", run + ".json"), "w"), indent=1)
    plot(run, truth, P, rf, A, res)
    return A, rf


def table(rows):
    H = ["run", "start", "result", "S1 real/best s", "S2 real/best s", "S3 real/best s", "END", "tile count", "tiles found",
         "piece on", "collisions", "contact flags", "min clearance mm"]
    out = ["| " + " | ".join(H) + " |", "|" + "---|" * len(H)]
    csvrows = []
    for A, rf in rows:
        s = A["sections"]

        def rb(k):
            if not s.get(k):
                return "-"
            r = rf.get(k.lower())
            return "%.1f / %.1f" % (s[k]["time"], r["time"]) if r else "%.1f" % s[k]["time"]
        cov = A.get("s2_coverage")
        tl = A["tiles"]
        row = [A["run"], A["start"], A["summary"]["note"], rb("S1"), rb("S2"), rb("S3"), A["summary"]["real"],
               {True: "right", False: "WRONG", None: "-"}[A["maze_count_ok"]],
               ("%d/%d" % (tl["found_right"], tl["true"]) + (" +%d false" % len(tl["false_or_wrong_colour"]) if tl["false_or_wrong_colour"] else "")) if cov else "-",
               ("%s (%s)" % (A["s3"]["placed_at"], "right" if A["s3"]["correct"] else "WRONG")) if A["s3"]["placed_at"] else "not placed",
               A["collisions"], A["judge_collisions"], A["clearance_mm"]]
        out.append("| " + " | ".join(str(v) for v in row) + " |")
        csvrows.append(row)
    return "\n".join(out), H, csvrows


def causes(rows):
    agg = {}
    for A, _ in rows:
        if "end" not in A["marks"]:
            continue                                         # unfinished runs would swamp the averages
        for sec, g in A["gaps"].items():
            if not g:
                continue
            a = agg.setdefault(sec, {})
            for k, v in g.items():
                a[k] = a.get(k, 0) + v
            a["n"] = a.get("n", 0) + 1
    lines = ["| section | runs | avg gap s | longer path | turning | stops | below top speed | other |", "|---|---|---|---|---|---|---|---|"]
    for sec in ("S1", "S2", "S3"):
        a = agg.get(sec)
        if not a:
            continue
        n = a["n"]
        lines.append("| %s | %d | %.1f | %.1f | %.1f | %.1f | %.1f | %.1f |" % (
            sec, n, a["gap"] / n, a["longer path"] / n, a["turning"] / n, a["stops"] / n, a["below top speed"] / n, a["other"] / n))
    return "\n".join(lines)


if __name__ == "__main__":
    runs = sys.argv[1:]
    if runs == ["--all"]:
        runs = sorted(os.path.basename(p) for p in glob.glob(os.path.join(TEST, "results", "runs", "*")) if
                      os.path.exists(os.path.join(p, "judge_result.json")))
    rows = []
    for r in runs:
        A, rf = analyze(r)
        rows.append((A, rf))
        print(r, A["summary"], {k: v for k, v in A["gaps"].items()})
    tab, H, csvrows = table(rows)
    md = ("# Phase C - real runs vs the best possible run\n\n" + tab +
          "\n\n## Where the extra time goes (average seconds per section, finished runs only)\n\n" + causes(rows) + "\n")
    open(os.path.join(TEST, "results", "phaseC.md"), "w").write(md)
    w = csv.writer(open(os.path.join(TEST, "results", "phaseC.csv"), "w", newline=""))
    w.writerow(H)
    w.writerows(csvrows)

"""compare.py - before/after table for a fix (Phase F of docs/TEST_PLAN.md).

usage: python test/tools/compare.py <before analysis dir> [<after analysis dir>] [--md out.md]
       default after = test/results/analysis

For every run in both: finished?, END time, tile count, piece, collisions. Then the verdict the plan asks for:
keep a change only if it is faster or equal on most worlds and breaks nothing.
"""
import glob
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TEST = os.path.dirname(HERE)


def load(d):
    out = {}
    for f in glob.glob(os.path.join(d, "*.json")):
        a = json.load(open(f))
        out[a["run"]] = a
    return out


def row(a):
    if a is None:
        return None
    fin = "end" in a["marks"]
    return {"fin": fin, "end": a["marks"].get("end"), "tiles": a.get("maze_count_ok"),
            "piece": a["s3"]["correct"], "coll": a.get("collisions", 0)}


def ok(r):
    return r and r["fin"] and r["tiles"] is not False and r["piece"] and not r["coll"]


def main():
    argv = sys.argv[1:]
    md = None
    if "--md" in argv:
        i = argv.index("--md")
        md = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    args = argv
    before = load(args[0])
    after = load(args[1] if len(args) > 1 else os.path.join(TEST, "results", "analysis"))
    lines = ["| run | before | after | change |", "|---|---|---|---|"]
    faster = slower = same = fixed = broke = new = 0
    for run in sorted(set(before) | set(after)):
        b, a = row(before.get(run)), row(after.get(run))
        fmt = lambda r: "-" if r is None else ("%.1f s" % r["end"] if r["fin"] else "DID NOT FINISH") + \
            ("" if ok(r) or not r["fin"] else " (" + ", ".join(x for x, bad in (("tiles", r["tiles"] is False),
                                                                                 ("piece", not r["piece"]), ("collision", r["coll"])) if bad) + ")")
        ch = ""
        if b is None:
            ch, new = "new world", new + 1
        elif a is None:
            ch = "not run"
        elif ok(a) and not ok(b):
            ch, fixed = "**FIXED**", fixed + 1
        elif ok(b) and not ok(a):
            ch, broke = "**BROKEN**", broke + 1
        elif a["fin"] and b["fin"]:
            d = a["end"] - b["end"]
            ch = "%+.1f s" % d
            if d < -0.5:
                faster += 1
            elif d > 0.5:
                slower += 1
            else:
                same += 1
        lines.append("| %s | %s | %s | %s |" % (run, fmt(b), fmt(a), ch))
    good_after = sum(1 for r in after.values() if ok(row(r)))
    summary = ("**%d fixed, %d broken, %d faster, %d slower, %d same (±0.5 s), %d new.** Runs fully right after the change: %d of %d."
               % (fixed, broke, faster, slower, same, new, good_after, len(after)))
    text = "\n".join(lines) + "\n\n" + summary + "\n"
    print(text)
    if md:
        open(md, "w").write(text)


if __name__ == "__main__":
    main()

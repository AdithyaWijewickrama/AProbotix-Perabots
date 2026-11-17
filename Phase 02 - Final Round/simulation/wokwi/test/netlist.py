"""netlist.py - flattens ../diagram.json into wiring.txt for the co-simulation (cosim.cpp)."""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
d = json.load(open(os.path.join(HERE, "..", "diagram.json")))
parent = {}


def find(x):
    parent.setdefault(x, x)
    while parent[x] != x:
        parent[x] = parent[parent[x]]
        x = parent[x]
    return x


for a, b, *_ in d["connections"]:
    parent[find(a)] = find(b)
# the Mega's GND pins are one net, so are its 5V pins
for p in list(parent):
    if p.startswith("mega:GND"):
        parent[find(p)] = find("mega:GND.1")
groups = {}
for p in parent:
    groups.setdefault(find(p), []).append(p)
out = []
for part in d["parts"]:
    out.append("part %s %s %s" % (part["id"], part["type"], " ".join("%s=%s" % kv for kv in part["attrs"].items())))
for k, (root, members) in enumerate(sorted(groups.items())):
    out.append("net n%d %s" % (k, " ".join(sorted(members))))
open(sys.argv[1], "w").write("\n".join(out) + "\n")
print("%d parts, %d nets" % (len(d["parts"]), len(groups)))

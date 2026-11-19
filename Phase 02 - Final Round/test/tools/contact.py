"""contact.py - tile the per-run pictures (test/results/plots/*.png) into overview sheets.
usage: python test/tools/contact.py [pattern] [cols]   -> test/results/plots/overview_<n>.png"""
import glob
import os
import sys
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
P = os.path.join(os.path.dirname(HERE), "results", "plots")
pat = sys.argv[1] if len(sys.argv) > 1 else "w*.png"
cols = int(sys.argv[2]) if len(sys.argv) > 2 else 5
fs = sorted(f for f in glob.glob(os.path.join(P, pat)) if "overview" not in f)
per = cols * 2
for k in range(0, len(fs), per):
    ims = [Image.open(f).convert("RGB") for f in fs[k:k + per]]
    w, h = ims[0].size
    sheet = Image.new("RGB", (cols * w, ((len(ims) + cols - 1) // cols) * h), "white")
    for i, im in enumerate(ims):
        sheet.paste(im.resize((w, h)), ((i % cols) * w, (i // cols) * h))
    out = os.path.join(P, "overview_%d.png" % (k // per + 1))
    sheet.save(out)
    print(out, sheet.size)

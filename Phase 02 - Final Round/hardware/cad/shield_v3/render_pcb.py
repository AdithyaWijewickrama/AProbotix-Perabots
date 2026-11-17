"""
render_pcb.py - top-view picture of a KiCad board (copper both sides + silkscreen + outline).
    python3 render_pcb.py board.kicad_pcb out.png
Needs: KiCad 7 python (pcbnew), rsvg-convert and ImageMagick (convert).
"""
import os
import subprocess
import sys
import tempfile

import pcbnew


def render(pcb_path, out_png, dpi=300):
    board = pcbnew.LoadBoard(pcb_path)
    tmp = tempfile.mkdtemp()
    pc = pcbnew.PLOT_CONTROLLER(board)
    po = pc.GetPlotOptions()
    po.SetOutputDirectory(tmp)
    po.SetPlotFrameRef(False)
    po.SetBlackAndWhite(True)
    po.SetDrillMarksType(pcbnew.DRILL_MARKS_FULL_DRILL_SHAPE)
    po.SetMirror(False)
    po.SetScale(1.0)
    pngs = {}
    for name, lid in (("bcu", pcbnew.B_Cu), ("fcu", pcbnew.F_Cu), ("silk", pcbnew.F_SilkS), ("edge", pcbnew.Edge_Cuts)):
        pc.SetLayer(lid)
        pc.OpenPlotfile(name, pcbnew.PLOT_FORMAT_SVG, name)
        pc.PlotLayer()
        pc.ClosePlot()
        svg = pc.GetPlotFileName()
        png = os.path.join(tmp, name + ".png")
        subprocess.run(["rsvg-convert", "-d", str(dpi), "-p", str(dpi), "-b", "white", svg, "-o", png], check=True)
        pngs[name] = png
    # every layer was plotted on the same page, so the images line up; turn each into a coloured mask
    col = {"bcu": "#4a7fd0", "fcu": "#e0a030", "silk": "#f4f4f4", "edge": "#202020"}
    layers = []
    for name in ("bcu", "fcu", "silk", "edge"):
        m = os.path.join(tmp, name + "_c.png")
        subprocess.run(["convert", pngs[name], "-colorspace", "gray", "-negate", "-threshold", "40%",
                        "(", "+clone", "-fill", col[name], "-colorize", "100", ")", "+swap",
                        "-alpha", "off", "-compose", "CopyOpacity", "-composite", m], check=True)
        layers.append(m)
    base = os.path.join(tmp, "base.png")
    # board area = flood fill inside the outline, from the board centre (SVG page is 1:1 in mm)
    bb = board.GetBoardEdgesBoundingBox()
    cx = int(pcbnew.ToMM(bb.GetCenter().x) * dpi / 25.4)
    cy = int(pcbnew.ToMM(bb.GetCenter().y) * dpi / 25.4)
    subprocess.run(["convert", pngs["edge"], "-fill", "#0f4d2a", "-fuzz", "20%", "-draw",
                    "color %d,%d floodfill" % (cx, cy), base], check=True)
    cmd = ["convert", base] + sum([[l, "-compose", "over", "-composite"] for l in layers], []) + \
          ["-trim", "+repage", "-bordercolor", "white", "-border", "20", out_png]
    subprocess.run(cmd, check=True)
    print("wrote", out_png)


if __name__ == "__main__":
    render(sys.argv[1], sys.argv[2])

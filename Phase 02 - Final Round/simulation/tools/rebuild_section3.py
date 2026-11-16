"""Rebuild Section 3 of Arena.proto from rulebook Figure 1 (exact dimensions)."""
import re, sys
p = sys.argv[1]
t = open(p).read()
BLK = "PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 metalness 0 }"
def flat(name, cx, cy, sx, sy, app=BLK, z=0.0015):
    return ('      Solid { translation %.4f %.4f %.4f children [ Shape { appearance %s geometry Box { size %.4f %.4f 0.001 } } ] name "%s" }'
            % (cx, cy, z, app, sx, sy, name))
L = []
L.append("""      # ========================================================================
      # SECTION 3: THE NINE-POINT BOARD  (rebuilt from rulebook Figure 1)
      # World frame: x = 0 (END side) .. 2.44 (START side), y = 2.44 .. 3.66
      # Board 122 cm (x 0.49-1.71) x 92 cm (y 2.59-3.51), 2 cm lines.
      # Robot enters the board heading -x at (1.70, 3.05).  Cells (positions):
      #   entrance row x=1.50 :  6 (y 2.75)  1 (y 3.05)  8 (y 3.35)
      #   middle row   x=1.10 :  7           5           3
      #   far row      x=0.70 :  2           9           4
      # ========================================================================""")
W = 0.02
L.append(flat("s3_line_exit", 2.21, 2.75, W, 0.62))                 # maze exit -> junction
L.append(flat("s3_line_entry", 1.96, 3.05, 0.52, W))                # junction -> board edge
L.append(flat("s3_line_collect", 2.006, 3.05, W, 0.70))            # green .. blue
L.append(flat("s3_collect_green_pad", 2.006, 2.70, 0.10, 0.10, "PBRAppearance { baseColor 0.05 0.65 0.15 roughness 1 metalness 0 }", 0.0018))
L.append(flat("s3_collect_blue_pad", 2.006, 3.40, 0.10, 0.10, "PBRAppearance { baseColor 0.03 0.05 0.55 roughness 1 metalness 0 }", 0.0018))
for i, y in enumerate((2.60, 2.90, 3.20, 3.50)):
    L.append(flat("board_long_%d" % i, 1.10, y, 1.22, W))
for i, x in enumerate((0.50, 0.90, 1.30, 1.70)):
    L.append(flat("board_cross_%d" % i, x, 3.05, W, 0.92))
L.append(flat("s3_line_to_end", 0.375, 3.05, 0.27, W))
L.append(flat("end_zone_marker", 0.131, 3.05, 0.25, 0.25, "PBRAppearance { baseColor 1 0 0 roughness 1 metalness 0 }", 0.001))
# printed "1" in position 1 (as in Figure 8)
L.append(flat("pos1_digit_stem", 1.50, 3.05, 0.008, 0.06))
L.append(flat("pos1_digit_flag", 1.517, 3.043, 0.014, 0.006))
new = "\n".join(L) + "\n    ]\n  }\n}\n"
a = t.index("      # ========================================================================\n      # SECTION 3")
t = t[:a] + new
open(p, "w", newline="\n").write(t)
print("section 3 rebuilt")

"""
Trace the shortest path through the official rotated maze and count turns.
"""
from collections import deque

# In the rotated frame:
# Maze grid: 4 cols (c in [0, 3]), 9 rows (r in [0, 8]).
# Row 0 is at top (Y=2.22), Row 8 is at bottom (Y=0.22).
# Entrance is at (r=0, c=0).
# Exit is at (r=8, c=3).

# Let's map the walls from Arena.proto under the transformation:
# For Arena.proto:
# x in [0.095, 2.345], y in [1.33, 2.33]
# cell in Arena.proto:
# col_proto = int(round((x - 0.22) / 0.25)) -> 0 to 8
# row_proto = int(round((y - 1.455) / 0.25)) -> 0 to 3
# With transformation:
# r_new = 8 - col_proto
# c_new = row_proto

# Let's verify with the wall segments we plotted in opt1:
# opt1 had:
# Horizontal walls in new frame (blocking movement between r and r+1):
H_walls = set([
    (0, 0), (0, 1),
    (0, 2),
    (1, 1),
    (2, 1), (2, 2),
    (3, 0),
    (3, 2),
    (4, 2),
    (5, 1), (5, 2), (5, 3),
    (6, 1),
    (7, 0), # wait, bottom left green tile at (8, 0) - is it entered from (7, 0) or from (8, 1)?
])

# Let's find neighbors:
def get_nbrs(r, c):
    nbrs = []
    # Up (-r)
    if r > 0 and (r-1, c) not in H_walls:
        nbrs.append((r-1, c))
    # Down (+r)
    if r < 8 and (r, c) not in H_walls:
        nbrs.append((r+1, c))
    # Left (-c)
    if c > 0 and (r, c-1) not in V_walls:
        nbrs.append((r, c-1))
    # Right (+c)
    if c < 3 and (r, c) not in V_walls:
        nbrs.append((r, c+1))
    return nbrs

# Let's verify by inspecting the exact walls from Arena.proto!
with open(r'D:\PeraBots\PeraBots_UC\protos\Arena.proto', 'r') as f:
    text = f.read()

import re
# Find all maze_wall
matches = re.findall(r'name "(maze_wall_[vh]_[^"]+)"', text)
print(f"Total maze walls in proto: {len(matches)}")
for m in matches:
    print(m)

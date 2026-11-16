"""
Exact topology of the PeraBots 9x4 maze directly from Arena.proto wall definitions.
In Arena.proto:
9 columns: c1 to c9 (c = 1 to 9, index 0 to 8)
4 rows: r1 to r4 (r = 1 to 4, index 0 to 3)
Entrance: chamber at c1, r1
Exit: end_marker at c9, r4
"""

# Walls in Arena.proto frame:
# V walls: between (c, c+1) at row r
v_walls = set()
# H walls: between (r, r+1) at col c
h_walls = set()

# Parse the 18 walls directly:
# maze_wall_v_c1c2_r1r2
v_walls.add((0, 0)) # between c1 and c2 at r1
v_walls.add((0, 1)) # between c1 and c2 at r2
# maze_wall_v_c2c3_r2
v_walls.add((1, 1))
# maze_wall_v_c3c4_r4
v_walls.add((2, 3))
# maze_wall_v_c4c5_r3
v_walls.add((3, 2))
# maze_wall_v_c5c6_r1
v_walls.add((4, 0))
# maze_wall_v_c6c7_r4
v_walls.add((5, 3))
# maze_wall_v_c7c8_r3
v_walls.add((6, 2))
# maze_wall_v_c8c9_r2
v_walls.add((7, 1))
# maze_wall_v_c8c9_r1
v_walls.add((7, 0))
# maze_wall_v_c8c9_r4
v_walls.add((7, 3))

# H walls: between (r, r+1) at col c (r=0 is r1, r=1 is r2, r=2 is r3, r=3 is r4)
# maze_wall_h_r4r3_c2c3 -> between r3 and r4 (index 2) at c2, c3 (index 1, 2)
h_walls.add((2, 1))
h_walls.add((2, 2))
# maze_wall_h_r4r3_c6 -> between r3 and r4 (index 2) at c6 (index 5)
h_walls.add((2, 5))
# maze_wall_h_r4r3_c8 -> between r3 and r4 (index 2) at c8 (index 7)
h_walls.add((2, 7))

# maze_wall_h_r3r2_c4c5 -> between r2 and r3 (index 1) at c4, c5 (index 3, 4)
h_walls.add((1, 3))
h_walls.add((1, 4))
# maze_wall_h_r3r2_c7 -> between r2 and r3 (index 1) at c7 (index 6)
h_walls.add((1, 6))

# maze_wall_h_r2r1_c2 -> between r1 and r2 (index 0) at c2 (index 1)
h_walls.add((0, 1))
# maze_wall_h_r2r1_c4c5 -> between r1 and r2 (index 0) at c4, c5 (index 3, 4)
h_walls.add((0, 3))
h_walls.add((0, 4))
# maze_wall_h_r2r1_c7c8 -> between r1 and r2 (index 0) at c7, c8 (index 6, 7)
h_walls.add((0, 6))
h_walls.add((0, 7))

def neighbors(c, r):
    nbrs = []
    # Left (c-1)
    if c > 0 and (c-1, r) not in v_walls:
        nbrs.append((c-1, r))
    # Right (c+1)
    if c < 8 and (c, r) not in v_walls:
        nbrs.append((c+1, r))
    # Down (r-1)
    if r > 0 and (r-1, c) not in h_walls:
        nbrs.append((c, r-1))
    # Up (r+1)
    if r < 3 and (r, c) not in h_walls:
        nbrs.append((c, r+1))
    return nbrs

# BFS from entrance (c=0, r=0) to exit (c=8, r=3)
from collections import deque
q = deque([((0, 0), [(0, 0)])])
vis = set([(0, 0)])
shortest = None

while q:
    curr, path = q.popleft()
    if curr == (8, 3):
        shortest = path
        break
    for nbr in neighbors(*curr):
        if nbr not in vis:
            vis.add(nbr)
            q.append((nbr, path + [nbr]))

print("Shortest path in Arena.proto frame (c, r):")
print(shortest)
print("Path length:", len(shortest))

# Now let's transform to the Final Round orientation (Option A):
# c_rot = r_proto
# r_rot = 8 - c_proto
print("\nIn Final Round rotated frame (col in 0..3, row in 0..8):")
rot_path = [(r, 8 - c) for (c, r) in shortest]
print(rot_path)

# Find dead ends:
print("\nDead ends in Arena.proto frame:")
for c in range(9):
    for r in range(4):
        nbrs = neighbors(c, r)
        if len(nbrs) == 1 and (c, r) not in [(0, 0), (8, 3)]:
            print(f"c{c+1} r{r+1} -> rotated: col={r}, row={8-c}")

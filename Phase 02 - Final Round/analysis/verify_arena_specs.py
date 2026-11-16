"""
Verify and build complete specifications for the PeraBots 2026 Final Round Arena.
"""

# Arena dimensions
ARENA_W = 3.66  # along X
ARENA_H = 2.44  # along Y
SEC_W = 1.22    # section width along X

# Section bounds
# Section 1: [0.00, 1.22] x [0.00, 2.44]
# Section 2: [1.22, 2.44] x [0.00, 2.44]
# Section 3: [2.44, 3.66] x [0.00, 2.44]

# Section 2 (Maze) parameters:
# 4 columns along X, 9 rows along Y
MAZE_X0 = 1.33
MAZE_Y0 = 0.095
CELL = 0.25

def maze_cell_center(col, row):
    # col: 0 to 3 (left to right)
    # row: 0 to 8 (top to bottom: row 0 is top at Y=2.345-0.125, row 8 is bottom at Y=0.095+0.125)
    cx = MAZE_X0 + (col + 0.5) * CELL
    cy = (MAZE_Y0 + 9 * CELL) - (row + 0.5) * CELL
    return round(cx, 4), round(cy, 4)

print("Maze cell centers:")
for r in range(9):
    row_str = f"Row {r}: "
    for c in range(4):
        cx, cy = maze_cell_center(c, r)
        row_str += f"({cx:.3f},{cy:.3f}) "
    print(row_str)

# Verify colored tile positions:
# Tile 1: Blue at row 1, col 2
print("Tile 1 (Blue, r1c2):", maze_cell_center(2, 1))
# Tile 2: Blue at row 3, col 0
print("Tile 2 (Blue, r3c0):", maze_cell_center(0, 3))
# Tile 3: Green at row 4, col 2
print("Tile 3 (Green, r4c2):", maze_cell_center(2, 4))
# Tile 4: Blue at row 6, col 3
print("Tile 4 (Blue, r6c3):", maze_cell_center(3, 6))
# Tile 5: Green at row 8, col 0
print("Tile 5 (Green, r8c0):", maze_cell_center(0, 8))

"""
Test Phase 3 logic:
1. Clue computation from maze map
2. Magic square mapping
3. Winning move detection
"""

# Magic Square layout:
# Row 2 (top):    [2, 9, 4]
# Row 1 (middle): [7, 5, 3]
# Row 0 (bottom): [6, 1, 8]
# Cols: 0 (left), 1 (center), 2 (right)

MAGIC_BOARD = [
    [6, 1, 8],  # Row 0 (entrance side)
    [7, 5, 3],  # Row 1
    [2, 9, 4],  # Row 2
]

POS_TO_CELL = {}
for r in range(3):
    for c in range(3):
        POS_TO_CELL[MAGIC_BOARD[r][c]] = (r, c)

print("Position to (row, col):")
for pos in range(1, 10):
    print(f"Pos {pos} -> {POS_TO_CELL[pos]}")

# Given clues:
# Clue 1: Blue tiles = 3 -> Cylinder 1 at Pos 3
# Clue 2: Green tiles = 2 -> Cube 1 at Pos 2
# Clue 3: Right turns = 3 (or 5) -> Cylinder 2 at Pos 5
# Clue 4: Left turns = 2 (or 4) -> Cube 2 at Pos 4
# Pre-placed pieces:
board_state = {}
board_state[POS_TO_CELL[3]] = 'BLUE'
board_state[POS_TO_CELL[5]] = 'BLUE'
board_state[POS_TO_CELL[2]] = 'GREEN'
board_state[POS_TO_CELL[4]] = 'GREEN'

print("\nBoard State:")
for r in range(2, -1, -1):
    row_str = f"Row {r}: "
    for c in range(3):
        piece = board_state.get((r, c), 'EMPTY')
        row_str += f"{piece:6s} "
    print(row_str)

# Find winning move for BLUE:
# Lines: 3 rows, 3 cols, 2 diagonals
lines = [
    # Rows
    [((0, 0), (0, 1), (0, 2)), "Row 0 (horizontal)"],
    [((1, 0), (1, 1), (1, 2)), "Row 1 (horizontal)"],
    [((2, 0), (2, 1), (2, 2)), "Row 2 (horizontal)"],
    # Cols
    [((0, 0), (1, 0), (2, 0)), "Col 0 (vertical)"],
    [((0, 1), (1, 1), (2, 1)), "Col 1 (vertical)"],
    [((0, 2), (1, 2), (2, 2)), "Col 2 (vertical)"],
    # Diags
    [((0, 0), (1, 1), (2, 2)), "Diag main"],
    [((0, 2), (1, 1), (2, 0)), "Diag anti"],
]

for my_clan in ['BLUE', 'GREEN']:
    print(f"\n--- Searching Winning Move for {my_clan} ---")
    candidates = []
    for line, desc in lines:
        pieces = [board_state.get(cell, 'EMPTY') for cell in line]
        if pieces.count(my_clan) == 2 and pieces.count('EMPTY') == 1:
            empty_cell = line[pieces.index('EMPTY')]
            pos = MAGIC_BOARD[empty_cell[0]][empty_cell[1]]
            is_horiz = "horizontal" in desc
            candidates.append((is_horiz, desc, empty_cell, pos))
            print(f"Potential Win on {desc}: target cell {empty_cell} = Position {pos}")
            
    # Prefer horizontal
    candidates.sort(key=lambda x: not x[0])
    best = candidates[0]
    print(f"Selected Winning Move for {my_clan}: {best[1]} at Cell {best[2]} (Position {best[3]})")

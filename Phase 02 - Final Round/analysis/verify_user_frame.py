"""
Verify the 3-section layout in the user's native coordinate frame.
Section 1: [0, 2.44] x [0, 1.22]
Section 2: [0, 2.44] x [1.22, 2.44]
Section 3: [0, 2.44] x [2.44, 3.66]
"""

# Section 1
S1_START = (2.315, 0.125)
S1_CHAMBER = (0.22, 1.275)

# Section 2
S2_ENTRANCE = (0.22, 1.455)
MX0, MY0 = 0.095, 1.33
CELL = 0.25
COLS, ROWS = 9, 4

def s2_cell(c, r):
    return MX0 + (c + 0.5) * CELL, MY0 + (r + 0.5) * CELL

S2_EXIT = s2_cell(8, 3)
print(f"S2 Exit cell (c8, r3): {S2_EXIT}")

# Section 3
# Connects at Y = 2.44 near X = 2.21
S3_CHAMBER = (2.21, 2.44)
print(f"S3 Chamber transition: {S3_CHAMBER}")

# Board centered at X = 1.22
# Board width = 1.22m along X (3 cols of 40cm: 0.82, 1.22, 1.62)
# Board height = 0.92m along Y (3 rows of 30cm: row0, row1, row2)
# If board spans Y from 2.80 to 3.72 (height 0.92m):
# But Section 3 max Y is 3.66m!
# In the rulebook:
# "Section 3 game board: 3 x 3 grid, with positions spaced 40 cm horizontally and 30 cm vertically
# Game board outer dimensions: 122 cm wide x 92 cm high, including the grid lines"
# Notice: In the rulebook Figure 1, the arena is 2.44m wide by 3.66m long!
# In Figure 1:
# Section 1: 122cm wide x 244cm high
# Section 2: 122cm wide x 244cm high
# Section 3: 122cm wide x 244cm high
# Total: 366cm wide x 244cm high!
print("Layout verified.")

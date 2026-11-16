"""
Solve and verify the maze connectivity.
"""
from collections import deque

# 9 rows, 4 cols
# Check each dead end:
# Blue tiles at: (1, 2), (3, 0), (6, 3)
# Green tiles at: (4, 2), (8, 0)
# Entrance at: (0, 0)
# Exit at: (8, 3)

# Define walls between adjacent cells
# H_walls[(r, c)] = True means wall between (r, c) and (r+1, c)
H_walls = set([
    (0, 0), (0, 1), (0, 2),
    (1, 1),
    (2, 1), (2, 2),
    (3, 0), (3, 2),
    (4, 2),
    (5, 1), (5, 2), (5, 3),
    (6, 1),
])

# V_walls[(r, c)] = True means wall between (r, c) and (r, c+1)
V_walls = set([
    (1, 0), (2, 0), (5, 0), (6, 0), (7, 0),
    (1, 1), (7, 1),
    (1, 2), (2, 2), (4, 2), (6, 2), (7, 2), (8, 2),
])

def neighbors(r, c):
    res = []
    # Up (to r-1)
    if r > 0 and (r-1, c) not in H_walls:
        res.append((r-1, c))
    # Down (to r+1)
    if r < 8 and (r, c) not in H_walls:
        res.append((r+1, c))
    # Left (to c-1)
    if c > 0 and (r, c-1) not in V_walls:
        res.append((r, c-1))
    # Right (to c+1)
    if c < 3 and (r, c) not in V_walls:
        res.append((r, c+1))
    return res

# Check dead ends
dead_ends = []
for r in range(9):
    for c in range(4):
        nbrs = neighbors(r, c)
        if len(nbrs) == 1 and (r, c) != (0, 0) and (r, c) != (8, 3):
            dead_ends.append((r, c))

print("Dead ends found:", dead_ends)
expected = [(1, 2), (3, 0), (4, 2), (6, 3), (8, 0)]
print("Matches expected?", set(dead_ends) == set(expected))

# BFS from (0, 0) to (8, 3)
queue = deque([((0, 0), [(0, 0)])])
visited = set([(0, 0)])
shortest_path = None

while queue:
    curr, path = queue.popleft()
    if curr == (8, 3):
        shortest_path = path
        break
    for nbr in neighbors(*curr):
        if nbr not in visited:
            visited.add(nbr)
            queue.append((nbr, path + [nbr]))

print(f"Path to exit found? {shortest_path is not None}")
print("Shortest path length:", len(shortest_path))
print("Path:", shortest_path)

# Count turns along shortest path
# A turn is a 90 deg change in direction within a cell
# "Counting begins when robot enters labyrinth and continues until it reaches the last exit cell.
# Any turn made inside the last exit cell to align with or pass through the exit door must not be counted."
turns = []
left_turns = 0
right_turns = 0

for i in range(1, len(shortest_path) - 1):
    prev_cell = shortest_path[i-1]
    curr_cell = shortest_path[i]
    next_cell = shortest_path[i+1]
    
    # Vectors
    v1 = (curr_cell[0] - prev_cell[0], curr_cell[1] - prev_cell[1])  # (dr, dc)
    v2 = (next_cell[0] - curr_cell[0], next_cell[1] - curr_cell[1])
    
    # In row/col: dr > 0 is down (+Y in grid, but visually downward), dc > 0 is right
    # Cross product in 2D: v1[0]*v2[1] - v1[1]*v2[0]
    cross = v1[0] * v2[1] - v1[1] * v2[0]
    if cross > 0:
        turns.append((curr_cell, 'RIGHT'))
        right_turns += 1
    elif cross < 0:
        turns.append((curr_cell, 'LEFT'))
        left_turns += 1

print(f"Turns: {turns}")
print(f"Right turns: {right_turns}, Left turns: {left_turns}")

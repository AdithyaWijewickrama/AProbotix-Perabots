import re
from collections import deque

with open('webots_project/protos/Arena.proto', 'r', encoding='utf-8') as f:
    text = f.read()

# Let's extract all wall lines
walls_v = re.findall(r'translation\s+([-\d\.]+)\s+([-\d\.]+)\s+0.05.*?name\s+"(maze_wall_v_[^"]+)"', text)
walls_h = re.findall(r'translation\s+([-\d\.]+)\s+([-\d\.]+)\s+0.05.*?name\s+"(maze_wall_h_[^"]+)"', text)

print(f"Found {len(walls_v)} vertical walls, {len(walls_h)} horizontal walls")

# Build grid edges:
# 9 columns (0..8), 4 rows (0..3)
# Edge between (c, r) and (c+1, r) blocked if vertical wall exists at x = MX0 + (c+1)*CELL
# Edge between (c, r) and (c, r+1) blocked if horizontal wall exists at y = MY0 + (r+1)*CELL

MX0, MY0 = 0.095, 1.33
CELL = 0.25

# Let's check which walls exist
v_blocked = set() # (c, r) blocked to (c+1, r)
for x, y, name in walls_v:
    x, y = float(x), float(y)
    col = round((x - MX0) / CELL)
    # Check y span
    # Each wall has size sx sy sz. In Arena.proto, size is typically 0.012 sy 0.1
    # Let's find size for this wall:
    match = re.search(re.escape(name) + r'.*?size\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)', text)
    if not match:
        # Search backwards
        idx = text.find(name)
        substr = text[max(0, idx-300):idx+100]
        match = re.search(r'size\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)', substr)
    sy = float(match.group(2)) if match else 0.26
    num_cells = round(sy / CELL)
    r_center = (y - MY0) / CELL - 0.5
    # rows covered:
    for r in range(4):
        cell_y = MY0 + (r + 0.5) * CELL
        if abs(cell_y - y) <= sy / 2.0:
            v_blocked.add((col - 1, r))
            print(f"V wall {name}: blocks col {col-1} <-> {col} at row {r}")

h_blocked = set() # (c, r) blocked to (c, r+1)
for x, y, name in walls_h:
    x, y = float(x), float(y)
    row = round((y - MY0) / CELL)
    idx = text.find(name)
    substr = text[max(0, idx-300):idx+100]
    match = re.search(r'size\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)', substr)
    sx = float(match.group(1)) if match else 0.26
    for c in range(9):
        cell_x = MX0 + (c + 0.5) * CELL
        if abs(cell_x - x) <= sx / 2.0:
            h_blocked.add((c, row - 1))
            print(f"H wall {name}: blocks row {row-1} <-> {row} at col {c}")

# Now BFS from (0, 0) to (8, 3)
q = deque([((0, 0), [(0, 0)])])
vis = set([(0, 0)])
shortest = None

while q:
    curr, path = q.popleft()
    if curr == (8, 3):
        shortest = path
        break
    c, r = curr
    # 4 directions: North (0, 1), East (1, 0), South (0, -1), West (-1, 0)
    # North:
    if r < 3 and (c, r) not in h_blocked and (c, r + 1) not in vis:
        vis.add((c, r + 1))
        q.append(((c, r + 1), path + [(c, r + 1)]))
    # East:
    if c < 8 and (c, r) not in v_blocked and (c + 1, r) not in vis:
        vis.add((c + 1, r))
        q.append(((c + 1, r), path + [(c + 1, r)]))
    # South:
    if r > 0 and (c, r - 1) not in h_blocked and (c, r - 1) not in vis:
        vis.add((c, r - 1))
        q.append(((c, r - 1), path + [(c, r - 1)]))
    # West:
    if c > 0 and (c - 1, r) not in v_blocked and (c - 1, r) not in vis:
        vis.add((c - 1, r))
        q.append(((c - 1, r), path + [(c - 1, r)]))

print("\n--- SHORTEST PATH ---")
print("Path length:", len(shortest) if shortest else "None")
print(shortest)

right_turns = 0
left_turns = 0
if shortest:
    for i in range(1, len(shortest) - 1):
        p0 = shortest[i - 1]
        p1 = shortest[i]
        p2 = shortest[i + 1]
        v1 = (p1[0] - p0[0], p1[1] - p0[1])
        v2 = (p2[0] - p1[0], p2[1] - p1[1])
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        # cross = v1_x * v2_y - v1_y * v2_x
        # If cross < 0 -> Clockwise = RIGHT turn
        # If cross > 0 -> Counter-clockwise = LEFT turn
        if cross < 0:
            right_turns += 1
            print(f"Step {i}: {p0} -> {p1} -> {p2} is RIGHT TURN")
        elif cross > 0:
            left_turns += 1
            print(f"Step {i}: {p0} -> {p1} -> {p2} is LEFT TURN")

print(f"\nFinal Result: Right Turns = {right_turns}, Left Turns = {left_turns}")

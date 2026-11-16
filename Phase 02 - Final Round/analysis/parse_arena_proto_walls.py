with open(r'D:\PeraBots\PeraBots_UC\protos\Arena.proto', 'r') as f:
    text = f.read()

import re

# Find all maze walls in Arena.proto
# Wall names: maze_wall_v_... and maze_wall_h_...
# Let's extract their translations and sizes
lines = text.split('\n')
for i, line in enumerate(lines):
    if 'name "maze_wall_' in line:
        name = line.strip().split('"')[1]
        trans_line = lines[i-15:i+5]
        for t in trans_line:
            if 'translation ' in t:
                trans = t.strip().replace('translation ', '')
            if 'size ' in t:
                size = t.strip().replace('size ', '')
        print(f"{name:25s} trans={trans:25s} size={size}")

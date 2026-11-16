import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

with open(r'D:\PeraBots\PeraBots_UC\protos\Obstacles.proto', 'r') as f:
    text = f.read()

translations = re.findall(r'translation\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', text)
xs = [float(t[0]) for t in translations]
ys = [float(t[1]) for t in translations]

# Option A: new_x = old_y, new_y = 2.44 - old_x
# Option B: new_x = 1.22 - old_y, new_y = 2.44 - old_x
# Option C: new_x = old_y, new_y = old_x
# Option D: new_x = 1.22 - old_y, new_y = old_x

fig, axs = plt.subplots(1, 4, figsize=(20, 10))

opts = [
    ("A: y, 2.44-x", [y for y in ys], [2.44 - x for x in xs]),
    ("B: 1.22-y, 2.44-x", [1.22 - y for y in ys], [2.44 - x for x in xs]),
    ("C: y, x", [y for y in ys], [x for x in xs]),
    ("D: 1.22-y, x", [1.22 - y for y in ys], [x for x in xs]),
]

for ax, (title, nx, ny) in zip(axs, opts):
    ax.scatter(nx, ny, color='black', s=50, marker='s')
    ax.set_xlim(0, 1.22)
    ax.set_ylim(0, 2.44)
    ax.set_title(title)

plt.tight_layout()
plt.savefig(r'D:\PeraBots\Final Round\pdf_images\obstacles_options.png')
print("Saved obstacles_options.png")

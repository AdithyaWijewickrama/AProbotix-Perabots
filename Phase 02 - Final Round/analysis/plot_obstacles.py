import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

with open(r'D:\PeraBots\PeraBots_UC\protos\Obstacles.proto', 'r') as f:
    text = f.read()

translations = re.findall(r'translation\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', text)
xs = [float(t[0]) for t in translations]
ys = [float(t[1]) for t in translations]

plt.figure(figsize=(6, 12))
plt.scatter(xs, ys, color='black', s=100, marker='s')
plt.xlim(0, 2.44)
plt.ylim(0, 1.22)
plt.title(f"Obstacles from Obstacles.proto ({len(xs)} cubes)")
plt.savefig(r'D:\PeraBots\Final Round\pdf_images\obstacles_plot.png')
print(f"Plotted {len(xs)} obstacles")

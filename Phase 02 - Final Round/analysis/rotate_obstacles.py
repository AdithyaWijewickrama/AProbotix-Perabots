import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

with open(r'D:\PeraBots\PeraBots_UC\protos\Obstacles.proto', 'r') as f:
    text = f.read()

translations = re.findall(r'translation\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)', text)
xs = [float(t[0]) for t in translations]
ys = [float(t[1]) for t in translations]

# In crop_sec1.png:
# START is at bottom-left (X=0, Y=0).
# In Obstacles.proto, START was at (2.315, 0.125).
# So in Obstacles.proto, the robot started at X=2.44, Y=0.
# If we map:
# new_y = 2.44 - old_x
# new_x = old_y
# Let's check where START (2.315, 0.125) goes:
# new_y = 2.44 - 2.315 = 0.125 (bottom!)
# new_x = 0.125 (left!)
# EXACTLY BOTTOM-LEFT!

new_xs = [y for y in ys]
new_ys = [2.44 - x for x in xs]

plt.figure(figsize=(6, 12))
plt.scatter(new_xs, new_ys, color='black', s=100, marker='s')
plt.xlim(0, 1.22)
plt.ylim(0, 2.44)
plt.title("Rotated Obstacles (new_x = old_y, new_y = 2.44 - old_x)")
plt.savefig(r'D:\PeraBots\Final Round\pdf_images\obstacles_rotated.png')
print("Saved obstacles_rotated.png")

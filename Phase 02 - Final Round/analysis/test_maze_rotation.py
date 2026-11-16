import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

walls = [
    # v walls (running along Y in Arena.proto)
    (0.345, 1.58, 0.012, 0.512),
    (0.595, 1.705, 0.012, 0.262),
    (0.845, 2.205, 0.012, 0.262),
    (1.095, 1.955, 0.012, 0.262),
    (1.345, 1.455, 0.012, 0.262),
    (1.595, 2.205, 0.012, 0.262),
    (1.845, 1.955, 0.012, 0.262),
    (2.095, 1.705, 0.012, 0.262),
    (2.095, 1.46, 0.012, 0.262),
    (2.095, 2.205, 0.012, 0.262),
    # h walls (running along X in Arena.proto)
    (0.595, 2.08, 0.512, 0.012),
    (1.47, 2.08, 0.262, 0.012),
    (1.97, 2.08, 0.262, 0.012),
    (1.095, 1.83, 0.512, 0.012),
    (1.72, 1.83, 0.262, 0.012),
    (0.47, 1.58, 0.262, 0.012),
    (1.095, 1.58, 0.512, 0.012),
    (1.845, 1.58, 0.512, 0.012),
]

# In Arena.proto, the maze floor was [0, 2.44] in X, [1.22, 2.44] in Y.
# Let y_local = y - 1.22, which is in [0, 1.22].
# Let's test the 4 rotation/reflection options to see which matches sec2_exact_crop.png:

fig, axs = plt.subplots(1, 4, figsize=(20, 10))

for ax, mode in zip(axs, ["opt1", "opt2", "opt3", "opt4"]):
    for x, y, sx, sy in walls:
        yl = y - 1.22
        if mode == "opt1": # new_x = yl, new_y = 2.44 - x
            nx, ny = yl, 2.44 - x
            nsx, nsy = sy, sx
        elif mode == "opt2": # new_x = 1.22 - yl, new_y = 2.44 - x
            nx, ny = 1.22 - yl, 2.44 - x
            nsx, nsy = sy, sx
        elif mode == "opt3": # new_x = yl, new_y = x
            nx, ny = yl, x
            nsx, nsy = sy, sx
        elif mode == "opt4": # new_x = 1.22 - yl, new_y = x
            nx, ny = 1.22 - yl, x
            nsx, nsy = sy, sx
            
        if nsx > nsy:
            ax.plot([nx - nsx/2, nx + nsx/2], [ny, ny], 'b-', lw=3)
        else:
            ax.plot([nx, nx], [ny - nsy/2, ny + nsy/2], 'b-', lw=3)
    ax.set_xlim(0, 1.22)
    ax.set_ylim(0, 2.44)
    ax.set_title(mode)

plt.tight_layout()
plt.savefig(r'D:\PeraBots\Final Round\pdf_images\maze_rotated_options.png')
print("Saved maze_rotated_options.png")

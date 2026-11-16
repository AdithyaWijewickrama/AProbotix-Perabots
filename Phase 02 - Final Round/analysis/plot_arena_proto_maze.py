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

# Let's plot in Arena.proto frame: X in [0.095, 2.345], Y in [1.33, 2.33]
plt.figure(figsize=(12, 6))
for x, y, sx, sy in walls:
    plt.plot([x - sx/2, x + sx/2], [y, y], 'b-', lw=3) if sx > sy else plt.plot([x, x], [y - sy/2, y + sy/2], 'b-', lw=3)

# Boundary walls
# s2_wall_bottom: trans=1.34 1.33 size=2.002 0.012
plt.plot([1.34-1.001, 1.34+1.001], [1.33, 1.33], 'k-', lw=4)
# s2_wall_top: trans=1.215 2.33 size=2.25 0.012
plt.plot([1.215-1.125, 1.215+1.125], [2.33, 2.33], 'k-', lw=4)
# s2_wall_left: trans=0.095 1.83 size=0.012 1.012
plt.plot([0.095, 0.095], [1.83-0.506, 1.83+0.506], 'k-', lw=4)
# s2_wall_right: trans=2.335 1.83 size=0.012 1.012
plt.plot([2.335, 2.335], [1.83-0.506, 1.83+0.506], 'k-', lw=4)

plt.xlim(0, 2.44)
plt.ylim(1.22, 2.44)
plt.title("Arena.proto Maze Walls")
plt.savefig(r'D:\PeraBots\Final Round\pdf_images\arena_proto_maze.png')
print("Saved arena_proto_maze.png")

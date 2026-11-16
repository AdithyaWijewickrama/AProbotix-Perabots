from PIL import Image, ImageDraw
import numpy as np

im = Image.open(r'D:\PeraBots\Final Round\pdf_images\sec2_exact_crop.png')
w, h = im.size

# Let's find the inner white boundary of the maze
# Outer white boundary lines:
arr = np.array(im)[:, :, :3]
# White lines have high values: r>200, g>200, b>200
white = np.all(arr > 200, axis=2)

# Find top, bottom, left, right limits of the white maze boundary
ys, xs = np.where(white)
print(f"White pixels x range: {xs.min()} to {xs.max()}, y range: {ys.min()} to {ys.max()}")

# Create grid overlay
grid_im = im.copy()
draw = ImageDraw.Draw(grid_im)

# Let's find the outer border coordinates:
# From visual inspection of sec2_exact_crop:
# Top white wall is near y ~ 54
# Bottom white wall is near y ~ 428
# Left white wall is near x ~ 37
# Right white wall is near x ~ 197

# Let's refine them by finding the actual border lines
x_left = 37
x_right = 197
y_top = 54
y_bottom = 428

col_w = (x_right - x_left) / 4.0
row_h = (y_bottom - y_top) / 9.0

print(f"Col width: {col_w:.2f}, Row height: {row_h:.2f}")

# Draw grid lines in yellow
for c in range(5):
    x = int(x_left + c * col_w)
    draw.line([(x, y_top), (x, y_bottom)], fill=(255, 255, 0), width=1)

for r in range(10):
    y = int(y_top + r * row_h)
    draw.line([(x_left, y), (x_right, y)], fill=(255, 255, 0), width=1)

# Label cells (r, c)
# r from 0 (top) to 8 (bottom), c from 0 (left) to 3 (right)
for r in range(9):
    for c in range(4):
        cx = int(x_left + (c + 0.5) * col_w)
        cy = int(y_top + (r + 0.5) * row_h)
        draw.text((cx - 8, cy - 5), f"{r},{c}", fill=(255, 200, 200))

grid_im.save(r'D:\PeraBots\Final Round\pdf_images\sec2_grid_overlay.png')
print("Saved sec2_grid_overlay.png")

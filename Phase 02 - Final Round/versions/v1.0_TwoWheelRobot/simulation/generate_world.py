"""
Generate the complete PeraBots 2026 Webots world file with accurate dimensions.
All measurements from the official rulebook:
- Arena: 2.44m x 3.66m (3 sections of 2.44m x 1.22m each)
- Maze: 9x4 grid, 25cm cell pitch, 12mm walls, 100mm high
- Game board: 3x3, 40cm horizontal spacing, 30cm vertical spacing
- Obstacles: 10cm cubes
- Robot max: 20cm x 20cm x 20cm

Coordinate system:
- Arena laid out with length along X (3.66m), width along Z (2.44m)
- Section 1 (obstacles): X from -1.83 to -0.61
- Section 2 (maze): X from -0.61 to 0.61  
- Section 3 (board): X from 0.61 to 1.83
- Y is up
"""

import os

# Arena dimensions
ARENA_LENGTH = 3.66  # X
ARENA_WIDTH = 2.44   # Z
SECTION_LENGTH = 1.22

# Maze parameters
MAZE_COLS = 9     # along Z  
MAZE_ROWS = 4     # along X
CELL_PITCH = 0.25
WALL_THICK = 0.012
WALL_HEIGHT = 0.10

# Section centers
SEC1_CX = -SECTION_LENGTH    # -1.22
SEC2_CX = 0.0
SEC3_CX = SECTION_LENGTH     # 1.22

# Maze grid origin (bottom-left cell center)
# Maze is centered in Section 2
# 9 cols along Z: total span = 8 * 0.25 = 2.0m, centered in 2.44m -> start at Z = -1.0
# 4 rows along X: total span = 3 * 0.25 = 0.75m, centered in 1.22m -> start at X = -0.375
MAZE_Z_START = -1.0
MAZE_X_START = -0.375

def cell_center(row, col):
    """Get world coordinates of maze cell center."""
    x = MAZE_X_START + row * CELL_PITCH
    z = MAZE_Z_START + col * CELL_PITCH
    return x, z

def maze_wall_h(x, z, length, name):
    """Horizontal wall (blocks Z movement, extends along X)."""
    return f"""Solid {{
  translation {x:.4f} {WALL_HEIGHT/2:.4f} {z:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.75 0.75 0.75
        roughness 0.5
        metalness 0
      }}
      geometry Box {{
        size {length:.4f} {WALL_HEIGHT:.4f} {WALL_THICK:.4f}
      }}
    }}
  ]
  name "{name}"
  boundingObject Box {{
    size {length:.4f} {WALL_HEIGHT:.4f} {WALL_THICK:.4f}
  }}
}}
"""

def maze_wall_v(x, z, length, name):
    """Vertical wall (blocks X movement, extends along Z)."""
    return f"""Solid {{
  translation {x:.4f} {WALL_HEIGHT/2:.4f} {z:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.75 0.75 0.75
        roughness 0.5
        metalness 0
      }}
      geometry Box {{
        size {WALL_THICK:.4f} {WALL_HEIGHT:.4f} {length:.4f}
      }}
    }}
  ]
  name "{name}"
  boundingObject Box {{
    size {WALL_THICK:.4f} {WALL_HEIGHT:.4f} {length:.4f}
  }}
}}
"""

def obstacle_cube(x, z, name):
    """10cm black cube obstacle."""
    return f"""Solid {{
  translation {x:.4f} 0.05 {z:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.12 0.12 0.12
        roughness 0.7
        metalness 0
      }}
      geometry Box {{
        size 0.10 0.10 0.10
      }}
    }}
  ]
  name "{name}"
  boundingObject Box {{
    size 0.10 0.10 0.10
  }}
}}
"""

def colored_tile(x, z, color_rgb, name):
    """Colored tile on maze floor."""
    r, g, b = color_rgb
    return f"""Solid {{
  translation {x:.4f} 0.004 {z:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {r} {g} {b}
        roughness 0.3
        metalness 0
      }}
      geometry Box {{
        size 0.20 0.003 0.20
      }}
    }}
  ]
  name "{name}"
}}
"""

def blue_cylinder(x, z, name):
    """Blue game piece cylinder (5cm diameter, 5cm height)."""
    return f"""Solid {{
  translation {x:.4f} 0.025 {z:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0 0.15 0.85
        roughness 0.4
        metalness 0
      }}
      geometry Cylinder {{
        height 0.05
        radius 0.025
      }}
    }}
  ]
  name "{name}"
  boundingObject Cylinder {{
    height 0.05
    radius 0.025
  }}
}}
"""

def green_cube(x, z, name):
    """Green game piece cube (5cm x 5cm x 5cm)."""
    return f"""Solid {{
  translation {x:.4f} 0.025 {z:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0 0.55 0.1
        roughness 0.4
        metalness 0
      }}
      geometry Box {{
        size 0.05 0.05 0.05
      }}
    }}
  ]
  name "{name}"
  boundingObject Box {{
    size 0.05 0.05 0.05
  }}
}}
"""

def black_line(x, z, sx, sz, name):
    """Black guide line on the floor."""
    return f"""Solid {{
  translation {x:.4f} 0.003 {z:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0 0 0
        roughness 0.5
        metalness 0
      }}
      geometry Box {{
        size {sx:.4f} 0.002 {sz:.4f}
      }}
    }}
  ]
  name "{name}"
}}
"""

def red_zone(x, z, sx, sz, name):
    """Red zone marker (start/end/transition)."""
    return f"""Solid {{
  translation {x:.4f} 0.002 {z:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.9 0 0
        roughness 0.5
        metalness 0
      }}
      geometry Box {{
        size {sx:.4f} 0.002 {sz:.4f}
      }}
    }}
  ]
  name "{name}"
}}
"""

def boundary_wall(x, z, sx, sz, name):
    """Arena boundary wall (dark)."""
    return f"""Solid {{
  translation {x:.4f} 0.05 {z:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.08 0.08 0.08
        roughness 0.6
        metalness 0
      }}
      geometry Box {{
        size {sx:.4f} 0.10 {sz:.4f}
      }}
    }}
  ]
  name "{name}"
  boundingObject Box {{
    size {sx:.4f} 0.10 {sz:.4f}
  }}
}}
"""

def generate_world():
    lines = []
    
    # Header
    lines.append("""#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"

WorldInfo {
  title "PeraBots 2026 - The Nine Seals of Pera"
  info [
    "PeraBots 2026 Final Round - University Category"
    "Arena: 2.44m x 3.66m with 3 sections"
    "Section 1: Obstacle field (Fallen Stones)"
    "Section 2: 9x4 Maze (Labyrinth of Hidden Signs)"
    "Section 3: 3x3 Nine-Point Board"
  ]
  basicTimeStep 16
  contactProperties [
    ContactProperties {
      material1 "default"
      material2 "default"
      coulombFriction [1.0]
    }
  ]
}
Viewpoint {
  orientation -0.577 0.577 0.577 2.094
  position 0 4.5 0
  follow ""
  followType "None"
}
TexturedBackground {
}
TexturedBackgroundLight {
}
""")

    # ================================================================
    # ARENA FLOOR
    # ================================================================
    # Section 1 floor - white
    lines.append(f"""Solid {{
  translation {SEC1_CX:.4f} 0 0
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.95 0.95 0.95
        roughness 0.8
        metalness 0
      }}
      geometry Box {{
        size {SECTION_LENGTH:.4f} 0.01 {ARENA_WIDTH:.4f}
      }}
    }}
  ]
  name "floor_section1"
  boundingObject Box {{
    size {SECTION_LENGTH:.4f} 0.01 {ARENA_WIDTH:.4f}
  }}
}}
""")

    # Section 2 floor - dark gray (for maze)
    lines.append(f"""Solid {{
  translation {SEC2_CX:.4f} 0 0
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.18 0.18 0.18
        roughness 0.8
        metalness 0
      }}
      geometry Box {{
        size {SECTION_LENGTH:.4f} 0.01 {ARENA_WIDTH:.4f}
      }}
    }}
  ]
  name "floor_section2"
  boundingObject Box {{
    size {SECTION_LENGTH:.4f} 0.01 {ARENA_WIDTH:.4f}
  }}
}}
""")

    # Section 3 floor - white
    lines.append(f"""Solid {{
  translation {SEC3_CX:.4f} 0 0
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.95 0.95 0.95
        roughness 0.8
        metalness 0
      }}
      geometry Box {{
        size {SECTION_LENGTH:.4f} 0.01 {ARENA_WIDTH:.4f}
      }}
    }}
  ]
  name "floor_section3"
  boundingObject Box {{
    size {SECTION_LENGTH:.4f} 0.01 {ARENA_WIDTH:.4f}
  }}
}}
""")

    # ================================================================
    # ARENA BOUNDARY WALLS
    # ================================================================
    hw = ARENA_WIDTH / 2   # 1.22
    hl = ARENA_LENGTH / 2  # 1.83
    wt = 0.02  # wall thickness

    # North wall (Z = +1.22) - full length
    lines.append(boundary_wall(0, hw + wt/2, ARENA_LENGTH + wt, wt, "wall_north"))
    
    # South wall (Z = -1.22) - full length
    lines.append(boundary_wall(0, -(hw + wt/2), ARENA_LENGTH + wt, wt, "wall_south"))
    
    # West wall - with START gap at bottom (Z ~ -1.095)
    # Gap from Z = -1.22 to Z = -0.97 (25cm gap)
    west_top_len = 2.44 - 0.25  # wall above gap
    west_top_z = (-0.97 + hw) / 2  # center of top section
    lines.append(boundary_wall(-hl - wt/2, west_top_z, wt, west_top_len, "wall_west_top"))
    
    # East wall - with exit gap at top (Z ~ +1.095) for END zone
    east_bottom_len = 2.44 - 0.25
    east_bottom_z = (0.97 - hw) / 2
    lines.append(boundary_wall(hl + wt/2, east_bottom_z, wt, east_bottom_len, "wall_east_bottom"))

    # ================================================================
    # SECTION DIVIDERS (with transition gaps)
    # ================================================================
    # Divider S1-S2 at X = -0.61, gap at bottom (Z ~ -1.1)
    div12_top_len = 2.44 - 0.25
    div12_top_z = (-0.97 + hw) / 2
    lines.append(boundary_wall(-SECTION_LENGTH/2, div12_top_z, wt, div12_top_len, "divider_12"))
    
    # Divider S2-S3 at X = 0.61, gap at top (Z ~ +1.1)
    div23_bot_len = 2.44 - 0.25
    div23_bot_z = (0.97 - hw) / 2
    lines.append(boundary_wall(SECTION_LENGTH/2, div23_bot_z, wt, div23_bot_len, "divider_23"))

    # ================================================================
    # START AND END ZONES
    # ================================================================
    lines.append(red_zone(-hl + 0.125, -hw + 0.125, 0.25, 0.25, "start_zone"))
    lines.append(red_zone(hl - 0.125, hw - 0.125, 0.25, 0.25, "end_zone"))
    
    # Transition markers
    lines.append(red_zone(-SECTION_LENGTH/2, -hw + 0.125, 0.10, 0.25, "transition_12"))
    lines.append(red_zone(SECTION_LENGTH/2, hw - 0.125, 0.10, 0.25, "transition_23"))

    # ================================================================
    # SECTION 1: OBSTACLE FIELD
    # Based on the image, obstacles form winding S-curve paths
    # ================================================================
    
    # Define obstacle positions creating a winding path from START (bottom-left)
    # to transition (bottom-right of Section 1)
    # Section 1 X range: -1.83 to -0.61, Z range: -1.22 to 1.22
    
    obstacle_positions = [
        # Bottom path going right then up
        (-1.55, -0.85), (-1.45, -0.85), (-1.35, -0.85),
        (-1.25, -0.75), (-1.25, -0.65),
        
        # Left wall going up
        (-1.70, -0.55), (-1.70, -0.45), (-1.65, -0.45),
        (-1.60, -0.35), (-1.50, -0.35), (-1.50, -0.25),
        
        # Middle S-curve
        (-1.35, -0.15), (-1.25, -0.15), (-1.15, -0.05),
        (-1.05, -0.05), (-0.95, 0.05), (-0.85, 0.05),
        
        # Upper portion - going right
        (-1.65, 0.15), (-1.65, 0.25), (-1.55, 0.25),
        (-1.45, 0.25), (-1.35, 0.35), (-1.25, 0.35),
        
        # Upper S-curve
        (-1.15, 0.45), (-1.05, 0.45), (-0.95, 0.55),
        (-0.85, 0.55),
        
        # Top area curves
        (-0.85, 0.75), (-0.95, 0.75), (-1.05, 0.75), (-1.15, 0.75),
        (-1.25, 0.85), (-1.35, 0.85), (-1.45, 0.85),
        (-1.55, 0.95), (-1.65, 0.95),
        (-1.65, 1.05), (-1.55, 1.05), (-1.45, 1.05),
        (-1.15, 0.95), (-1.05, 0.95),
        (-0.95, 1.05), (-0.85, 1.05),
    ]
    
    for i, (ox, oz) in enumerate(obstacle_positions):
        lines.append(obstacle_cube(ox, oz, f"obstacle_{i+1}"))

    # ================================================================
    # SECTION 2: MAZE (9 columns x 4 rows)
    # Based on the images showing the maze layout
    # Entrance at bottom-left, exit at top-right
    # ================================================================
    
    # Define maze walls using grid edge representation
    # H-walls block movement between rows (placed between col cells)
    # V-walls block movement between columns (placed between row cells)
    
    # Maze boundary
    # Outer walls of maze
    maze_left = MAZE_X_START - CELL_PITCH/2    # -0.500
    maze_right = MAZE_X_START + 3*CELL_PITCH + CELL_PITCH/2  # 0.500
    maze_bottom = MAZE_Z_START - CELL_PITCH/2  # -1.125
    maze_top = MAZE_Z_START + 8*CELL_PITCH + CELL_PITCH/2    # 1.125
    
    maze_x_span = maze_right - maze_left    # 1.0
    maze_z_span = maze_top - maze_bottom    # 2.25
    
    # Bottom outer wall (with entrance gap at row 0, col 0)
    # Entrance gap at X = maze_left (left side)
    entrance_gap_start = maze_bottom
    entrance_gap_end = maze_bottom + CELL_PITCH
    
    # Bottom wall - left portion (before entrance) - not needed since entrance is at bottom-left
    # Bottom wall right portion (after entrance gap)
    lines.append(maze_wall_v(
        maze_left, 
        (entrance_gap_end + maze_top) / 2,
        maze_top - entrance_gap_end,
        "maze_outer_left_top"
    ))
    
    # Right outer wall (with exit gap at top-right)
    exit_gap_start = maze_top - CELL_PITCH
    exit_gap_end = maze_top
    lines.append(maze_wall_v(
        maze_right,
        (maze_bottom + exit_gap_start) / 2,
        exit_gap_start - maze_bottom,
        "maze_outer_right_bottom"
    ))
    
    # Top outer wall - full
    lines.append(maze_wall_h(
        (maze_left + maze_right) / 2,
        maze_top,
        maze_x_span,
        "maze_outer_top"
    ))
    
    # Bottom outer wall - full
    lines.append(maze_wall_h(
        (maze_left + maze_right) / 2,
        maze_bottom,
        maze_x_span,
        "maze_outer_bottom"
    ))
    
    # ---- Internal maze walls ----
    # Based on the images, creating a complex maze with dead ends
    # Grid: rows 0-3 (X), cols 0-8 (Z)
    # Wall between cell (r,c) and cell (r+1,c) = vertical wall at X boundary
    # Wall between cell (r,c) and cell (r,c+1) = horizontal wall at Z boundary
    
    wall_idx = 0
    
    # Define horizontal walls (between columns, blocking Z movement)
    # Format: (row, col_boundary, length_in_cells)
    # col_boundary N means wall between col N-1 and col N
    h_walls = [
        # Various internal horizontal walls creating the maze pattern
        (0, 1, 1), (1, 1, 1),              # between col 0-1
        (3, 1, 1),
        (0, 2, 1), (2, 2, 1), (3, 2, 1),   # between col 1-2
        (1, 3, 1), (2, 3, 1),               # between col 2-3
        (0, 4, 1), (3, 4, 1),               # between col 3-4
        (1, 5, 1), (2, 5, 1),               # between col 4-5
        (0, 5, 1), (3, 5, 1),
        (1, 6, 1), (3, 6, 1),               # between col 5-6
        (0, 7, 1), (2, 7, 1),               # between col 6-7
        (1, 8, 1), (3, 8, 1),               # between col 7-8
    ]
    
    for row, col_boundary, length in h_walls:
        cx, _ = cell_center(row, 0)
        _, cz = cell_center(0, col_boundary)
        wall_z = cz - CELL_PITCH / 2
        wall_idx += 1
        lines.append(maze_wall_h(cx, wall_z, CELL_PITCH * length, f"maze_hw_{wall_idx}"))
    
    # Define vertical walls (between rows, blocking X movement)
    # Format: (row_boundary, col, length_in_cells)
    v_walls = [
        # Various internal vertical walls
        (1, 0, 1), (1, 2, 1),
        (2, 1, 1), (2, 3, 1),
        (3, 0, 1), (3, 2, 1), (3, 4, 1),
        (1, 4, 1), (1, 6, 1),
        (2, 5, 1), (2, 7, 1),
        (3, 6, 1), (3, 8, 1),
        (1, 8, 1),
    ]
    
    for row_boundary, col, length in v_walls:
        _, cz = cell_center(0, col)
        cx, _ = cell_center(row_boundary, 0)
        wall_x = cx - CELL_PITCH / 2
        wall_idx += 1
        lines.append(maze_wall_v(wall_x, cz, CELL_PITCH * length, f"maze_vw_{wall_idx}"))

    # ================================================================
    # COLORED TILES IN MAZE
    # Blue tiles in dead-end cells, Green tiles in dead-end cells
    # Blue > Green means Blue Sentinels, Green > Blue means Green Keepers
    # ================================================================
    
    blue_color = (0, 0.15, 0.85)
    green_color = (0, 0.6, 0.1)
    
    # Blue tiles (3 of them - more blue = Blue Sentinels side)
    blue_tile_cells = [(1, 2), (3, 4), (3, 7)]  # (row, col) in maze grid
    for i, (r, c) in enumerate(blue_tile_cells):
        tx, tz = cell_center(r, c)
        lines.append(colored_tile(tx, tz, blue_color, f"blue_tile_{i+1}"))
    
    # Green tiles (2 of them)
    green_tile_cells = [(0, 6), (2, 3)]
    for i, (r, c) in enumerate(green_tile_cells):
        tx, tz = cell_center(r, c)
        lines.append(colored_tile(tx, tz, green_color, f"green_tile_{i+1}"))

    # ================================================================
    # SECTION 3: NINE-POINT BOARD
    # 3x3 grid, 40cm horizontal, 30cm vertical spacing
    # Board dimensions: 122cm x 92cm
    # Board is centered in Section 3
    # Magic square numbering (position 1 = middle of entrance row):
    #   2 7 6   (top row from robot entrance)
    #   9 5 1   (middle row)
    #   4 3 8   (bottom row)
    # ================================================================
    
    board_cx = SEC3_CX  # 1.22
    board_cz = 0.0
    h_spacing = 0.40
    v_spacing = 0.30
    
    # Grid positions (row 0=top from entrance, col 0=left)
    # In our coord system, robot enters from left (lower X)
    # Top = positive Z, Bottom = negative Z in terms of the board
    grid_positions = {}
    for row in range(3):
        for col in range(3):
            gx = board_cx - h_spacing + col * h_spacing
            gz = board_cz + v_spacing - row * v_spacing
            grid_positions[(row, col)] = (gx, gz)
    
    # Draw grid lines (black, 20mm wide)
    board_w = 0.82   # total width (2 * 40cm + margins)
    board_h = 0.62   # total height (2 * 30cm + margins)
    
    # Horizontal lines (4 lines)
    for i in range(4):
        lz = board_cz + v_spacing + v_spacing/2 - i * v_spacing
        lines.append(black_line(board_cx, lz, board_w + 0.02, 0.02, f"board_hline_{i}"))
    
    # Vertical lines (4 lines)
    for i in range(4):
        lx = board_cx - h_spacing - h_spacing/2 + i * h_spacing
        lines.append(black_line(lx, board_cz, 0.02, board_h + 0.02, f"board_vline_{i}"))
    
    # ================================================================
    # GAME PIECES ON BOARD
    # Based on image analysis - 4 pieces already placed
    # Mapping clue values to positions:
    #   blue_tiles -> cylinder 1 position
    #   green_tiles -> cube 1 position
    #   right_turns -> cylinder 2 position
    #   left_turns -> cube 2 position
    # ================================================================
    
    # Example game state (teams must deduce this from maze clues):
    # Two green cubes and two blue cylinders already placed
    
    # Green cube at position (0, 2) - top-right
    gx, gz = grid_positions[(0, 2)]
    lines.append(green_cube(gx, gz, "board_green_1"))
    
    # Green cube at position (2, 2) - bottom-right  
    gx, gz = grid_positions[(2, 2)]
    lines.append(green_cube(gx, gz, "board_green_2"))
    
    # Blue cylinder at position (1, 1) - center
    bx, bz = grid_positions[(1, 1)]
    lines.append(blue_cylinder(bx, bz, "board_blue_1"))
    
    # Blue cylinder at position (2, 1) - bottom-center
    bx, bz = grid_positions[(2, 1)]
    lines.append(blue_cylinder(bx, bz, "board_blue_2"))

    # ================================================================
    # GAME PIECE COLLECTION POINTS
    # Green cube pickup and blue cylinder pickup near the board
    # ================================================================
    
    # Green cube collection point (top-left of board area)
    lines.append(green_cube(board_cx - h_spacing - 0.15, v_spacing + 0.15, "collect_green"))
    
    # Blue cylinder collection point (bottom-left of board area)
    lines.append(blue_cylinder(board_cx - h_spacing - 0.15, -(v_spacing + 0.15), "collect_blue"))

    # ================================================================
    # NAVIGATION BLACK LINES
    # Connect START -> S1 -> S2 entrance -> maze -> S2 exit -> S3 -> Board -> END
    # ================================================================
    
    # Line from start zone going up
    start_x = -hl + 0.125
    start_z = -hw + 0.125
    lines.append(black_line(start_x, start_z + 0.20, 0.02, 0.40, "line_start_up"))
    
    # Line across bottom of Section 1 to transition
    lines.append(black_line(
        (-hl + 0.125 + (-SECTION_LENGTH/2)) / 2,
        -hw + 0.125,
        abs(-hl + 0.125 - (-SECTION_LENGTH/2)) + 0.02,
        0.02,
        "line_s1_bottom"
    ))
    
    # Line from transition 2-3 area
    lines.append(black_line(
        (SECTION_LENGTH/2 + board_cx - h_spacing - 0.15) / 2,
        hw - 0.125,
        abs(SECTION_LENGTH/2 - (board_cx - h_spacing - 0.15)),
        0.02,
        "line_s23_top"
    ))
    
    # Left side path line along board
    path_x = board_cx - h_spacing - h_spacing/2
    lines.append(black_line(path_x, 0, 0.02, ARENA_WIDTH * 0.75, "line_board_left"))
    
    # Connection lines to collection points
    lines.append(black_line(
        (path_x + board_cx - h_spacing - 0.15) / 2,
        v_spacing + 0.15,
        abs(path_x - (board_cx - h_spacing - 0.15)),
        0.02,
        "line_to_green_collect"
    ))
    
    lines.append(black_line(
        (path_x + board_cx - h_spacing - 0.15) / 2,
        -(v_spacing + 0.15),
        abs(path_x - (board_cx - h_spacing - 0.15)),
        0.02,
        "line_to_blue_collect"
    ))

    # ================================================================
    # ROBOT - E-puck at START position
    # ================================================================
    lines.append(f"""E-puck {{
  translation {start_x:.4f} 0 {start_z:.4f}
  rotation 0 1 0 1.5708
  name "e-puck"
  controller "robot_controller"
  groundSensorsSlot [
    E-puckGroundSensors {{
    }}
  ]
}}
""")

    # Add the ground sensors PROTO reference at the top
    return lines


def main():
    world_lines = generate_world()
    world_content = "\n".join(world_lines)
    
    # Add the ground sensors EXTERNPROTO to the header
    world_content = world_content.replace(
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"',
        'EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puck.proto"\nEXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/robots/gctronic/e-puck/protos/E-puckGroundSensors.proto"'
    )
    
    out_path = r"D:\PeraBots\Final Round\webots_project\worlds\perabots_arena.wbt"
    with open(out_path, 'w') as f:
        f.write(world_content)
    
    print(f"World file written to: {out_path}")
    print(f"Total size: {len(world_content)} bytes")

if __name__ == '__main__':
    main()

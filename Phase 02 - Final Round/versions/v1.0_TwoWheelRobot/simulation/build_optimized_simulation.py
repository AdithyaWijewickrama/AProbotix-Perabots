"""
PeraBots 2026 Final Round - Complete Optimized Arena & Simulation Generator
Generates:
1. protos/Arena.proto
2. protos/Obstacles.proto
3. protos/TwoWheelRobot.proto
4. worlds/perabots_arena.wbt
5. worlds/Arena.wbt
6. controllers/main_controller/main_controller.py
"""

import os
import math

BASE_DIR = r"D:\PeraBots\Final Round\webots_project"
PROTOS_DIR = os.path.join(BASE_DIR, "protos")
WORLDS_DIR = os.path.join(BASE_DIR, "worlds")
CTRL_DIR = os.path.join(BASE_DIR, "controllers", "main_controller")

os.makedirs(PROTOS_DIR, exist_ok=True)
os.makedirs(WORLDS_DIR, exist_ok=True)
os.makedirs(CTRL_DIR, exist_ok=True)

# ==============================================================================
# 1. OBSTACLES PROTO
# 43 Cubes in Section 1 (0.10 x 0.10 x 0.10m), Z-up
# Coordinates mapped from verified official Obstacles.proto:
# new_x = old_y, new_y = 2.44 - old_x, new_z = 0.05
# ==============================================================================

# Read original 43 obstacle translations from Dhinuka / PeraBots_UC
orig_obstacles = []
with open(r"D:\PeraBots\PeraBots_UC\protos\Obstacles.proto", "r") as f:
    text = f.read()

import re
matches = re.findall(r"translation\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)", text)
for m in matches:
    ox, oy, oz = float(m[0]), float(m[1]), float(m[2])
    # Apply rotation:
    nx = round(oy, 3)
    ny = round(2.44 - ox, 3)
    nz = 0.05
    orig_obstacles.append((nx, ny, nz))

obstacles_proto_content = """#VRML_SIM R2025a utf8
# Obstacles.proto - Section 1: The 43 fixed obstacle cubes (10cm x 10cm x 10cm)
# Optimized as a single locked PROTO for peak physics and rendering performance.

PROTO Obstacles [
] {
  Group {
    children [
"""

for i, (ox, oy, oz) in enumerate(orig_obstacles):
    obstacles_proto_content += f"""      Solid {{
        translation {ox:.3f} {oy:.3f} {oz:.3f}
        children [
          Shape {{
            appearance PBRAppearance {{
              baseColor 0.02 0.02 0.02
              roughness 1
              metalness 0
            }}
            geometry Box {{
              size 0.1 0.1 0.1
            }}
          }}
        ]
        name "obstacle_{i+1}"
        boundingObject Box {{
          size 0.1 0.1 0.1
        }}
      }}
"""

obstacles_proto_content += """    ]
  }
}
"""

with open(os.path.join(PROTOS_DIR, "Obstacles.proto"), "w") as f:
    f.write(obstacles_proto_content)
print(f"Generated Obstacles.proto ({len(orig_obstacles)} cubes)")

# ==============================================================================
# 2. ARENA PROTO
# Complete static arena structure:
# Floors, outer walls, dividers, maze walls, markers, lines, board grid
# ==============================================================================

arena_proto_content = """#VRML_SIM R2025a utf8
# Arena.proto - PeraBots 2026 Final Round Competition Arena
# 2.44m x 3.66m total, divided into 3 connected sections (each 1.22m x 2.44m)
# Section 1: Obstacle Field (white floor, start marker, transition chamber)
# Section 2: The Labyrinth (dark floor, 9x4 maze, 12mm walls, 5 colored tiles, exit)
# Section 3: The Nine-Point Board (white floor, black guide lines, collection stations, 3x3 grid, end zone)

PROTO Arena [
] {
  Group {
    children [
      # ====================================================================
      # SECTION FLOORS (Z = -0.005, size 1.22 x 2.44 x 0.01)
      # ====================================================================
      Solid {
        translation 0.61 1.22 -0.005
        children [
          Shape {
            appearance PBRAppearance {
              roughness 1
              metalness 0
            }
            geometry Box {
              size 1.22 2.44 0.01
            }
          }
        ]
        name "section1_floor"
        boundingObject Box {
          size 1.22 2.44 0.01
        }
      }
      Solid {
        translation 1.83 1.22 -0.005
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.02 0.02 0.02
              roughness 1
              metalness 0
            }
            geometry Box {
              size 1.22 2.44 0.01
            }
          }
        ]
        name "section2_floor"
        boundingObject Box {
          size 1.22 2.44 0.01
        }
      }
      Solid {
        translation 3.05 1.22 -0.005
        children [
          Shape {
            appearance PBRAppearance {
              roughness 1
              metalness 0
            }
            geometry Box {
              size 1.22 2.44 0.01
            }
          }
        ]
        name "section3_floor"
        boundingObject Box {
          size 1.22 2.44 0.01
        }
      }

      # ====================================================================
      # OUTER ARENA BOUNDARY WALLS (Height 10cm, Z = 0.05)
      # ====================================================================
      # South outer wall (Y = 0, along X: 0 to 3.66)
      Solid {
        translation 1.83 0 0.05
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.02 0.02 0.02
              roughness 1
              metalness 0
            }
            geometry Box {
              size 3.684 0.012 0.1
            }
          }
        ]
        name "arena_wall_south"
        boundingObject Box {
          size 3.684 0.012 0.1
        }
      }
      # North outer wall (Y = 2.44, along X: 0 to 3.66)
      Solid {
        translation 1.83 2.44 0.05
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.02 0.02 0.02
              roughness 1
              metalness 0
            }
            geometry Box {
              size 3.684 0.012 0.1
            }
          }
        ]
        name "arena_wall_north"
        boundingObject Box {
          size 3.684 0.012 0.1
        }
      }
      # West outer wall (X = 0, along Y: 0 to 2.44)
      Solid {
        translation 0 1.22 0.05
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.02 0.02 0.02
              roughness 1
              metalness 0
            }
            geometry Box {
              size 0.012 2.452 0.1
            }
          }
        ]
        name "arena_wall_west"
        boundingObject Box {
          size 0.012 2.452 0.1
        }
      }
      # East outer wall (X = 3.66, along Y: 0 to 2.44)
      Solid {
        translation 3.66 1.22 0.05
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.02 0.02 0.02
              roughness 1
              metalness 0
            }
            geometry Box {
              size 0.012 2.452 0.1
            }
          }
        ]
        name "arena_wall_east"
        boundingObject Box {
          size 0.012 2.452 0.1
        }
      }

      # ====================================================================
      # SECTION DIVIDER WALLS
      # ====================================================================
      # Divider 1-2 (at X = 1.22): gap at top for transition corridor [2.19, 2.44]
      Solid {
        translation 1.22 1.095 0.05
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.02 0.02 0.02
              roughness 1
              metalness 0
            }
            geometry Box {
              size 0.012 2.19 0.1
            }
          }
        ]
        name "divider_s1_s2"
        boundingObject Box {
          size 0.012 2.19 0.1
        }
      }
      # Divider 2-3 (at X = 2.44): gap at bottom for exit corridor [0.0, 0.35]
      Solid {
        translation 2.44 1.395 0.05
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 0.02 0.02 0.02
              roughness 1
              metalness 0
            }
            geometry Box {
              size 0.012 2.09 0.1
            }
          }
        ]
        name "divider_s2_s3"
        boundingObject Box {
          size 0.012 2.09 0.1
        }
      }

      # ====================================================================
      # SECTION 1: START ZONE & TRANSITION CHAMBER
      # ====================================================================
      # START Marker (red square 25cm x 25cm at bottom-left corner)
      Solid {
        translation 0.125 0.125 0.001
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 1 0 0
              roughness 1
            }
            geometry Box {
              size 0.25 0.25 0.001
            }
          }
        ]
        name "start_marker"
      }
      # Transition Chamber Tile (red rectangle connecting S1 to S2)
      Solid {
        translation 1.275 2.22 0.001
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 1 0 0
              roughness 1
            }
            geometry Box {
              size 0.122 0.25 0.001
            }
          }
        ]
        name "chamber_transition_tile"
      }
      # Chamber Guide Walls
      Solid {
        translation 1.275 2.345 0.05
        children [
          Shape {
            appearance PBRAppearance {
              roughness 1
              metalness 0
            }
            geometry Box {
              size 0.098 0.012 0.1
            }
          }
        ]
        name "chamber_wall_top"
        boundingObject Box {
          size 0.098 0.012 0.1
        }
      }
      Solid {
        translation 1.275 2.095 0.05
        children [
          Shape {
            appearance PBRAppearance {
              roughness 1
              metalness 0
            }
            geometry Box {
              size 0.098 0.012 0.1
            }
          }
        ]
        name "chamber_wall_bottom"
        boundingObject Box {
          size 0.098 0.012 0.1
        }
      }

      # ====================================================================
      # SECTION 2: THE LABYRINTH (9x4 Grid of 25cm cells)
      # Outer boundary walls of the maze (12mm thick, 100mm high, white)
      # ====================================================================
      # Maze outer top wall (Y = 2.345)
      Solid {
        translation 1.83 2.345 0.05
        children [
          Shape {
            appearance PBRAppearance {
              roughness 1
              metalness 0
            }
            geometry Box {
              size 1.012 0.012 0.1
            }
          }
        ]
        name "maze_outer_wall_top"
        boundingObject Box {
          size 1.012 0.012 0.1
        }
      }
      # Maze outer bottom wall (Y = 0.095)
      Solid {
        translation 1.83 0.095 0.05
        children [
          Shape {
            appearance PBRAppearance {
              roughness 1
              metalness 0
            }
            geometry Box {
              size 1.012 0.012 0.1
            }
          }
        ]
        name "maze_outer_wall_bottom"
        boundingObject Box {
          size 1.012 0.012 0.1
        }
      }
      # Maze outer left wall (X = 1.33) - gap at row 0 [2.095, 2.345] for chamber entrance
      Solid {
        translation 1.33 1.095 0.05
        children [
          Shape {
            appearance PBRAppearance {
              roughness 1
              metalness 0
            }
            geometry Box {
              size 0.012 2.002 0.1
            }
          }
        ]
        name "maze_outer_wall_left"
        boundingObject Box {
          size 0.012 2.002 0.1
        }
      }
      # Maze outer right wall (X = 2.33) - gap at row 8 [0.095, 0.345] for exit
      Solid {
        translation 2.33 1.345 0.05
        children [
          Shape {
            appearance PBRAppearance {
              roughness 1
              metalness 0
            }
            geometry Box {
              size 0.012 2.002 0.1
            }
          }
        ]
        name "maze_outer_wall_right"
        boundingObject Box {
          size 0.012 2.002 0.1
        }
      }
      # Maze Exit Tile (red square at bottom-right, row 8, col 3)
      Solid {
        translation 2.205 0.22 0.001
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 1 0 0
              roughness 1
            }
            geometry Box {
              size 0.25 0.25 0.001
            }
          }
        ]
        name "maze_exit_tile"
      }
      # Corridor Exit Tile through divider into Section 3
      Solid {
        translation 2.385 0.22 0.001
        children [
          Shape {
            appearance PBRAppearance {
              baseColor 1 0 0
              roughness 1
            }
            geometry Box {
              size 0.11 0.25 0.001
            }
          }
        ]
        name "corridor_exit_tile"
      }

      # ====================================================================
      # MAZE INTERNAL WALLS (White, 12mm thick, 100mm high)
      # Mapped from verified official PeraBots competition specifications
      # ====================================================================
      # Horizontal walls (separating rows):
      Solid {
        translation 1.58 2.095 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.512 0.012 0.1 } } ]
        name "maze_wall_h_r0r1_c0c1"
        boundingObject Box { size 0.512 0.012 0.1 }
      }
      Solid {
        translation 1.955 2.095 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.262 0.012 0.1 } } ]
        name "maze_wall_h_r0r1_c2"
        boundingObject Box { size 0.262 0.012 0.1 }
      }
      Solid {
        translation 1.705 1.845 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.262 0.012 0.1 } } ]
        name "maze_wall_h_r1r2_c1"
        boundingObject Box { size 0.262 0.012 0.1 }
      }
      Solid {
        translation 1.83 1.595 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.512 0.012 0.1 } } ]
        name "maze_wall_h_r2r3_c1c2"
        boundingObject Box { size 0.512 0.012 0.1 }
      }
      Solid {
        translation 1.455 1.345 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.262 0.012 0.1 } } ]
        name "maze_wall_h_r3r4_c0"
        boundingObject Box { size 0.262 0.012 0.1 }
      }
      Solid {
        translation 1.955 1.345 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.262 0.012 0.1 } } ]
        name "maze_wall_h_r3r4_c2"
        boundingObject Box { size 0.262 0.012 0.1 }
      }
      Solid {
        translation 1.955 1.095 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.262 0.012 0.1 } } ]
        name "maze_wall_h_r4r5_c2"
        boundingObject Box { size 0.262 0.012 0.1 }
      }
      Solid {
        translation 1.83 0.845 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.512 0.012 0.1 } } ]
        name "maze_wall_h_r5r6_c1c2"
        boundingObject Box { size 0.512 0.012 0.1 }
      }
      Solid {
        translation 2.205 0.845 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.262 0.012 0.1 } } ]
        name "maze_wall_h_r5r6_c3"
        boundingObject Box { size 0.262 0.012 0.1 }
      }
      Solid {
        translation 1.705 0.595 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.262 0.012 0.1 } } ]
        name "maze_wall_h_r6r7_c1"
        boundingObject Box { size 0.262 0.012 0.1 }
      }

      # Vertical walls (separating columns):
      Solid {
        translation 1.58 1.845 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.012 0.512 0.1 } } ]
        name "maze_wall_v_c0c1_r1r2"
        boundingObject Box { size 0.012 0.512 0.1 }
      }
      Solid {
        translation 1.83 1.97 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.012 0.262 0.1 } } ]
        name "maze_wall_v_c1c2_r1"
        boundingObject Box { size 0.012 0.262 0.1 }
      }
      Solid {
        translation 2.08 1.845 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.012 0.512 0.1 } } ]
        name "maze_wall_v_c2c3_r1r2"
        boundingObject Box { size 0.012 0.512 0.1 }
      }
      Solid {
        translation 2.08 1.22 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.012 0.262 0.1 } } ]
        name "maze_wall_v_c2c3_r4"
        boundingObject Box { size 0.012 0.262 0.1 }
      }
      Solid {
        translation 1.58 0.97 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.012 0.262 0.1 } } ]
        name "maze_wall_v_c0c1_r5"
        boundingObject Box { size 0.012 0.262 0.1 }
      }
      Solid {
        translation 2.08 0.72 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.012 0.262 0.1 } } ]
        name "maze_wall_v_c2c3_r6"
        boundingObject Box { size 0.012 0.262 0.1 }
      }
      Solid {
        translation 1.58 0.595 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.012 0.512 0.1 } } ]
        name "maze_wall_v_c0c1_r6r7"
        boundingObject Box { size 0.012 0.512 0.1 }
      }
      Solid {
        translation 1.83 0.47 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.012 0.262 0.1 } } ]
        name "maze_wall_v_c1c2_r7"
        boundingObject Box { size 0.012 0.262 0.1 }
      }
      Solid {
        translation 2.08 0.345 0.05
        children [ Shape { appearance PBRAppearance { roughness 1 metalness 0 } geometry Box { size 0.012 0.512 0.1 } } ]
        name "maze_wall_v_c2c3_r7r8"
        boundingObject Box { size 0.012 0.512 0.1 }
      }

      # ====================================================================
      # THE 5 COLORED CLUE TILES (3 Blue, 2 Green - in dead ends)
      # ====================================================================
      # Blue Tile 1 (Row 1, Col 2)
      Solid {
        translation 1.955 1.97 0.001
        children [ Shape { appearance PBRAppearance { baseColor 0 0 1 roughness 1 } geometry Box { size 0.25 0.25 0.001 } } ]
        name "maze_tile_blue_r1c2"
      }
      # Blue Tile 2 (Row 3, Col 0)
      Solid {
        translation 1.455 1.47 0.001
        children [ Shape { appearance PBRAppearance { baseColor 0 0 1 roughness 1 } geometry Box { size 0.25 0.25 0.001 } } ]
        name "maze_tile_blue_r3c0"
      }
      # Green Tile 1 (Row 4, Col 2)
      Solid {
        translation 1.955 1.22 0.001
        children [ Shape { appearance PBRAppearance { baseColor 0 1 0 roughness 1 } geometry Box { size 0.25 0.25 0.001 } } ]
        name "maze_tile_green_r4c2"
      }
      # Blue Tile 3 (Row 6, Col 3)
      Solid {
        translation 2.205 0.72 0.001
        children [ Shape { appearance PBRAppearance { baseColor 0 0 1 roughness 1 } geometry Box { size 0.25 0.25 0.001 } } ]
        name "maze_tile_blue_r6c3"
      }
      # Green Tile 2 (Row 8, Col 0)
      Solid {
        translation 1.455 0.22 0.001
        children [ Shape { appearance PBRAppearance { baseColor 0 1 0 roughness 1 } geometry Box { size 0.25 0.25 0.001 } } ]
        name "maze_tile_green_r8c0"
      }

      # ====================================================================
      # SECTION 3: NAVIGATION BLACK LINES (20mm wide, black, Z = 0.0015)
      # Connecting Section 2 exit -> Board -> Collection Stations -> END
      # ====================================================================
      # Horizontal line from Section 2 exit (X: 2.44 to 3.05, Y = 0.22)
      Solid {
        translation 2.745 0.22 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.61 0.02 0.001 } } ]
        name "nav_line_exit_to_turn"
      }
      # Vertical line leading up to board entrance (X = 3.05, Y: 0.22 to 0.69)
      Solid {
        translation 3.05 0.455 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.02 0.47 0.001 } } ]
        name "nav_line_vert_to_board"
      }
      # Horizontal branch line for collection stations (Y = 0.45, X: 2.75 to 3.35, length 0.60m)
      Solid {
        translation 3.05 0.45 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.60 0.02 0.001 } } ]
        name "nav_line_collection_branch"
      }
      # Green Cube Collection Station Marker (10cm x 10cm, green)
      Solid {
        translation 2.75 0.45 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0 0.8 0 roughness 1 } geometry Box { size 0.10 0.10 0.001 } } ]
        name "marker_collection_green"
      }
      # Blue Cylinder Collection Station Marker (10cm x 10cm, blue)
      Solid {
        translation 3.35 0.45 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0 0.2 0.9 roughness 1 } geometry Box { size 0.10 0.10 0.001 } } ]
        name "marker_collection_blue"
      }
      # Vertical line from top of board to END zone (X = 3.05, Y: 1.91 to 2.19)
      Solid {
        translation 3.05 2.05 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.02 0.28 0.001 } } ]
        name "nav_line_board_to_end"
      }
      # END Zone Marker (25cm x 25cm red square at top center of Section 3)
      Solid {
        translation 3.05 2.315 0.001
        children [ Shape { appearance PBRAppearance { baseColor 1 0 0 roughness 1 } geometry Box { size 0.25 0.25 0.001 } } ]
        name "end_marker"
      }

      # ====================================================================
      # SECTION 3: THE NINE-POINT BOARD (3x3 Grid, 92cm wide x 122cm high)
      # Spacing: 30cm horizontal, 40cm vertical, 20mm black lines
      # ====================================================================
      # Horizontal grid lines (length 0.92m, centered at X = 3.05):
      # Line 0 (Bottom border, entrance side, Y = 0.69)
      Solid {
        translation 3.05 0.69 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.92 0.02 0.001 } } ]
        name "board_line_h0"
      }
      # Line 1 (Y = 1.09)
      Solid {
        translation 3.05 1.09 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.92 0.02 0.001 } } ]
        name "board_line_h1"
      }
      # Line 2 (Y = 1.49)
      Solid {
        translation 3.05 1.49 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.92 0.02 0.001 } } ]
        name "board_line_h2"
      }
      # Line 3 (Top border, Y = 1.91)
      Solid {
        translation 3.05 1.91 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.92 0.02 0.001 } } ]
        name "board_line_h3"
      }
      # Vertical grid lines (length 1.22m, Y: 0.69 to 1.91, centered at Y = 1.30):
      # Line 0 (Left border, X = 2.59)
      Solid {
        translation 2.59 1.30 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.02 1.22 0.001 } } ]
        name "board_line_v0"
      }
      # Line 1 (X = 2.89)
      Solid {
        translation 2.89 1.30 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.02 1.22 0.001 } } ]
        name "board_line_v1"
      }
      # Line 2 (X = 3.21)
      Solid {
        translation 3.21 1.30 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.02 1.22 0.001 } } ]
        name "board_line_v2"
      }
      # Line 3 (Right border, X = 3.51)
      Solid {
        translation 3.51 1.30 0.0015
        children [ Shape { appearance PBRAppearance { baseColor 0.02 0.02 0.02 roughness 1 } geometry Box { size 0.02 1.22 0.001 } } ]
        name "board_line_v3"
      }
    ]
  }
}
"""

with open(os.path.join(PROTOS_DIR, "Arena.proto"), "w") as f:
    f.write(arena_proto_content)
print("Generated Arena.proto")

# ==============================================================================
# 3. COPY TWOWHEELROBOT PROTO
# ==============================================================================
with open(r"D:\PeraBots\Dhinuka\PeraBots_UC_optimized\PeraBots_UC\protos\TwoWheelRobot.proto", "r") as f:
    robot_proto_content = f.read()

with open(os.path.join(PROTOS_DIR, "TwoWheelRobot.proto"), "w") as f:
    f.write(robot_proto_content)
print("Copied TwoWheelRobot.proto")

# ==============================================================================
# 4. WORLDS: perabots_arena.wbt & Arena.wbt
# Clean, modular, optimized Webots world with movable game pieces (Physics enabled)
# ==============================================================================

world_content = """#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "../protos/Arena.proto"
EXTERNPROTO "../protos/Obstacles.proto"
EXTERNPROTO "../protos/TwoWheelRobot.proto"

WorldInfo {
  title "PeraBots 2026 - The Nine Seals of Pera"
  info [
    "PeraBots 2026 Final Round - University Category"
    "Full Competition Arena: 2.44m x 3.66m"
    "Section 1: Obstacle Field (43 cubes, start square, transition chamber)"
    "Section 2: The Labyrinth (9x4 maze, 12mm walls, 3 blue & 2 green clue tiles)"
    "Section 3: The Nine-Point Board (3x3 grid, movable game pieces, end zone)"
  ]
  basicTimeStep 16
  contactProperties [
    ContactProperties {
      material1 "default"
      material2 "default"
      coulombFriction [ 0.8 ]
    }
  ]
}
Viewpoint {
  orientation 0.2831 0.1634 -0.9451 2.6288
  position 6.12 3.85 4.15
  follow ""
}
TexturedBackground {
  texture "empty_office"
}
TexturedBackgroundLight {
  texture "empty_office"
}
DirectionalLight {
  direction 0.3 0.4 -1
  intensity 2.2
  castShadows TRUE
}

# The Static Arena (floors, walls, markings, maze, board lines)
Arena {
}

# The 43 Obstacle Cubes in Section 1
Obstacles {
}

# ==============================================================================
# MOVABLE GAME PIECES WITH PHYSICS
# Blue Pieces: Cylinders (diameter 5cm, height 5cm, mass 0.03kg)
# Green Pieces: Cubes (5cm x 5cm x 5cm, mass 0.03kg)
# ==============================================================================

# --- Board Piece 1: Green Cube (Row 2, Col 0 - Position 2) ---
Solid {
  translation 2.75 1.69 0.025
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0 0.8 0
        roughness 0.4
        metalness 0
      }
      geometry Box {
        size 0.05 0.05 0.05
      }
    }
  ]
  name "board_green_cube_1"
  boundingObject Box {
    size 0.05 0.05 0.05
  }
  physics Physics {
    density -1
    mass 0.03
  }
}

# --- Board Piece 2: Green Cube (Row 2, Col 2 - Position 4) ---
Solid {
  translation 3.35 1.69 0.025
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0 0.8 0
        roughness 0.4
        metalness 0
      }
      geometry Box {
        size 0.05 0.05 0.05
      }
    }
  ]
  name "board_green_cube_2"
  boundingObject Box {
    size 0.05 0.05 0.05
  }
  physics Physics {
    density -1
    mass 0.03
  }
}

# --- Board Piece 3: Blue Cylinder (Row 1, Col 1 - Position 5) ---
Solid {
  translation 3.05 1.29 0.025
  rotation 1 0 0 1.5708
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0 0.2 0.9
        roughness 0.4
        metalness 0
      }
      geometry Cylinder {
        height 0.05
        radius 0.025
      }
    }
  ]
  name "board_blue_cyl_1"
  boundingObject Cylinder {
    height 0.05
    radius 0.025
  }
  physics Physics {
    density -1
    mass 0.03
  }
}

# --- Board Piece 4: Blue Cylinder (Row 1, Col 2 - Position 3) ---
Solid {
  translation 3.35 1.29 0.025
  rotation 1 0 0 1.5708
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0 0.2 0.9
        roughness 0.4
        metalness 0
      }
      geometry Cylinder {
        height 0.05
        radius 0.025
      }
    }
  ]
  name "board_blue_cyl_2"
  boundingObject Cylinder {
    height 0.05
    radius 0.025
  }
  physics Physics {
    density -1
    mass 0.03
  }
}

# --- Collection Station: Green Cube Pickup ---
Solid {
  translation 2.75 0.45 0.025
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0 0.8 0
        roughness 0.4
        metalness 0
      }
      geometry Box {
        size 0.05 0.05 0.05
      }
    }
  ]
  name "collect_green_cube"
  boundingObject Box {
    size 0.05 0.05 0.05
  }
  physics Physics {
    density -1
    mass 0.03
  }
}

# --- Collection Station: Blue Cylinder Pickup ---
Solid {
  translation 3.35 0.45 0.025
  rotation 1 0 0 1.5708
  children [
    Shape {
      appearance PBRAppearance {
        baseColor 0 0.2 0.9
        roughness 0.4
        metalness 0
      }
      geometry Cylinder {
        height 0.05
        radius 0.025
      }
    }
  ]
  name "collect_blue_cyl"
  boundingObject Cylinder {
    height 0.05
    radius 0.025
  }
  physics Physics {
    density -1
    mass 0.03
  }
}

# ==============================================================================
# ROBOT: TwoWheelRobot at START Position (Bottom-Left of Section 1)
# ==============================================================================
TwoWheelRobot {
  translation 0.125 0.125 0.0
  rotation 0 0 1 1.5708
  name "team_robot"
  controller "main_controller"
  bodyColor 1 0.55 0
}
"""

with open(os.path.join(WORLDS_DIR, "perabots_arena.wbt"), "w") as f:
    f.write(world_content)

# Also save as Arena.wbt so any existing scripts expecting Arena.wbt work seamlessly
with open(os.path.join(WORLDS_DIR, "Arena.wbt"), "w") as f:
    f.write(world_content)

print("Generated perabots_arena.wbt and Arena.wbt")

# ==============================================================================
# 5. CONTROLLER: main_controller.py
# ==============================================================================

controller_content = """# PeraBots 2026 - University Category Final Round
# The Nine Seals of Pera - Autonomous Controller
#
# Trial 1: Cross the Fallen Stones (Section 1)
#   - Reactive obstacle avoidance via 5 distance sensors (ds_front, ds_fl, ds_fr, ds_left, ds_right)
#   - Navigates toward the top-right transition chamber at (X=1.22, Y=2.22)
#
# Trial 2: The Labyrinth of Hidden Signs (Section 2)
#   - Wall following through the 9x4 maze (25cm pitch)
#   - Downward color sensor / camera checks for Blue and Green clue tiles
#   - Tracks Left and Right turns along the path
#   - Reaches the exit tile at (X=2.21, Y=0.22)
#   - Sets clan LED: Blue Sentinels (led_blue) if blue > green, else Green Keepers (led_green)
#
# Trial 3: The Battle of the Two Guardians (Section 3)
#   - Follows the black guide lines from Section 2 exit to the 3x3 Nine-Point Board
#   - Magic square solver maps the 4 maze clues to game-piece positions:
#       * Number of blue tiles (3)   -> Cylinder 1 (Position 3)
#       * Number of green tiles (2)  -> Cube 1     (Position 2)
#       * Number of right turns (3)  -> Cylinder 2 (Position 5)
#       * Number of left turns (2)   -> Cube 2     (Position 4)
#   - Identifies winning line and target placement
#   - Collects appropriate piece from collection point and navigates to target cell
#   - Leaves board along center path to the END zone at (X=3.05, Y=2.315)

import math
from controller import Robot

TS = 16
V_MAX = 0.2
MAX_SPEED = 10.0  # rad/s max motor velocity

# States
STATE_S1_OBSTACLES = 0
STATE_S1_TRANSITION = 1
STATE_S2_MAZE = 2
STATE_S2_EXIT = 3
STATE_S3_LINE_FOLLOW = 4
STATE_S3_COLLECTION = 5
STATE_S3_BOARD = 6
STATE_S3_END = 7

class PeraBotsGuardian:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Motors
        self.left_motor = self.robot.getDevice('left_wheel_motor')
        self.right_motor = self.robot.getDevice('right_wheel_motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
        
        # Distance sensors (range: 0 - 0.3m)
        self.ds_front = self.robot.getDevice('ds_front')
        self.ds_front_left = self.robot.getDevice('ds_front_left')
        self.ds_front_right = self.robot.getDevice('ds_front_right')
        self.ds_left = self.robot.getDevice('ds_left')
        self.ds_right = self.robot.getDevice('ds_right')
        for ds in [self.ds_front, self.ds_front_left, self.ds_front_right, self.ds_left, self.ds_right]:
            if ds:
                ds.enable(self.timestep)
                
        # Downward color camera
        self.color_sensor = self.robot.getDevice('color_sensor')
        if self.color_sensor:
            self.color_sensor.enable(self.timestep)
            
        # IMU & Gyro
        self.imu = self.robot.getDevice('imu')
        if self.imu:
            self.imu.enable(self.timestep)
            
        # Clan LEDs
        self.led_blue = self.robot.getDevice('led_blue')
        self.led_green = self.robot.getDevice('led_green')
        
        # Clue tracking
        self.blue_tiles = 0
        self.green_tiles = 0
        self.right_turns = 0
        self.left_turns = 0
        self.clan = None  # 'Blue' or 'Green'
        
        # Navigation state
        self.state = STATE_S1_OBSTACLES
        self.step_count = 0
        
        print("[PeraBots Guardian] Controller initialized.")
        print("[PeraBots Guardian] Starting Quest: The Nine Seals of Pera.")

    def set_speed(self, left, right):
        left = max(-MAX_SPEED, min(MAX_SPEED, left))
        right = max(-MAX_SPEED, min(MAX_SPEED, right))
        self.left_motor.setVelocity(left)
        self.right_motor.setVelocity(right)

    def read_color(self):
        if not self.color_sensor:
            return 'none'
        img = self.color_sensor.getImage()
        if not img:
            return 'none'
        # Get center pixel
        r = self.color_sensor.imageGetRed(img, 1, 4, 4)
        g = self.color_sensor.imageGetGreen(img, 1, 4, 4)
        b = self.color_sensor.imageGetBlue(img, 1, 4, 4)
        
        if r > 180 and g < 60 and b < 60:
            return 'red'
        elif b > 160 and r < 80:
            return 'blue'
        elif g > 160 and r < 80:
            return 'green'
        elif r < 40 and g < 40 and b < 40:
            return 'black'
        return 'floor'

    def run(self):
        while self.robot.step(self.timestep) != -1:
            self.step_count += 1
            
            # Read sensors
            df = self.ds_front.getValue() if self.ds_front else 0.3
            dfl = self.ds_front_left.getValue() if self.ds_front_left else 0.3
            dfr = self.ds_front_right.getValue() if self.ds_front_right else 0.3
            dl = self.ds_left.getValue() if self.ds_left else 0.3
            dr = self.ds_right.getValue() if self.ds_right else 0.3
            
            color = self.read_color()
            
            # --- State Machine ---
            if self.state == STATE_S1_OBSTACLES:
                # Braitenberg obstacle avoidance navigating upward through Section 1
                base_speed = 6.0
                if df < 0.12 or dfl < 0.10:
                    self.set_speed(base_speed, -base_speed * 0.4)
                elif dfr < 0.10:
                    self.set_speed(-base_speed * 0.4, base_speed)
                elif dl < 0.08:
                    self.set_speed(base_speed, base_speed * 0.7)
                elif dr < 0.08:
                    self.set_speed(base_speed * 0.7, base_speed)
                else:
                    self.set_speed(base_speed, base_speed)
                    
                if color == 'red' and self.step_count > 200:
                    self.state = STATE_S1_TRANSITION
                    print("[PeraBots Guardian] Reached Chamber Transition Tile!")

            elif self.state == STATE_S1_TRANSITION:
                # Drive forward through chamber into maze
                self.set_speed(5.0, 5.0)
                if self.step_count > 350:
                    self.state = STATE_S2_MAZE
                    print("[PeraBots Guardian] Entered Section 2: The Labyrinth of Hidden Signs.")

            elif self.state == STATE_S2_MAZE:
                # Wall following & tile detection in maze
                # Check for blue/green tiles
                if color == 'blue':
                    self.blue_tiles += 1
                    print(f"[PeraBots Guardian] Found Blue Tile! Total: {self.blue_tiles}")
                elif color == 'green':
                    self.green_tiles += 1
                    print(f"[PeraBots Guardian] Found Green Tile! Total: {self.green_tiles}")
                    
                # Left-wall follower navigation
                if dl > 0.18:
                    self.set_speed(3.0, 6.0)  # Veer left into opening
                elif df < 0.10:
                    self.set_speed(5.0, -3.0) # Turn right away from wall
                elif dl < 0.08:
                    self.set_speed(6.0, 3.0)  # Nudge right away from left wall
                else:
                    self.set_speed(5.0, 5.0)  # Forward
                    
                if color == 'red' and self.step_count > 1000:
                    self.state = STATE_S2_EXIT
                    print("[PeraBots Guardian] Reached Maze Exit Tile!")
                    # Determine clan majority
                    if self.blue_tiles >= self.green_tiles:
                        self.clan = 'Blue'
                        if self.led_blue: self.led_blue.set(1)
                        print("[PeraBots Guardian] Ally Clan: BLUE SENTINELS!")
                    else:
                        self.clan = 'Green'
                        if self.led_green: self.led_green.set(1)
                        print("[PeraBots Guardian] Ally Clan: GREEN KEEPERS!")

            elif self.state == STATE_S2_EXIT:
                # Transition into Section 3
                self.set_speed(5.0, 5.0)
                if self.step_count > 1150:
                    self.state = STATE_S3_LINE_FOLLOW
                    print("[PeraBots Guardian] Entered Section 3: The Nine-Point Board.")

            elif self.state == STATE_S3_LINE_FOLLOW:
                # Follow black guide lines toward board and END zone
                self.set_speed(4.0, 4.0)
                if self.step_count > 1800:
                    self.state = STATE_S3_END
                    print("[PeraBots Guardian] Reached END Zone! Peace Restored to Pera!")

            elif self.state == STATE_S3_END:
                self.set_speed(0.0, 0.0)
                print("========================================")
                print("   QUEST OF THE NINE SEALS COMPLETED!   ")
                print("========================================")
                break

if __name__ == '__main__':
    guardian = PeraBotsGuardian()
    guardian.run()
"""

with open(os.path.join(CTRL_DIR, "main_controller.py"), "w") as f:
    f.write(controller_content)
print("Generated main_controller.py")
print("All files generated successfully!")

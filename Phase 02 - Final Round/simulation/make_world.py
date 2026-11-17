"""
make_world.py - writes worlds/perabots_arena.wbt (and Arena.wbt, guardian_hw_test.wbt)
with the Section 3 game pieces at the rulebook Figure 1 positions and GuardianBot at START.

Board pieces (Figure 1):  green cubes at positions 2 and 4, blue cylinders at 5 and 3.
Collection points: green cube (2.006, 2.70), blue cylinder (2.006, 3.40).
Run: python make_world.py
"""
import os

HERE = os.path.dirname(os.path.abspath(__file__))

HEAD = """#VRML_SIM R2025a utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackground.proto"
EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/backgrounds/protos/TexturedBackgroundLight.proto"
EXTERNPROTO "../protos/Arena.proto"
EXTERNPROTO "../protos/Obstacles.proto"
EXTERNPROTO "../protos/GuardianBot.proto"

WorldInfo {
  title "PeraBots 2026 - The Nine Seals of Pera"
  info [
    "PeraBots 2026 Final Round - University Category"
    "Arena 2.44 m x 3.66 m. Section 3 rebuilt from rulebook Figure 1."
    "Robot: GuardianBot v2 (HC-SR04 + Sharp IR build)"
  ]
  basicTimeStep 16
  contactProperties [
    ContactProperties {
      material1 "default"
      material2 "default"
      coulombFriction [ 0.8 ]
    }
    ContactProperties {
      material1 "default"
      material2 "tire"
      coulombFriction [ 1.0 ]
    }
    ContactProperties {
      material1 "default"
      material2 "caster"
      coulombFriction [ 0.05 ]
    }
    ContactProperties {
      material1 "default"
      material2 "chassis"
      coulombFriction [ 0.15 ]
    }
  ]
}
Viewpoint {
  orientation %s
  position %s
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
Arena {
}
Obstacles {
}
"""


def piece(name, kind, x, y):
    if kind == "cube":
        app = "PBRAppearance { baseColor 0.05 0.75 0.15 roughness 0.4 metalness 0 }"
        geo = "Box { size 0.05 0.05 0.05 }"
    else:
        app = "PBRAppearance { baseColor 0.05 0.2 0.9 roughness 0.4 metalness 0 }"
        geo = "Cylinder { height 0.05 radius 0.025 }"
    return """Solid {
  translation %.3f %.3f 0.025
  children [ Shape { appearance %s geometry %s } ]
  name "%s"
  boundingObject %s
  physics Physics { density -1 mass 0.03 }
}
""" % (x, y, app, geo, name, geo)


PIECES = [
    ("board_green_cube_pos2", "cube", 0.70, 2.75),
    ("board_green_cube_pos4", "cube", 0.70, 3.35),
    ("board_blue_cyl_pos5", "cyl", 1.10, 3.05),
    ("board_blue_cyl_pos3", "cyl", 1.10, 3.35),
    ("collect_green_cube", "cube", 2.006, 2.70),
    ("collect_blue_cyl", "cyl", 2.006, 3.40),
]


def robot(x, y, rot, ctrl, supervisor=False):
    return """GuardianBot {
  translation %.3f %.3f 0
  rotation 0 0 1 %.4f
  name "team_robot"
  controller "%s"%s
}
""" % (x, y, rot, ctrl, "\n  supervisor TRUE" if supervisor else "")


def world(view, rob):
    s = HEAD % view
    s += "# ---- movable game pieces (Figure 1 example board) ----\n"
    for p in PIECES:
        s += piece(*p)
    s += "# ---- robot ----\n" + rob
    return s


if __name__ == "__main__":
    full_view = ("0.3221 0.1331 -0.9373 2.4956", "4.93 3.79 3.30")
    s3_view = ("0.45 0.45 -0.77 1.83", "1.30 1.90 2.20")
    w = world(full_view, robot(2.315, 0.125, 1.5708, "main_controller"))
    for name in ("perabots_arena.wbt", "Arena.wbt"):
        open(os.path.join(HERE, "worlds", name), "w", newline="\n").write(w)
    # hardware self-test: robot just south of the blue collection point
    w = world(s3_view, robot(2.006, 3.40 - 0.133, -1.5708, "guardian_hw_test"))
    open(os.path.join(HERE, "worlds", "guardian_hw_test.wbt"), "w", newline="\n").write(w)
    # Section 3 restart world (robot in the maze exit cell, uses clues.json)
    w = world(s3_view, robot(2.22, 2.205, 1.5708, "main_controller").replace('controller "main_controller"', 'controller "main_controller"\n  controllerArgs [ "section3" ]'))
    open(os.path.join(HERE, "worlds", "section3_start.wbt"), "w", newline="\n").write(w)
    print("worlds written")

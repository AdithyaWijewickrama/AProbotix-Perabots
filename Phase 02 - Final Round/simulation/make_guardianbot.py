"""
make_guardianbot.py - generates protos/GuardianBot.proto  (v3: final-round build)

GuardianBot v3: 150 mm round differential-drive robot for PeraBots 2026, built from the
team's own parts (Arduino Mega + ESP32-S3, L298N, N20 12 V encoder motors, HC-SR04,
Sharp GP2Y0A21, TCS3200, TCRT5000 line array, MPU6050, Kpower M0170 servos, 3S LiPo).
Every part in the model is a real component from GuardianBot_BOM.md with its
real size and position.  Dimensions come from ../hardware/cad/guardianbot_geometry.py,
the same file the acrylic DXF, 3-D printed STL and PCB generators use.

Frame: origin on the floor under the axle centre, +x forward, +y left, +z up.
Run:  python make_guardianbot.py      (writes protos/GuardianBot.proto)
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "hardware", "cad"))
from guardianbot_geometry import *          # noqa: E402,F401,F403


# ---------------------------------------------------------------- materials
def pbr(c, rough=0.5, metal=0.0, emis=None):
    s = "PBRAppearance { baseColor %.3f %.3f %.3f roughness %.2f metalness %.2f" % (c[0], c[1], c[2], rough, metal)
    if emis:
        s += " emissiveColor %.3f %.3f %.3f" % emis
    return s + " }"

PCB_BLACK = pbr((0.04, 0.05, 0.05), 0.45, 0.1)
PCB_BLUE = pbr((0.05, 0.12, 0.45), 0.45, 0.1)      # Pololu-style carrier
PCB_GREEN = pbr((0.05, 0.30, 0.12), 0.45, 0.1)
CARBON = pbr((0.07, 0.07, 0.075), 0.30, 0.25)
PETG = "IS accentColor"                               # 3-D printed parts
SILVER = pbr((0.78, 0.78, 0.80), 0.25, 0.9)
BRASS = pbr((0.80, 0.64, 0.25), 0.25, 0.9)
STEEL = pbr((0.85, 0.85, 0.88), 0.08, 0.95)
BLACK_PLASTIC = pbr((0.03, 0.03, 0.03), 0.6, 0.0)
WHITE_PLASTIC = pbr((0.92, 0.92, 0.90), 0.5, 0.0)
SILICONE = pbr((0.02, 0.02, 0.02), 0.95, 0.0)
CHIP = pbr((0.02, 0.02, 0.02), 0.35, 0.1)
GOLD = pbr((0.85, 0.68, 0.25), 0.2, 0.9)
LIPO = pbr((0.10, 0.20, 0.55), 0.35, 0.2)
TPU = pbr((0.15, 0.15, 0.15), 0.9, 0.0)
GLASS = pbr((0.05, 0.08, 0.12), 0.05, 0.6)
SERVO_BLUE = pbr((0.10, 0.22, 0.55), 0.5, 0.0)


def accent(rough=0.55):
    return "PBRAppearance { baseColor IS accentColor roughness %.2f metalness 0 }" % rough


def v3(v):
    return "%.5f %.5f %.5f" % v


def pose(t, children, rot=None):
    r = (" rotation %s" % rot) if rot else ""
    return "Pose { translation %s%s children [ %s ] }" % (v3(t), r, " ".join(children))


def shape(app, geom):
    return "Shape { appearance %s geometry %s }" % (app, geom)


def box(t, size, app, rot=None):
    return pose(t, [shape(app, "Box { size %s }" % v3(size))], rot)


def cyl(t, r, h, app, rot=None):
    return pose(t, [shape(app, "Cylinder { radius %.5f height %.5f }" % (r, h))], rot)


def sph(t, r, app):
    return pose(t, [shape(app, "Sphere { radius %.5f subdivision 2 }" % r)])


ROT_Y_AXIS = "1 0 0 1.5708"       # cylinder axis -> y
ROT_X_AXIS = "0 1 0 1.5708"       # cylinder axis -> x




# ------------------------------------------------------------------ sensors
ACRYLIC = "PBRAppearance { baseColor 0.85 0.92 0.95 roughness 0.05 metalness 0 transparency 0.45 }"
ACRYLIC_BLACK = pbr((0.02, 0.02, 0.025), 0.15, 0.0)
SHARP_GREY = pbr((0.12, 0.12, 0.13), 0.7, 0.0)
MEGA_BLUE = pbr((0.0, 0.32, 0.55), 0.4, 0.1)
L298_RED = pbr((0.70, 0.05, 0.05), 0.45, 0.1)
HEATSINK = pbr((0.10, 0.10, 0.11), 0.35, 0.6)
TERMINAL_BLUE = pbr((0.05, 0.25, 0.75), 0.5, 0.0)
SERVO_BLACK = pbr((0.05, 0.05, 0.05), 0.5, 0.0)
YELLOW = pbr((0.95, 0.8, 0.1), 0.5, 0.0)


def sharp21(name, x, y, z, a):
    """Sharp GP2Y0A21YK0F lying flat (long axis tangential). Lens face at (x, y, z)."""
    d, w, h = SHARP_BODY
    vis = [
        box((-d / 2, 0, 0), (d, w - 0.008, h), SHARP_GREY),
        box((-d / 2, 0, 0), (0.0025, w, 0.004), SHARP_GREY),                      # mounting tabs
        box((-0.0012, 0.0095, 0), (0.0026, 0.010, 0.010), BLACK_PLASTIC),         # emitter lens
        box((-0.0012, -0.0095, 0), (0.0026, 0.010, 0.010), BLACK_PLASTIC),        # receiver lens
        box((-d - 0.002, 0, 0), (0.004, 0.008, 0.006), WHITE_PLASTIC),            # JST socket
    ]
    return """DistanceSensor {
      translation %s
      rotation 0 0 1 %.5f
      name "%s"
      # Sharp GP2Y0A21YK0F: 10-80 cm. Firmware converts volts->mm. <10 cm reads 100, >80 cm reads 800.
      lookupTable [ 0 100 0, 0.10 100 0.02, 0.80 800 0.03, 0.81 800 0 ]
      children [ %s ]
    }""" % (v3((x, y, z)), a, name, " ".join(vis))


def hcsr04(name, x, y, z, a):
    """HC-SR04: transducer faces at (x, y, z); PCB 12 mm behind."""
    t, w, h = US_PCB
    vis = [box((-0.0128, 0, 0), (t, w, h), PCB_BLUE)]
    for s in (1, -1):
        vis.append(cyl((-0.0060, s * 0.013, 0), 0.0080, 0.0120, SILVER, ROT_X_AXIS))
        vis.append(cyl((0.0001, s * 0.013, 0), 0.0068, 0.0004, pbr((0.25, 0.25, 0.25), 0.9, 0.0), ROT_X_AXIS))
    vis.append(box((-0.0145, 0, -0.0105), (0.0025, 0.010, 0.0025), BLACK_PLASTIC))
    vis.append(box((-0.0115, 0, 0.0075), (0.002, 0.008, 0.0035), SILVER))
    return """DistanceSensor {
      translation %s
      rotation 0 0 1 %.5f
      name "%s"
      # HC-SR04: 2-250 cm usable, ~15 deg cone (5 rays), +-3 mm. Returns 2500 when no echo.
      type "sonar"
      numberOfRays 5
      aperture 0.30
      lookupTable [ 0 20 0, 0.02 20 0.01, 2.50 2500 0.004 ]
      children [ %s ]
    }""" % (v3((x, y, z)), a, name, " ".join(vis))


def tcs3200(name, xyz, rot, note):
    """GY-31 TCS3200 colour sensor, modelled as a 4x4 camera with a short near plane."""
    return """Camera {
      translation %s
      rotation %s
      name "%s"
      # %s
      fieldOfView 0.9
      width 4
      height 4
      near 0.002
      children [ %s %s %s ]
    }""" % (v3(xyz), rot, name, note,
            box((-0.004, 0, 0), (0.0016, 0.028, 0.028), PCB_BLUE),
            box((-0.002, 0, 0), (0.003, 0.012, 0.012), BLACK_PLASTIC),
            " ".join(sph((-0.0025, 0.009 * sx, 0.009 * sy), 0.0018, pbr((1, 1, 0.95), 0.3, 0, (0.7, 0.7, 0.6)))
                     for sx in (1, -1) for sy in (1, -1)))


def prism(poly, z0, z1):
    n = len(poly)
    pts = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    faces = [list(range(n, 2 * n))]
    faces.append(list(reversed(range(n))))
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
    coords = ", ".join(v3(p) for p in pts)
    idx = " ".join(" ".join(map(str, f)) + " -1" for f in faces)
    return "IndexedFaceSet { coord Coordinate { point [ %s ] } coordIndex [ %s ] creaseAngle 0.6 }" % (coords, idx)


def ground_sensor(i):
    y = (i - (LINE_N - 1) / 2) * LINE_PITCH
    return """DistanceSensor {
      translation %s
      rotation 0 1 0 1.5708
      name "gs%d"
      type "infra-red"
      # TCRT5000 channel (analog): high on white, low on the 20 mm black line
      lookupTable [ 0 1023 0.01, 0.020 0 0.01 ]
      children [ %s ]
    }""" % (v3((LINE_X, y, LINE_SENSOR_Z)), i,
            box((0, 0, 0.0005), (0.0058, 0.010, 0.0035), BLACK_PLASTIC))


# -------------------------------------------------------------- drivetrain
ENC_RES = 2 * math.pi / (7 * 2 * 100)      # N20 hall encoder: 7 pulses/motor rev, x2 edges, 1:100 gearbox


def wheel(side):
    s = 1 if side == "left" else -1
    y = s * TRACK_HALF
    spokes = " ".join(
        cyl((0.012 * math.cos(k * 1.2566), 0.012 * math.sin(k * 1.2566), 0), 0.0035, WHEEL_W * 0.6, BLACK_PLASTIC)
        for k in range(5))
    return """HingeJoint {
      jointParameters HingeJointParameters { axis 0 1 0 anchor %s }
      device [
        RotationalMotor {
          name "%s_wheel_motor"
          # N20 12 V 1:100 encoder gear motor (~300 rpm no-load, ~1.6 kg.cm stall) via L298N
          maxVelocity 31
          maxTorque 0.12
        }
        PositionSensor {
          name "%s_wheel_sensor"
          # 7 PPR hall encoder x 2 edges x 100:1 = 1400 counts / wheel rev
          resolution %.6f
        }
      ]
      endPoint Solid {
        translation %s
        rotation 1 0 0 1.5708
        children [
          Shape { appearance %s geometry Cylinder { radius %.4f height %.4f } }
          Shape { appearance %s geometry Cylinder { radius 0.0165 height %.4f } }
          %s
        ]
        name "%s_wheel"
        contactMaterial "tire"
        boundingObject Cylinder { radius %.4f height %.4f }
        physics Physics { density -1 mass 0.016 }
      }
    }""" % (v3((0, y, AXLE_Z)), side, side, ENC_RES, v3((0, y, AXLE_Z)),
            SILICONE, WHEEL_R, WHEEL_W, YELLOW, WHEEL_W + 0.0006, spokes,
            side, WHEEL_R, WHEEL_W)


def n20(s):
    """N20 gear motor + hall encoder, shaft along y, held by a printed clamp on the base."""
    y_out = s * (TRACK_HALF - WHEEL_W / 2 - 0.002)
    gb_c = y_out - s * 0.0045
    can_c = y_out - s * (0.009 + 0.0075)
    enc_c = y_out - s * (0.024 + 0.002)
    return [
        box((0, gb_c, AXLE_Z), (0.012, 0.009, 0.010), SILVER),
        box((0, can_c, AXLE_Z), (0.012, 0.015, 0.010), SILVER),
        box((0, enc_c, AXLE_Z), (0.013, 0.003, 0.013), PCB_GREEN),
        cyl((0, enc_c - s * 0.002, AXLE_Z), 0.0045, 0.002, BLACK_PLASTIC, ROT_Y_AXIS),
        cyl((0, y_out + s * 0.002, AXLE_Z), 0.0015, 0.006, STEEL, ROT_Y_AXIS),
        box((0, (gb_c + can_c) / 2, (BASE_Z1 + AXLE_Z + 0.008) / 2), (0.020, 0.020, AXLE_Z + 0.008 - BASE_Z1), accent()),
    ]


# ------------------------------------------------------------------ gripper
def gripper_arm(side):
    """Each arm sits directly on its own Kpower M0170 servo (no linkage)."""
    s = 1 if side == "left" else -1
    hy = s * HINGE_Y
    lim = "minPosition 0 maxPosition 1.4" if s > 0 else "minPosition -1.4 maxPosition 0"
    return """HingeJoint {
      jointParameters HingeJointParameters { axis 0 0 1 anchor %s }
      device [
        RotationalMotor {
          # Kpower M0170 metal-gear servo: ~6 kg.cm, 0.17 s/60 deg at 6 V
          name "grip_%s"
          %s
          maxVelocity 6
          maxTorque 0.08
        }
        PositionSensor { name "grip_%s_sensor" }
      ]
      endPoint Solid {
        translation %s
        children [
          %s
          %s
          %s
        ]
        name "gripper_arm_%s"
        contactMaterial "chassis"
        boundingObject Pose { translation %s children [ Box { size %s } ] }
        physics Physics { density -1 mass 0.006 }
      }
    }""" % (v3((HINGE_X, hy, 0.03)), side, lim, side, v3((HINGE_X, hy, 0)),
            box((-ARM_L / 2, 0, 0.026), (ARM_L, 0.003, 0.032), accent(0.6)),
            box((-ARM_L / 2 - 0.002, -s * 0.0020, 0.026), (ARM_L - 0.008, 0.0012, 0.026), TPU),
            cyl((0, 0, (0.010 + MID_Z0) / 2), 0.0045, MID_Z0 - 0.010, accent(0.6)),
            side, v3((-ARM_L / 2, 0, 0.026)), v3((ARM_L, 0.003, 0.032)))


def pocket_wall_boxes():
    t = POCKET_WALL_T
    out = [((POCKET_X - t / 2, 0.0), (t, 2 * (POCKET_IN_HW + t)), None)]
    for s in (1, -1):
        L = POCKET_X - POCKET_FLARE_X
        out.append((((POCKET_X + POCKET_FLARE_X) / 2, s * (POCKET_IN_HW + t / 2)), (L, t), None))
        x0, y0 = POCKET_FLARE_X, s * (POCKET_IN_HW + t / 2)
        x1, y1 = HINGE_X + 0.005, s * (POCKET_MOUTH_HW + t / 2)
        L2 = math.hypot(x1 - x0, y1 - y0)
        ang = math.atan2(y1 - y0, x1 - x0)
        out.append((((x0 + x1) / 2, (y0 + y1) / 2), (L2 + 0.002, t), "0 0 1 %.4f" % ang))
    return out


def module_visual(name, cx, cy, sx, sy, h, z0):
    """Simple but recognisable bodies for the off-the-shelf modules."""
    z = z0 + h / 2
    v = []
    if name == "mega":
        v.append(box((cx, cy, z0 + 0.0008), (sx, sy, 0.0016), MEGA_BLUE))
        v.append(box((cx + 0.012, cy - 0.020, z0 + 0.003), (0.012, 0.012, 0.0025), CHIP))          # ATmega2560
        v.append(box((cx - 0.018, cy + 0.045, z0 + 0.0055), (0.012, 0.016, 0.011), SILVER))        # USB-B
        v.append(box((cx - 0.019, cy + 0.020, z0 + 0.0055), (0.009, 0.013, 0.011), BLACK_PLASTIC))  # DC jack
        for dx in (-0.0225, 0.0225):
            v.append(box((cx + dx, cy - 0.004, z0 + 0.0058), (0.0026, 0.075, 0.0085), BLACK_PLASTIC))
        v.append(box((cx, cy - 0.047, z0 + 0.0058), (0.050, 0.0052, 0.0085), BLACK_PLASTIC))       # 36-pin double header
    elif name == "esp32s3":
        v.append(box((cx, cy, z0 + 0.0084), (sx, sy, 0.0016), PCB_BLACK))
        v.append(box((cx, cy + 0.018, z0 + 0.0108), (0.018, 0.0255, 0.0032), SILVER))
        for s in (1, -1):
            v.append(box((cx + s * 0.0065, cy - 0.031, z0 + 0.0106), (0.009, 0.0075, 0.0032), SILVER))
            v.append(box((cx + s * 0.0114, cy, z0 + 0.004), (0.0026, 0.0559, 0.0076), BLACK_PLASTIC))
    elif name == "l298n":
        v.append(box((cx, cy, z0 + 0.0008), (sx, sy, 0.0016), L298_RED))
        v.append(box((cx - 0.004, cy, z0 + 0.014), (0.016, 0.023, 0.025), HEATSINK))
        for dy in (-0.017, 0.017):
            v.append(box((cx + 0.016, cy + dy, z0 + 0.006), (0.0075, 0.0085, 0.010), TERMINAL_BLUE))
        v.append(box((cx - 0.017, cy, z0 + 0.006), (0.0075, 0.013, 0.010), TERMINAL_BLUE))
        v.append(cyl((cx + 0.010, cy + 0.006, z0 + 0.007), 0.004, 0.011, BLACK_PLASTIC))
    elif name.startswith("buck"):
        v.append(box((cx, cy, z0 + 0.0008), (sx, sy, 0.0016), PCB_BLUE))
        v.append(box((cx - 0.008, cy, z0 + 0.0055), (0.012, 0.012, 0.008), BLACK_PLASTIC))         # inductor
        v.append(cyl((cx + 0.010, cy + 0.004, z0 + 0.007), 0.004, 0.011, BLACK_PLASTIC))
        v.append(box((cx + 0.014, cy - 0.006, z0 + 0.004), (0.006, 0.0035, 0.0065), TERMINAL_BLUE))  # trimmer
    else:
        v.append(box((cx, cy, z), (sx, sy, h), PCB_BLUE))
        v.append(box((cx, cy, z0 + h + 0.0006), (0.004, 0.004, 0.0012), CHIP))
    return v


# ------------------------------------------------------------------- build
def build():
    polys = chassis_polys()
    vis, dev, bnd = [], [], []

    # decks: black acrylic base, clear acrylic mid + top
    for p in polys:
        vis.append(shape(ACRYLIC_BLACK, prism(p, BASE_Z0, BASE_Z1)))
    vis.append(cyl((0, 0, (MID_Z0 + MID_Z1) / 2), MID_R, MID_Z1 - MID_Z0, ACRYLIC))
    vis.append(cyl((0, 0, (TOP_Z0 + TOP_Z1) / 2), TOP_R, TOP_Z1 - TOP_Z0, ACRYLIC))
    for x, y in BASE_STANDOFFS:
        vis.append(cyl((x, y, (BASE_Z1 + MID_Z0) / 2), 0.0025, MID_Z0 - BASE_Z1, BRASS))
    for x, y in TOP_STANDOFFS:
        vis.append(cyl((x, y, (MID_Z1 + TOP_Z0) / 2), 0.0025, TOP_Z0 - MID_Z1, BRASS))
        vis.append(cyl((x, y, TOP_Z1 + 0.0012), 0.0028, 0.0024, SILVER))

    # rear pocket (printed)
    wz0, wz1 = 0.006, MID_Z0
    wzc, wh = (wz0 + wz1) / 2, wz1 - wz0
    pocket_boxes = pocket_wall_boxes()
    for c, sz, rot in pocket_boxes:
        vis.append(box((c[0], c[1], wzc), (sz[0], sz[1], wh), accent(), rot))

    # drivetrain, caster, line array, battery
    for s in (1, -1):
        vis += n20(s)
    vis.append(cyl((CASTER_X, 0, (2 * CASTER_BALL_R + BASE_Z0) / 2), 0.009, BASE_Z0 - 2 * CASTER_BALL_R + 0.002, accent()))
    vis.append(box((LINE_X, 0, LINE_SENSOR_Z + 0.004), LINE_BOARD, PCB_BLUE))
    for s in (1, -1):
        vis.append(cyl((LINE_X, s * 0.026, (LINE_SENSOR_Z + 0.004 + BASE_Z0) / 2), 0.002,
                       BASE_Z0 - LINE_SENSOR_Z - 0.004, BRASS))
    (bx, by), bs = BATTERY
    vis.append(box((bx, by, BASE_Z1 + bs[2] / 2), bs, LIPO))
    vis.append(box((bx, by, BASE_Z1 + bs[2] / 2), (0.010, bs[1] + 0.002, bs[2] + 0.001), pbr((0.1, 0.1, 0.1), 0.9)))
    vis.append(box((bx - 0.022, by + 0.020, BASE_Z1 + 0.006), (0.010, 0.016, 0.008), YELLOW))     # XT60

    # Sharp brackets (printed), visual only
    for n, (x, y, z, a) in SHARP.items():
        d = SHARP_BODY[0]
        cx, cy = x - (d + 0.003) * math.cos(a), y - (d + 0.003) * math.sin(a)
        vis.append(box((cx, cy, (BASE_Z1 + z) / 2), (0.006, 0.020, z - BASE_Z1), accent(), "0 0 1 %.4f" % a))

    # modules on the decks
    # Mega shield PCB (green) above the Mega
    shz = TOP_Z1 + SHIELD["z"]
    vis.append(box(((SHIELD["x0"] + SHIELD["x1"]) / 2, (SHIELD["y0"] + SHIELD["y1"]) / 2, shz + SHIELD["t"] / 2),
                   (SHIELD["x1"] - SHIELD["x0"], SHIELD["y1"] - SHIELD["y0"], SHIELD["t"]), PCB_GREEN))
    for k in range(9):                                           # rear rows of white JST-XH connectors
        for row, xr in ((0, SHIELD["x0"] + 0.005), (1, SHIELD["x0"] + 0.016)):
            vis.append(box((xr, SHIELD["y1"] - 0.008 - k * 0.0108, shz + SHIELD["t"] + 0.003), (0.0058, 0.0085, 0.006), WHITE_PLASTIC))
    for k in range(6):                                           # front row
        vis.append(box((SHIELD["x1"] - 0.012, SHIELD["y1"] - 0.010 - k * 0.0165, shz + SHIELD["t"] + 0.003), (0.0058, 0.013, 0.006), WHITE_PLASTIC))
    for name, ((cx, cy), (sx, sy, h), deck) in MODULES.items():
        z0 = MID_Z1 if deck == "mid" else (shz + SHIELD["t"]) if deck == "shield" else TOP_Z1
        vis += module_visual(name, cx, cy, sx, sy, h, z0)

    # gripper servos on the mid deck (shaft down through the deck to the arm hubs)
    L, W, H = SERVO_BODY
    for s in (1, -1):
        cx = HINGE_X + L / 2 - 0.007
        vis.append(box((cx, s * HINGE_Y, MID_Z1 + H / 2), (L, W, H), SERVO_BLACK))
        vis.append(box((cx, s * HINGE_Y, MID_Z1 + 0.019), (0.038, W, 0.0025), SERVO_BLACK))
        vis.append(cyl((HINGE_X, s * HINGE_Y, MID_Z0 - 0.002), 0.003, 0.004, WHITE_PLASTIC))

    # buttons and clan LEDs on the top deck
    for k, (x, y) in TOP_BUTTONS.items():
        vis.append(box((x, y, TOP_Z1 + 0.002), (0.012, 0.012, 0.004), BLACK_PLASTIC))
        vis.append(cyl((x, y, TOP_Z1 + 0.0048), 0.0035, 0.0015,
                       pbr((0.85, 0.1, 0.1) if k == "start" else (0.95, 0.85, 0.1), 0.4)))

    # ---- sensors
    for n, (x, y, z, a) in SHARP.items():
        dev.append(sharp21(n, x, y, z, a))
    for n, (x, y, z, a) in US.items():
        dev.append(hcsr04(n, x, y, z, a))
    for i in range(LINE_N):
        dev.append(ground_sensor(i))
    dev.append(tcs3200("tcs_floor", TCS_FLOOR, "0 1 0 1.5708",
                       "TCS3200 #1 facing the floor: maze tiles, red zones, collection pads"))
    dev.append(tcs3200("tcs_front", TCS_FRONT, "0 0 1 0",
                       "TCS3200 #2 facing forward at piece height: piece colour"))
    # ESP32-CAM on its mast (front of the top deck), lens pitched down
    bw, bh, bt = ESP32CAM_BOARD
    vis.append(box((CAM_X - 0.010, 0, TOP_Z1 + 0.0015), (0.022, 0.034, 0.003), accent()))            # mast foot
    vis.append(box((CAM_X - 0.012, 0, (TOP_Z1 + CAM_Z) / 2 + 0.004), (0.004, 0.030, CAM_Z - TOP_Z1 + 0.012), accent()))
    vis.append(box((CAM_X - 0.008, 0, CAM_Z - 0.008), (bt, bw, bh), PCB_BLACK, "0 1 0 %.4f" % CAM_PITCH))
    dev.append("""Camera {
      translation %s
      rotation 0 1 0 %.4f
      name "cam"
      # ESP32-CAM OV2640, cropped and averaged to a %dx%d grid by firmware/esp32cam_vision
      fieldOfView %.3f
      width %d
      height %d
      near 0.01
      children [ %s %s ]
    }""" % (v3((CAM_X, 0, CAM_Z)), CAM_PITCH, CAM_W, CAM_H, CAM_FOV, CAM_W, CAM_H,
            cyl((-0.002, 0, 0), 0.0042, 0.005, BLACK_PLASTIC, ROT_X_AXIS),
            cyl((0.0006, 0, 0), 0.0025, 0.0005, GLASS, ROT_X_AXIS)))
    (ix, iy), _, _ = MODULES["mpu6050"]
    imu_xyz = v3((ix, iy, MID_Z1 + 0.003))
    dev.append('InertialUnit { translation %s name "imu" noise 0.001 }' % imu_xyz)
    dev.append('Gyro { translation %s name "gyro" }' % imu_xyz)
    dev.append('Accelerometer { translation %s name "accel" }' % imu_xyz)
    for n, (x, y) in TOP_LEDS.items():
        col = (0, 0.2, 1) if n == "led_blue" else (0, 1, 0.2)
        dev.append("""LED {
      translation %s
      name "%s"
      color [ %.1f %.1f %.1f ]
      children [ Shape { appearance PBRAppearance { baseColor 0.9 0.9 0.9 roughness 0.2 metalness 0 emissiveColor 0 0 0 } geometry Sphere { radius 0.0028 subdivision 2 } } ]
    }""" % (v3((x, y, TOP_Z1 + 0.0025)), n, *col))
    dev.append(wheel("left"))
    dev.append(wheel("right"))
    dev.append(gripper_arm("left"))
    dev.append(gripper_arm("right"))
    dev.append("""BallJoint {
      jointParameters BallJointParameters { anchor %s }
      endPoint Solid {
        translation %s
        children [ Shape { appearance %s geometry Sphere { radius %.5f subdivision 2 } } ]
        name "caster_ball"
        contactMaterial "caster"
        boundingObject Sphere { radius %.5f }
        physics Physics { density -1 mass 0.008 }
      }
    }""" % (v3((CASTER_X, 0, CASTER_BALL_R)), v3((CASTER_X, 0, CASTER_BALL_R)), STEEL, CASTER_BALL_R, CASTER_BALL_R))
    for s in (1, -1):
        dev.append("""BallJoint {
      jointParameters BallJointParameters { anchor %s }
      endPoint Solid {
        translation %s
        children [ Shape { appearance %s geometry Sphere { radius 0.0035 subdivision 2 } } ]
        name "skid_%s"
        contactMaterial "caster"
        boundingObject Sphere { radius 0.0035 }
        physics Physics { density -1 mass 0.001 }
      }
    }""" % (v3((-0.050, s * 0.050, 0.0050)), v3((-0.050, s * 0.050, 0.0050)), WHITE_PLASTIC, "left" if s > 0 else "right"))

    # ---- collision envelope
    for p in polys:
        bnd.append(prism(p, BASE_Z0, BASE_Z1))
    for c, sz, rot in pocket_boxes:
        bnd.append("Pose { translation %s %s children [ Box { size %s } ] }" % (
            v3((c[0], c[1], 0.038)), ("rotation " + rot) if rot else "", v3((sz[0], sz[1], 0.064))))
    bnd.append("Pose { translation %s children [ Cylinder { radius %.4f height %.4f } ] }" % (
        v3((0, 0, (MID_Z0 + TOP_Z1) / 2)), TOP_R, TOP_Z1 - MID_Z0))
    bnd.append("Pose { translation %s children [ Box { size 0.035 0.074 0.026 } ] }" % v3((BATTERY[0][0], 0, BASE_Z1 + 0.013)))
    bnd.append("Pose { translation %s children [ Box { size 0.060 0.040 0.030 } ] }" % v3((0.010, 0, 0.050)))

    proto = """#VRML_SIM R2025a utf8
# license: Apache License 2.0
# GuardianBot v3 (final-round build) - PeraBots 2026 "Nine Seals of Pera".
# 150 mm round, 3 acrylic decks, rear capture pocket with 2 servo arms.
# Controllers: Arduino Mega 2560 (real-time I/O) + ESP32-S3 (mission brain); L298N; 3S LiPo.
# Sensors: 4 x HC-SR04, 5 x Sharp GP2Y0A21YK0F, 5-ch TCRT5000 line array,
#          2 x TCS3200 (floor + front), ESP32-CAM (cam), MPU6050, N20 hall encoders.
# Generated by simulation/make_guardianbot.py from hardware/cad/guardianbot_geometry.py.
#
# Devices
#   left_wheel_motor right_wheel_motor  left_wheel_sensor right_wheel_sensor
#   us_front us_left us_right us_back            (mm, 20..2500)
#   ir_front ir_fl ir_fr ir_left ir_right         (mm, 100..800)
#   gs0..gs4 line array (gs0 = right-most)        tcs_floor tcs_front (4x4 colour)
#   grip_left grip_right  (0 = open, +-1.05 = closed)   imu gyro accel   led_blue led_green

PROTO GuardianBot [
  field SFVec3f    translation    0 0 0
  field SFRotation rotation       0 0 1 0
  field SFString   name           "GuardianBot"
  field SFString   controller     "main_controller"
  field MFString   controllerArgs []
  field SFString   window         "<generic>"
  field SFBool     supervisor     FALSE
  field SFBool     synchronization TRUE
  field SFColor    accentColor    1 0.45 0.05
  field MFNode     extensionSlot  []
]
{
  Robot {
    translation IS translation
    rotation IS rotation
    name IS name
    controller IS controller
    controllerArgs IS controllerArgs
    window IS window
    supervisor IS supervisor
    synchronization IS synchronization
    contactMaterial "chassis"
    children [
      %s
      %s
      Group { children IS extensionSlot }
    ]
    boundingObject Group { children [ %s ] }
    physics Physics {
      density -1
      mass 0.58
      centerOfMass [ 0.012 0 0.048 ]
    }
  }
}
""" % ("\n      ".join(vis), "\n    ".join(dev), "\n      ".join(bnd))
    return proto


if __name__ == "__main__":
    out = os.path.join(HERE, "protos", "GuardianBot.proto")
    with open(out, "w", newline="\n") as f:
        f.write(build())
    print("wrote", out)

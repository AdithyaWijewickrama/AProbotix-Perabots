"""
make_guardianbot.py - generates protos/GuardianBot.proto

GuardianBot: a buildable 130 mm round differential-drive robot for
PeraBots 2026 (Nine Seals of Pera).  Every part in the model corresponds
to a real component in GuardianBot_BOM.md, with its real size, position
and (approximate) mass.

Frame: origin on the floor directly under the wheel axle centre.
       +x = forward, +y = left, +z = up.  Positive wheel speed = forward.

Run:  python make_guardianbot.py   (writes ../protos/GuardianBot.proto)
"""
import math
import os

R_BODY = 0.065          # chassis radius (130 mm diameter)
WHEEL_R = 0.020         # Pololu 40x7 mm wheel
WHEEL_W = 0.007
TRACK_HALF = 0.0555     # wheel centre-plane distance from centre line
AXLE_Z = WHEEL_R
PLATE_Z0, PLATE_Z1 = 0.0124, 0.0140     # bottom deck: 1.6 mm FR4 PCB
TOP_Z0, TOP_Z1 = 0.0620, 0.0635         # top deck: 1.5 mm carbon-fibre plate
POCKET_X = -0.008       # front face of the rear pocket (back wall inner face)
POCKET_HW = 0.031       # pocket inner half-width (62 mm for 50 mm pieces)
HINGE_X, HINGE_Y = -0.050, 0.0335
ARM_L = 0.032
TOF_R, TOF_Z = 0.057, 0.050
CAM_X, CAM_Z, CAM_PITCH = 0.048, 0.092, 0.4
LINE_X = 0.040
CASTER_X = 0.055
ENC_RES = 2 * math.pi / (12 * 51.45)    # 12 CPR encoder x 51.45:1 gearbox

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


# ---------------------------------------------------------- chassis outline
def arc(a0, a1, n, r=R_BODY):
    return [(r * math.cos(a0 + (a1 - a0) * i / n), r * math.sin(a0 + (a1 - a0) * i / n)) for i in range(n + 1)]


def chassis_polys():
    """Circle minus the rear pocket, split into three convex polygons (CCW)."""
    t0 = math.acos(POCKET_X / R_BODY)                      # ~97 deg
    t1 = math.pi - math.asin(POCKET_HW / R_BODY)          # ~151 deg
    front = arc(-t0, t0, 40)                               # closes along x = POCKET_X
    left = [(POCKET_X, POCKET_HW)] + arc(t0, t1, 12)
    right = [(p[0], -p[1]) for p in reversed(left)]
    return [front, left, right]


def prism(poly, z0, z1):
    n = len(poly)
    pts = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    faces = [list(range(n, 2 * n))]                        # top (CCW from above)
    faces.append(list(reversed(range(n))))                 # bottom
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, n + j, n + i])
    coords = ", ".join(v3(p) for p in pts)
    idx = " ".join(" ".join(map(str, f)) + " -1" for f in faces)
    return "IndexedFaceSet { coord Coordinate { point [ %s ] } coordIndex [ %s ] creaseAngle 0.6 }" % (coords, idx)


# ------------------------------------------------------------------ sensors
def tof(name, x, y, z, a, mount=True):
    """Pololu VL53L4CD carrier (#3692) on a printed bracket."""
    vis = [
        box((-0.0009, 0, 0), (0.0016, 0.0127, 0.0178), PCB_BLUE),
        box((0.0002, 0.0, 0.0030), (0.0010, 0.0048, 0.0024), CHIP),
        sph((0.0005, 0.0010, 0.0030), 0.0005, GLASS),
        sph((0.0005, -0.0010, 0.0030), 0.0005, GLASS),
        box((-0.0009, 0, -0.0074), (0.0017, 0.0110, 0.0016), GOLD),
    ]
    if mount:
        vis.append(box((-0.0035, 0, 0), (0.0035, 0.0150, 0.0200), accent()))
    return """DistanceSensor {
      translation %s
      rotation 0 0 1 %.5f
      name "%s"
      # VL53L4CD: 1 mm resolution, ~1.3 m max range (returns 1300 when nothing is seen)
      lookupTable [ 0 0 0, 0.02 20 0.05, 1.30 1300 0.006 ]
      children [ %s ]
    }""" % (v3((x, y, z)), a, name, " ".join(vis))


def ground_sensor(i):
    y = (i - 3.5) * 0.008
    return """DistanceSensor {
      translation %s
      rotation 0 1 0 1.5708
      name "gs%d"
      type "infra-red"
      # QTRX-MD-08A channel: high on white, low on the 20 mm black line
      lookupTable [ 0 1023 0.01, 0.020 0 0.01 ]
      children [ %s ]
    }""" % (v3((LINE_X, y, PLATE_Z0 - 0.0031)), i,
            box((0, 0, 0.0005), (0.0032, 0.0028, 0.0012), CHIP))


# -------------------------------------------------------------- drivetrain
def wheel(side):
    s = 1 if side == "left" else -1
    y = s * TRACK_HALF
    # dark lightening pockets on the outer face of the white hub
    spokes = " ".join(
        cyl((0.0105 * math.cos(k * 1.2566), 0.0105 * math.sin(k * 1.2566), 0), 0.0032, WHEEL_W + 0.0008, BLACK_PLASTIC)
        for k in range(5))
    return """HingeJoint {
      jointParameters HingeJointParameters { axis 0 1 0 anchor %s }
      device [
        RotationalMotor {
          name "%s_wheel_motor"
          # Pololu 50:1 HPCB micro metal gearmotor: 650 rpm @ 6 V, 0.74 kg.cm stall
          maxVelocity 68
          maxTorque 0.085
        }
        PositionSensor {
          name "%s_wheel_sensor"
          # 12 CPR magnetic encoder x 51.45:1 = 617 counts / wheel rev
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
          %s
        ]
        name "%s_wheel"
        contactMaterial "tire"
        boundingObject Cylinder { radius %.4f height %.4f }
        physics Physics { density -1 mass 0.0045 }
      }
    }""" % (v3((0, y, AXLE_Z)), side, side, ENC_RES, v3((0, y, AXLE_Z)),
            SILICONE, WHEEL_R, WHEEL_W, WHITE_PLASTIC, WHEEL_W + 0.0006,
            spokes, cyl((0, 0, 0), 0.004, WHEEL_W + 0.0012, WHITE_PLASTIC),
            side, WHEEL_R, WHEEL_W)


def gearmotor(s):
    """Micro metal gearmotor + bracket + magnetic encoder, shaft along y."""
    y_out = s * (TRACK_HALF - WHEEL_W / 2 - 0.0020)   # gearbox output face
    parts = []
    gb_c = y_out - s * 0.0045
    can_c = y_out - s * (0.009 + 0.008)
    enc_c = y_out - s * (0.025 + 0.0015)
    parts.append(box((0, gb_c, AXLE_Z), (0.010, 0.009, 0.012), SILVER))
    parts.append(box((0, can_c, AXLE_Z), (0.010, 0.016, 0.012), SILVER))
    parts.append(box((0, can_c, AXLE_Z), (0.0102, 0.010, 0.0122), BLACK_PLASTIC))
    parts.append(box((0, enc_c, AXLE_Z), (0.012, 0.0012, 0.013), PCB_GREEN))
    parts.append(cyl((0, enc_c - s * 0.0015, AXLE_Z), 0.003, 0.002, BLACK_PLASTIC, ROT_Y_AXIS))
    parts.append(cyl((0, y_out + s * 0.0010, AXLE_Z), 0.0015, 0.004, STEEL, ROT_Y_AXIS))
    # bracket (Pololu #989 style) holding the motor down to the bottom PCB
    parts.append(box((0, gb_c, (PLATE_Z1 + AXLE_Z + 0.007) / 2), (0.018, 0.010, AXLE_Z + 0.007 - PLATE_Z1), BLACK_PLASTIC))
    return parts


# ------------------------------------------------------------------ gripper
def gripper_arm(side):
    s = 1 if side == "left" else -1
    hy = s * HINGE_Y
    lim = "minPosition 0 maxPosition 1.3" if s > 0 else "minPosition -1.3 maxPosition 0"
    mult = 1 if s > 0 else -1
    return """HingeJoint {
      jointParameters HingeJointParameters { axis 0 0 1 anchor %s }
      device [
        RotationalMotor {
          # one MG90S servo drives both arms through a push-rod linkage;
          # modelled as two coupled motors (same "gripper::" prefix)
          name "gripper::%s"
          multiplier %d
          %s
          maxVelocity 8
          maxTorque 0.12
        }
        PositionSensor { name "gripper_sensor_%s" }
      ]
      endPoint Solid {
        translation %s
        children [
          %s
          %s
          %s
          %s
        ]
        name "gripper_arm_%s"
        boundingObject Pose { translation %s children [ Box { size %s } ] }
        physics Physics { density -1 mass 0.004 }
      }
    }""" % (v3((HINGE_X, hy, 0.03)), side, mult, lim, side, v3((HINGE_X, hy, 0)),
            box((-ARM_L / 2, 0, 0.024), (ARM_L, 0.003, 0.032), accent(0.6)),
            box((-ARM_L / 2 - 0.002, -s * 0.0020, 0.024), (ARM_L - 0.008, 0.0012, 0.026), TPU),
            cyl((0, 0, 0.024), 0.0040, 0.032, accent(0.6)),
            cyl((0, 0, (0.040 + TOP_Z1) / 2), 0.0015, TOP_Z1 - 0.040 + 0.002, STEEL),
            side, v3((-ARM_L / 2, 0, 0.024)), v3((ARM_L, 0.003, 0.032)))


# ------------------------------------------------------------------- build
def build():
    polys = chassis_polys()
    vis = []

    # bottom deck (custom PCB) with gold silkscreen ring
    for p in polys:
        vis.append(shape(PCB_BLACK, prism(p, PLATE_Z0, PLATE_Z1)))
    # top deck (carbon fibre disc) + orange edge ring
    vis.append(cyl((0, 0, (TOP_Z0 + TOP_Z1) / 2), R_BODY, TOP_Z1 - TOP_Z0, CARBON))   # Webots cylinders are z-axis

    # 4 aluminium M2.5 standoffs between decks
    for x, y in ((0.041, 0.030), (0.041, -0.030), (-0.040, 0.048), (-0.040, -0.048)):
        vis.append(cyl((x, y, (PLATE_Z1 + TOP_Z0) / 2), 0.0022, TOP_Z0 - PLATE_Z1, SILVER))
        vis.append(cyl((x, y, TOP_Z1 + 0.0008), 0.0023, 0.0016, BLACK_PLASTIC))

    # rear pocket: printed PETG back wall + side walls (hold a 50 mm piece)
    wz0, wz1 = 0.006, TOP_Z0
    wzc, wh = (wz0 + wz1) / 2, wz1 - wz0
    vis.append(box((POCKET_X - 0.0015, 0, wzc), (0.003, 2 * (POCKET_HW + 0.003), wh), accent()))
    side_len = POCKET_X - (HINGE_X + 0.004)
    for s in (1, -1):
        vis.append(box(((POCKET_X + HINGE_X + 0.004) / 2, s * (POCKET_HW + 0.0015), wzc),
                       (side_len, 0.003, wh), accent()))

    # drivetrain hardware
    for s in (1, -1):
        vis += gearmotor(s)

    # front ball caster (Pololu 3/8" metal ball) -> separate low-friction Solid below
    vis.append(cyl((CASTER_X, 0, (0.0070 + PLATE_Z0) / 2), 0.0075, PLATE_Z0 - 0.0070, BLACK_PLASTIC))

    # QTRX-MD-08A line array under the front of the bottom deck
    vis.append(box((LINE_X, 0, PLATE_Z0 - 0.0015), (0.020, 0.061, 0.003), PCB_BLACK))
    vis.append(box((LINE_X - 0.008, 0, PLATE_Z0 - 0.0031), (0.003, 0.050, 0.0004), GOLD))

    # 2S 450 mAh LiPo on the bottom deck, ahead of the axle (keeps CoM forward)
    vis.append(box((0.030, 0, PLATE_Z1 + 0.0068), (0.030, 0.055, 0.0135), LIPO))
    vis.append(box((0.030, 0, PLATE_Z1 + 0.0137), (0.012, 0.030, 0.0002), WHITE_PLASTIC))
    vis.append(box((0.010, 0.020, PLATE_Z1 + 0.004), (0.006, 0.006, 0.008), pbr((0.8, 0.7, 0.1), 0.5)))  # XT30

    # ---------------- top deck electronics
    tz = TOP_Z1
    # ESP32-S3-DevKitC-1 (69 x 25.4 mm) across the rear of the top deck
    vis.append(box((-0.010, 0, tz + 0.0060), (0.0254, 0.069, 0.0016), PCB_BLACK))
    vis.append(box((-0.010, 0.018, tz + 0.0083), (0.018, 0.0255, 0.0030), SILVER))
    vis.append(box((-0.010, -0.031, tz + 0.0080), (0.009, 0.0075, 0.0030), SILVER))   # USB-C
    for s in (1, -1):    # header rows / female sockets
        vis.append(box((-0.010 + s * 0.0115, 0, tz + 0.0030), (0.0025, 0.056, 0.0050), BLACK_PLASTIC))
    # BNO085 IMU near the turning centre
    vis.append(box((0.016, 0, tz + 0.0012), (0.018, 0.025, 0.0016), PCB_BLUE))
    vis.append(box((0.016, 0, tz + 0.0026), (0.0052, 0.0038, 0.0011), CHIP))
    # TB6612FNG dual motor driver
    vis.append(box((0.030, 0.026, tz + 0.0012), (0.0152, 0.0203, 0.0016), PCB_BLUE))
    vis.append(box((0.030, 0.026, tz + 0.0025), (0.006, 0.0075, 0.0011), CHIP))
    # 5 V 2.5 A buck regulator
    vis.append(box((0.030, -0.026, tz + 0.0012), (0.0152, 0.0178, 0.0016), PCB_GREEN))
    vis.append(box((0.030, -0.026, tz + 0.0030), (0.005, 0.005, 0.0030), CHIP))
    # MCP23017 I/O expander (drives the 10 ToF XSHUT lines) soldered on the bottom PCB
    vis.append(box((0.002, 0, PLATE_Z1 + 0.0012), (0.0076, 0.018, 0.0024), CHIP))
    # power switch, START and CAL buttons at the rear edge
    vis.append(box((-0.030, 0.026, tz + 0.003), (0.006, 0.012, 0.006), BLACK_PLASTIC))
    vis.append(box((-0.030, 0.026, tz + 0.0065), (0.002, 0.004, 0.003), WHITE_PLASTIC))
    for y, col in ((-0.021, (0.85, 0.1, 0.1)), (-0.032, (0.95, 0.85, 0.1))):   # START (red), CAL (yellow)
        vis.append(box((-0.030, y, tz + 0.0020), (0.0065, 0.0065, 0.0035), BLACK_PLASTIC))
        vis.append(cyl((-0.030, y, tz + 0.0045), 0.0018, 0.0020, pbr(col, 0.4)))
    # MG90S gripper servo standing on the top deck above the pocket + push rods
    vis.append(box((-0.040, 0, tz + 0.0115), (0.0225, 0.0122, 0.0225), SERVO_BLUE))
    vis.append(box((-0.040, 0, tz + 0.0170), (0.032, 0.0122, 0.0025), SERVO_BLUE))
    vis.append(cyl((-0.046, 0, tz + 0.0245), 0.0025, 0.004, WHITE_PLASTIC))
    for s in (1, -1):
        dx, dy = HINGE_X - (-0.046), s * HINGE_Y
        L = math.hypot(dx, dy)
        a = math.atan2(dy, dx)
        vis.append(box(((-0.046 + HINGE_X) / 2, s * HINGE_Y / 2, tz + 0.0030), (L, 0.0012, 0.0012), STEEL,
                       "0 0 1 %.4f" % a))
        vis.append(cyl((HINGE_X, s * HINGE_Y, tz + 0.0025), 0.0045, 0.0030, accent()))

    # XIAO ESP32S3 Sense camera on a printed mast, pitched 23 deg down
    vis.append(box((CAM_X - 0.006, 0, (TOP_Z1 + CAM_Z) / 2 - 0.004), (0.004, 0.018, CAM_Z - TOP_Z1 - 0.004), accent()))

    # anti-tip PTFE skids under the rear lobes (1.5 mm above the floor)
    for s in (1, -1):
        vis.append(cyl((-0.040, s * 0.045, (0.0080 + PLATE_Z0) / 2), 0.0030, PLATE_Z0 - 0.0080, SILVER))

    # ---------------- devices
    dev = []
    sens = [("ds_front", 0.0), ("ds_fl1", 0.4), ("ds_fr1", -0.4), ("ds_fl2", 0.8), ("ds_fr2", -0.8),
            ("ds_fl3", 1.2), ("ds_fr3", -1.2), ("ds_left", math.pi / 2), ("ds_right", -math.pi / 2)]
    sens_table = {}
    for n, a in sens:
        x, y = TOF_R * math.cos(a), TOF_R * math.sin(a)
        dev.append(tof(n, x, y, TOF_Z, a))
        sens_table[n] = (x, y, a)
    # rear ToF inside the pocket: sees walls behind AND confirms a held piece
    dev.append(tof("ds_back", POCKET_X - 0.0005, 0, 0.040, math.pi, mount=False))
    sens_table["ds_back"] = (POCKET_X - 0.0005, 0.0, math.pi)

    for i in range(8):
        dev.append(ground_sensor(i))

    dev.append("""Camera {
      translation %s
      rotation 0 1 0 %.4f
      name "cam"
      # XIAO ESP32S3 Sense (OV3660) streaming a down-sampled 48x32 frame
      fieldOfView 0.8
      width 48
      height 32
      near 0.01
      children [ %s %s %s %s ]
    }""" % (v3((CAM_X, 0, CAM_Z)), CAM_PITCH,
            box((-0.004, 0, 0), (0.0035, 0.0175, 0.021), PCB_BLACK),
            box((-0.0015, 0, 0), (0.0012, 0.008, 0.008), CHIP),
            cyl((0.0008, 0, 0), 0.0030, 0.0035, BLACK_PLASTIC, ROT_X_AXIS),
            cyl((0.0027, 0, 0), 0.0020, 0.0004, GLASS, ROT_X_AXIS)))

    dev.append("""Camera {
      translation %s
      rotation 0 0 1 3.14159
      name "piece_cam"
      # TCS34725 colour sensor on the pocket back wall: identifies the held piece
      fieldOfView 1.0
      width 4
      height 4
      near 0.0005
      children [ %s %s ]
    }""" % (v3((POCKET_X - 0.0005, 0, 0.020)),
            box((-0.0009, 0, 0), (0.0016, 0.020, 0.020), PCB_BLUE),
            sph((0.0002, 0, 0.004), 0.0012, pbr((1, 1, 0.95), 0.3, 0, (0.6, 0.6, 0.5)))))

    dev.append("""InertialUnit {
      translation %s
      name "imu"
      noise 0.0005
    }""" % v3((0.016, 0, TOP_Z1 + 0.003)))
    dev.append('Gyro { translation %s name "gyro" }' % v3((0.016, 0, TOP_Z1 + 0.003)))
    dev.append('Accelerometer { translation %s name "accel" }' % v3((0.016, 0, TOP_Z1 + 0.003)))

    for n, y, col in (("led_blue", 0.016, (0, 0.2, 1)), ("led_green", -0.016, (0, 1, 0.2))):
        dev.append("""LED {
      translation %s
      name "%s"
      color [ %.1f %.1f %.1f ]
      children [ Shape { appearance PBRAppearance { baseColor 0.9 0.9 0.9 roughness 0.2 metalness 0 emissiveColor 0 0 0 } geometry Box { size 0.005 0.005 0.0016 } } ]
    }""" % (v3((0.052, y, TOP_Z1 + 0.0008)), n, *col))

    dev.append(wheel("left"))
    dev.append(wheel("right"))
    dev.append(gripper_arm("left"))
    dev.append(gripper_arm("right"))

    # front ball caster: a real free-rolling 3/8" steel ball on a ball joint
    dev.append("""BallJoint {
      jointParameters BallJointParameters { anchor %s }
      endPoint Solid {
        translation %s
        children [ Shape { appearance %s geometry Sphere { radius 0.00476 subdivision 2 } } ]
        name "caster_ball"
        contactMaterial "caster"
        boundingObject Sphere { radius 0.00476 }
        physics Physics { density -1 mass 0.0035 }
      }
    }""" % (v3((CASTER_X, 0, 0.00476)), v3((CASTER_X, 0, 0.00476)), STEEL))
    # PTFE anti-tip skids (1.5 mm above the floor, only touch if the robot pitches back)
    for s in (1, -1):
        vis.append(sph((-0.040, s * 0.045, 0.0050), 0.0035, WHITE_PLASTIC))

    # ---------------- collision envelope of the body
    bnd = []
    for p in polys:
        bnd.append(prism(p, PLATE_Z0 + 0.001, TOP_Z1))
    bnd.append("Pose { translation %s children [ Box { size %s } ] }" % (
        v3((POCKET_X - 0.0015, 0, 0.034)), v3((0.003, 2 * (POCKET_HW + 0.003), 0.056))))
    for s in (1, -1):
        bnd.append("Pose { translation %s children [ Box { size %s } ] }" % (
            v3(((POCKET_X + HINGE_X + 0.004) / 2, s * (POCKET_HW + 0.0015), 0.034)), v3((side_len, 0.003, 0.056))))
    bnd.append("Pose { translation %s children [ Cylinder { radius %.4f height %.4f } ] }" % (
        v3((0, 0, (TOP_Z0 + TOP_Z1) / 2)), R_BODY, TOP_Z1 - TOP_Z0))
    bnd.append("Pose { translation %s children [ Box { size 0.030 0.020 0.035 } ] }" % v3((CAM_X - 0.004, 0, 0.080)))
    for s in (1, -1):
        bnd.append("Pose { translation %s children [ Sphere { radius 0.0035 } ] }" % v3((-0.040, s * 0.045, 0.0050)))

    proto = """#VRML_SIM R2025a utf8
# license: Apache License 2.0
# GuardianBot - PeraBots 2026 "Nine Seals of Pera" competition robot.
# 130 mm round differential-drive robot with a rear capture pocket and a
# servo-driven two-arm gripper.  Every part matches GuardianBot_BOM.md.
# Generated by make_guardianbot.py - edit the generator, not this file.
#
# Frame: origin on the floor under the axle centre, +x forward, +y left.
# Positive wheel velocity drives forward.
#
# Devices
#   left_wheel_motor / right_wheel_motor  + *_wheel_sensor (617 cpr)
#   ds_front ds_fl1 ds_fr1 ds_fl2 ds_fr2 ds_fl3 ds_fr3 ds_left ds_right ds_back   (VL53L4CD, mm)
#   gs0..gs7      QTRX-MD-08A line array (gs0 = right-most, gs7 = left-most)
#   cam           XIAO ESP32S3 Sense camera, 48x32, pitched 0.4 rad down
#   piece_cam     TCS34725 colour sensor looking at the held piece
#   gripper::left / gripper::right  coupled; setPosition(0)=open, ~1.0=closed
#   imu gyro accel  (BNO085)       led_blue led_green

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
    children [
      %s
      %s
      Group { children IS extensionSlot }
    ]
    boundingObject Group { children [ %s ] }
    physics Physics {
      density -1
      mass 0.215
      centerOfMass [ 0.010 0 0.028 ]
    }
  }
}
""" % ("\n      ".join(vis), "\n    ".join(dev), "\n      ".join(bnd))
    return proto, sens_table


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, "protos", "GuardianBot.proto")
    if not os.path.isdir(os.path.dirname(out)):
        out = os.path.join(here, "..", "protos", "GuardianBot.proto")
    text, table = build()
    with open(out, "w", newline="\n") as f:
        f.write(text)
    print("wrote", os.path.abspath(out))
    print("SENS = {")
    for n, (x, y, a) in table.items():
        print("    %r: (%.4f, %.4f, %.4f)," % (n, x, y, a))
    print("}")

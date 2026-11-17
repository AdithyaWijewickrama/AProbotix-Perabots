"""
make_guardianbot.py - generates protos/GuardianBot.proto  (v2: HC-SR04 + Sharp IR build)

GuardianBot: buildable 130 mm round differential-drive robot for PeraBots 2026.
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

PLATE_Z0, PLATE_Z1 = BASE_Z0, BASE_Z1       # names used by the part helpers
TOP_Z0, TOP_Z1 = PCB_Z0, PCB_Z1
ENC_RES = 2 * math.pi / (12 * 51.45)        # 12 CPR encoder x 51.45:1 gearbox

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
SOLDERMASK = pbr((0.02, 0.18, 0.10), 0.35, 0.1)
SHARP_GREY = pbr((0.12, 0.12, 0.13), 0.7, 0.0)


def sharp(name, x, y, z, a):
    """Sharp GP2Y0A41SK0F, long axis vertical. Lens face at (x,y,z)."""
    d, w, h = SHARP_BODY
    vis = [
        box((-d / 2, 0, 0), (d, w, h - 0.004), SHARP_GREY),
        box((-d / 2, 0, 0), (0.0025, w + 0.0075, 0.004), SHARP_GREY),       # mounting tabs
        box((-0.0012, 0, 0.0075), (0.0026, 0.0095, 0.0095), BLACK_PLASTIC),  # emitter lens
        box((-0.0012, 0, -0.0075), (0.0026, 0.0095, 0.0095), BLACK_PLASTIC), # receiver lens
        box((-d - 0.002, 0, -0.008), (0.004, 0.008, 0.006), WHITE_PLASTIC),  # JST-PH socket
        box((-d / 2 - 0.004, 0, 0), (0.003, 0.018, 0.034), accent()),        # printed bracket
    ]
    return """DistanceSensor {
      translation %s
      rotation 0 0 1 %.5f
      name "%s"
      # Sharp GP2Y0A41SK0F: 4-30 cm. Firmware converts volts->mm. <4 cm reads 40, >30 cm reads 300.
      lookupTable [ 0 40 0, 0.04 40 0.02, 0.30 300 0.03, 0.31 300 0 ]
      children [ %s ]
    }""" % (v3((x, y, z)), a, name, " ".join(vis))


def hcsr04(name, x, y, z, a):
    """HC-SR04: transducer faces at (x,y,z); PCB 12 mm behind."""
    t, w, h = US_PCB
    vis = [box((-0.0128, 0, 0), (t, w, h), PCB_BLUE)]
    for s in (1, -1):
        vis.append(cyl((-0.0060, s * 0.013, 0), 0.0080, 0.0120, SILVER, ROT_X_AXIS))
        vis.append(cyl((0.0001, s * 0.013, 0), 0.0068, 0.0004, pbr((0.25, 0.25, 0.25), 0.9, 0.0), ROT_X_AXIS))
    vis.append(box((-0.0145, 0, -0.0105), (0.0025, 0.010, 0.0025), BLACK_PLASTIC))   # 4-pin header
    vis.append(box((-0.0115, 0, 0.0075), (0.002, 0.008, 0.0035), SILVER))             # crystal
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
          maxTorque 0.06
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
        contactMaterial "chassis"
        boundingObject Pose { translation %s children [ Box { size %s } ] }
        physics Physics { density -1 mass 0.004 }
      }
    }""" % (v3((HINGE_X, hy, 0.03)), side, mult, lim, side, v3((HINGE_X, hy, 0)),
            box((-ARM_L / 2, 0, 0.024), (ARM_L, 0.003, 0.032), accent(0.6)),
            box((-ARM_L / 2 - 0.002, -s * 0.0020, 0.024), (ARM_L - 0.008, 0.0012, 0.026), TPU),
            cyl((0, 0, 0.024), 0.0040, 0.032, accent(0.6)),
            cyl((0, 0, (0.040 + TOP_Z1) / 2), 0.0015, TOP_Z1 - 0.040 + 0.002, STEEL),
            side, v3((-ARM_L / 2, 0, 0.024)), v3((ARM_L, 0.003, 0.032)))



def pcb_parts(tz):
    """Simple 3-D bodies for every footprint on the main board (top side above, bottom side below)."""
    from pcb_design import PARTS
    out = []
    for ref, val, fpid, x, y, rot, side, pinmap, desc in PARTS:
        if ref in ("J1", "J3"):
            continue                                   # DevKit sockets drawn separately
        fp = fpid.split(":")[1]
        n = len(pinmap)
        x, y = x / 1000.0, y / 1000.0
        # footprint +X (KiCad) maps to robot -y at rot 0; rotation is CCW in the KiCad view
        a = math.radians(rot) - math.pi / 2
        if "JST_XH" in fp:
            size, col, h = (0.0025 * n + 0.0024, 0.00575), WHITE_PLASTIC, 0.007
            off = 0.0025 * (n - 1) / 2
        elif "JST_SH" in fp:
            size, col, h, off = (0.006, 0.0042), WHITE_PLASTIC, 0.003, 0.0
        elif "PinHeader" in fp or "PinSocket" in fp:
            size, col, h = (0.00254 * n, 0.00254), BLACK_PLASTIC, 0.0085
            off = 0.00254 * (n - 1) / 2
        elif "SSOP" in fp or "SOIC" in fp:
            size, col, h, off = ((0.0179, 0.0075) if "SOIC" in fp else (0.0082, 0.0053)), CHIP, 0.0018, 0.0
            a += math.pi / 2
        elif "XT30" in fp:
            size, col, h, off = (0.010, 0.0052), pbr((0.85, 0.75, 0.1), 0.5), 0.010, 0.0025
        elif "SW_PUSH" in fp:
            size, col, h, off = (0.006, 0.006), BLACK_PLASTIC, 0.0035, 0.0
        elif "CP_Elec" in fp:
            size, col, h, off = (0.0066, 0.0066), SILVER, 0.0077, 0.0
        elif "SOT-23" in fp:
            size, col, h, off = (0.003, 0.0014), CHIP, 0.0011, 0.0
        else:                                          # 0805 / 1206 passives and LEDs
            size, col, h, off = ((0.0032, 0.0016) if "1206" in fp else (0.002, 0.00125)), \
                (pbr((0.85, 0.85, 0.85), 0.3, 0, (0.2, 0.35, 1.0) if val == "BLUE" else (0.1, 0.9, 0.2))
                 if val in ("BLUE", "GREEN") else CHIP), 0.0008, 0.0
        if side == "B":
            a2 = -a
            cx, cy = x + off * math.cos(a2), y + off * math.sin(a2)
            z = PCB_Z0 - h / 2
            aa = a2
        else:
            cx, cy = x + off * math.cos(a), y + off * math.sin(a)
            z = tz + h / 2
            aa = a
        out.append(box((cx, cy, z), (size[0], size[1], h), col, "0 0 1 %.4f" % aa))
        if "SW_PUSH" in fp:
            out.append(cyl((cx, cy, z + h / 2 + 0.0008), 0.0017, 0.0016,
                           pbr((0.85, 0.1, 0.1) if val == "START" else (0.95, 0.85, 0.1), 0.4)))
    return out


# ------------------------------------------------------------------- build
def pocket_wall_boxes():
    """Back wall + straight side walls + flared lead-ins: [(centre xy, size xy, rotation-or-None)]."""
    t = POCKET_WALL_T
    out = [((POCKET_X - t / 2, 0.0), (t, 2 * (POCKET_IN_HW + t)), None)]
    for s in (1, -1):
        L = POCKET_X - POCKET_FLARE_X
        out.append((((POCKET_X + POCKET_FLARE_X) / 2, s * (POCKET_IN_HW + t / 2)), (L, t), None))
        x0, y0 = POCKET_FLARE_X, s * (POCKET_IN_HW + t / 2)
        x1, y1 = HINGE_X + 0.004, s * (POCKET_MOUTH_HW + t / 2)
        L2 = math.hypot(x1 - x0, y1 - y0)
        ang = math.atan2(y1 - y0, x1 - x0)
        out.append((((x0 + x1) / 2, (y0 + y1) / 2), (L2 + 0.002, t), "0 0 1 %.4f" % ang))
    return out


def build():
    polys = chassis_polys()
    vis, dev, bnd = [], [], []

    # 3 mm acrylic base plate (laser cut, black)
    for p in polys:
        vis.append(shape(ACRYLIC_BLACK, prism(p, BASE_Z0, BASE_Z1)))
    # main PCB = top deck (green soldermask, gold edge ring)
    vis.append(cyl((0, 0, (PCB_Z0 + PCB_Z1) / 2), R_BODY, PCB_Z1 - PCB_Z0, SOLDERMASK))
    # crown: clear 3 mm acrylic disc carrying the 4 ultrasonics
    vis.append(cyl((0, 0, (CROWN_Z0 + CROWN_Z1) / 2), CROWN_R, CROWN_Z1 - CROWN_Z0, ACRYLIC))

    for x, y in BASE_STANDOFFS:
        vis.append(cyl((x, y, (BASE_Z1 + PCB_Z0) / 2), 0.0025, PCB_Z0 - BASE_Z1, BRASS))
    for x, y in CROWN_STANDOFFS:
        vis.append(cyl((x, y, (PCB_Z1 + CROWN_Z0) / 2), 0.0025, CROWN_Z0 - PCB_Z1, BRASS))
        vis.append(cyl((x, y, CROWN_Z1 + 0.0012), 0.0028, 0.0024, SILVER))

    # rear pocket (PETG print: back wall + 2 side walls)
    wz0, wz1 = 0.006, PCB_Z0
    wzc, wh = (wz0 + wz1) / 2, wz1 - wz0
    pocket_boxes = pocket_wall_boxes()
    for c, sz, rot in pocket_boxes:
        vis.append(box((c[0], c[1], wzc), (sz[0], sz[1], wh), accent(), rot))
    # drivetrain
    for s in (1, -1):
        vis += gearmotor(s)
    vis.append(cyl((CASTER_X, 0, (0.0070 + BASE_Z0) / 2), 0.0075, BASE_Z0 - 0.0070, BLACK_PLASTIC))
    # line array
    vis.append(box((LINE_X, 0, BASE_Z0 - 0.0015), LINE_BOARD, PCB_BLACK))
    # battery + strap
    (bx, by), bs = BATTERY
    vis.append(box((bx, by, BASE_Z1 + bs[2] / 2), bs, LIPO))
    vis.append(box((bx, by, BASE_Z1 + bs[2] / 2), (0.010, bs[1] + 0.002, bs[2] + 0.001), pbr((0.1, 0.1, 0.1), 0.9)))
    # anti-tip PTFE skid holders (the low-friction domes are modelled as free balls below)
    for s in (1, -1):
        vis.append(cyl((-0.040, s * 0.045, (0.0075 + BASE_Z0) / 2), 0.0040, BASE_Z0 - 0.0075, WHITE_PLASTIC))

    # ---- PCB population
    tz = PCB_Z1
    (dx, dy), (dsx, dsy) = DEVKIT
    for s in (1, -1):                                   # female headers
        vis.append(box((dx + s * 0.01143, dy, tz + 0.00425), (0.0026, 0.0559, 0.0085), BLACK_PLASTIC))
    vis.append(box((dx, dy, tz + 0.0093), (dsx, dsy, 0.0016), PCB_BLACK))
    vis.append(box((dx, dy + 0.018, tz + 0.0118), (0.018, 0.0255, 0.0032), SILVER))
    for s in (1, -1):
        vis.append(box((dx + s * 0.0065, dy - 0.031, tz + 0.0115), (0.009, 0.0075, 0.0032), SILVER))  # 2 x USB-C
    # every part of the real main board, at its real position (from hardware/cad/pcb_design.py)
    vis += pcb_parts(tz)
    # BNO085 IMU breakout on top of the crown (Qwiic cable to J16)
    (ix, iy), (isx, isy) = BNO085
    vis.append(box((ix, iy, CROWN_Z1 + 0.0010), (isx, isy, 0.0016), PCB_BLUE))
    vis.append(box((ix, iy, CROWN_Z1 + 0.0024), (0.0052, 0.0038, 0.0011), CHIP))
    # gripper servo (MG90S) + push-rods + hinge collars
    sx, sy = SERVO_XY
    vis.append(box((sx, sy, tz + 0.0115), (0.0225, 0.0122, 0.0225), SERVO_BLUE))
    vis.append(box((sx, sy, tz + 0.0170), (0.032, 0.0122, 0.0025), SERVO_BLUE))
    hx, hy = SERVO_HORN
    vis.append(cyl((hx, hy, tz + 0.0245), 0.0025, 0.004, WHITE_PLASTIC))
    for s in (1, -1):
        ddx, ddy = HINGE_X - hx, s * HINGE_Y - hy
        L = math.hypot(ddx, ddy)
        vis.append(box(((hx + HINGE_X) / 2, s * HINGE_Y / 2, tz + 0.0030), (L, 0.0012, 0.0012), STEEL,
                       "0 0 1 %.4f" % math.atan2(ddy, ddx)))
        vis.append(cyl((HINGE_X, s * HINGE_Y, tz + 0.0025), 0.0045, 0.0030, accent()))
    # camera mast on the crown front edge
    vis.append(box((CAM_X - 0.006, 0, (CROWN_Z1 + CAM_Z) / 2 - 0.004), (0.004, 0.018, CAM_Z - CROWN_Z1 - 0.004), accent()))

    # ---- sensors
    for n, (x, y, z, a) in SHARP.items():
        dev.append(sharp(n, x, y, z, a))
    for n, (x, y, z, a) in US.items():
        dev.append(hcsr04(n, x, y, z, a))
    for i in range(8):
        dev.append(ground_sensor(i))
    dev.append("""Camera {
      translation %s
      rotation 0 1 0 %.4f
      name "cam"
      # XIAO ESP32S3 Sense (OV2640/OV3660) down-sampled to 48x32 for tile colour
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
      # TCS34725 colour sensor (with white LED) on the pocket back wall
      fieldOfView 1.0
      width 4
      height 4
      near 0.0005
      children [ %s %s ]
    }""" % (v3((POCKET_X - 0.0005, 0, 0.020)),
            box((-0.0009, 0, 0), (0.0016, 0.020, 0.020), PCB_BLUE),
            sph((0.0002, 0, 0.004), 0.0012, pbr((1, 1, 0.95), 0.3, 0, (0.6, 0.6, 0.5)))))
    (ix, iy), _ = BNO085
    dev.append('InertialUnit { translation %s name "imu" noise 0.0005 }' % v3((ix, iy, CROWN_Z1 + 0.003)))
    dev.append('Gyro { translation %s name "gyro" }' % v3((ix, iy, CROWN_Z1 + 0.003)))
    dev.append('Accelerometer { translation %s name "accel" }' % v3((ix, iy, CROWN_Z1 + 0.003)))
    for n, y, col in (("led_blue", 0.012, (0, 0.2, 1)), ("led_green", -0.012, (0, 1, 0.2))):
        dev.append("""LED {
      translation %s
      name "%s"
      color [ %.1f %.1f %.1f ]
      children [ Shape { appearance PBRAppearance { baseColor 0.9 0.9 0.9 roughness 0.2 metalness 0 emissiveColor 0 0 0 } geometry Box { size 0.005 0.005 0.0016 } } ]
    }""" % (v3((0.057, 0.020 if n == "led_blue" else -0.020, PCB_Z1 + 0.0012)), n, *col))
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
        physics Physics { density -1 mass 0.0035 }
      }
    }""" % (v3((CASTER_X, 0, CASTER_BALL_R)), v3((CASTER_X, 0, CASTER_BALL_R)), STEEL, CASTER_BALL_R, CASTER_BALL_R))

    for s in (1, -1):        # PTFE anti-tip skids: 1.5 mm above the floor, slide with ~no friction
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
    }""" % (v3((-0.040, s * 0.045, 0.0050)), v3((-0.040, s * 0.045, 0.0050)), WHITE_PLASTIC, "left" if s > 0 else "right"))

    # ---- collision envelope
    for p in polys:
        bnd.append(prism(p, BASE_Z0, PCB_Z1))
    for c, sz, rot in pocket_boxes:
        bnd.append("Pose { translation %s %s children [ Box { size %s } ] }" % (
            v3((c[0], c[1], 0.034)), ("rotation " + rot) if rot else "", v3((sz[0], sz[1], 0.056))))
    bnd.append("Pose { translation %s children [ Cylinder { radius %.4f height %.4f } ] }" % (
        v3((0, 0, (PCB_Z0 + PCB_Z1) / 2)), R_BODY, PCB_Z1 - PCB_Z0))
    bnd.append("Pose { translation %s children [ Cylinder { radius %.4f height %.4f } ] }" % (
        v3((0, 0, 0.0835)), 0.0675, 0.034))          # ultrasonic ring + crown
    bnd.append("Pose { translation %s children [ Box { size 0.012 0.020 0.020 } ] }" % v3((CAM_X - 0.004, 0, CAM_Z - 0.002)))


    proto = """#VRML_SIM R2025a utf8
# license: Apache License 2.0
# GuardianBot v2 - PeraBots 2026 "Nine Seals of Pera" competition robot.
# 130 mm round differential drive, rear capture pocket + servo gripper.
# Sensors: 4 x HC-SR04 ultrasonic (crown), 4 x Sharp GP2Y0A41SK0F IR, QTRX-MD-08A line array,
#          XIAO ESP32S3 Sense camera, TCS34725 piece colour, BNO085 IMU, 617 cpr wheel encoders.
# Generated by make_guardianbot.py - edit the generator / hardware/cad/guardianbot_geometry.py, not this file.
#
# Devices
#   left_wheel_motor right_wheel_motor  left_wheel_sensor right_wheel_sensor
#   us_front us_left us_right us_back        (mm, 20..2500)
#   ir_fl ir_fr ir_left ir_right             (mm, 40..300)
#   gs0..gs7  line array (gs0 = right-most)  cam  piece_cam
#   gripper::left (+ coupled gripper::right)  0 = open, 1.05 = closed
#   imu gyro accel   led_blue led_green

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
      mass 0.255
      centerOfMass [ 0.008 0 0.040 ]
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

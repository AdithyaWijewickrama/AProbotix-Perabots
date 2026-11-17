"""
guardianbot_geometry.py - single source of truth for GuardianBot dimensions.

Used by:  make_guardianbot.py (Webots PROTO), cad/make_acrylic_dxf.py,
          cad/make_printed_parts.py, cad/make_pcb.py
All units are METRES in robot frame:
  origin on the floor under the wheel-axle centre, +x forward, +y left, +z up.
"""
import math

# ---------------------------------------------------------------- body
R_BODY = 0.065                 # 130 mm round footprint
WHEEL_R, WHEEL_W = 0.020, 0.007   # Pololu 40x7 mm wheel
TRACK_HALF = 0.0555            # 111 mm track
AXLE_Z = WHEEL_R

# decks
BASE_Z0, BASE_Z1 = 0.0110, 0.0140      # 3 mm acrylic base plate (laser cut)
PCB_Z0, PCB_Z1 = 0.0620, 0.0636        # 1.6 mm main PCB (top deck)
CROWN_Z0, CROWN_Z1 = 0.0985, 0.1015    # 3 mm acrylic crown (ultrasonic ring)
CROWN_R = 0.058

# rear capture pocket + gripper
POCKET_X = -0.008              # back-wall inner face
POCKET_HW = 0.031              # base-plate notch half-width (62 mm)
POCKET_IN_HW = 0.0275          # pocket inner half-width at the back wall (55 mm: a 50 mm cube can't twist)
POCKET_FLARE_X = -0.036        # straight walls end here, then flare out to the arm hinges
POCKET_MOUTH_HW = 0.0315       # half-width at the mouth (63 mm) - funnels the piece in
POCKET_WALL_T = 0.003
HINGE_X, HINGE_Y = -0.050, 0.0335
ARM_L = 0.032
SERVO_XY = (-0.028, 0.0)       # MG90S body centre on the PCB
SERVO_HORN = (-0.034, 0.0)     # output shaft position

# standoffs (x, y)
BASE_STANDOFFS = [(0.052, 0.0), (0.0, 0.0), (-0.047, 0.042), (-0.047, -0.042)]   # base <-> PCB, M3 x 48
CROWN_STANDOFFS = [(0.032, 0.038), (0.032, -0.038), (-0.032, 0.038), (-0.032, -0.038)]  # PCB <-> crown, M3 x 35

# ---------------------------------------------------------------- sensors
# Sharp GP2Y0A41SK0F (4-30 cm), mounted VERTICALLY (long axis = z).
#   body 29.5 (z) x 13.5 (tangential) x 13 (radial); (x, y, z) = centre of the lens face
SHARP = {
    "ir_fl": (0.0424, 0.0424, 0.040, math.pi / 4),
    "ir_fr": (0.0424, -0.0424, 0.040, -math.pi / 4),
    "ir_left": (-0.030, 0.0530, 0.035, math.pi / 2),
    "ir_right": (-0.030, -0.0530, 0.035, -math.pi / 2),
}
SHARP_BODY = (0.013, 0.0135, 0.0295)        # radial, tangential, vertical
SHARP_MIN, SHARP_MAX = 0.04, 0.30

# HC-SR04 ultrasonic, hung under the crown; (x, y, z) = centre of the transducer faces
US = {
    "us_front": (0.067, 0.0, 0.083, 0.0),
    "us_left": (0.0, 0.067, 0.083, math.pi / 2),
    "us_right": (0.0, -0.067, 0.083, -math.pi / 2),
    "us_back": (-0.067, 0.0, 0.083, math.pi),
}
US_PCB = (0.0016, 0.045, 0.020)             # thickness, width, height
US_PCB_R = 0.055                            # radius of the PCB plane
US_MIN, US_MAX = 0.02, 2.50

# QTRX-MD-08A line array under the base
LINE_X = 0.040
LINE_PITCH = 0.008
LINE_BOARD = (0.020, 0.061, 0.003)

# camera (XIAO ESP32S3 Sense) on the crown front edge
CAM_X, CAM_Z, CAM_PITCH = 0.062, 0.112, 0.4

# caster
CASTER_X = 0.055
CASTER_BALL_R = 0.00476

# battery (2S LiPo 450-550 mAh)
BATTERY = ((0.030, 0.0), (0.030, 0.055, 0.0135))   # centre xy, size (x, y, z)

# electronics on the PCB (centre xy, size x, y)
DEVKIT = ((0.005, 0.0), (0.0254, 0.0699))
BNO085 = ((0.022, 0.020), (0.018, 0.025))       # on the crown (top)


def chassis_polys(r=R_BODY, px=POCKET_X, phw=POCKET_HW, n=40):
    """Base-plate outline: circle minus rear pocket, as 3 convex CCW polygons."""
    def arc(a0, a1, k):
        return [(r * math.cos(a0 + (a1 - a0) * i / k), r * math.sin(a0 + (a1 - a0) * i / k)) for i in range(k + 1)]
    t0 = math.acos(px / r)
    t1 = math.pi - math.asin(phw / r)
    front = arc(-t0, t0, n)
    left = [(px, phw)] + arc(t0, t1, 12)
    right = [(p[0], -p[1]) for p in reversed(left)]
    return [front, left, right]


def base_outline(r=R_BODY, px=POCKET_X, phw=POCKET_HW, n=120):
    """Single closed outline (CCW) of the base plate: circle with the rear notch."""
    t1 = math.pi - math.asin(phw / r)           # where y = +phw on the circle (rear-left)
    pts = []
    for i in range(n + 1):                      # from -t1 ... +t1 going through the front
        a = -t1 + 2 * t1 * i / n
        pts.append((r * math.cos(a), r * math.sin(a)))
    pts += [(px, phw), (px, -phw)]
    return pts

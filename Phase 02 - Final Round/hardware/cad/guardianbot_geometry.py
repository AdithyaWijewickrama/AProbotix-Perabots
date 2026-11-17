"""
guardianbot_geometry.py - single source of truth for GuardianBot v3 (final-round build).

Built from the parts the team already has:
  Arduino Mega 2560 (real-time I/O) + ESP32-S3-DevKitC-1 (mission brain), L298N driver,
  2 x N20 12 V encoder gear-motors with 43 mm wheels, 3S 11.1 V LiPo (XT60),
  4 x HC-SR04, 5 x Sharp GP2Y0A21YK0F (10-80 cm), 2 x TCS3200 colour sensors,
  5-channel TCRT5000 line array, MPU6050, 2 x Kpower M0170 metal-gear servos (gripper).

Used by: simulation/make_guardianbot.py (Webots PROTO), make_acrylic_dxf.py, make_printed_parts.py
Units: METRES, robot frame: origin on the floor under the axle centre, +x forward, +y left, +z up.
"""
import math

# ---------------------------------------------------------------- body
R_BODY = 0.075                 # 150 mm round footprint (maze cell is 238 mm wide inside)
WHEEL_R, WHEEL_W = 0.0215, 0.018   # N20 43 mm wheel
TRACK_HALF = 0.062             # 124 mm track
AXLE_Z = WHEEL_R

# decks (laser-cut acrylic); mid deck: L298N, 2 bucks, MPU6050, level shifter, servos
# top deck: Arduino Mega (centre), ESP32-S3 (front strip), HC-SR04 ring underneath, buttons + LEDs (rear)
BASE_Z0, BASE_Z1 = 0.0125, 0.0155      # 3 mm base: motors, caster, line array, TCS3200 floor, LiPo
MID_Z0, MID_Z1 = 0.0700, 0.0730        # 3 mm mid deck
TOP_Z0, TOP_Z1 = 0.1050, 0.1080        # 3 mm top deck
MID_R = 0.072
TOP_R = 0.075

# rear capture pocket + 2-servo gripper (each arm on its own M0170 servo, no linkage)
POCKET_X = -0.008              # back-wall inner face
POCKET_HW = 0.031              # base notch half width
POCKET_IN_HW = 0.0275          # 55 mm throat: a 50 mm cube cannot twist
POCKET_FLARE_X = -0.040
POCKET_MOUTH_HW = 0.0315
POCKET_WALL_T = 0.003
HINGE_X, HINGE_Y = -0.060, 0.0345
ARM_L = 0.032
SERVO_BODY = (0.0285, 0.013, 0.027)    # Kpower M0170 body (L x W x H), output shaft 7 mm from one end

# standoffs (x, y)
BASE_STANDOFFS = [(0.060, 0.030), (0.060, -0.030), (-0.030, 0.055), (-0.030, -0.055)]   # base <-> mid, M3 x 54.5
TOP_STANDOFFS = [(0.040, 0.050), (0.040, -0.050), (-0.040, 0.050), (-0.040, -0.050)]   # mid <-> top, M3 x 32

# ---------------------------------------------------------------- sensors
# Sharp GP2Y0A21YK0F (10-80 cm, blind below 10 cm). Body 44.5 (tangential) x 13.5 (radial) x 13.5 (z).
# The side pair is mounted on both faces of a 5 mm centre plate (printed sharp_side_tower) so a wall
# 11.9 cm from the robot centre
# (robot centred in a maze cell) is still >= 10 cm from the lens.
SHARP = {
    "ir_left": (-0.004, 0.0165, 0.046, math.pi / 2),
    "ir_right": (-0.004, -0.0165, 0.046, -math.pi / 2),
    "ir_front": (0.0400, 0.0, 0.058, 0.0),
    "ir_fl": (0.0389, 0.0389, 0.058, math.pi / 4),
    "ir_fr": (0.0389, -0.0389, 0.058, -math.pi / 4),
}
SHARP_BODY = (0.0135, 0.0445, 0.0135)        # radial, tangential, vertical
SHARP_MIN, SHARP_MAX = 0.10, 0.80

# HC-SR04 hung under the top deck; (x, y, z, heading) of the transducer faces
US = {
    "us_front": (0.075, 0.0, 0.092, 0.0),
    "us_left": (0.0, 0.075, 0.092, math.pi / 2),
    "us_right": (0.0, -0.075, 0.092, -math.pi / 2),
    "us_back": (-0.075, 0.0, 0.092, math.pi),
}
US_PCB = (0.0016, 0.045, 0.020)
US_MIN, US_MAX = 0.02, 2.50

# 5-channel TCRT5000 line array under the base
LINE_X = 0.045
LINE_N = 5
LINE_PITCH = 0.012
LINE_BOARD = (0.020, 0.064, 0.0016)
LINE_SENSOR_Z = 0.007                         # sensor face height above the floor

# TCS3200 colour sensors
TCS_FLOOR = (0.012, 0.030, 0.008)            # facing down, under the base: tiles / red zones / pads
TCS_FRONT = (0.071, 0.0, 0.028)              # facing forward at game-piece height: piece colour

# caster (metal ball caster under the front)
CASTER_X = 0.060
CASTER_BALL_R = 0.0063

# LiPo 3S 1300-1550 mAh, XT60
BATTERY = ((0.038, 0.0), (0.035, 0.074, 0.026))

# module footprints on the decks: name -> ((x, y), (size x, size y, height), deck)
MODULES = {
    "mega": ((0.000, 0.000), (0.0533, 0.1016, 0.015), "top"),        # Arduino Mega 2560 (long axis along y)
    "esp32s3": ((0.045, 0.0), (0.0254, 0.069, 0.012), "top"),        # ESP32-S3-DevKitC-1 (2 x USB-C)
    "l298n": ((-0.020, 0.0), (0.043, 0.043, 0.027), "mid"),          # L298N with heat sink
    "buck5": ((0.030, 0.025), (0.043, 0.021, 0.014), "mid"),         # LM2596 -> 5.1 V logic + sensors
    "buck6": ((0.030, -0.025), (0.043, 0.021, 0.014), "mid"),        # LM2596 -> 6.0 V servos
    "mpu6050": ((0.000, 0.048), (0.016, 0.021, 0.003), "mid"),
    "level": ((0.000, -0.048), (0.013, 0.016, 0.003), "mid"),        # 5 V <-> 3.3 V level shifter (UART)
}
TOP_BUTTONS = {"start": (-0.055, 0.020), "cal": (-0.055, -0.020)}
TOP_LEDS = {"led_blue": (-0.045, 0.036), "led_green": (-0.045, -0.036)}


def chassis_polys(r=R_BODY, px=POCKET_X, phw=POCKET_HW, n=40):
    """Base outline (circle minus rear pocket) as 3 convex CCW polygons (for the Webots model)."""
    def arc(a0, a1, k):
        return [(r * math.cos(a0 + (a1 - a0) * i / k), r * math.sin(a0 + (a1 - a0) * i / k)) for i in range(k + 1)]
    t0 = math.acos(px / r)
    t1 = math.pi - math.asin(phw / r)
    front = arc(-t0, t0, n)
    left = [(px, phw)] + arc(t0, t1, 12)
    right = [(p[0], -p[1]) for p in reversed(left)]
    return [front, left, right]

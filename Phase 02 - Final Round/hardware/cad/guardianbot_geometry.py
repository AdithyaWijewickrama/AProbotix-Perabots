"""
guardianbot_geometry.py - single source of truth for GuardianBot v4.0 (final-round build).

Built from the team's parts list (Nov 17):
  Arduino Mega 2560 (real-time I/O, mounted upside-down on the PCB mid deck)
  + ESP32-CAM (brain AND camera: dual-core ESP32 + OV2640) on a front mast,
  DRV8833 driver, 2 x N20 6 V 300 rpm encoder gear-motors with 43 mm wheels, 2S 7.4 V 1500 mAh LiPo,
  8 x VL53L0X ToF (GY-VL53L0XV2), 2 x HC-SR04, 2 x TCS3200, Pololu QTR-8RC line array,
  GY-50 L3G4200D gyro, 2 x Kpower M0170 servos (gripper), OLED 128x32 + 1x4 keypad.

Used by: simulation/make_guardianbot.py (Webots PROTO), make_acrylic_dxf.py, make_printed_parts.py
Units: METRES, robot frame: origin on the floor under the axle centre, +x forward, +y left, +z up.
"""
import math

# ---------------------------------------------------------------- body
R_BODY = 0.075                 # 150 mm round footprint (maze cell is 238 mm wide inside)
WHEEL_R, WHEEL_W = 0.0215, 0.018   # N20 43 mm wheel
TRACK_HALF = 0.062             # 124 mm track
AXLE_Z = WHEEL_R

# decks: acrylic base (motors, caster, QTR-8RC, floor TCS3200, 2S LiPo, side ToF);
# PCB mid deck (all wiring, Mega upside-down on top, DRV8833 + gyro, bucks underneath, gripper servos);
# acrylic top deck (camera mast, ESP32-S3 carrier, OLED, keypad, LEDs; HC-SR04 pair and back ToF underneath)
BASE_Z0, BASE_Z1 = 0.0125, 0.0155      # 3 mm acrylic base: motors, caster, line array, TCS3200 floor, LiPo
MID_Z0, MID_Z1 = 0.0700, 0.0716        # 1.6 mm PCB mid deck (all wiring; Mega upside-down on top of it)
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
TOP_STANDOFFS = [(0.040, 0.050), (0.040, -0.050), (-0.040, 0.050), (-0.040, -0.050)]   # PCB deck <-> top, M3 x 33.4

# ---------------------------------------------------------------- sensors
# VL53L0X time-of-flight (GY-VL53L0XV2, 25 deg cone, 30 mm .. ~1.2 m). (x, y, z, heading) of the sensor face.
# Two per side (66 mm apart) so the robot can measure its angle to a maze wall and straighten itself.
# Side sensors at z 40 mm also see the 50 mm game pieces; the front three hang under the PCB deck,
# the back one hangs under the top deck (a 50 mm piece held in the pocket must not hide the wall behind).
TOF = {
    "tof_front": (0.068, 0.0, 0.060, 0.0),
    "tof_fl": (0.050, 0.046, 0.060, math.pi / 4),
    "tof_fr": (0.050, -0.046, 0.060, -math.pi / 4),
    "tof_lf": (0.030, 0.064, 0.040, math.pi / 2),
    "tof_lr": (-0.036, 0.064, 0.040, math.pi / 2),
    "tof_rf": (0.030, -0.064, 0.040, -math.pi / 2),
    "tof_rr": (-0.036, -0.064, 0.040, -math.pi / 2),
    "tof_back": (-0.066, 0.030, 0.090, math.pi),        # under the top deck, beside the back sonar (sees over a held piece)
}
TOF_BOARD = (0.0025, 0.0107, 0.025)          # thickness, width, height of the GY-VL53L0XV2 board (upright)
TOF_MIN, TOF_MAX = 0.03, 1.20
SIDE_PAIR_DX = 0.066                         # distance between the two side sensors (wall-angle fix)

# HC-SR04 hung under the top deck; (x, y, z, heading) of the transducer faces
US = {                                          # 2 x HC-SR04 for long range (Section 1 obstacle field)
    "us_front": (0.075, 0.0, 0.092, 0.0),
    "us_back": (-0.075, 0.0, 0.092, math.pi),
}
US_PCB = (0.0016, 0.045, 0.020)
US_MIN, US_MAX = 0.02, 2.50

# Pololu QTR-8RC reflectance array under the base (8 sensors, 9.525 mm pitch, board 75 x 12.7 mm)
LINE_X = 0.045
LINE_N = 8
LINE_PITCH = 0.009525
LINE_BOARD = (0.0127, 0.075, 0.0016)
LINE_SENSOR_Z = 0.005                         # QTR works best 3-6 mm above the floor

# TCS3200 colour sensors
TCS_FLOOR = (0.012, 0.030, 0.008)            # facing down, under the base: tiles / red zones / pads
TCS_FRONT = (0.071, 0.0, 0.028)              # facing forward at game-piece height: piece colour

# caster (metal ball caster under the front)
CASTER_X = 0.060
CASTER_BALL_R = 0.0063

# ESP32-CAM vision board (OV2640) on a printed mast at the front of the top deck.
# CAM = lens position (x, z) and the downward pitch (rad). CAM_FOV = horizontal field of view
# after the ESP32-CAM crops its 160x120 frame to a 48x32 RGB565 grid (see firmware/esp32cam_brain, camera task).
CAM_X, CAM_Z, CAM_PITCH = 0.062, 0.140, 0.50
CAM_FOV = 0.90
CAM_W, CAM_H = 48, 32
ESP32CAM_BOARD = (0.027, 0.0405, 0.0045)     # (v4.0) width (y), height (z), PCB thickness
# v4.1: OV7670 module (34 x 34 mm board, M12 lens holder) on the mast. The ESP32-S3 N16R8 DevKitC-1
# (63 x 25.5 mm) lies FLAT on a 30 x 70 mm vero carrier on the top deck, just behind the mast: the board
# plugs face-up into female headers (pins hidden, solder underneath), USB port towards the +y edge.
# A 10 cm 18-wire female-female ribbon goes from the carrier up to the OV7670.
# The maze code needs CAM_FOV = 0.90 rad (52 deg) of picture: the lens must be at least that wide
# (stock OV7670 lenses vary, some are only ~25 deg; an M12 wide-angle lens fixes that).
OV7670_BOARD = (0.034, 0.034, 0.0016)         # width (y), height (z), PCB thickness
OV7670_LENS = (0.014, 0.014, 0.016)           # lens holder + lens (y, z, length)
S3_BOARD = (0.0255, 0.063, 0.0016)            # ESP32-S3-DevKitC-1 N16R8: width (x), length (y), thickness
S3_CARRIER = (0.0055, 0.0, 0.030, 0.070)      # vero carrier on the top deck: centre x, y, size x, y
S3_CARRIER_HOLES = [(0.0055, 0.0325), (0.0055, -0.0325)]   # 2 x M3 on the centre line (under the S3's ends)
S3_STANDOFF = 0.006                           # carrier sits on 6 mm nylon spacers (room for the solder joints)
S3_HEADER_H = 0.0085                          # female header height: S3 board bottom above the carrier
CAM_NOISE = 0.05                              # OV7670: noisier and less colour-accurate than the OV2640

# LiPo 2S 7.4 V 1500 mAh (XT60)
BATTERY = ((0.038, 0.0), (0.034, 0.072, 0.019))

# module footprints on the decks: name -> ((x, y), (size x, size y, height), deck)
MODULES = {
    "mega": ((0.000, 0.000), (0.0533, 0.1016, 0.015), "mid_flip"),  # Arduino Mega, upside-down on the PCB deck
    "drv8833": ((0.048, 0.036), (0.0127, 0.0203, 0.003), "mid"),     # Pololu DRV8833 carrier (wired to J16/J17)
    "gyro": ((0.049, -0.022), (0.016, 0.022, 0.003), "mid"),         # GY-50 L3G4200D (foam tape, near the axle line)
    "buck5": ((0.036, 0.022), (0.043, 0.021, 0.014), "under_r"),     # LM2596 -> 5.0 V, stuck under the PCB deck
    "buck6": ((0.036, -0.022), (0.043, 0.021, 0.014), "under_r"),    # HW-319 -> 5.5 V servos, under the PCB deck
    "oled": ((-0.030, 0.012), (0.038, 0.012, 0.004), "top"),           # 128x32 I2C OLED
    "keypad": ((-0.050, -0.022), (0.020, 0.040, 0.001), "top"),      # 1x4 membrane keypad
}
TOP_BUTTONS = {}                              # v4: the 1x4 keypad replaces START / CAL
SERVO_TOP_HOLES = [(-0.046, 0.0345), (-0.046, -0.0345)]   # servo cradle caps screw up into the top deck
TOP_LEDS = {"led_blue": (-0.030, 0.058), "led_green": (-0.030, -0.058)}
MEGA_GAP = 0.011                              # Mega PCB sits 11 mm above the deck (long male headers)


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

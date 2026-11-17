"""
deck_design.py - GuardianBot v4.0 PCB mid deck: parts, positions and every net.

The PCB IS the middle floor of the robot (150 mm round family, R 73 mm, rear pocket notch, servo
notches). The Arduino Mega plugs in UPSIDE-DOWN on long male headers on the top side (11 mm gap);
almost every connector is on the BOTTOM side, because the sensors, motors, bucks and DRV8833 all
live in the space between the base and this deck. Top side: OLED, keypad, LEDs, ESP32-CAM link,
the two HC-SR04, the back ToF and the servos.

All parts are through-hole (easy to solder, local PCB shop friendly).
Coordinates are ROBOT millimetres seen from the top: X forward, Y left, origin = wheel-axle centre.
Pin functions match firmware/mega_body/pins.h and firmware/esp32cam_brain/config.h.
"""
import math

FP = "/usr/share/kicad/footprints"      # Windows: r"C:\\Program Files\\KiCad\\7.0\\share\\kicad\\footprints"

R_DECK = 72.0                                        # = MID_R in guardianbot_geometry.py
# The capture pocket is only in the base: a 50 mm piece fits under this deck (z 70 mm), so the deck is a full disc.
HUBS = [(-60.0, 34.5), (-60.0, -34.5)]               # M0170 output shafts pass down through the deck to the arms
HUB_D = 8.0
M3 = 3.2
STANDOFFS = [(60.0, 30.0), (60.0, -30.0), (-30.0, 55.0), (-30.0, -55.0),   # base <-> deck
             (40.0, 50.0), (40.0, -50.0), (-40.0, 50.0), (-40.0, -50.0)]   # deck <-> top
SERVO_HOLES = []                     # v4: the servo cradles screw up into the TOP deck (no holes in the PCB)
MEGA_HOLES_UV = [(13.97, 2.54), (15.24, 50.8), (90.17, 50.8), (96.52, 2.54)]  # 4 corner holes: M3 x 11 mm spacers
# Mega parts that hang down into the 11 mm gap (the Mega is upside-down): keep the deck free (cut-outs)
CUTOUTS = [(-18.5, 40.0, -3.5, 62.0),     # under the Mega USB-B (also leaves room for the plug)
           (12.5, 38.5, 25.0, 58.0)]      # under the DC jack and the big capacitors next to it

# ---------------------------------------------------------------- Mega pads (upside-down)
# Arduino Mega 2560 R3 header geometry in board coordinates (u along the board from the USB end,
# v across from the POWER/ANALOG edge), after github.com/Alarm-Siren/arduino-kicad-library (CC BY-SA 4.0).
# The Mega is flipped face-down about its long axis, so robot X = 26.67 - v and Y = 50.8 - u:
# POWER/ANALOG header at the front (X = +24.1), digital header at the rear (X = -24.1), USB end on the left.
MEGA_UV = {"NC": (27.94, 2.54), "IORF": (30.48, 2.54), "RST1": (33.02, 2.54), "3V3": (35.56, 2.54),
           "5V1": (38.1, 2.54), "GND2": (40.64, 2.54), "GND3": (43.18, 2.54), "VIN": (45.72, 2.54)}
for i in range(8):
    MEGA_UV["A%d" % i] = (50.8 + 2.54 * i, 2.54)
    MEGA_UV["A%d" % (8 + i)] = (73.66 + 2.54 * i, 2.54)
MEGA_UV.update({"SCL": (18.796, 50.8), "SDA": (21.336, 50.8), "AREF": (23.876, 50.8), "GND1": (26.416, 50.8)})
for k, n in enumerate(range(13, 7, -1)):
    MEGA_UV["D%d" % n] = (28.956 + 2.54 * k, 50.8)
for k, n in enumerate(range(7, -1, -1)):
    MEGA_UV["D%d" % n] = (45.72 + 2.54 * k, 50.8)
for k, n in enumerate(range(14, 22)):
    MEGA_UV["D%d" % n] = (68.58 + 2.54 * k, 50.8)
MEGA_UV.update({"5V3": (93.98, 50.8), "5V4": (96.52, 50.8), "GND5": (93.98, 7.62), "GND6": (96.52, 7.62)})
for k in range(16):
    MEGA_UV["D%d" % (22 + 2 * k)] = (93.98, 48.26 - 2.54 * k)
    MEGA_UV["D%d" % (23 + 2 * k)] = (96.52, 48.26 - 2.54 * k)
MEGA = {n: (26.67 - v, 50.8 - u) for n, (u, v) in MEGA_UV.items()}

MEGA_NET = {
    "5V1": "+5V", "5V3": "+5V", "5V4": "+5V",
    "GND1": "GND", "GND2": "GND", "GND3": "GND", "GND5": "GND", "GND6": "GND",
    "A0": "VBAT_DIV", "A1": "LED_B", "A2": "LED_G",
    "D2": "ENC_LA", "D3": "ENC_LB", "D18": "ENC_RA", "D19": "ENC_RB",
    "D4": "DRV_SLP", "D5": "AIN1", "D6": "AIN2", "D7": "BIN1", "D8": "BIN2",
    "D9": "BUZ", "D10": "ECHO_F", "D11": "ECHO_B", "D39": "TRIG_F", "D40": "TRIG_B",
    "D16": "MEGA_TX2", "D17": "MEGA_RX2", "D20": "SDA", "D21": "SCL",
    "D22": "QTR1", "D23": "QTR2", "D24": "QTR3", "D25": "QTR4", "D26": "QTR5", "D27": "QTR6",
    "D28": "QTR7", "D29": "QTR8", "D38": "QTR_LED",
    "D30": "XS_F", "D31": "XS_FL", "D32": "XS_FR", "D33": "XS_LF", "D34": "XS_LR", "D35": "XS_RF",
    "D36": "XS_RR", "D37": "XS_B",
    "D41": "TCS_S2", "D42": "TCS_S3", "D43": "TCS_LED_FLOOR", "D44": "TCS_LED_FRONT",
    "D50": "TCS_OUT_FLOOR", "D51": "TCS_OUT_FRONT",
    "D45": "SERVO_L", "D46": "SERVO_R",
    "D47": "KEY1", "D48": "KEY2", "D49": "KEY3", "D53": "KEY4",
}


def pins(*nets):
    return {str(i + 1): n for i, n in enumerate(nets)}


XH = "Connector_JST:JST_XH_B%dB-XH-A_1x%02d_P2.50mm_Vertical"
HDR = "Connector_PinHeader_2.54mm:PinHeader_1x%02d_P2.54mm_Vertical"
R_AX = "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal"
TO92 = "Package_TO_SOT_THT:TO-92_Inline_Wide"
DISC = "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm"


def tof(xs):
    return pins("+5V", "GND", "SDA", "SCL", xs)


MEGA_HOLES = [(26.67 - v, 50.8 - u) for u, v in MEGA_HOLES_UV]
HOLES = [(x, y, M3) for x, y in STANDOFFS + SERVO_HOLES + MEGA_HOLES]


# Connector rows. Every row runs along -Y (robot left -> right); a part's length along the row:
def _len(fpid):
    n = int(fpid.split("1x")[1][:2])
    return (n - 1) * (2.5 if "JST" in fpid else 2.54) + (4.9 if "JST" in fpid else 2.54)


def row(x, y_start, side, items, gap=3.0):  # noqa: C901
    """items: (ref, value, footprint, {pad: net}, note) -> PARTS entries with pad 1 at the left end."""
    out, y = [], y_start
    for ref, val, fpid, pm, note in items:
        L = _len(fpid)
        m = 2.45 if "JST" in fpid else 1.27
        out.append((ref, val, fpid, x, y - m, "-Y", side, pm, note))
        y -= L + gap
    if y + gap < -R_DECK:
        raise SystemExit("row at X=%g too long" % x)
    return out


# (ref, value, footprint, X, Y of pad 1, direction pad1 -> pad2, side 'T'/'B', {pad: net}, note)
PARTS = []
# Rows are ordered so each connector sits close to the Mega pins it uses: the 2x18 digital header
# (D22..D53) is on the robot's RIGHT, the analog header at the FRONT, D0..D21 at the REAR.
# ===== BOTTOM side, under the Mega: everything that goes down to the base =====
PARTS += row(17.0, 36.5, "B", [
    ("J6", "TOF_F", XH % (5, 5), tof("XS_F"), "VL53L0X: VIN GND SDA SCL XSHUT"),
    ("J11", "TOF_FL", XH % (5, 5), tof("XS_FL"), "VL53L0X: VIN GND SDA SCL XSHUT"),
    ("J2", "TCS_FLOOR", XH % (8, 8), pins("+5V", "GND", "+5V", "GND", "TCS_S2", "TCS_S3", "TCS_OUT_FLOOR", "TCS_LED_FLOOR"),
     "TCS3200: VCC GND S0 S1 S2 S3 OUT LED"),
], gap=4.0)
PARTS += row(5.5, 50.0, "B", [
    ("J12", "TOF_FR", XH % (5, 5), tof("XS_FR"), "VL53L0X: VIN GND SDA SCL XSHUT"),
    ("J3", "TCS_FRONT", XH % (8, 8), pins("+5V", "GND", "+5V", "GND", "TCS_S2", "TCS_S3", "TCS_OUT_FRONT", "TCS_LED_FRONT"),
     "TCS3200: VCC GND S0 S1 S2 S3 OUT LED"),
], gap=24.0)
PARTS += row(-6.0, 37.0, "B", [
    ("J7", "TOF_LF", XH % (5, 5), tof("XS_LF"), "VL53L0X: VIN GND SDA SCL XSHUT"),
    ("J8", "TOF_LR", XH % (5, 5), tof("XS_LR"), "VL53L0X: VIN GND SDA SCL XSHUT"),
    ("J9", "TOF_RF", XH % (5, 5), tof("XS_RF"), "VL53L0X: VIN GND SDA SCL XSHUT"),
    ("J10", "TOF_RR", XH % (5, 5), tof("XS_RR"), "VL53L0X: VIN GND SDA SCL XSHUT"),
])
PARTS += row(-17.0, 31.0, "B", [
    ("J4", "MOTOR_L", XH % (6, 6), pins("MA1", "MA2", "GND", "+5V", "ENC_LA", "ENC_LB"), "N20 encoder motor: M1 M2 GND VCC A B"),
    ("J5", "MOTOR_R", XH % (6, 6), pins("MB1", "MB2", "GND", "+5V", "ENC_RA", "ENC_RB"), "N20 encoder motor: M1 M2 GND VCC A B"),
], gap=4.0)
PARTS += row(-17.0, -9.0, "B", [
    ("J1", "QTR-8RC", XH % (11, 11), pins("+5V", "GND", "QTR_LED", "QTR1", "QTR2", "QTR3", "QTR4", "QTR5", "QTR6", "QTR7", "QTR8"),
     "VCC GND LEDON 1..8 (sensor 1 = right-most)"),
])
# ===== BOTTOM side, front: power (the two bucks are stuck under the deck here) + small parts =====
PARTS += row(56.0, 26.0, "B", [
    ("J13", "BAT_IN", XH % (2, 2), pins("VBAT", "GND"), "from the power switch (2S LiPo 7.4 V)"),
    ("J14", "BUCK_5V", XH % (4, 4), pins("VBAT", "GND", "+5V", "GND"), "LM2596 set to 5.0 V: IN+ IN- OUT+ OUT-"),
    ("J15", "BUCK_SERVO", XH % (4, 4), pins("VBAT", "GND", "+5V5", "GND"), "HW-319 set to 5.5 V: IN+ IN- OUT+ OUT-"),
])
PARTS += [
    ("C1", "470uF 16V", "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", 46.5, -12.0, "-Y", "B", pins("+5V", "GND"), "5 V bulk"),
    ("C2", "470uF 16V", "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", 46.5, -26.0, "-Y", "B", pins("+5V5", "GND"), "servo bulk"),
    ("C3", "100uF 16V", "Capacitor_THT:CP_Radial_D6.3mm_P2.50mm", 56.0, -16.0, "-Y", "B", pins("VBAT", "GND"), "battery"),
]
for k, (ref, val, a, b, note) in enumerate([
        ("R1", "20k", "VBAT", "VBAT_DIV", "battery divider top"),
        ("R2", "10k", "VBAT_DIV", "GND", "battery divider bottom (A0 = VBAT / 3)"),
        ("R7", "330", "LED_B", "LED_BA", "blue LED"),
        ("R8", "330", "LED_G", "LED_GA", "green LED")]):
    PARTS.append((ref, val, R_AX, 39.0, 7.0 - 3.6 * k, "+X", "B", pins(a, b), note))
PARTS.append(("C4", "100nF", DISC, 49.5, 7.0, "+X", "B", pins("VBAT_DIV", "GND"), "ADC filter"))
# ===== TOP side, front: user interface and the front sonar =====
PARTS += row(33.0, 50.0, "T", [
    ("J20", "OLED", XH % (4, 4), pins("GND", "+5V", "SCL", "SDA"), "SSD1306 128x32: GND VCC SCL SDA"),
    ("J22", "GYRO", XH % (4, 4), pins("+5V", "GND", "SCL", "SDA"), "GY-50 L3G4200D: VCC GND SCL SDA"),
    ("J19", "US_FRONT", XH % (4, 4), pins("+5V", "TRIG_F", "ECHO_F", "GND"), "HC-SR04: VCC TRIG ECHO GND"),
    ("J25", "LEDS", XH % (3, 3), pins("LED_BA", "LED_GA", "GND"), "blue LED +, green LED +, common GND"),
    ("J21", "KEYPAD", XH % (5, 5), pins("GND", "KEY1", "KEY2", "KEY3", "KEY4"), "1x4 membrane keypad: COM K1 K2 K3 K4"),
])
# ===== TOP side, rear (behind the Mega, between the gripper servos) =====
PARTS += [
    # DRV8833 wiring headers right behind D4..D8 (the carrier sits on foam tape behind them)
    ("J16", "DRV_CTRL", HDR % 6, -29.5, 9.14, "-Y", "T", pins("BIN2", "BIN1", "AIN2", "AIN1", "DRV_SLP", "GND"),
     "wire to the DRV8833 carrier: BIN2 BIN1 AIN2 AIN1 SLP GND"),
    ("J17", "DRV_PWR", HDR % 6, -33.3, 9.14, "-Y", "T", pins("VBAT", "GND", "MA1", "MA2", "MB1", "MB2"),
     "wire to the DRV8833 carrier: VIN GND AOUT1 AOUT2 BOUT1 BOUT2"),
    # buzzer driver next to D9
    ("R9", "1k", R_AX, -29.5, 14.5, "-X", "T", pins("BUZ", "BUZ_B"), "buzzer transistor base"),
    ("Q2", "BC547", TO92, -41.0, 24.0, "-Y", "T", pins("BUZ_K", "BUZ_B", "GND"), "buzzer driver (C B E)"),
    ("J28", "BUZZER", XH % (2, 2), -47.0, 23.5, "-Y", "T", pins("+5V", "BUZ_K"), "5 V active buzzer: + -"),
    # link circuit next to TX2/RX2 (D16/D17)
    ("R3", "1.8k", R_AX, -29.5, -10.0, "-X", "T", pins("MEGA_TX2", "ESP_RX"), "Mega TX2 -> ESP RX divider top"),
    ("R4", "3.3k", R_AX, -29.5, -13.6, "-X", "T", pins("ESP_RX", "GND"), "divider bottom: 5 V -> 3.2 V"),
    ("R5", "10k", R_AX, -29.5, -17.2, "-X", "T", pins("ESP_TX", "+3V3"), "level shifter pull-up, 3.3 V side"),
    ("R6", "10k", R_AX, -29.5, -20.8, "-X", "T", pins("MEGA_RX2", "+5V"), "level shifter pull-up, 5 V side"),
    ("Q1", "2N7000", TO92, -41.0, -9.0, "-Y", "T", pins("ESP_TX", "+3V3", "MEGA_RX2"), "level shifter ESP TX -> Mega RX2 (S G D)"),
    ("C5", "100nF", DISC, -41.0, -18.0, "-Y", "T", pins("+3V3", "GND"), "3.3 V decoupling at the shifter"),
    ("J18", "ESP32CAM", XH % (5, 5), -47.0, -8.5, "-Y", "T", pins("+5V", "GND", "ESP_RX", "ESP_TX", "+3V3"),
     "to the ESP32-CAM on the mast: 5V GND U0R(GPIO15 RX) GPIO14(TX) 3V3 (3.3 V comes from the CAM board)"),
    ("J26", "SERVO_L", HDR % 3, -31.0, 30.0, "+Y", "T", pins("SERVO_L", "+5V5", "GND"), "M0170 servo: S + -"),
    ("J27", "SERVO_R", HDR % 3, -31.0, -30.0, "-Y", "T", pins("SERVO_R", "+5V5", "GND"), "M0170 servo: S + -"),
]
PARTS += row(-54.0, 23.5, "T", [
    ("J23", "US_BACK", XH % (4, 4), pins("+5V", "TRIG_B", "ECHO_B", "GND"), "HC-SR04: VCC TRIG ECHO GND"),
    ("J24", "TOF_BACK", XH % (5, 5), tof("XS_B"), "VL53L0X: VIN GND SDA SCL XSHUT"),
])

POWER_W = {"GND": 0.6, "+5V": 0.6, "+5V5": 0.8, "VBAT": 0.8, "MA1": 0.6, "MA2": 0.6, "MB1": 0.6, "MB2": 0.6,
           "+3V3": 0.4}
SIGNAL_W = 0.3
CLEARANCE = 0.25
VIA = (1.0, 0.6)


def outline():
    """Deck outline as a shapely polygon (robot mm): disc minus cut-outs, with the holes as interiors."""
    from shapely.geometry import Point, box
    p = Point(0, 0).buffer(R_DECK, 256)
    for x0, y0, x1, y1 in CUTOUTS:
        p = p.difference(box(x0, y0, x1, y1))
    for hx, hy in HUBS:
        p = p.difference(Point(hx, hy).buffer(HUB_D / 2, 64))
    for hx, hy, d in HOLES:
        p = p.difference(Point(hx, hy).buffer(d / 2, 32))
    return p

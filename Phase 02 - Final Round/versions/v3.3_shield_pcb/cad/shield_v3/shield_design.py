"""
shield_design.py - GuardianBot v3 "Mega shield": parts, positions and every net.

The shield plugs onto the Arduino Mega 2560 (top deck). It carries the ESP32-S3-DevKitC-1 on two
22-pin sockets, the 3.3 V <-> 5 V link circuit (2N7000 + divider, no level-shifter module needed),
the battery divider, buzzer driver, bulk/decoupling capacitors and a keyed JST-XH connector for
every sensor, so nothing on the robot is a loose Dupont jumper.

All parts are through-hole: easy to solder, and a Sri Lankan PCB shop can make it
(2 layers, 0.3 mm tracks, 0.25 mm clearance, 1.0/0.6 mm vias, 1.0 mm pin holes).

Coordinates are SHIELD mm: u along the Mega (0 = USB / power-jack end, 101.6 = far end),
v across the Mega (0 = rear edge with the POWER/ANALOG headers, 53.34 = front edge with the
digital header). The shield extends 26 mm behind the Mega (v = -26) for two connector rows, 3.7 mm in
front of it and 5.4 mm past its far end (u = 107) so tracks can leave the 2x18 header from both sides.
Mega pad positions are the Arduino Mega 2560 R3 shield geometry
(after github.com/Alarm-Siren/arduino-kicad-library, CC BY-SA 4.0).
Pin functions match firmware/mega_body/pins.h and firmware/esp32s3_brain/config.h.
"""

FP = "/usr/share/kicad/footprints"      # Windows: r"C:\\Program Files\\KiCad\\7.0\\share\\kicad\\footprints"

U_MIN, U_MAX, V_MIN, V_MAX = 0.0, 107.0, -26.0, 57.0
REAR_RIGHT_CHAMFER = 12.0               # keeps the corner inside the 150 mm robot circle

# ---------------------------------------------------------------- Mega header pads (u, v)
MEGA = {"NC": (27.94, 2.54), "IORF": (30.48, 2.54), "RST1": (33.02, 2.54), "3V3": (35.56, 2.54),
        "5V1": (38.1, 2.54), "GND2": (40.64, 2.54), "GND3": (43.18, 2.54), "VIN": (45.72, 2.54)}
for i in range(8):
    MEGA["A%d" % i] = (50.8 + 2.54 * i, 2.54)
    MEGA["A%d" % (8 + i)] = (73.66 + 2.54 * i, 2.54)
MEGA.update({"SCL": (18.796, 50.8), "SDA": (21.336, 50.8), "AREF": (23.876, 50.8), "GND1": (26.416, 50.8)})
for k, n in enumerate(range(13, 7, -1)):
    MEGA["D%d" % n] = (28.956 + 2.54 * k, 50.8)
for k, n in enumerate(range(7, -1, -1)):
    MEGA["D%d" % n] = (45.72 + 2.54 * k, 50.8)
for k, n in enumerate(range(14, 22)):
    MEGA["D%d" % n] = (68.58 + 2.54 * k, 50.8)
MEGA.update({"5V3": (93.98, 50.8), "5V4": (96.52, 50.8), "GND5": (93.98, 7.62), "GND6": (96.52, 7.62)})
for k in range(16):
    MEGA["D%d" % (22 + 2 * k)] = (93.98, 48.26 - 2.54 * k)
    MEGA["D%d" % (23 + 2 * k)] = (96.52, 48.26 - 2.54 * k)
MEGA_HOLES = [(13.97, 2.54), (15.24, 50.8), (66.04, 7.62), (66.04, 35.56), (90.17, 50.8), (96.52, 2.54)]

# Mega pin -> net (only what the robot uses; the rest of the pins are plugged in but not connected)
MEGA_NET = {
    "5V1": "+5V", "5V3": "+5V", "5V4": "+5V",
    "GND1": "GND", "GND2": "GND", "GND3": "GND", "GND5": "GND", "GND6": "GND",
    "A0": "IR_F", "A1": "IR_FL", "A2": "IR_FR", "A3": "IR_L", "A4": "IR_R", "A5": "VBAT_DIV",
    "A8": "GS0", "A9": "GS1", "A10": "GS2", "A11": "GS3", "A12": "GS4",
    "D2": "ENC_LA", "D3": "ENC_LB", "D18": "ENC_RA", "D19": "ENC_RB",
    "D5": "ENA", "D6": "ENB", "D22": "IN1", "D23": "IN2", "D24": "IN3", "D25": "IN4",
    "D7": "SERVO_L", "D8": "SERVO_R", "D9": "BUZ",
    "D10": "ECHO_F", "D11": "ECHO_L", "D12": "ECHO_R", "D13": "ECHO_B",
    "D30": "TRIG_F", "D31": "TRIG_L", "D32": "TRIG_R", "D33": "TRIG_B",
    "D16": "MEGA_TX2", "D17": "MEGA_RX2", "D20": "SDA", "D21": "SCL",
    "D34": "TCS_S2", "D35": "TCS_S3", "D38": "TCS_LED_FLOOR", "D39": "TCS_LED_FRONT",
    "D50": "TCS_OUT_FLOOR", "D51": "TCS_OUT_FRONT",
    "D46": "BTN_START", "D47": "BTN_CAL", "D48": "LED_B", "D49": "LED_G",
}

# ---------------------------------------------------------------- ESP32-S3-DevKitC-1 sockets
# J1 runs along u at v = 17.0 (pin 1 at u = 30.0), J3 at v = 39.86. Check your board's silkscreen:
# J1 = 3V3 3V3 RST IO4 IO5 IO6 IO7 IO15 IO16 IO17 IO18 IO8 IO3 IO46 IO9 IO10 IO11 IO12 IO13 IO14 5V GND
ESP_U0, ESP_V1, ESP_V3 = 30.0, 17.0, 39.86
ESP_J1 = {1: "+3V3", 2: "+3V3", 8: "CAM_TX", 9: "CAM_RX", 10: "ESP_TX", 11: "ESP_RX", 21: "+5V", 22: "GND"}
ESP_J3 = {1: "GND", 21: "GND", 22: "GND"}
# socket segments actually fitted (1x2 + 1x4 + 1x2 on J1, 1x3 + 1x3 on J3): the DevKit still sits on 14 pins
ESP_SOCKET_PINS = {"U1": {1, 2, 8, 9, 10, 11, 21, 22}, "U2": {1, 2, 3, 20, 21, 22}}


def pins(*nets):
    return {str(i + 1): n for i, n in enumerate(nets)}


XH = "Connector_JST:JST_XH_B%dB-XH-A_1x%02d_P2.50mm_Vertical"
R_AX = "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal"
TERM = "TerminalBlock_Phoenix:TerminalBlock_Phoenix_MKDS-1,5-2-5.08_1x02_P5.08mm_Horizontal"

# (ref, value, footprint, u, v, direction of pin 1 -> pin 2 ('+u', '-u', '+v', '-v'), {pad: net}, description)
PARTS = [
    # ---- rear row A (v = -21): ultrasonics, Sharp supply capacitors, buzzer
    ("J1", "US_FRONT", XH % (4, 4), 3.0, -21.0, "+u", pins("+5V", "TRIG_F", "ECHO_F", "GND"), "HC-SR04 front"),
    ("J2", "US_LEFT", XH % (4, 4), 16.5, -21.0, "+u", pins("+5V", "TRIG_L", "ECHO_L", "GND"), "HC-SR04 left"),
    ("J3", "US_RIGHT", XH % (4, 4), 30.0, -21.0, "+u", pins("+5V", "TRIG_R", "ECHO_R", "GND"), "HC-SR04 right"),
    ("J4", "US_BACK", XH % (4, 4), 43.5, -21.0, "+u", pins("+5V", "TRIG_B", "ECHO_B", "GND"), "HC-SR04 back"),
] + [
    ("C%d" % (1 + k), "10uF", "Capacitor_THT:CP_Radial_D5.0mm_P2.00mm", 57.0 + 6.0 * k, -21.0, "+u",
     pins("+5V", "GND"), "Sharp supply decoupling")
    for k in range(5)
] + [
    ("J25", "BUZZER", XH % (2, 2), 88.0, -21.0, "+u", pins("+5V", "BUZ_K"), "5 V active buzzer: + -"),
    # ---- rear row B (v = -10): Sharps, encoders, ESP32-CAM
    ("J5", "IR_FRONT", XH % (3, 3), 3.0, -10.0, "+u", pins("IR_F", "GND", "+5V"), "Sharp GP2Y0A21 front: Vo GND VCC"),
    ("J6", "IR_FL", XH % (3, 3), 14.0, -10.0, "+u", pins("IR_FL", "GND", "+5V"), "Sharp front-left"),
    ("J7", "IR_FR", XH % (3, 3), 25.0, -10.0, "+u", pins("IR_FR", "GND", "+5V"), "Sharp front-right"),
    ("J8", "IR_LEFT", XH % (3, 3), 36.0, -10.0, "+u", pins("IR_L", "GND", "+5V"), "Sharp left"),
    ("J9", "IR_RIGHT", XH % (3, 3), 47.0, -10.0, "+u", pins("IR_R", "GND", "+5V"), "Sharp right"),
    ("J15", "ENC_L", XH % (4, 4), 41.0, 45.5, "+u", pins("+5V", "GND", "ENC_LA", "ENC_LB"), "left N20 encoder: VCC GND A B"),
    ("J16", "ENC_R", XH % (4, 4), 78.0, 45.5, "+u", pins("+5V", "GND", "ENC_RA", "ENC_RB"), "right N20 encoder"),
    ("J17", "ESP32CAM", XH % (4, 4), 79.0, -10.0, "+u", pins("+5V", "GND", "CAM_TX", "CAM_RX"),
     "ESP32-CAM: 5V GND U0TXD U0RXD"),
    # ---- front row (v = 45): colour sensors, line array, L298N
    ("J10", "TCS_FLOOR", XH % (8, 8), 31.0, 10.0, "+u",
     pins("+5V", "GND", "+5V", "GND", "TCS_S2", "TCS_S3", "TCS_OUT_FLOOR", "TCS_LED_FLOOR"),
     "TCS3200 floor: VCC GND S0 S1 S2 S3 OUT LED"),
    ("J11", "TCS_FRONT", XH % (8, 8), 71.0, 10.0, "+u",
     pins("+5V", "GND", "+5V", "GND", "TCS_S2", "TCS_S3", "TCS_OUT_FRONT", "TCS_LED_FRONT"),
     "TCS3200 front: VCC GND S0 S1 S2 S3 OUT LED"),
    ("J12", "LINE", XH % (7, 7), 58.0, -10.0, "+u", pins("+5V", "GND", "GS0", "GS1", "GS2", "GS3", "GS4"),
     "5-ch TCRT5000: VCC GND OUT1(right-most)..OUT5"),
    ("J13", "L298N", XH % (8, 8), 54.5, 45.5, "+u", pins("ENA", "IN1", "IN2", "IN3", "IN4", "ENB", "+5V", "GND"),
     "L298N logic: ENA IN1 IN2 IN3 IN4 ENB +5V GND (remove its 5 V jumper)"),
    # ---- middle row (v = 10, between the POWER/ANALOG header and the ESP32 sockets)
    ("J18", "BUTTONS", XH % (3, 3), 16.5, 45.5, "+u", pins("BTN_START", "BTN_CAL", "GND"), "START, CAL (to GND)"),
    ("J19", "LEDS", XH % (3, 3), 27.5, 45.5, "+u", pins("LED_BA", "LED_GA", "GND"), "clan LEDs: blue, green anodes"),
    ("J14", "MPU6050", XH % (4, 4), 3.0, 45.5, "+u", pins("+5V", "GND", "SCL", "SDA"), "MPU6050: VCC GND SCL SDA"),
    # ---- left end: power inputs, servos, bulk capacitors, buzzer driver
    ("J20", "5V_IN", TERM, 6.0, 6.0, "+v", pins("+5V", "GND"), "from LM2596 #1 (5.1 V)"),
    ("J21", "6V_IN", TERM, 6.0, 17.5, "+v", pins("+6V", "GND"), "from LM2596 #2 (6.0 V, servos)"),
    ("J22", "VBAT", TERM, 6.0, 29.0, "+v", pins("VBAT", "GND"), "switched battery + (sense only)"),
    ("J23", "SERVO_L", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", 14.5, 6.6, "+v",
     pins("SERVO_L", "+6V", "GND"), "left gripper servo: S + -"),
    ("J24", "SERVO_R", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", 18.5, 6.6, "+v",
     pins("SERVO_R", "+6V", "GND"), "right gripper servo: S + -"),
    ("C6", "470uF 16V", "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", 15.0, 19.5, "+u", pins("+5V", "GND"), "5 V bulk"),
    ("C7", "470uF 16V", "Capacitor_THT:CP_Radial_D10.0mm_P5.00mm", 15.0, 31.0, "+u", pins("+6V", "GND"), "servo bulk"),
    ("Q2", "BC547", "Package_TO_SOT_THT:TO-92_Inline_Wide", 25.5, 21.0, "+v", pins("BUZ_K", "BUZ_B", "GND"),
     "buzzer driver (C B E)"),
    ("R1", "1k", R_AX, 13.0, 39.5, "+u", pins("BUZ", "BUZ_B"), "buzzer base"),
    # ---- under the ESP32-S3 (low parts only)
    ("R2", "20k", R_AX, 33.0, 21.0, "+u", pins("VBAT", "VBAT_DIV"), "battery divider top"),
    ("R3", "10k", R_AX, 33.0, 24.5, "+u", pins("VBAT_DIV", "GND"), "battery divider bottom"),
    ("C8", "100nF", "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm", 44.0, 24.5, "+u", pins("VBAT_DIV", "GND"),
     "ADC filter"),
    ("R4", "1.8k", R_AX, 33.0, 28.0, "+u", pins("MEGA_TX2", "ESP_RX"), "Mega TX2 -> ESP RX divider (top)"),
    ("R5", "3.3k", R_AX, 33.0, 31.5, "+u", pins("ESP_RX", "GND"), "divider (bottom): 5 V -> 3.2 V"),
    ("Q1", "2N7000", "Package_TO_SOT_THT:TO-92_Inline_Wide", 44.0, 34.0, "+u", pins("ESP_TX", "+3V3", "MEGA_RX2"),
     "ESP TX -> Mega RX2 level shifter (S G D)"),
    ("R6", "10k", R_AX, 56.0, 21.0, "+u", pins("ESP_TX", "+3V3"), "shifter pull-up 3.3 V side"),
    ("R7", "10k", R_AX, 56.0, 24.5, "+u", pins("MEGA_RX2", "+5V"), "shifter pull-up 5 V side"),
    ("R8", "330", R_AX, 56.0, 28.0, "+u", pins("LED_B", "LED_BA"), "blue LED"),
    ("R9", "330", R_AX, 56.0, 31.5, "+u", pins("LED_G", "LED_GA"), "green LED"),
    ("C9", "100nF", "Capacitor_THT:C_Disc_D3.0mm_W1.6mm_P2.50mm", 70.0, 21.0, "+u", pins("+5V", "GND"),
     "ESP32 5 V decoupling"),
    ("C10", "100uF 10V", "Capacitor_THT:CP_Radial_D5.0mm_P2.00mm", 70.0, 26.0, "+u", pins("+5V", "GND"),
     "ESP32-CAM supply bursts"),
]

POWER_W = {"GND": 0.6, "+5V": 0.6, "+6V": 0.8, "VBAT": 0.4, "+3V3": 0.4}
SIGNAL_W = 0.3
CLEARANCE = 0.25
VIA = (1.0, 0.6)                    # diameter, drill (large: easy for local shops, can be pinned by hand)

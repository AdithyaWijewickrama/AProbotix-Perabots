"""
pcb_design.py - GuardianBot main board: parts, footprints, placement and every net.
Shared by make_pcb.py (KiCad board + autorouting + Gerbers) and make_schematic.py.

Board: 130 mm round, 2 layers, 1.6 mm, top deck of the robot (component side up).
Coordinates below are ROBOT coordinates in mm (x = forward, y = left), seen from the top.
In KiCad:  X = -y,  Y = -x   (front of the robot is at the top of the board drawing).
"""

FP = "/usr/share/kicad/footprints"   # Windows: r"C:\Program Files\KiCad\7.0\share\kicad\footprints"

# --------------------------------------------------------------------------- ESP32-S3-DevKitC-1 pinout
# J1 (left header, pin 1 at the antenna end) and J3 (right header).  Verify against your board's silkscreen.
J1 = ["3V3", "3V3", "EN", "IO4", "IO5", "IO6", "IO7", "IO15", "IO16", "IO17", "IO18", "IO8", "IO3", "IO46",
      "IO9", "IO10", "IO11", "IO12", "IO13", "IO14", "5V", "GND"]
J3 = ["GND", "IO43", "IO44", "IO1", "IO2", "IO42", "IO41", "IO40", "IO39", "IO38", "IO37", "IO36", "IO35",
      "IO0", "IO45", "IO48", "IO47", "IO21", "IO20", "IO19", "GND", "GND"]

# GPIO -> net function
GPIO = {
    "IO1": "LINE1", "IO2": "LINE2", "IO4": "LINE3", "IO5": "LINE4", "IO6": "LINE5", "IO7": "LINE6",
    "IO8": "LINE7", "IO9": "LINE8",
    "IO10": "IR_FL", "IO11": "IR_L", "IO12": "IR_R", "IO13": "IR_FR",
    "IO14": "VBAT_SENSE",
    "IO15": "PWMA", "IO16": "PWMB",
    "IO17": "ENC_LA", "IO18": "ENC_LB", "IO21": "ENC_RA", "IO47": "ENC_RB",
    "IO40": "ECHO_F", "IO41": "ECHO_L", "IO42": "ECHO_R", "IO39": "ECHO_B",
    "IO3": "SDA", "IO46": "SCL",
    "IO48": "SERVO",
    "3V3": "+3V3", "5V": "+5V", "GND": "GND",
}


def devkit_nets(header):
    return {str(i + 1): GPIO.get(p, "") for i, p in enumerate(header)}


# MCP23017 (SOIC-28) pin -> net
MCP = {1: "AIN1", 2: "AIN2", 3: "BIN1", 4: "BIN2", 5: "LED_B", 6: "LED_G", 7: "", 8: "",
       9: "+3V3", 10: "GND", 11: "", 12: "SCL", 13: "SDA", 14: "", 15: "GND", 16: "GND", 17: "GND",
       18: "MCP_RST", 19: "", 20: "",
       21: "TRIG_F", 22: "TRIG_L", 23: "TRIG_R", 24: "TRIG_B", 25: "LINE_CTRL", 26: "BTN_START", 27: "BTN_CAL",
       28: "STBY"}

# TB6612FNG (SSOP-24) pin -> net
TB = {1: "MOT_L1", 2: "MOT_L1", 3: "GND", 4: "GND", 5: "MOT_L2", 6: "MOT_L2", 7: "MOT_R2", 8: "MOT_R2",
      9: "GND", 10: "GND", 11: "MOT_R1", 12: "MOT_R1", 13: "VBAT", 14: "VBAT", 15: "PWMB", 16: "BIN2",
      17: "BIN1", 18: "GND", 19: "STBY", 20: "+3V3", 21: "AIN1", 22: "AIN2", 23: "PWMA", 24: "VBAT"}


def pins(*nets):
    return {str(i + 1): n for i, n in enumerate(nets)}


# (ref, value, footprint, x, y, kicad_rot, side, {pad: net}, description)
PARTS = [
    # ---- the two DevKit sockets (pin 1 at robot y = +26.67, rows at robot x = -6.43 / +16.43)
    ("J1", "DevKitC J1", "Connector_PinSocket_2.54mm:PinSocket_1x22_P2.54mm_Vertical", -6.43, 26.67, 90, "F",
     devkit_nets(J1), "ESP32-S3-DevKitC-1 N16R8, left header (female socket)"),
    ("J3", "DevKitC J3", "Connector_PinSocket_2.54mm:PinSocket_1x22_P2.54mm_Vertical", 16.43, 26.67, 90, "F",
     devkit_nets(J3), "ESP32-S3-DevKitC-1 N16R8, right header (female socket)"),
    # ---- motor driver
    ("U2", "TB6612FNG", "Package_SO:SSOP-24_5.3x8.2mm_P0.65mm", 36, 22, 270, "F",
     {str(k): v for k, v in TB.items()}, "Dual H-bridge 1.2 A / 3.2 A peak"),
    ("C1", "10uF 25V", "Capacitor_SMD:C_1206_3216Metric", 27, 28, 90, "F", pins("VBAT", "GND"), "TB6612 VM"),
    ("C2", "100nF", "Capacitor_SMD:C_0805_2012Metric", 27, 22, 90, "F", pins("VBAT", "GND"), "TB6612 VM"),
    ("C3", "100nF", "Capacitor_SMD:C_0805_2012Metric", 30, 17, 0, "F", pins("+3V3", "GND"), "TB6612 VCC"),
    # ---- power input: XT30 -> external switch -> reverse-polarity P-MOSFET -> VBAT
    ("J20", "XT30 battery", "Connector_AMASS:AMASS_XT30UPB-M_1x02_P5.0mm_Vertical", 48, 31, 90, "B",
     pins("BAT_RAW", "GND"), "2S LiPo input (bottom side)"),
    ("J21", "Power switch", "Connector_JST:JST_XH_B2B-XH-A_1x02_P2.50mm_Vertical", 49, 18, 90, "B",
     pins("BAT_RAW", "BAT_SW"), "to a 3 A panel/slide switch"),
    ("Q1", "AO3401A", "Package_TO_SOT_SMD:SOT-23", 42, 36, 0, "F", pins("GND_G1", "VBAT", "BAT_SW"),
     "P-MOSFET reverse-battery protection (G, S, D)"),
    ("R1", "10k", "Resistor_SMD:R_0805_2012Metric", 48, 40, 90, "F", pins("GND_G1", "GND"), "Q1 gate pull-down"),
    ("C4", "220uF 16V", "Capacitor_SMD:CP_Elec_6.3x7.7", 40, 44, 0, "F", pins("VBAT", "GND"), "VBAT bulk"),
    ("U4", "Pololu D24V22F5", "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical", 46, -19, 90, "F",
     pins("", "", "VBAT", "GND", "+5V"), "5 V 2.5 A buck module: PG, EN, VIN, GND, VOUT (check silkscreen)"),
    ("C5", "47uF 10V", "Capacitor_SMD:C_1206_3216Metric", 24, -30, 0, "F", pins("+5V", "GND"), "5 V bulk"),
    ("R2", "100k", "Resistor_SMD:R_0805_2012Metric", -12, -15, 90, "F", pins("VBAT", "VBAT_SENSE"), "VBAT divider"),
    ("R3", "33k", "Resistor_SMD:R_0805_2012Metric", -12, -20, 90, "F", pins("VBAT_SENSE", "GND"), "VBAT divider"),
    ("C6", "100nF", "Capacitor_SMD:C_0805_2012Metric", -16, -18, 90, "F", pins("VBAT_SENSE", "GND"), "ADC filter"),
    # ---- I/O expander
    ("U3", "MCP23017-E/SO", "Package_SO:SOIC-28W_7.5x17.9mm_P1.27mm", 2, -38, 90, "F",
     {str(k): v for k, v in MCP.items()}, "16-bit I2C I/O expander @0x20"),
    ("C7", "100nF", "Capacitor_SMD:C_0805_2012Metric", 13, -33, 90, "F", pins("+3V3", "GND"), "MCP decoupling"),
    ("R4", "10k", "Resistor_SMD:R_0805_2012Metric", 13, -38, 90, "F", pins("MCP_RST", "+3V3"), "MCP reset pull-up"),
    ("R5", "4.7k", "Resistor_SMD:R_0805_2012Metric", -11, 20, 0, "F", pins("SDA", "+3V3"), "I2C pull-up"),
    ("R6", "4.7k", "Resistor_SMD:R_0805_2012Metric", -11, 24, 0, "F", pins("SCL", "+3V3"), "I2C pull-up"),
    # ---- HC-SR04 connectors (VCC, TRIG, ECHO, GND) + echo dividers 10k/20k
    ("J4", "US front", "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", 45, 13, 90, "F",
     pins("+5V", "TRIG_F", "ECHO_F5", "GND"), "HC-SR04 front"),
    ("J5", "US left", "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", -24, 46, 90, "F",
     pins("+5V", "TRIG_L", "ECHO_L5", "GND"), "HC-SR04 left"),
    ("J6", "US right", "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", -24, -46, 90, "F",
     pins("+5V", "TRIG_R", "ECHO_R5", "GND"), "HC-SR04 right"),
    ("J7", "US back", "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", -54, 4, 270, "F",
     pins("+5V", "TRIG_B", "ECHO_B5", "GND"), "HC-SR04 back"),
] + [
    (r, v, "Resistor_SMD:R_0805_2012Metric", x, y, 90, "F", pins(a, b), d)
    for r, v, x, y, a, b, d in [
        ("R7", "10k", 22, -1, "ECHO_F5", "ECHO_F", "echo divider top"),
        ("R8", "20k", 22, -4, "ECHO_F", "GND", "echo divider bottom"),
        ("R9", "10k", 22, -7, "ECHO_L5", "ECHO_L", "echo divider top"),
        ("R10", "20k", 22, -10, "ECHO_L", "GND", "echo divider bottom"),
        ("R11", "10k", 22, -13, "ECHO_R5", "ECHO_R", "echo divider top"),
        ("R12", "20k", 22, -16, "ECHO_R", "GND", "echo divider bottom"),
        ("R13", "10k", 22, -19, "ECHO_B5", "ECHO_B", "echo divider top"),
        ("R14", "20k", 22, -22, "ECHO_B", "GND", "echo divider bottom"),
    ]
] + [
    # ---- Sharp GP2Y0A41SK0F connectors (Vo, GND, Vcc) on the bottom side + 10 uF each
    ("J8", "IR front-left", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", 30, 47, 90, "B",
     pins("IR_FL", "GND", "+5V"), "Sharp GP2Y0A41 front-left"),
    ("J9", "IR front-right", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", 38, -45, 90, "B",
     pins("IR_FR", "GND", "+5V"), "Sharp GP2Y0A41 front-right"),
    ("J10", "IR left", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", -34, 47, 90, "B",
     pins("IR_L", "GND", "+5V"), "Sharp GP2Y0A41 left"),
    ("J11", "IR right", "Connector_JST:JST_XH_B3B-XH-A_1x03_P2.50mm_Vertical", -34, -47, 90, "B",
     pins("IR_R", "GND", "+5V"), "Sharp GP2Y0A41 right"),
    ("C8", "10uF", "Capacitor_SMD:C_0805_2012Metric", 38, 45, 0, "B", pins("+5V", "GND"), "Sharp supply"),
    ("C9", "10uF", "Capacitor_SMD:C_0805_2012Metric", 26, -45, 0, "B", pins("+5V", "GND"), "Sharp supply"),
    ("C10", "10uF", "Capacitor_SMD:C_0805_2012Metric", -26, 41, 0, "B", pins("+5V", "GND"), "Sharp supply"),
    ("C11", "10uF", "Capacitor_SMD:C_0805_2012Metric", -26, -41, 0, "B", pins("+5V", "GND"), "Sharp supply"),
    # ---- motors + encoders (M1, M2, VCC, GND, A, B) bottom side
    ("J12", "Motor L", "Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical", 10, 53, 90, "B",
     pins("MOT_L1", "MOT_L2", "+3V3", "GND", "ENC_LA", "ENC_LB"), "left gearmotor + encoder"),
    ("J13", "Motor R", "Connector_JST:JST_XH_B6B-XH-A_1x06_P2.50mm_Vertical", 10, -53, 90, "B",
     pins("MOT_R1", "MOT_R2", "+3V3", "GND", "ENC_RA", "ENC_RB"), "right gearmotor + encoder"),
    # ---- line array QTRX-MD-08A (VCC, GND, CTRL, 1..8) bottom side
    ("J14", "Line array", "Connector_PinHeader_2.54mm:PinHeader_1x11_P2.54mm_Vertical", 5, 12.7, 90, "B",
     pins("+3V3", "GND", "LINE_CTRL", "LINE1", "LINE2", "LINE3", "LINE4", "LINE5", "LINE6", "LINE7", "LINE8"),
     "QTRX-MD-08A"),
    # ---- servo, I2C, camera link
    ("J15", "Servo", "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical", -19, 14, 90, "F",
     pins("SERVO", "+5V", "GND"), "MG90S gripper servo (S, +, -)"),
    ("J16", "Qwiic IMU", "Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal", -22, 26, 90, "F",
     pins("GND", "+3V3", "SDA", "SCL") | {"MP": "GND"}, "BNO085 (Qwiic)"),
    ("J17", "Qwiic colour", "Connector_JST:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal", -22, -26, 90, "F",
     pins("GND", "+3V3", "SDA", "SCL") | {"MP": "GND"}, "TCS34725 (Qwiic)"),
    ("J18", "XIAO cam", "Connector_JST:JST_XH_B4B-XH-A_1x04_P2.50mm_Vertical", 36, -8, 90, "F",
     pins("+5V", "GND", "SDA", "SCL"), "XIAO ESP32S3 Sense (I2C slave 0x42)"),
    # ---- buttons + LEDs
    ("SW1", "START", "Button_Switch_THT:SW_PUSH_6mm", 33, -24, 0, "F", pins("BTN_START", "BTN_START", "GND", "GND"),
     "start button"),
    ("SW2", "CAL", "Button_Switch_THT:SW_PUSH_6mm", 24, -37, 0, "F", pins("BTN_CAL", "BTN_CAL", "GND", "GND"),
     "calibration button (rule 5.3)"),
    ("D1", "BLUE", "LED_SMD:LED_0805_2012Metric", 57, 20, 90, "F", pins("GND", "LED_BA"), "clan LED blue"),
    ("D2", "GREEN", "LED_SMD:LED_0805_2012Metric", 57, -20, 90, "F", pins("GND", "LED_GA"), "clan LED green"),
    ("R15", "330R", "Resistor_SMD:R_0805_2012Metric", 53, 20, 90, "F", pins("LED_B", "LED_BA"), "LED resistor"),
    ("R16", "330R", "Resistor_SMD:R_0805_2012Metric", 53, -20, 90, "F", pins("LED_G", "LED_GA"), "LED resistor"),
]

# mounting / mechanical holes (robot mm) -> footprint
HOLES = (
    [("H%d" % (i + 1), "MountingHole:MountingHole_3.2mm_M3", x, y) for i, (x, y) in
     enumerate([(52, 0), (0, 0), (-47, 42), (-47, -42)])]                       # base standoffs
    + [("H%d" % (i + 5), "MountingHole:MountingHole_3.2mm_M3", x, y) for i, (x, y) in
       enumerate([(32, 38), (32, -38), (-32, 38), (-32, -38)])]                 # crown standoffs
    + [("H9", "MountingHole:MountingHole_4.3mm_M4", -50, 33.5),
       ("H10", "MountingHole:MountingHole_4.3mm_M4", -50, -33.5)]                # gripper hinge bushings
    + [("H11", "MountingHole:MountingHole_2.7mm_M2.5", -15, 36),
       ("H12", "MountingHole:MountingHole_2.7mm_M2.5", -15, -36)]                # pocket flange
    + [("H%d" % (13 + i), "MountingHole:MountingHole_2.2mm_M2", x, y) for i, (x, y) in
       enumerate([(-43.5, 6.5), (-43.5, -6.5), (-12.5, 6.5), (-12.5, -6.5)])]       # servo cradle
)
BOARD_R = 65.0

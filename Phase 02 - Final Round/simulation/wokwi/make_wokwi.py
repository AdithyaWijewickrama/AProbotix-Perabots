"""
make_wokwi.py - builds the Wokwi project that runs the REAL Arduino Mega firmware (firmware/mega_body)
with every part of GuardianBot v4.1 wired exactly as on the PCB deck (pin map = firmware/mega_body/pins.h).

    python make_wokwi.py          -> project/   (upload these files to a new Arduino Mega project on wokwi.com)
                                      and diagram.json + wokwi.toml here (Wokwi for VS Code)
    python make_wokwi.py --wasm   -> also compiles the custom chips to chips/*.chip.wasm (needs clang + wasi-libc)

Parts: Wokwi's own OLED (128x64; ours is 128x32, the text uses the top half), 2 x HC-SR04, 2 servos,
4 keys, 2 LEDs, buzzer, a slide pot as the battery voltage, a logic analyzer on the link; and our custom
chips (chips/): 8 x VL53L0X, L3G4200D, 2 x TCS3200, QTR-8RC, 2 x DRV8833+N20 encoder motor, and a
"brain stub" that plays the ESP32-S3 brain on Serial2 (sliders for wheel speeds, servos and LEDs).
"""
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, "..", "..", "firmware", "mega_body"))
CHIPS = ["vl53l0x", "l3g4200d", "tcs3200", "qtr8rc", "n20motor", "brainstub"]

parts, conns = [], []


def part(ptype, pid, top, left, attrs=None, rotate=0):
    p = {"type": ptype, "id": pid, "top": top, "left": left, "attrs": {k: str(v) for k, v in (attrs or {}).items()}}
    if rotate:
        p["rotate"] = rotate
    parts.append(p)


def wire(a, b, color="green"):
    conns.append([a, b, color, []])


def mega(pin):
    return "mega:" + str(pin)


GND, V5 = "mega:GND.1", "mega:5V"

# ---------------------------------------------------------------- board
part("wokwi-arduino-mega", "mega", 300, 380)

# ---------------------------------------------------------------- I2C: OLED, gyro, 8 ToF (all on D20 SDA / D21 SCL)
part("wokwi-ssd1306", "oled", 120, 760, {"i2cAddress": "0x3c"})
for pin, to in (("GND", GND), ("VCC", V5), ("SCL", mega(21)), ("SDA", mega(20))):
    wire("oled:" + pin, to, "black" if pin == "GND" else "red" if pin == "VCC" else "blue" if pin == "SCL" else "green")
part("chip-l3g4200d", "gyro", 140, 1000, {"rate": 0, "bias": 8})
for pin, to in (("VCC", V5), ("GND", GND), ("SCL", mega(21)), ("SDA", mega(20))):
    wire("gyro:" + pin, to)
TOF = [("tof_front", 300), ("tof_fl", 450), ("tof_fr", 450), ("tof_lf", 120), ("tof_lr", 125), ("tof_rf", 140),
       ("tof_rr", 135), ("tof_back", 800)]
for i, (name, d) in enumerate(TOF):
    pid = name
    part("chip-vl53l0x", pid, 20 + 110 * (i % 4), 20 + 170 * (i // 4), {"distance": d, "noise": 2})
    for pin, to in (("VIN", V5), ("GND", GND), ("SCL", mega(21)), ("SDA", mega(20)), ("XSHUT", mega(30 + i))):
        wire(pid + ":" + pin, to, "orange" if pin == "XSHUT" else "green")

# ---------------------------------------------------------------- sonars
for k, (pid, trig, echo, d) in enumerate((("us_front", 39, 10, 600), ("us_back", 40, 11, 900))):
    part("wokwi-hc-sr04", pid, 520 + 130 * k, 20, {"distance": d})
    for pin, to in (("VCC", V5), ("TRIG", mega(trig)), ("ECHO", mega(echo)), ("GND", GND)):
        wire(pid + ":" + pin, to, "purple")

# ---------------------------------------------------------------- colour sensors (S0 = 5 V, S1 = GND as on the deck)
for k, (pid, out, led, rgb) in enumerate((("tcs_floor", 50, 43, (200, 200, 200)), ("tcs_front", 51, 44, (40, 60, 200)))):
    part("chip-tcs3200", pid, 300 + 130 * k, 1000, {"red": rgb[0], "green": rgb[1], "blue": rgb[2]})
    for pin, to in (("VCC", V5), ("GND", GND), ("S0", V5), ("S1", GND), ("S2", mega(41)), ("S3", mega(42)),
                    ("OUT", mega(out)), ("LED", mega(led))):
        wire(pid + ":" + pin, to, "gray")

# ---------------------------------------------------------------- QTR-8RC on D22..D29 (sensor 1 = right-most), LEDON D38
part("chip-qtr8rc", "qtr", 560, 1000, {"line_mm": 0, "line_on": 1, "width_mm": 20})
wire("qtr:VCC", V5); wire("qtr:GND", GND); wire("qtr:LEDON", mega(38))
for i in range(8):
    wire("qtr:%d" % (i + 1), mega(22 + i), "yellow")

# ---------------------------------------------------------------- motors: DRV8833 + N20 with encoder
for k, (pid, in1, in2, ea, eb, rev) in enumerate((("motor_l", 5, 6, 2, 3, 0), ("motor_r", 7, 8, 18, 19, 1))):
    part("chip-n20motor", pid, 700 + 120 * k, 1000, {"vbat_dV": 74, "load": 20, "cpr": 2800, "reverse": rev})
    for pin, to in (("IN1", mega(in1)), ("IN2", mega(in2)), ("SLP", mega(4)), ("GND", GND), ("VCC", V5),
                    ("ENC_A", mega(ea)), ("ENC_B", mega(eb))):
        wire(pid + ":" + pin, to, "brown")

# ---------------------------------------------------------------- the ESP32-S3 brain stand-in on Serial2
part("chip-brainstub", "brain", 120, 1260, {"enable": 1, "v_left": 0, "v_right": 0, "servo_left": 90,
                                            "servo_right": 90, "outputs": 0, "zero": 0})
wire("brain:RX", mega(16), "cyan"); wire("brain:TX", mega(17), "cyan"); wire("brain:GND", GND, "black")
part("wokwi-logic-analyzer", "logic", 330, 1260, {})
for ch, pin in ((0, 16), (1, 17), (2, 5), (3, 2), (4, 3), (5, 22)):
    wire("logic:D%d" % ch, mega(pin), "cyan")
wire("logic:GND", GND, "black")

# ---------------------------------------------------------------- servos, keys, LEDs, buzzer, battery
for k, (pid, pin) in enumerate((("servo_l", 45), ("servo_r", 46))):
    part("wokwi-servo", pid, 820 + 110 * k, 120, {})
    wire(pid + ":GND", GND, "black"); wire(pid + ":V+", V5, "red"); wire(pid + ":PWM", mega(pin), "orange")
for k, (pin, lab, col) in enumerate(((47, "K1 GO", "green"), (48, "K2 MAZE", "blue"), (49, "K3 S3", "yellow"),
                                     (53, "K4 CAL", "red"))):
    pid = "key%d" % (k + 1)
    part("wokwi-pushbutton", pid, 820, 420 + 95 * k, {"color": col, "key": str(k + 1), "label": lab})
    wire(pid + ":1.l", mega(pin), "green"); wire(pid + ":2.l", GND, "black")
for k, (pid, pin, col) in enumerate((("led_blue", "A1", "blue"), ("led_green", "A2", "green"))):
    part("wokwi-led", pid, 820, 820 + 60 * k, {"color": col})
    part("wokwi-resistor", "r_" + pid, 860, 880 + 60 * k, {"value": "330"})
    wire(pid + ":A", "r_%s:2" % pid, "green"); wire("r_%s:1" % pid, mega(pin), "green"); wire(pid + ":C", GND, "black")
part("wokwi-buzzer", "buzzer", 1000, 820, {"volume": "0.1"})
wire("buzzer:1", GND, "black"); wire("buzzer:2", mega(9), "red")
part("wokwi-slide-potentiometer", "battery", 1060, 380, {"travelLength": "30"})
wire("battery:VCC", V5, "red"); wire("battery:GND", GND, "black"); wire("battery:SIG", mega("A0"), "green")

diagram = {"version": 1, "author": "AProbotix - GuardianBot v4.1", "editor": "wokwi",
           "parts": parts, "connections": conns, "dependencies": {}}


def main():
    out = os.path.join(HERE, "project")
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out)
    js = json.dumps(diagram, indent=2)
    for d in (out, HERE):
        open(os.path.join(d, "diagram.json"), "w").write(js)
    shutil.copy(os.path.join(FW, "mega_body.ino"), os.path.join(out, "sketch.ino"))
    for f in ("pins.h", "protocol.h"):
        shutil.copy(os.path.join(FW, f), os.path.join(out, f))
    open(os.path.join(out, "libraries.txt"), "w").write("# Wokwi Library List\nVL53L0X\nSSD1306Ascii\n")
    for c in CHIPS:
        for ext in (".chip.c", ".chip.json"):
            shutil.copy(os.path.join(HERE, "chips", c + ext), os.path.join(out, c + ext))
    # Wokwi for VS Code: firmware built with arduino-cli into build/, chips as wasm
    toml = ["[wokwi]", "version = 1", 'firmware = "build/sketch.ino.hex"', 'elf = "build/sketch.ino.elf"', ""]
    for c in CHIPS:
        toml += ["[[chip]]", 'name = "%s"' % c, 'binary = "chips/%s.chip.wasm"' % c, ""]
    open(os.path.join(HERE, "wokwi.toml"), "w").write("\n".join(toml))
    if "--wasm" in sys.argv:
        for c in CHIPS:
            src = os.path.join(HERE, "chips", c + ".chip.c")
            subprocess.run(["clang", "--target=wasm32-unknown-wasi", "--sysroot=/usr", "-I/usr/include/wasm32-wasi",
                            "-nostartfiles", "-Wl,--import-memory", "-Wl,--export-table", "-Wl,--no-entry",
                            "-Wall", "-Wno-unused-function", "-O2", "-o", src[:-2] + ".wasm", src, "-lm"], check=True)
    print("parts %d, wires %d -> %s" % (len(parts), len(conns), out))


if __name__ == "__main__":
    main()

"""
testbench_sim.py - the robot TEST BENCH web page, driving the SIMULATED robot in Webots.

It serves exactly the page of the real test-bench firmware (firmware/esp32s3_testbench/web/index.html)
with the same /api/... calls, so you can try the page - and drive the robot with the joystick - before
the hardware exists. Open worlds/testbench.wbt in Webots, press play, then browse to http://localhost:8080

What it simulates: the Mega's telemetry (encoders, gyro yaw, 2 sonars, 8 ToF, QTR-8RC decay times,
both TCS3200, battery), the Mega's wheel speed control (acceleration-limited), the 300 ms deadman,
servos and LEDs, the camera picture, and the calibration routines (1 m drive, 360 deg spin, white balance).
Calibration values are kept in cal.json next to this file (the real robot keeps them in flash).
Port: environment variable TESTBENCH_PORT (default 8080).
"""
import json
import math
import os
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from controller import Robot

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.normpath(os.path.join(HERE, "..", "..", "..", "firmware", "esp32s3_testbench", "web", "index.html"))
CAL_FILE = os.path.join(HERE, "cal.json")
PORT = int(os.environ.get("TESTBENCH_PORT", "8080"))
R_WHEEL, HALF_TRACK = 0.0215, 0.062
V_LIMIT, DEADMAN = 600, 0.3
GRIP_CLOSED_RAD = 1.05

robot = Robot()
TS = int(robot.getBasicTimeStep())
dev = robot.getDevice
lm, rm = dev("left_wheel_motor"), dev("right_wheel_motor")
for m in (lm, rm):
    m.setPosition(float("inf"))
    m.setVelocity(0)
le, re_ = dev("left_wheel_sensor"), dev("right_wheel_sensor")
imu, gyro = dev("imu"), dev("gyro")
US = [dev(n) for n in ("us_front", "us_back")]
TOF = [dev(n) for n in ("tof_front", "tof_fl", "tof_fr", "tof_lf", "tof_lr", "tof_rf", "tof_rr", "tof_back")]
GS = [dev("gs%d" % i) for i in range(8)]
tf, tx, cam = dev("tcs_floor"), dev("tcs_front"), dev("cam")
for d in [le, re_, imu, gyro, tf, tx, cam] + US + TOF + GS:
    d.enable(TS)
gl, gr = dev("grip_left"), dev("grip_right")
led_b, led_g = dev("led_blue"), dev("led_green")

lock = threading.Lock()
cal = {"cpr": 2800.0, "lens": 0.90, "grip": [30, 90, 150, 90], "tofo": [0] * 8,
       "whiteF": [300, 300, 300], "whiteX": [300, 300, 300], "cam": [1.0, 1.0, 1.0]}
if os.path.exists(CAL_FILE):
    cal.update(json.load(open(CAL_FILE)))


def cal_save():
    json.dump(cal, open(CAL_FILE, "w"), indent=1)


st = {"tele": None, "frame": b"", "grid": None, "cmd": {"vl": 0, "vr": 0, "sl": cal["grip"][1], "sr": cal["grip"][3], "out": 0},
      "web_t": 0.0, "web_v": [0, 0], "ctl": 0, "routine": None, "rmsg": "-", "rname": "", "rresult": False,
      "log_on": False, "log": [], "log_want": 0, "v": [0.0, 0.0]}


def classify(r, g, b):                         # firmware/esp32s3_brain/colour.h
    m = max(r, g, b)
    if m < 50:
        return 0
    if b > 45 and b > 1.5 * g + 15 and b > 1.5 * r + 15:
        return 1
    if g > 45 and g > 1.5 * b + 15 and g > 1.5 * r + 15:
        return 2
    if r > 100 and r > 2 * g and r > 2 * b:
        return 0
    return -1


def tcs_rgb(c, white):
    return [min(255.0, 255.0 * c[k] / max(1, white[k])) for k in range(3)]


def avg_rgb(c):
    img = c.getImage()
    w, h = c.getWidth(), c.getHeight()
    r = g = b = 0
    for y in range(h):
        for x in range(w):
            r += c.imageGetRed(img, w, x, y)
            g += c.imageGetGreen(img, w, x, y)
            b += c.imageGetBlue(img, w, x, y)
    n = w * h
    # counts per 6 ms window like the TCS3200: white paper ~300 with the LEDs on
    return [int(r / n * 1.18), int(g / n * 1.18), int(b / n * 1.18), int((r + g + b) / n * 0.45)]


def cam_frame():
    """Webots camera -> 4-byte header (w, h) + RGB565 little-endian, and the 48x32 grid."""
    img = cam.getImage()
    w, h = cam.getWidth(), cam.getHeight()
    px = []
    for i in range(0, len(img), 4):
        b, g, r = img[i], img[i + 1], img[i + 2]
        px.append(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3))
    return struct.pack("<HH", w, h) + struct.pack("<%dH" % len(px), *px), (w, h, px)


# ---------------------------------------------------------------- web API (same as the firmware)
class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def reply(self, code, ctype, body):
        if isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        p = u.path
        gi = lambda k, d: int(float(q.get(k, d)))
        with lock:
            if p in ("/", "/index.html"):
                return self.reply(200, "text/html; charset=utf-8", open(PAGE, encoding="utf-8").read())
            if p == "/api/state":
                return self.reply(200, "application/json", json.dumps(state_json()))
            if p == "/api/cmd":
                st["web_v"] = [max(-V_LIMIT, min(V_LIMIT, gi("vl", 0))), max(-V_LIMIT, min(V_LIMIT, gi("vr", 0)))]
                c = st["cmd"]
                c["sl"] = max(0, min(180, gi("sl", c["sl"])))
                c["sr"] = max(0, min(180, gi("sr", c["sr"])))
                if st["routine"] != "wb":
                    c["out"] = gi("out", c["out"]) & 0x1F
                st["web_t"] = time.time()
                if any(st["web_v"]) and st["routine"]:
                    stop_routine("stopped: you took over the wheels")
                return self.reply(200, "text/plain", "ok")
            if p == "/api/stop":
                st["web_v"] = [0, 0]
                stop_routine("STOPPED")
                return self.reply(200, "text/plain", "stopped")
            if p == "/api/ctl":
                if gi("zero", 0):
                    st["ctl"] |= 1
                return self.reply(200, "text/plain", "ok")
            if p == "/api/frame":
                return self.reply(200, "application/octet-stream", st["frame"])
            if p == "/api/grid":
                return self.reply(200, "application/json", json.dumps(grid_json()))
            if p == "/api/routine":
                start_routine(q.get("name", ""))
                return self.reply(200, "text/plain", "ok")
            if p == "/api/set":
                k = q.get("key")
                if k == "cpr":
                    cal["cpr"] = max(100.0, min(20000.0, float(q["val"])))
                elif k == "lens":
                    cal["lens"] = max(0.2, min(2.5, float(q["val"])))
                elif k == "grip" and 0 <= gi("i", -1) < 4:
                    cal["grip"][gi("i", 0)] = max(0, min(180, gi("val", 90)))
                else:
                    return self.reply(400, "text/plain", "unknown key")
                cal_save()
                return self.reply(200, "text/plain", "saved")
            if p == "/api/tofzero":
                i, ref, t = gi("i", -1), gi("ref", 100), st["tele"]
                if not t or not 0 <= i < 8 or t["tof"][i] >= 1200:
                    return self.reply(400, "text/plain", "no reading")
                cal["tofo"][i] = ref - t["tof"][i]
                cal_save()
                return self.reply(200, "text/plain", "saved")
            if p == "/api/log":
                st["log"], st["log_want"], st["log_on"] = [], max(1, min(60, gi("secs", 30))) * 50, True
                return self.reply(200, "text/plain", "recording")
            if p == "/api/log.csv":
                return self.reply(200, "text/csv", log_csv())
        return self.reply(404, "text/plain", "not found")


def state_json():
    t = st["tele"] or {}
    if not t:
        return {"fw": "Webots simulation", "link": {"ok": False, "fps": 0, "bad": 0, "age": 9999}}
    frgb, xrgb = tcs_rgb(t["tcsFloor"], cal["whiteF"]), tcs_rgb(t["tcsFront"], cal["whiteX"])
    r = st["routine"]
    return {"fw": "Webots simulation", "ms": t["ms"], "link": {"ok": True, "fps": 1000 // TS, "bad": 0, "age": 0},
            "enc": t["enc"], "v": [int(v) for v in st["v"]], "yaw": t["yaw"], "gz": t["gz"], "vbat": 7400, "keys": 0,
            "flags": 0x0E, "us": t["us"], "tof": t["tof"], "line": t["line"], "tcsFloor": t["tcsFloor"],
            "tcsFront": t["tcsFront"], "floorRGB": frgb, "frontRGB": xrgb, "floorCls": classify(*frgb), "frontCls": classify(*xrgb),
            "cmd": st["cmd"], "cam": {"ok": True, "pid": 0x76, "fps": 1000 // (TS * 4)}, "psram": 8 << 20,
            "routine": {"name": st["rname"], "running": r is not None, "result": st["rresult"], "msg": st["rmsg"]},
            "log": {"on": st["log_on"], "rows": len(st["log"])},
            "cal": {"cpr": cal["cpr"], "lens": cal["lens"], "grip": cal["grip"], "tofo": cal["tofo"],
                    "whiteF": cal["whiteF"], "whiteX": cal["whiteX"]}}


def grid_json():
    g = st["grid"]
    if not g:
        return {"w": 48, "h": 32, "px": [0] * 1536, "cls": "." * 1536, "gain": cal["cam"], "lens": cal["lens"], "crop": 0.9}
    w, h, px = g
    k = cal["cam"]
    cls = ""
    for v in px:
        r = min(255, ((v >> 11) & 31) * 255 / 31 * k[0])
        gg = min(255, ((v >> 5) & 63) * 255 / 63 * k[1])
        b = min(255, (v & 31) * 255 / 31 * k[2])
        cls += {1: "B", 2: "G", 0: "F"}.get(classify(r, gg, b), ".")
    return {"w": w, "h": h, "px": px, "cls": cls, "gain": k, "lens": cal["lens"], "crop": min(0.9, cal["lens"])}


def log_csv():
    out = ["ms,encL,encR,yaw_deg,gz_dps,us_f,us_b,tof_f,tof_fl,tof_fr,tof_lf,tof_lr,tof_rf,tof_rr,tof_b,"
           "q1,q2,q3,q4,q5,q6,q7,q8,fR,fG,fB,fC,xR,xG,xB,xC,vbat_mv,keys,flags,cmd_vl,cmd_vr,servo_l,servo_r"]
    for t, c in st["log"]:
        row = [t["ms"], *t["enc"], "%.2f" % t["yaw"], "%.2f" % t["gz"], *t["us"], *t["tof"], *t["line"], *t["tcsFloor"],
               *t["tcsFront"], 7400, 0, 14, c["vl"], c["vr"], c["sl"], c["sr"]]
        out.append(",".join(str(x) for x in row))
    return "\n".join(out) + "\n"


# ---------------------------------------------------------------- routines (same logic as the firmware)
R = {}


def start_routine(name):
    if name not in ("drive1m", "spin360", "wb") or not st["tele"]:
        return
    t = st["tele"]
    R.update(t0=time.time(), sim0=robot.getTime(), encL=t["enc"][0], encR=t["enc"][1], yaw=t["yaw"], acc=[0.0] * 6, n=0)
    st["routine"], st["rname"], st["rmsg"], st["rresult"] = name, name, "running...", False


def stop_routine(why):
    if st["routine"] and why:
        st["rmsg"] = why
    st["routine"] = None
    st["cmd"]["vl"] = st["cmd"]["vr"] = 0


def routine_step(t):
    mmpt = 2 * math.pi * R_WHEEL * 1000 / cal["cpr"]
    dl, dr = (t["enc"][0] - R["encL"]) * mmpt, (t["enc"][1] - R["encR"]) * mmpt
    dyaw = t["yaw"] - R["yaw"]
    el = robot.getTime() - R["sim0"]
    c = st["cmd"]
    if st["routine"] == "drive1m":
        d = (dl + dr) / 2
        left = 1000 - d
        if left <= 0 or el > 15:
            c["vl"] = c["vr"] = 0
            st["rresult"], st["routine"] = True, None
            st["rmsg"] = "Done: encoders say %.0f mm (L %.0f, R %.0f), heading drift %.1f deg. Measure the real distance below." % (d, dl, dr, dyaw)
            return
        v = min(200, max(40, left * 1.5))
        steer = -dyaw * 4
        c["vl"], c["vr"] = int(v - steer), int(v + steer)
        st["rmsg"] = "driving... %.0f mm" % d
    elif st["routine"] == "spin360":
        left = 360 - dyaw
        if left <= 0.5 or el > 15:
            c["vl"] = c["vr"] = 0
            track = (dr - dl) / (2 * math.pi * max(1, dyaw) / 360) / 2
            st["rresult"], st["routine"] = True, None
            st["rmsg"] = ("Done: gyro %.1f deg. Encoders imply half-track %.1f mm (design 62). "
                          "If the robot does not face the start direction, the gyro scale is off." % (dyaw, track))
            return
        w = min(120, max(25, left * 1.2)) * math.pi / 180
        v = int(w * HALF_TRACK * 1000)
        c["vl"], c["vr"] = -v, v
        st["rmsg"] = "turning... %.0f deg" % dyaw
    elif st["routine"] == "wb":
        c["vl"] = c["vr"] = 0
        c["out"] |= 0x0C
        if el < 0.3:
            return
        for k in range(3):
            R["acc"][k] += t["tcsFloor"][k]
            R["acc"][3 + k] += t["tcsFront"][k]
        R["n"] += 1
        if R["n"] >= 15:
            cal["whiteF"] = [max(1, R["acc"][k] / R["n"]) for k in range(3)]
            cal["whiteX"] = [max(1, R["acc"][3 + k] / R["n"]) for k in range(3)]
            w, h, px = st["grid"]
            s = [0.0, 0.0, 0.0]
            n = 0
            for v in range(h // 2, h):
                for u in range(w // 4, 3 * w // 4):
                    p = px[v * w + u]
                    s[0] += ((p >> 11) & 31) * 255 / 31
                    s[1] += ((p >> 5) & 63) * 255 / 63
                    s[2] += (p & 31) * 255 / 31
                    n += 1
            if min(s) > 30 * n:
                m = max(s)
                cal["cam"] = [m / s[0], m / s[1], m / s[2]]
            cal_save()
            c["out"] &= ~0x0C
            st["rresult"], st["routine"] = True, None
            st["rmsg"] = "Saved. Floor white %.0f/%.0f/%.0f, front %.0f/%.0f/%.0f, camera gains %.2f/%.2f/%.2f" % (
                *cal["whiteF"], *cal["whiteX"], *cal["cam"])


threading.Thread(target=lambda: ThreadingHTTPServer(("0.0.0.0", PORT), H).serve_forever(), daemon=True).start()
print("test bench page: http://localhost:%d  (same page as the robot's Wi-Fi test firmware)" % PORT, flush=True)

enc0 = [0.0, 0.0]
yaw_u, prev_yaw, yaw0 = 0.0, None, 0.0
wl = wr = 0.0                                   # wheel speeds after the acceleration limit, m/s
prev_ticks = [0, 0]
k = 0
while robot.step(TS) != -1:
    k += 1
    y = imu.getRollPitchYaw()[2]
    if prev_yaw is not None:
        yaw_u += (y - prev_yaw + math.pi) % (2 * math.pi) - math.pi
    prev_yaw = y
    cpr = cal["cpr"]
    ticks = [int((le.getValue() - enc0[0]) / (2 * math.pi) * cpr), int((re_.getValue() - enc0[1]) / (2 * math.pi) * cpr)]
    tof_raw = [max(30, min(1200, int(d.getValue()))) for d in TOF]
    tele = {"ms": int(robot.getTime() * 1000), "enc": ticks, "yaw": math.degrees(yaw_u - yaw0), "gz": math.degrees(gyro.getValues()[2]),
            "us": [min(2500, int(d.getValue())) for d in US], "tof": tof_raw,
            "line": [int(2500 - 2.2 * max(0, min(1023, d.getValue()))) for d in GS]}
    if k % 3 == 0 or "tcsFloor" not in (st["tele"] or {}):
        tele["tcsFloor"], tele["tcsFront"] = avg_rgb(tf), avg_rgb(tx)
    else:
        tele["tcsFloor"], tele["tcsFront"] = st["tele"]["tcsFloor"], st["tele"]["tcsFront"]
    frame = grid = None
    if k % 4 == 0:
        frame, grid = cam_frame()
    with lock:
        st["tele"] = tele
        if frame:
            st["frame"], st["grid"] = frame, grid
        dt = TS / 1000.0
        mmpt = 2 * math.pi * R_WHEEL * 1000 / cpr
        for i in range(2):
            st["v"][i] = 0.6 * st["v"][i] + 0.4 * (ticks[i] - prev_ticks[i]) * mmpt / dt
        prev_ticks = ticks
        if st["ctl"] & 1:
            enc0 = [le.getValue(), re_.getValue()]
            yaw0 = yaw_u
            st["ctl"] = 0
        c = st["cmd"]
        if st["routine"]:
            routine_step(tele)
        elif time.time() - st["web_t"] > DEADMAN:
            c["vl"] = c["vr"] = 0
        else:
            c["vl"], c["vr"] = st["web_v"]
        if st["log_on"]:
            st["log"].append((tele, dict(c)))
            if len(st["log"]) >= st["log_want"]:
                st["log_on"] = False
        tl, tr = c["vl"] / 1000.0, c["vr"] / 1000.0
        sl, sr, out = c["sl"], c["sr"], c["out"]
    step = 2.5 * TS / 1000.0                     # Mega: acceleration-limited wheel speed control
    wl += max(-step, min(step, tl - wl))
    wr += max(-step, min(step, tr - wr))
    lm.setVelocity(max(-31, min(31, wl / R_WHEEL)))
    rm.setVelocity(max(-31, min(31, wr / R_WHEEL)))
    g = cal["grip"]
    gl.setPosition(max(0.0, min(1.4, (sl - g[0]) / float(max(1, g[1] - g[0])) * GRIP_CLOSED_RAD)))
    gr.setPosition(-max(0.0, min(1.4, (g[2] - sr) / float(max(1, g[2] - g[3])) * GRIP_CLOSED_RAD)))
    led_b.set(1 if out & 1 else 0)
    led_g.set(1 if out & 2 else 0)

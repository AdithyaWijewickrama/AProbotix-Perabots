"""
mega_sil.py - software-in-the-loop test of the REAL robot firmware.

This Webots controller plays the part of the Arduino Mega (firmware/mega_body):
every 16 ms it sends a binary Telemetry frame (firmware/mega_body/protocol.h) to
the ESP32-CAM brain - compiled for the PC by firmware/sil/build_sil.sh - and
applies the Command frame the brain sends back (wheel speeds, servos, LEDs).
It also plays the ESP32-CAM camera task: every 4th step it sends
the 48x32 camera grid as an RGB565 'V' frame, one frame late (real latency).
So the exact C++ that will be flashed to the ESP32-CAM drives the simulated robot.

Use: build the brain (firmware/sil/build_sil.sh), set the robot's controller to
"mega_sil" (controllerArgs "section3" = hold CAL+START for a Section 3 restart).
"""
import math
import os
import struct
import subprocess
import sys
from controller import Robot

HERE = os.path.dirname(os.path.abspath(__file__))
BRAIN = os.path.normpath(os.path.join(HERE, '..', '..', '..', 'firmware', 'sil',
                                      'brain_sil.exe' if os.name == 'nt' else 'brain_sil'))
TELE = struct.Struct('<Iiiff2H8H8H4H4HHBBB')     # firmware/mega_body/protocol.h (v4.0)
CMD = struct.Struct('<hhBBBB')
ENC_CPR, R_WHEEL = 2800.0, 0.0215
SERVO_L_OPEN, SERVO_L_CLOSED, SERVO_R_OPEN, SERVO_R_CLOSED = 30, 90, 150, 90
GRIP_CLOSED_RAD = 1.05

robot = Robot()
TS = int(robot.getBasicTimeStep())
dev = robot.getDevice
lm, rm = dev('left_wheel_motor'), dev('right_wheel_motor')
for m in (lm, rm):
    m.setPosition(float('inf'))
    m.setVelocity(0)
le, re_ = dev('left_wheel_sensor'), dev('right_wheel_sensor')
imu, gyro = dev('imu'), dev('gyro')
US = [dev(n) for n in ('us_front', 'us_back')]
TOF = [dev(n) for n in ('tof_front', 'tof_fl', 'tof_fr', 'tof_lf', 'tof_lr', 'tof_rf', 'tof_rr', 'tof_back')]
GS = [dev('gs%d' % i) for i in range(8)]
tf, tx = dev('tcs_floor'), dev('tcs_front')
cam = dev('cam')
cam.enable(TS)
for d in [le, re_, imu, gyro, tf, tx] + US + TOF + GS:
    d.enable(TS)
gl, gr = dev('grip_left'), dev('grip_right')
led_b, led_g = dev('led_blue'), dev('led_green')

section3 = 'section3' in sys.argv[1:]
log = open(os.path.join(HERE, 'sil_log.txt'), 'w')
cam_r, cam_w = os.pipe()                     # ESP32-CAM stream: the brain reads fd SIL_CAM_FD
brain = subprocess.Popen([BRAIN], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log,
                         cwd=HERE, bufsize=0, pass_fds=(cam_r,), env=dict(os.environ, SIL_CAM_FD=str(cam_r)))
os.close(cam_r)
cam_seq = 0
cam_pending = None


def cam_frame():
    """Webots BGRA image -> 'V' frame with little-endian RGB565 pixels."""
    global cam_seq
    img = cam.getImage()
    out = bytearray()
    for i in range(0, len(img), 4):
        b, g, r = img[i], img[i + 1], img[i + 2]
        v = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
        out += bytes((v & 0xFF, v >> 8))
    hdr = bytes([0xA5, 0x5A, ord('V'), cam_seq & 0xFF, len(out) & 0xFF, len(out) >> 8])
    cam_seq += 1
    s = (sum(hdr[2:]) + sum(out)) & 0xFF
    return hdr + bytes(out) + bytes([s])


def rgb(cam):
    img = cam.getImage()
    w, h = cam.getWidth(), cam.getHeight()
    r = g = b = 0
    for y in range(h):
        for x in range(w):
            r += cam.imageGetRed(img, w, x, y)
            g += cam.imageGetGreen(img, w, x, y)
            b += cam.imageGetBlue(img, w, x, y)
    n = w * h
    r, g, b = r // n, g // n, b // n
    return [r, g, b, r + g + b]


def frame(t, payload):
    s = (t + len(payload) + sum(payload)) & 0xFF
    return bytes([0xA5, 0x5A, t, len(payload)]) + payload + bytes([s])


rx = bytearray()


def read_commands():
    """Block until at least one Command frame has arrived; return the newest one."""
    global rx
    newest = None
    while True:
        while True:                                   # parse every complete frame in the buffer
            i = rx.find(b'\xa5\x5a')
            if i < 0:
                rx = rx[-1:]
                break
            rx = rx[i:]
            if len(rx) < 5 + CMD.size:
                break
            t, n = rx[2], rx[3]
            if t == 0x58 and n == 42:                 # OLED text from the brain: show it in the console
                if len(rx) < 47:
                    break
                txt = bytes(rx[4:46]).decode('ascii', 'replace')
                print('OLED | %s | %s' % (txt[:21].rstrip(), txt[21:].rstrip()), flush=True)
                rx = rx[47:]
                continue
            if t != 0x43 or n != CMD.size:
                rx = rx[2:]
                continue
            body = bytes(rx[4:4 + n])
            if (t + n + sum(body)) & 0xFF == rx[4 + n]:
                newest = CMD.unpack(body)
            rx = rx[5 + n:]
        if newest is not None:
            return newest
        chunk = os.read(brain.stdout.fileno(), 4096)
        if not chunk:
            return None
        rx += chunk


enc0 = [0.0, 0.0]
yaw0 = 0.0
yaw_unwrapped = 0.0
prev_yaw = None
last_ctl = 0
k = 0
while robot.step(TS) != -1:
    k += 1
    y = imu.getRollPitchYaw()[2]
    if prev_yaw is not None:
        yaw_unwrapped += (y - prev_yaw + math.pi) % (2 * math.pi) - math.pi
    prev_yaw = y
    btn = 1 if 10 <= k < 16 and not section3 else 0   # K1 = START
    if section3 and 10 <= k < 16:
        btn |= 4                                      # K3 = Section 3 restart
    ticks = [int((le.getValue() - enc0[0]) / (2 * math.pi) * ENC_CPR), int((re_.getValue() - enc0[1]) / (2 * math.pi) * ENC_CPR)]
    us = [min(2500, int(d.getValue())) for d in US]
    tof = [max(30, min(1200, int(d.getValue()))) for d in TOF]
    # QTR-8RC: decay time in us, short on white, long (2500) on black; the model reads 0..1023 white-high
    line = [int(2500 - 2.2 * max(0, min(1023, d.getValue()))) for d in GS]
    flags = 0x02 | 0x08 | (0x04 if k % 2 == 0 else 0)
    p = TELE.pack(k * TS, ticks[0], ticks[1], yaw_unwrapped - yaw0, gyro.getValues()[2],
                  *us, *tof, *line, *rgb(tf), *rgb(tx), 7800, btn, flags, k & 0xFF)
    if k % 4 == 0:
        if cam_pending:
            try:
                os.write(cam_w, cam_pending)
            except OSError:
                pass
        cam_pending = cam_frame()
    try:
        brain.stdin.write(frame(0x54, p))
    except BrokenPipeError:
        break
    c = read_commands()
    if c is None:
        break
    vl, vr, sl, sr, out, ctl = c
    if ctl & 1 and not last_ctl & 1:
        enc0 = [le.getValue(), re_.getValue()]
        yaw0 = yaw_unwrapped
    last_ctl = ctl
    lm.setVelocity(max(-31, min(31, vl / 1000.0 / R_WHEEL)))
    rm.setVelocity(max(-31, min(31, vr / 1000.0 / R_WHEEL)))
    gl.setPosition(max(0.0, min(1.4, (sl - SERVO_L_OPEN) / float(SERVO_L_CLOSED - SERVO_L_OPEN) * GRIP_CLOSED_RAD)))
    gr.setPosition(-max(0.0, min(1.4, (SERVO_R_OPEN - sr) / float(SERVO_R_OPEN - SERVO_R_CLOSED) * GRIP_CLOSED_RAD)))
    led_b.set(1 if out & 1 else 0)
    led_g.set(1 if out & 2 else 0)
brain.kill()

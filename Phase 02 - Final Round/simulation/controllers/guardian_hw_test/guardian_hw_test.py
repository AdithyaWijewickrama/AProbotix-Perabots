"""
guardian_hw_test.py - GuardianBot hardware self-test (Webots controller).

Use it to check the robot on its own before running main_controller:
  1. prints every distance sensor, the 8 line sensors and the piece colour sensor
  2. backs the rear pocket over the blue collection piece (10 cm behind the robot)
  3. closes the gripper, spins 180 deg and drives 20 cm carrying the piece
  4. opens the gripper and drives away, leaving the piece behind

Set the robot's controller field to "guardian_hw_test" to run it.
The same sequence is the bench test to run on the real robot.
"""
import math
from controller import Robot

TS = 16
R_WHEEL, HALF_TRACK = 0.020, 0.0555

robot = Robot()
lm, rm = robot.getDevice("left_wheel_motor"), robot.getDevice("right_wheel_motor")
for m in (lm, rm):
    m.setPosition(float("inf"))
    m.setVelocity(0)
le, re_ = robot.getDevice("left_wheel_sensor"), robot.getDevice("right_wheel_sensor")
le.enable(TS)
re_.enable(TS)
imu = robot.getDevice("imu")
imu.enable(TS)
grip = robot.getDevice("gripper::left")          # coupled: also moves gripper::right
grip_pos = robot.getDevice("gripper_sensor_left")
grip_pos.enable(TS)
TOF = ["us_front", "us_left", "us_right", "us_back", "ir_fl", "ir_fr", "ir_left", "ir_right"]
tof = {n: robot.getDevice(n) for n in TOF}
for d in tof.values():
    d.enable(TS)
gs = [robot.getDevice("gs%d" % i) for i in range(8)]
for d in gs:
    d.enable(TS)
pcam = robot.getDevice("piece_cam")
pcam.enable(TS)

GRIP_OPEN, GRIP_CLOSED = 0.0, 1.05


def step(n=1):
    for _ in range(n):
        if robot.step(TS) == -1:
            raise SystemExit


def drive(dist, v=0.10):
    """Straight move using encoders (dist > 0 forward)."""
    l0, r0 = le.getValue(), re_.getValue()
    w = math.copysign(v / R_WHEEL, dist)
    lm.setVelocity(w)
    rm.setVelocity(w)
    while abs((le.getValue() - l0 + re_.getValue() - r0) * 0.5 * R_WHEEL) < abs(dist):
        step()
    lm.setVelocity(0)
    rm.setVelocity(0)
    step(10)


def yaw():
    return imu.getRollPitchYaw()[2]


def turn(angle):
    target = yaw() + angle
    while True:
        e = (target - yaw() + math.pi) % (2 * math.pi) - math.pi
        if abs(e) < 0.01:
            break
        w = max(0.4, min(3.0, 4.0 * abs(e))) * (1 if e > 0 else -1)
        lm.setVelocity(-w * HALF_TRACK / R_WHEEL)
        rm.setVelocity(w * HALF_TRACK / R_WHEEL)
        step()
    lm.setVelocity(0)
    rm.setVelocity(0)
    step(10)


def piece_colour():
    img = pcam.getImage()
    r = g = b = 0
    for y in range(pcam.getHeight()):
        for x in range(pcam.getWidth()):
            r += pcam.imageGetRed(img, pcam.getWidth(), x, y)
            g += pcam.imageGetGreen(img, pcam.getWidth(), x, y)
            b += pcam.imageGetBlue(img, pcam.getWidth(), x, y)
    if b > 1.4 * g and b > 1.4 * r:
        return "BLUE"
    if g > 1.4 * b and g > 1.4 * r:
        return "GREEN"
    return "none"


step(20)
print("distance sensors (mm):", {n: int(d.getValue()) for n, d in tof.items()})
print("line sensors:", [int(d.getValue()) for d in gs])

grip.setPosition(GRIP_OPEN)
print("backing into the piece ...")
drive(-0.10, 0.06)
print("colour sensor sees %s" % piece_colour())
grip.setPosition(GRIP_CLOSED)
step(40)
print("gripper closed at %.2f rad" % grip_pos.getValue())
turn(math.pi)
drive(0.20, 0.15)
print("after 180 deg turn + 20 cm: colour sensor sees %s" % piece_colour())
grip.setPosition(GRIP_OPEN)
step(40)
drive(0.10, 0.10)
print("released, colour sensor sees %s" % piece_colour())
print("HW TEST DONE")
while robot.step(TS) != -1:
    pass

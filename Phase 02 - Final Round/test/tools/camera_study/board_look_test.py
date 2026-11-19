"""Fix 4 camera study: hold still, set the camera tilt servo, save one 96x64 'cam_hi' frame (BGRA) and quit."""
import json, os
from controller import Supervisor
r = Supervisor()
TS = int(r.getBasicTimeStep())
cam = r.getDevice('cam_hi'); cam.enable(TS)
for n in ('left_wheel_motor', 'right_wheel_motor'):
    m = r.getDevice(n); m.setPosition(float('inf')); m.setVelocity(0)
r.getDevice('cam_tilt').setPosition(-float(os.environ['LOOK_TILT']))
for _ in range(60):
    r.step(TS)
out = os.environ['LOOK_OUT']
open(out + '_0.raw', 'wb').write(cam.getImage())
json.dump({'w': cam.getWidth(), 'h': cam.getHeight()}, open(out + '_meta.json', 'w'))
r.simulationQuit(0)

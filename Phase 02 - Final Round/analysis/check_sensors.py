import re

with open('webots_project/protos/TwoWheelRobot.proto', 'r', encoding='utf-8') as f:
    text = f.read()

pattern = r'DistanceSensor\s*\{\s*translation\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)\s+rotation\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)\s+name\s+"([^"]+)"'
for m in re.finditer(pattern, text):
    t = m.groups()
    print(f'{t[7]:10s} trans=({t[0]:>7s}, {t[1]:>7s}, {t[2]:>7s}) rot=({t[3]}, {t[4]}, {t[5]}, {t[6]:>7s})')

import re

with open(r'D:\PeraBots\PeraBots_UC\protos\Arena.proto', 'r') as f:
    text = f.read()

names = re.findall(r'name "([^"]+)"', text)
print(f"Total solids in Arena.proto: {len(names)}")
for n in names:
    print(n)

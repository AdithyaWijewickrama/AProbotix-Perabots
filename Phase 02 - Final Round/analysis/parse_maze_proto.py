with open(r'D:\PeraBots\PeraBots_UC\protos\Arena.proto', 'r') as f:
    lines = f.readlines()

current_name = None
current_trans = None
current_size = None

for line in lines:
    line_s = line.strip()
    if line_s.startswith('name "'):
        current_name = line_s.split('"')[1]
        print(f"{current_name:28s} trans={current_trans:30s} size={current_size}")
        current_name = None
    elif line_s.startswith('translation '):
        current_trans = line_s.replace('translation ', '')
    elif line_s.startswith('size '):
        current_size = line_s.replace('size ', '')

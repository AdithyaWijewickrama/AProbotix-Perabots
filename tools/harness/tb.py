import re, sys
d = sys.argv[1]
rows = []
for l in open(d + '/controllers/main_controller/ctrl_log.txt'):
    m = re.match(r'EST t=([\d.]+) axle=\(([-\d.]+),([-\d.]+)\) yaw=([-\d.]+) cmd=([-\d.]+),([-\d.]+)', l)
    if m: rows.append(tuple(map(float, m.groups())))
def seg(t0, t1):
    acc = {'pivot': 0, 'drive': 0, 'idle': 0}; npiv = 0; last = None
    for t, x, y, yw, L, R in rows:
        if not (t0 <= t < t1): continue
        if abs(L) < 0.05 and abs(R) < 0.05: k = 'idle'
        elif L * R < 0: k = 'pivot'
        else: k = 'drive'
        acc[k] += 0.016
        if k == 'pivot' and last != 'pivot': npiv += 1
        last = k
    return {k: round(v, 1) for k, v in acc.items()}, npiv
t = [float(x) for x in sys.argv[2:]]
for a, b in zip(t, t[1:]):
    print('%.1f-%.1f' % (a, b), seg(a, b))

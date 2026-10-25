import re, sys, math
d = sys.argv[1]
seq = []
for l in open(d + '/controllers/main_controller/ctrl_log.txt'):
    m = re.match(r'EST t=([\d.]+) axle=\(([-\d.]+),([-\d.]+)\)', l)
    if not m: continue
    t, x, y = map(float, m.groups())
    if y < 1.33: continue
    c, r = int((x - 0.095) // 0.25), int((y - 1.33) // 0.25)
    if not seq or seq[-1][1] != (c, r):
        seq.append((t, (c, r)))
print(' '.join('c%dr%d@%.0f' % (c + 1, r + 1, t) for t, (c, r) in seq))
print('cells entered:', len(seq), 'unique:', len(set(s[1] for s in seq)))

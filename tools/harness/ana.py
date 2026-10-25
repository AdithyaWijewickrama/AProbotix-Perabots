import sys, re, math
d = sys.argv[1]
est = {}
for l in open(d + '/controllers/main_controller/ctrl_log.txt'):
    m = re.match(r'EST t=([\d.]+) axle=\(([-\d.]+),([-\d.]+)\) yaw=([-\d.]+) cmd=([-\d.]+),([-\d.]+)', l)
    if m: est[round(float(m.group(1)), 3)] = tuple(map(float, m.groups()[1:]))
tru = {}
for l in open(d + '/controllers/judge/judge_log.txt'):
    m = re.match(r'TRUE t=([\d.]+) axle=\(([-\d.]+),([-\d.]+)\) yaw=([-\d.]+)', l)
    if m: tru[round(float(m.group(1)), 3)] = tuple(map(float, m.groups()[1:]))
ts = sorted(set(est) & set(tru))
tmax = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
prev = None
acc = {'pivot': 0, 'straight': 0, 'curve': 0, 'slow': 0}
for t in ts:
    if t > tmax: break
    e, g = est[t], tru[t]
    err = (e[0] - g[0], e[1] - g[1])
    if prev:
        de = (err[0] - prev[0], err[1] - prev[1])
        L, R = e[3], e[4]
        if abs(L + R) < 0.5 * abs(L - R): k = 'pivot'
        elif abs(L - R) < 0.3: k = 'straight'
        else: k = 'curve'
        if abs(L) + abs(R) < 3 and k != 'pivot': k = 'slow'
        acc[k] += math.hypot(*de)
    prev = err
print('drift by motion type (sum of |d err| m):', {k: round(v, 3) for k, v in acc.items()})
for t in ts[::31]:
    if t > tmax: break
    e, g = est[t], tru[t]
    print('t=%.2f err=(%+.3f,%+.3f) cmd=%.1f,%.1f yaw=%.2f' % (t, e[0]-g[0], e[1]-g[1], e[3], e[4], e[2]))

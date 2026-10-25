# makes a copy of the controller with different accel/brake/pivot numbers
# usage: python mkvar.py <name> <MAX_DV> <ACC> <pivot_decel>
# then:  $env:CTRL="...\<name>.py"; ..\harness\batch.ps1 -seeds 0,17,18
import os, re, sys
here = os.path.dirname(os.path.abspath(__file__))
src = os.path.join(here, '..', '..', 'controllers', 'main_controller', 'main_controller.py')
s = open(src, encoding='utf-8').read()
name, dv, acc, ad = sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4]
t = re.sub(r'^MAX_DV = [\d.]+', 'MAX_DV = ' + dv, s, flags=re.M)
t = re.sub(r'^ACC = [\d.]+', 'ACC = ' + acc, t, flags=re.M)
t = re.sub(r'math.sqrt\(2 \* [\d.]+ \* abs\(e\)\)', 'math.sqrt(2 * %s * abs(e))' % ad, t)
open(os.path.join(here, name + '.py'), 'w', encoding='utf-8').write(t)
print('wrote', name)

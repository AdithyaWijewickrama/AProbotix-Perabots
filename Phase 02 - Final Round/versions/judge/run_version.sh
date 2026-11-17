# usage: runold.sh tag srcdir world.wbt
set -e
R=/tmp/runs/$1; rm -rf $R; mkdir -p $R; cp -r $2/* $R/
mkdir -p $R/controllers/judge $R/protos; cp /tmp/judge/judge.py $R/controllers/judge/
cp /tmp/p2/simulation/protos/Arena.proto /tmp/p2/simulation/protos/Obstacles.proto $R/protos/
[ -f $R/controllers/main_controller/main_controller_twowheel.py ] && cp $R/controllers/main_controller/main_controller_twowheel.py $R/controllers/main_controller/main_controller.py
printf '\nRobot {\n  name "judge"\n  controller "judge"\n  supervisor TRUE\n}\n' >> $R/worlds/$3
cd $R && export WEBOTS_HOME=/usr/local/webots USER=claude
timeout 560 xvfb-run -a -s "-screen 0 1280x1024x24" webots --batch --mode=fast --stdout --stderr --minimize --no-rendering worlds/$3 > /tmp/runs/$1.log 2>&1 || true
grep "JUDGE RESULT" /tmp/runs/$1.log || tail -3 /tmp/runs/$1.log

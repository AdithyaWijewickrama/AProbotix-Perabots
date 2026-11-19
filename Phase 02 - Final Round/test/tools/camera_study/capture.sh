#!/bin/bash
# capture.sh <world> <tilt up, rad> [x y yaw] - one 96x64 board-look frame from a still robot (Fix 4 camera study).
# Default pose: the exit junction (2.21, 3.05) facing the board. Frames go to $LOOK_DIR (default /tmp/look).
# The detector (detect.py) then reads them:  python detect.py <camera pitch = 0.50 - tilt>
NAME=$1; TILT=$2; X=${3:-2.21}; Y=${4:-3.05}; A=${5:-3.14159}
T=$(cd "$(dirname "$0")/../.." && pwd); ROOT=$(dirname "$T")
P=$(python3 -c "print('%.4f' % (0.50 - $TILT))")
OUT=${LOOK_DIR:-/tmp/look}; R=$OUT/run_${NAME}_$P; rm -rf $R; mkdir -p $R/worlds $R/protos $R/controllers/board_look_test $R/out
cp $T/protos/Arena_$NAME.proto $T/protos/Obstacles_$NAME.proto $ROOT/simulation/protos/GuardianBot.proto $R/protos/
cp "$(dirname "$0")/board_look_test.py" $R/controllers/board_look_test/
sed -e 's|"../../simulation/protos/GuardianBot.proto"|"../protos/GuardianBot.proto"|' -e 's|controller "test_runner"|controller "board_look_test"\n  supervisor TRUE|' \
    -e "/^GuardianBot {/,/^}/{s/^  translation .*/  translation $X $Y 0/;s/^  rotation .*/  rotation 0 0 1 $A/}" $T/worlds/$NAME.wbt > $R/worlds/$NAME.wbt
export WEBOTS_HOME=${WEBOTS_HOME:-/usr/local/webots} USER=${USER:-claude} LOOK_OUT=$R/out/img LOOK_TILT=$TILT
timeout 120 xvfb-run -a -s "-screen 0 1280x1024x24" webots --batch --mode=fast --stdout --stderr --minimize --no-rendering \
  --port=$((1300 + RANDOM % 500)) $R/worlds/$NAME.wbt > $R/log.txt 2>&1
ls $R/out

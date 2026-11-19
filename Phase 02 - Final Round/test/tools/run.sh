#!/bin/bash
# run.sh <world name> [python|sil] - one headless judged run of a test world (Linux / Webots R2025a).
#   python (default): the Python controller simulation/controllers/main_controller
#   sil: the real ESP32-S3 brain firmware (software-in-the-loop) with mega_sil
# Builds a throwaway copy in $RUNS/<name>_<ctrl> (default /tmp/gb_runs), adds the judge,
# runs Webots in fast mode with no rendering, and copies the logs to test/results/runs/<name>_<ctrl>/.
set -e
NAME=$1; CTRL=${2:-python}
T=$(cd "$(dirname "$0")/.." && pwd); ROOT=$(dirname "$T")
RUNS=${RUNS:-/tmp/gb_runs}; RR=$RUNS/${NAME}_$CTRL; R=$RR/simulation      # same layout as the project: simulation/ + firmware/
rm -rf "$RR"; mkdir -p "$R/worlds" "$R/protos" "$R/controllers"
cp "$T/protos/Arena_$NAME.proto" "$T/protos/Obstacles_$NAME.proto" "$ROOT/simulation/protos/GuardianBot.proto" "$R/protos/"
cp -r "$ROOT/simulation/controllers/main_controller" "$R/controllers/"
mkdir -p "$R/controllers/judge"; cp "$T/tools/judge.py" "$R/controllers/judge/"
W="$R/worlds/$NAME.wbt"
sed -e 's|"../../simulation/protos/GuardianBot.proto"|"../protos/GuardianBot.proto"|' -e 's|controller "test_runner"|controller "main_controller"|' "$T/worlds/$NAME.wbt" > "$W"
python3 - "$T/truth/$NAME.json" "$R/controllers/main_controller/clues.json" <<'PY'
import json, sys
t = json.load(open(sys.argv[1])); c = t["clues"]
json.dump([c["blue"], c["green"], c["right"], c["left"]], open(sys.argv[2], "w"))
PY
if [ "$CTRL" = sil ]; then
  cp -r "$ROOT/simulation/controllers/mega_sil" "$R/controllers/"
  cp -r "$ROOT/firmware" "$RR/"
  ( cd "$RR" && bash firmware/sil/build_sil.sh >/dev/null )
  sed -i 's|controller "main_controller"|controller "mega_sil"\n  supervisor TRUE|' "$W"
fi
printf '\nRobot {\n  name "judge"\n  controller "judge"\n  supervisor TRUE\n}\n' >> "$W"
export WEBOTS_HOME=${WEBOTS_HOME:-/usr/local/webots} USER=${USER:-claude}
cd "$R/controllers/judge"
timeout ${TLIM:-700} xvfb-run -a -s "-screen 0 1280x1024x24" webots --batch --mode=fast --stdout --stderr --minimize --no-rendering --port=$((1300 + RANDOM % 500)) "$W" > "$R/webots_log.txt" 2>&1 || true
O="$T/results/runs/${NAME}_$CTRL"; mkdir -p "$O"
cp "$R/webots_log.txt" "$O/"; cp "$R/controllers/judge/judge_path.csv" "$R/controllers/judge/judge_result.json" "$O/" 2>/dev/null || echo "no judge result"
[ -f "$R/controllers/mega_sil/sil_log.txt" ] && cp "$R/controllers/mega_sil/sil_log.txt" "$O/"
grep "JUDGE RESULT" "$R/webots_log.txt" || tail -5 "$R/webots_log.txt"

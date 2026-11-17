#!/bin/sh
# Host-side compile check of both sketches with stub Arduino headers (warnings on).
cd "$(dirname "$0")"
set -e
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I. -I../mega_body -include Arduino.h -x c++ ../mega_body/mega_body.ino -c -o /dev/null
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I. -I../esp32cam_brain -include Arduino.h -x c++ ../esp32cam_brain/esp32cam_brain.ino -c -o /dev/null
echo "both sketches compile (host check)"

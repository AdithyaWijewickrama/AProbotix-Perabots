#!/bin/sh
# Host-side compile check of both sketches with stub Arduino headers (warnings on).
cd "$(dirname "$0")"
set -e
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I. -I../mega_body -include Arduino.h -x c++ ../mega_body/mega_body.ino -c -o /dev/null
g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I. -I../esp32s3_brain -include Arduino.h -x c++ ../esp32s3_brain/esp32s3_brain.ino -c -o /dev/null
if [ -d ../esp32s3_testbench ]; then
  g++ -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -I. -I../esp32s3_testbench -include Arduino.h -x c++ ../esp32s3_testbench/esp32s3_testbench.ino -c -o /dev/null
fi
# the shared files must stay identical copies
cmp ../mega_body/protocol.h ../esp32s3_brain/protocol.h
if [ -d ../esp32s3_testbench ]; then
  cmp ../esp32s3_brain/protocol.h ../esp32s3_testbench/protocol.h
  cmp ../esp32s3_brain/cam_ov7670.h ../esp32s3_testbench/cam_ov7670.h
  cmp ../esp32s3_brain/colour.h ../esp32s3_testbench/colour.h
fi
echo "all sketches compile (host check), shared headers identical"

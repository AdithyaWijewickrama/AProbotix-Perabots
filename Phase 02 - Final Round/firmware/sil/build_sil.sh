#!/bin/sh
# Build the ESP32-S3 brain as a PC program for software-in-the-loop tests in Webots
# (simulation/controllers/mega_sil plays the Arduino Mega). Webots runs at 16 ms steps.
cd "$(dirname "$0")"
g++ -O2 -std=gnu++17 -Wall -Wno-unused-parameter -DSTEP_S=0.016f -DGS_INVERT=false -DTCS_WHITE_DEFAULT=255.0f \
  -I. -I../esp32s3_brain -include Arduino.h -x c++ ../esp32s3_brain/esp32s3_brain.ino -x c++ main.cpp -o brain_sil && echo "built firmware/sil/brain_sil"

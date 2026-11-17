#!/bin/sh
# Co-simulation: the real Mega firmware (built by ../build_hex.sh) in simavr + the custom chips, wired from
# ../diagram.json. Needs (Linux / WSL): simavr libsimavr-dev libelf-dev gcc-avr avr-libc arduino-core-avr, g++.
set -e
cd "$(dirname "$0")"
./run_tests.sh > /dev/null                     # builds the chip objects (and checks them)
(cd .. && ./build_hex.sh)
python3 netlist.py _build/wiring.txt
g++ -O2 -w -std=c++17 -D__timer_t_defined -I. -c mock_wokwi.cpp -o _build/mock.o
g++ -O2 -w -std=c++17 -I. -o _build/cosim cosim.cpp _build/mock.o _build/vl53.o _build/gyro.o _build/tcs.o _build/qtr.o \
    _build/motor.o _build/brain.o -lsimavr -lelf -lm
./_build/cosim ../build/sketch.ino.elf _build/wiring.txt

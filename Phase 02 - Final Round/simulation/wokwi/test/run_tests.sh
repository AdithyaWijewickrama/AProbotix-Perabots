#!/bin/sh
# Unit tests for the Wokwi custom chips on a PC (g++). Downloads the Pololu VL53L0X library once.
set -e
cd "$(dirname "$0")"
mkdir -p _deps _build
[ -f _deps/VL53L0X.cpp ] || { curl -sSL -o _deps/VL53L0X.cpp https://raw.githubusercontent.com/pololu/vl53l0x-arduino/master/VL53L0X.cpp;
                              curl -sSL -o _deps/VL53L0X.h https://raw.githubusercontent.com/pololu/vl53l0x-arduino/master/VL53L0X.h; }
CF="-O1 -w -I../chips -D__timer_t_defined"
gcc $CF -Dchip_init=vl53_init -c ../chips/vl53l0x.chip.c -o _build/vl53.o
gcc $CF -Dchip_init=gyro_init -c ../chips/l3g4200d.chip.c -o _build/gyro.o
gcc $CF -Dchip_init=tcs_init -c ../chips/tcs3200.chip.c -o _build/tcs.o
gcc $CF -Dchip_init=qtr_init -c ../chips/qtr8rc.chip.c -o _build/qtr.o
gcc $CF -Dchip_init=motor_init -c ../chips/n20motor.chip.c -o _build/motor.o
gcc $CF -Dchip_init=brain_init -c ../chips/brainstub.chip.c -o _build/brain.o
g++ -O1 -w -std=c++17 -D__timer_t_defined -Ishim -I_deps -I. -o _build/test_chips test_chips.cpp mock_wokwi.cpp shim/shim.cpp _deps/VL53L0X.cpp \
    _build/vl53.o _build/gyro.o _build/tcs.o _build/qtr.o _build/motor.o _build/brain.o -lm
./_build/test_chips

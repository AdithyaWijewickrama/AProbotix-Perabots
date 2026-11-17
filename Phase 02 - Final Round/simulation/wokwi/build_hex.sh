#!/bin/sh
# Builds firmware/mega_body for the Arduino Mega with plain avr-gcc -> build/sketch.ino.hex / .elf
# (what wokwi.toml points at). On Windows simply use the Arduino IDE / arduino-cli instead:
#   arduino-cli compile -b arduino:avr:mega --output-dir build project
# Needs (Linux): gcc-avr avr-libc arduino-core-avr. The libraries VL53L0X (Pololu), SSD1306Ascii and Servo are
# downloaded into test/_deps/libs the first time (or set LIBS to your Arduino libraries folder).
set -e
cd "$(dirname "$0")"
CORE=${CORE:-/usr/share/arduino/hardware/arduino/avr}
LIBS=${LIBS:-test/_deps/libs}
if [ ! -f $LIBS/SSD1306Ascii/SSD1306Ascii.cpp ]; then          # fetch the three libraries once
  R=https://raw.githubusercontent.com
  mkdir -p $LIBS/VL53L0X $LIBS/Servo/avr $LIBS/SSD1306Ascii/fonts
  for f in VL53L0X.h VL53L0X.cpp; do curl -sSL -o $LIBS/VL53L0X/$f $R/pololu/vl53l0x-arduino/master/$f; done
  for f in Servo.h avr/Servo.cpp avr/ServoTimers.h; do curl -sSL -o $LIBS/Servo/$f $R/arduino-libraries/Servo/master/src/$f; done
  for f in SSD1306Ascii.h SSD1306Ascii.cpp SSD1306AsciiWire.h SSD1306init.h fonts/allFonts.h; do
    curl -sSL -o $LIBS/SSD1306Ascii/$f $R/greiman/SSD1306Ascii/master/src/$f; done
  for f in $(grep -o '#include "[^"]*"' $LIBS/SSD1306Ascii/fonts/allFonts.h | cut -d'"' -f2); do
    curl -sSL -o $LIBS/SSD1306Ascii/fonts/$f $R/greiman/SSD1306Ascii/master/src/fonts/$f; done
fi
B=build/obj; mkdir -p $B
F="-mmcu=atmega2560 -DF_CPU=16000000L -DARDUINO=10806 -DARDUINO_AVR_MEGA2560 -DARDUINO_ARCH_AVR -Os -ffunction-sections -fdata-sections -w"
I="-I$CORE/cores/arduino -I$CORE/variants/mega -I$CORE/libraries/Wire/src -I$CORE/libraries/Wire/src/utility \
   -I$LIBS/VL53L0X -I$LIBS/SSD1306Ascii -I$LIBS/Servo"
SRC="$(ls $CORE/cores/arduino/*.c $CORE/cores/arduino/*.cpp $CORE/libraries/Wire/src/*.cpp $CORE/libraries/Wire/src/utility/*.c \
       $LIBS/VL53L0X/VL53L0X.cpp $LIBS/SSD1306Ascii/SSD1306Ascii.cpp $LIBS/Servo/avr/Servo.cpp)"
OBJ=""
for s in $SRC; do
  o=$B/$(basename $s).o
  case $s in *.c) avr-gcc $F -std=gnu11 $I -c $s -o $o ;; *) avr-g++ $F -std=gnu++11 -fno-exceptions -fno-threadsafe-statics $I -c $s -o $o ;; esac
  OBJ="$OBJ $o"
done
cp ../../firmware/mega_body/mega_body.ino $B/sketch.cpp
sed -i '1i #include <Arduino.h>' $B/sketch.cpp
avr-g++ $F -std=gnu++11 -fno-exceptions -fno-threadsafe-statics $I -I../../firmware/mega_body -Wall -c $B/sketch.cpp -o $B/sketch.o
avr-gcc -mmcu=atmega2560 -Os -Wl,--gc-sections -o build/sketch.ino.elf $B/sketch.o $OBJ -lm
avr-objcopy -O ihex -R .eeprom build/sketch.ino.elf build/sketch.ino.hex
avr-size -C --mcu=atmega2560 build/sketch.ino.elf | grep -E "Program|Data"

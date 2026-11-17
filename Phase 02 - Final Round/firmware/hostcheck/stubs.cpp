#include "Arduino.h"
#include "Wire.h"
HardwareSerial Serial, Serial2;
TwoWire Wire;
volatile uint8_t PINB, PINE, PIND, PCMSK0, PCICR, TCCR3B, TCCR4B;

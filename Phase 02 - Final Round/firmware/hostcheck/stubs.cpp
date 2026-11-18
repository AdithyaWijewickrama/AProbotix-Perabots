#include "Arduino.h"
#include "Wire.h"
HardwareSerial Serial, Serial2;
TwoWire Wire;
volatile uint8_t PINB, PINE, PIND, PCMSK0, PCICR, PINA, PORTA, DDRA, TCCR3B, TCCR4B;
#include "SSD1306Ascii.h"
const DevType Adafruit128x32{};
const uint8_t System5x7[1] = {0};
WiFiClass WiFi; EspClass ESP;

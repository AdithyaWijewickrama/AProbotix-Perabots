// Minimal host-side stand-ins for the Arduino APIs used by GuardianBot firmware.
// Only for a syntax / type check on a PC:  ./check.sh   (real builds use the Arduino IDE)
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <algorithm>
using std::min; using std::max;
#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define CHANGE 1
#define PI 3.14159265358979f
#define A0 54
#define A1 55
#define A2 56
#define A3 57
#define A4 58
#define A5 59
#define A8 62
#define A9 63
#define A10 64
#define A11 65
#define A12 66
#define SERIAL_8N1 0
#define F(x) x
#define _BV(b) (1 << (b))
#define constrain(x, a, b) ((x) < (a) ? (a) : (x) > (b) ? (b) : (x))
#define digitalPinToInterrupt(p) (p)
#define ISR(v) void v(void)
inline void noInterrupts() {}
inline void interrupts() {}
inline uint32_t millis() { return 0; }
inline uint32_t micros() { return 0; }
inline void delay(uint32_t) {}
inline void delayMicroseconds(uint32_t) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return 1; }
inline int analogRead(int) { return 0; }
inline void analogWrite(int, int) {}
inline void attachInterrupt(int, void (*)(), int) {}
struct Stream {
  void begin(unsigned long) {}
  void begin(unsigned long, int, int, int) {}
  void setRxBufferSize(int) {}
  int available() { return 0; }
  int availableForWrite() { return 64; }
  int read() { return -1; }
  size_t write(uint8_t) { return 1; }
  size_t write(const uint8_t *, size_t n) { return n; }
  template <class T> void print(T) {}
  template <class T> void println(T) {}
  void printf(const char *f, ...) { va_list a; va_start(a, f); vprintf(f, a); va_end(a); }
};
typedef Stream HardwareSerial_;
struct HardwareSerial : Stream { HardwareSerial(int) {} HardwareSerial() {} };
extern HardwareSerial Serial, Serial2;
extern volatile uint8_t PINB, PINE, PIND, PCMSK0, PCICR;
#define PCINT2 2
#define PCINT3 3
#define PCINT4 4
#define PCINT5 5
#define PCINT6 6
#define PCINT7 7
#define PCIE0 0
// ---- v4 additions
#define A6 60
#define A7 61
#define A13 67
#define A14 68
#define A15 69
extern volatile uint8_t PINA, PORTA, DDRA, TCCR3B, TCCR4B;
inline void ledcAttach(int, int, int) {}
inline void ledcWrite(int, int) {}
inline size_t strnlen_s(const char *s, size_t n) { return strnlen(s, n); }

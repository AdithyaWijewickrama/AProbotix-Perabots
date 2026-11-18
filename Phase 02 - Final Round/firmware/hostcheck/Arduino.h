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
inline bool psramFound() { return true; }
// ---- v4.1 additions (test bench: String, WiFi, WebServer, ESP)
#include <string>
#define PROGMEM
#define PGM_P const char *
struct String {
  std::string s;
  String() {}
  String(const char *c) : s(c ? c : "") {}
  String(const std::string &x) : s(x) {}
  String(int v) : s(std::to_string(v)) {}
  String(unsigned v) : s(std::to_string(v)) {}
  String(long v) : s(std::to_string(v)) {}
  String(unsigned long v) : s(std::to_string(v)) {}
  void reserve(size_t n) { s.reserve(n); }
  const char *c_str() const { return s.c_str(); }
  long toInt() const { return atol(s.c_str()); }
  float toFloat() const { return (float)atof(s.c_str()); }
  String &operator+=(const String &o) { s += o.s; return *this; }
  String &operator+=(const char *o) { s += o; return *this; }
  String &operator+=(char c) { s += c; return *this; }
  String &operator+=(int v) { s += std::to_string(v); return *this; }
  String &operator+=(unsigned v) { s += std::to_string(v); return *this; }
  String &operator+=(uint16_t v) { s += std::to_string(v); return *this; }
  bool operator==(const char *o) const { return s == o; }
  friend String operator+(const String &a, const String &b) { return String(a.s + b.s); }
  friend String operator+(const String &a, const char *b) { return String(a.s + b); }
  friend String operator+(const char *a, const String &b) { return String(std::string(a) + b.s); }
};
struct IPAddress { String toString() const { return String("192.168.4.1"); } };
#define WIFI_AP 2
struct WiFiClass { void mode(int) {} bool softAP(const char *, const char *) { return true; } IPAddress softAPIP() { return {}; }
  int softAPgetStationNum() { return 0; } };
extern WiFiClass WiFi;
struct EspClass { uint32_t getPsramSize() { return 8u << 20; } };
extern EspClass ESP;
#define CONTENT_LENGTH_UNKNOWN ((size_t)-1)
struct WebServer {
  WebServer(int) {}
  void on(const char *, void (*)()) {}
  template <class F> void on(const char *, F) {}
  void begin() {}
  void handleClient() {}
  bool hasArg(const char *) { return false; }
  String arg(const char *) { return String(); }
  void send(int, const char *, const String &) {}
  void send(int, const char *, const char *) {}
  void send_P(int, const char *, const char *) {}
  void send_P(int, const char *, const char *, size_t) {}
  void setContentLength(size_t) {}
  void sendContent(const char *) {}
  void sendContent(const String &) {}
};

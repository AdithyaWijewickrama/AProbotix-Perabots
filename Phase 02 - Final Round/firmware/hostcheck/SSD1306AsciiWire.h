#pragma once
#include "SSD1306Ascii.h"
struct SSD1306AsciiWire { void begin(const DevType *, uint8_t) {} void setFont(const uint8_t *) {} void clear() {}
  void setCursor(int, int) {} void clearToEOL() {} template <class T> void print(T) {} template <class T> void print(T, int) {}
  template <class T> void println(T) {} };

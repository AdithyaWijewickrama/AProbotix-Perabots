#pragma once
#include <stdint.h>
struct TwoWire { void begin() {} void setClock(long) {} void setWireTimeout(int, bool) {} void beginTransmission(int) {}
  int endTransmission(bool = true) { return 0; } int requestFrom(int, int) { return 2; } int read() { return 0; } void write(uint8_t) {} };
extern TwoWire Wire;

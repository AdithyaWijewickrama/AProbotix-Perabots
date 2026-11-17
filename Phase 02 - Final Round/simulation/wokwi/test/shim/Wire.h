#pragma once
#include "Arduino.h"
class TwoWire {
 public:
  void begin() {}
  void setClock(uint32_t) {}
  void beginTransmission(uint8_t a);
  size_t write(uint8_t b);
  uint8_t endTransmission(bool stop = true);
  uint8_t requestFrom(uint8_t a, uint8_t n, uint8_t stop = 1);
  int available();
  int read();
 private:
  uint8_t addr = 0, buf[32] = {0}, n = 0, pos = 0, txn = 0, tx[32] = {0};
};
extern TwoWire Wire;

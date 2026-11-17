#pragma once
#include <stdint.h>
struct VL53L0X { void setTimeout(uint16_t) {} bool init(bool = true) { return true; } void setAddress(uint8_t) {}
  bool setMeasurementTimingBudget(uint32_t) { return true; } void startContinuous(uint32_t = 0) {}
  uint8_t readReg(uint8_t) { return 0; } uint16_t readReg16Bit(uint8_t) { return 0; } void writeReg(uint8_t, uint8_t) {} };

// Minimal Arduino shim so the real Pololu VL53L0X library runs against mock_wokwi on a PC.
#pragma once
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <cstring>
typedef uint8_t byte;
typedef bool boolean;
uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

// mock_wokwi.h - a tiny stand-in for the Wokwi simulator, to unit-test the custom chips on a PC.
// Pins with an "MCU side" driver, chip-side modes and pulls, edge watches, timers on a simulated
// nanosecond clock, an I2C bus master and UART capture.
#pragma once
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace mock {
void begin_chip(void (*init)(void), const std::map<std::string, int> &nets, const std::map<std::string, uint32_t> &attrs = {});
int net(const std::string &name);                 // net id (created on first use)
void mcu_drive(int net, int level);               // -1 = release (input, no pull)
int level(int net);
void set_attr(int chip, const std::string &name, uint32_t v);
int chip_count();
uint64_t now();
void run_for_ns(uint64_t ns);
inline void run_us(double us) { run_for_ns((uint64_t)(us * 1000)); }
// I2C master
bool i2c_start(uint8_t addr, bool read);
bool i2c_write(uint8_t b);
uint8_t i2c_read();
void i2c_stop();
// UART (MCU side)
std::vector<uint8_t> &uart_captured(int dev);
void uart_inject(int dev, const uint8_t *data, int n);
int conflicts();
// co-simulation hooks
void mcu_pullup(int net, bool on);                 // MCU pin is an input with its internal pull-up
extern std::function<void(int net, int level)> on_level;        // any net changed level
extern std::function<void(int dev, uint8_t byte)> on_uart_tx;   // a chip sent a UART byte
int net_count();
}

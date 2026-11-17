#include "Wire.h"
#include "../mock_wokwi.h"
TwoWire Wire;
uint32_t millis() { return (uint32_t)(mock::now() / 1000000ull); }
uint32_t micros() { return (uint32_t)(mock::now() / 1000ull); }
void delay(uint32_t ms) { mock::run_us(ms * 1000.0); }
void delayMicroseconds(uint32_t us) { mock::run_us(us); }
void TwoWire::beginTransmission(uint8_t a) { addr = a; txn = 0; }
size_t TwoWire::write(uint8_t b) { if (txn < 32) tx[txn++] = b; return 1; }
uint8_t TwoWire::endTransmission(bool stop) {
  mock::run_us(10.0 + txn * 22.5);                    // 400 kHz bus time
  if (!mock::i2c_start(addr, false)) { mock::i2c_stop(); return 2; }
  for (uint8_t i = 0; i < txn; i++) if (!mock::i2c_write(tx[i])) { mock::i2c_stop(); return 3; }
  if (stop) mock::i2c_stop();
  return 0;
}
uint8_t TwoWire::requestFrom(uint8_t a, uint8_t cnt, uint8_t stop) {
  mock::run_us(10.0 + cnt * 22.5);
  n = pos = 0;
  if (!mock::i2c_start(a, true)) { mock::i2c_stop(); return 0; }
  for (uint8_t i = 0; i < cnt && i < 32; i++) buf[n++] = mock::i2c_read();
  if (stop) mock::i2c_stop();
  return n;
}
int TwoWire::available() { return n - pos; }
int TwoWire::read() { return pos < n ? buf[pos++] : -1; }

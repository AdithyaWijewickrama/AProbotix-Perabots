// Software-in-the-loop Arduino layer: the ESP32-S3 brain runs as a PC program.
// Link (Serial1) = stdin/stdout binary pipe to the Webots "mega_sil" controller, Serial = stderr log.
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <chrono>
#include <algorithm>
using std::min; using std::max;
#define SERIAL_8N1 0
inline uint32_t millis() {
  using namespace std::chrono;
  static auto t0 = steady_clock::now();
  return (uint32_t)duration_cast<milliseconds>(steady_clock::now() - t0).count();
}
inline void delay(uint32_t ms) { usleep(ms * 1000); }
struct HardwareSerial {
  int in = -1, outFd = 2;
  uint8_t buf[4096]; int n = 0, p = 0;
  // port 1 = Mega link (stdin/stdout), port 2 = ESP32-CAM (fd $SIL_CAM_FD in, writes dropped), port 0 = log (stderr)
  HardwareSerial(int port = 0) { if (port == 1) { in = 0; outFd = 1; } else if (port == 2) { in = getenv("SIL_CAM_FD") ? atoi(getenv("SIL_CAM_FD")) : -1; outFd = -1; } }
  void begin(unsigned long) {}
  void begin(unsigned long, int, int, int) {}
  void setRxBufferSize(int) {}
  int available() {
    if (p < n) return n - p;
    if (in < 0) return 0;
    struct pollfd f = {in, POLLIN, 0};
    if (poll(&f, 1, 1) > 0) { n = ::read(in, buf, sizeof(buf)); p = 0; if (n <= 0) { if (in == 0) { fprintf(stderr, "link closed\n"); exit(0); } n = 0; in = -1; } }
    return n - p;
  }
  int read() { return available() ? buf[p++] : -1; }
  size_t write(const uint8_t *b, size_t k) { if (outFd < 0) return k; size_t w = ::write(outFd, b, k); return w; }
  void printf(const char *f, ...) { va_list a; va_start(a, f); vfprintf(stderr, f, a); va_end(a); fflush(stderr); }
};
extern HardwareSerial Serial;

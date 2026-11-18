#pragma once
#include <stddef.h>
struct Preferences { bool begin(const char *, bool = false) { return true; } void end() {}
  int getInt(const char *, int d = 0) { return d; } void putInt(const char *, int) {}
  size_t getBytes(const char *, void *, size_t) { return 0; } size_t putBytes(const char *, const void *, size_t n) { return n; }
  float getFloat(const char *, float d = 0) { return d; } size_t putFloat(const char *, float) { return 4; } };

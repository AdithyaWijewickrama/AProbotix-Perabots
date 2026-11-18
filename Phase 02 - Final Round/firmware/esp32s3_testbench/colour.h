// colour.h - colour classes for maze tiles (identical copy in esp32s3_brain/ and esp32s3_testbench/)
// Used for the camera grid (after camera white balance) and both TCS3200 sensors (after K4 white balance).
#pragma once
#include <math.h>
#include <stdint.h>

enum { FLOOR = 0, BLUE = 1, GREEN = 2, JUNK = -1 };

// TCS3200 counts (R, G, B, Clear per 6 ms window) -> 0..255 using the counts measured on white paper
inline void tcsRGB(const uint16_t *c, const float *white, float &r, float &g, float &b) {
  r = fminf(255.f, 255.f * c[0] / white[0]);
  g = fminf(255.f, 255.f * c[1] / white[1]);
  b = fminf(255.f, 255.f * c[2] / white[2]);
}

inline int classify(float r, float g, float b) {
  float m = fmaxf(r, fmaxf(g, b));
  if (m < 50) return FLOOR;
  if (b > 45 && b > 1.5f * g + 15 && b > 1.5f * r + 15) return BLUE;
  if (g > 45 && g > 1.5f * b + 15 && g > 1.5f * r + 15) return GREEN;
  if (r > 100 && r > 2 * g && r > 2 * b) return FLOOR;
  return JUNK;
}

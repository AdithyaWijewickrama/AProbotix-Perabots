// cam_ov7670.h - OV7670 camera on the ESP32-S3 N16R8 (identical copy in esp32s3_brain/ and esp32s3_testbench/)
// GuardianBot v4.1. The S3 lies flat on a vero carrier on the top deck, behind the camera mast; the camera
// connects through a 10 cm 18-wire ribbon. Keep it flat and in one piece; if the picture shows stripes,
// lower CAM_XCLK_HZ to 8 MHz.
//
// N16R8 notes: GPIO 35/36/37 belong to the octal PSRAM, 19/20 are USB, 0/3/45/46 are strapping pins,
// 43/44 are the USB-UART log. RESET is tied to 3V3 and PWDN to GND on the S3 carrier (saves two pins).
// SIOD / SIOC need 4.7 k pull-ups to 3V3 on most 18-pin OV7670 boards (they have none).
#pragma once
#ifndef SIL
#include "esp_camera.h"

#define CAM_PIN_D0    11
#define CAM_PIN_D1     9
#define CAM_PIN_D2     8
#define CAM_PIN_D3    10
#define CAM_PIN_D4    12
#define CAM_PIN_D5    18
#define CAM_PIN_D6    17
#define CAM_PIN_D7    16
#define CAM_PIN_PCLK  13
#define CAM_PIN_VSYNC  6
#define CAM_PIN_HREF   7
#define CAM_PIN_XCLK  15
#define CAM_PIN_SIOD   4
#define CAM_PIN_SIOC   5
#ifndef CAM_XCLK_HZ
#define CAM_XCLK_HZ 10000000    // 10 MHz: fine for QQVGA and kind to the ribbon; try 20 MHz for more fps
#endif
#ifndef CAM_SWAP_BYTES
#define CAM_SWAP_BYTES 1        // RGB565 arrives high byte first; set 0 if the colours look swapped
#endif
#ifndef CAM_VFLIP
#define CAM_VFLIP 0             // set so the floor is at the BOTTOM of the picture (check on the test page)
#endif
#ifndef CAM_HMIRROR
#define CAM_HMIRROR 0           // set so the robot's left is on the LEFT of the picture
#endif

static uint16_t camPID = 0;     // 0x76 for the OV7670 (VER 0x73)

inline bool camBegin() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = CAM_PIN_D0; c.pin_d1 = CAM_PIN_D1; c.pin_d2 = CAM_PIN_D2; c.pin_d3 = CAM_PIN_D3;
  c.pin_d4 = CAM_PIN_D4; c.pin_d5 = CAM_PIN_D5; c.pin_d6 = CAM_PIN_D6; c.pin_d7 = CAM_PIN_D7;
  c.pin_xclk = CAM_PIN_XCLK; c.pin_pclk = CAM_PIN_PCLK; c.pin_vsync = CAM_PIN_VSYNC; c.pin_href = CAM_PIN_HREF;
  c.pin_sccb_sda = CAM_PIN_SIOD; c.pin_sccb_scl = CAM_PIN_SIOC; c.pin_pwdn = -1; c.pin_reset = -1;
  c.xclk_freq_hz = CAM_XCLK_HZ;
  c.pixel_format = PIXFORMAT_RGB565;
  c.frame_size = FRAMESIZE_QQVGA;          // 160 x 120
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  if (esp_camera_init(&c) != ESP_OK) return false;
  sensor_t *s = esp_camera_sensor_get();
  camPID = s->id.PID;
  s->set_whitebal(s, 1); s->set_gain_ctrl(s, 1); s->set_exposure_ctrl(s, 1);   // the OV7670 driver supports these
  s->set_vflip(s, CAM_VFLIP);
  s->set_hmirror(s, CAM_HMIRROR);
  return true;
}

static inline uint16_t camPix(const uint16_t *p, int i) {
  uint16_t v = p[i];
#if CAM_SWAP_BYTES
  v = (v >> 8) | (v << 8);
#endif
  return v;
}

// Average the frame into a gw x gh RGB565 grid over the centre crop that spans cropFov of a lens with
// lensHfov (radians). full (optional) receives the whole frame, byte order fixed (W*H words).
inline bool camGrab(uint16_t *grid, int gw, int gh, float cropFov, float lensHfov, uint16_t *full = nullptr,
                    int *fw = nullptr, int *fh = nullptr) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;
  const int W = fb->width, H = fb->height;
  const uint16_t *px16 = (const uint16_t *)fb->buf;
  if (full) for (int i = 0; i < W * H; i++) full[i] = camPix(px16, i);
  if (fw) *fw = W;
  if (fh) *fh = H;
  int cw = cropFov >= lensHfov ? W : (int)(W * tanf(cropFov / 2) / tanf(lensHfov / 2));
  int ch = cw * gh / gw;
  if (ch > H) { ch = H; cw = ch * gw / gh; }
  int x0 = (W - cw) / 2, y0 = (H - ch) / 2;
  for (int gy = 0; gy < gh; gy++) {
    int ya = y0 + gy * ch / gh, yb = y0 + (gy + 1) * ch / gh;
    for (int gx = 0; gx < gw; gx++) {
      int xa = x0 + gx * cw / gw, xb = x0 + (gx + 1) * cw / gw;
      uint32_t r = 0, g = 0, b = 0, n = 0;
      for (int y = ya; y < yb; y++)
        for (int x = xa; x < xb; x++) {
          uint16_t v = camPix(px16, y * W + x);
          r += (v >> 11) & 0x1F; g += (v >> 5) & 0x3F; b += v & 0x1F; n++;
        }
      if (!n) n = 1;
      grid[gy * gw + gx] = ((r / n) << 11) | ((g / n) << 5) | (b / n);
    }
  }
  esp_camera_fb_return(fb);
  return true;
}
#endif

// white-balance gains (from K4 CAL on white paper) applied to an RGB565 pixel -> 0..255 floats
struct CamGain { float r = 1, g = 1, b = 1; };
static inline void camRGB(uint16_t v, const CamGain &k, float &r, float &g, float &b) {
  r = ((v >> 11) & 0x1F) * (255.0f / 31) * k.r;
  g = ((v >> 5) & 0x3F) * (255.0f / 63) * k.g;
  b = (v & 0x1F) * (255.0f / 31) * k.b;
  if (r > 255) r = 255;
  if (g > 255) g = 255;
  if (b > 255) b = 255;
}

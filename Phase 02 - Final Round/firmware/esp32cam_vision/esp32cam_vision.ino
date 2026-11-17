// =============================================================================
// GuardianBot v3 - ESP32-CAM (AI-Thinker, OV2640) "eye" firmware
//
// Grabs 160x120 RGB565 frames, crops the centre to the field of view the brain
// expects (CAM_FOV in esp32s3_brain/config.h) and averages it down to a 48x32 grid.
// Every frame (~15 fps) goes to the ESP32-S3 brain over UART0 (921600 baud):
//     A5 5A 'V' seq lenLo lenHi <48*32 RGB565, little-endian> sum8
// The brain does the colour classification and the projection onto the maze floor
// (same code as the Webots controller). Commands from the brain (1 byte + value):
//     'L' n : flash LED PWM 0..255 (0 = off; ~20 helps in a dark hall)
// Wi-Fi and Bluetooth are never started (rulebook 4.2).
//
// Board: "AI Thinker ESP32-CAM" (Arduino-ESP32 core). Upload through the ESP32-CAM-MB
// adapter, then wire U0TXD (GPIO1) -> ESP32-S3 GPIO15, U0RXD (GPIO3) <- ESP32-S3 GPIO16,
// 5V and GND from the 5.1 V buck. Nothing else may print on Serial.
// =============================================================================
#include <Arduino.h>
#include "esp_camera.h"

// ---- tuning
static const float LENS_HFOV = 0.94f;   // OV2640 standard lens, horizontal (rad). Calibrate: see README
static const float CAM_FOV   = 0.90f;   // must match esp32s3_brain/config.h
static const int GW = 48, GH = 32;      // output grid
static const uint32_t BAUD = 921600;

// ---- AI-Thinker pin map
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define FLASH_LED_PIN 4

static uint8_t out[6 + GW * GH * 2 + 1];
static uint8_t seq = 0;

bool camInit() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM; c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_RGB565;
  c.frame_size = FRAMESIZE_QQVGA;          // 160 x 120
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;        // always the newest frame (lowest latency)
  if (esp_camera_init(&c) != ESP_OK) return false;
  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 1);                   // auto white balance (lighting in the hall varies)
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_gain_ctrl(s, 1);
  s->set_saturation(s, 2);                 // stronger colours -> cleaner blue/green/red separation
  s->set_vflip(s, 1);                      // set 0 or 1 so the floor is at the BOTTOM of the image
  s->set_hmirror(s, 0);
  return true;
}

void setup() {
  Serial.begin(BAUD);
  ledcAttach(FLASH_LED_PIN, 5000, 8);
  ledcWrite(FLASH_LED_PIN, 0);
  if (!camInit()) {                        // blink the flash LED if the camera is missing
    for (;;) { ledcWrite(FLASH_LED_PIN, 30); delay(100); ledcWrite(FLASH_LED_PIN, 0); delay(900); }
  }
}

void loop() {
  while (Serial.available() >= 2) {
    int c = Serial.read(), v = Serial.read();
    if (c == 'L') ledcWrite(FLASH_LED_PIN, v & 0xFF);
  }
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  const int W = fb->width, H = fb->height;
  // centre crop with the brain's field of view and a 3:2 aspect
  int cw = (int)(W * tanf(CAM_FOV / 2) / tanf(LENS_HFOV / 2));
  if (cw > W) cw = W;
  int ch = cw * GH / GW;
  if (ch > H) { ch = H; cw = ch * GW / GH; }
  int x0 = (W - cw) / 2, y0 = (H - ch) / 2;
  const uint16_t *px = (const uint16_t *)fb->buf;
  uint8_t *p = out + 6;
  for (int gy = 0; gy < GH; gy++) {
    int ya = y0 + gy * ch / GH, yb = y0 + (gy + 1) * ch / GH;
    for (int gx = 0; gx < GW; gx++) {
      int xa = x0 + gx * cw / GW, xb = x0 + (gx + 1) * cw / GW;
      uint32_t r = 0, g = 0, b = 0, n = 0;
      for (int y = ya; y < yb; y++)
        for (int x = xa; x < xb; x++) {
          uint16_t v = px[y * W + x];
          v = (v >> 8) | (v << 8);          // the driver delivers RGB565 big-endian
          r += (v >> 11) & 0x1F; g += (v >> 5) & 0x3F; b += v & 0x1F; n++;
        }
      if (!n) n = 1;
      uint16_t o = ((r / n) << 11) | ((g / n) << 5) | (b / n);
      *p++ = o & 0xFF; *p++ = o >> 8;
    }
  }
  esp_camera_fb_return(fb);
  const uint16_t len = GW * GH * 2;
  out[0] = 0xA5; out[1] = 0x5A; out[2] = 'V'; out[3] = seq++; out[4] = len & 0xFF; out[5] = len >> 8;
  uint8_t s = 0;
  for (int i = 2; i < 6 + len; i++) s += out[i];
  out[6 + len] = s;
  Serial.write(out, sizeof(out));
}

#pragma once
#include <stdint.h>
#include <stddef.h>
enum { LEDC_CHANNEL_0, LEDC_TIMER_0, PIXFORMAT_RGB565, FRAMESIZE_QQVGA, CAMERA_FB_IN_PSRAM, CAMERA_GRAB_LATEST };
typedef int esp_err_t;
#define ESP_OK 0
struct camera_config_t { int ledc_channel, ledc_timer, pin_d0, pin_d1, pin_d2, pin_d3, pin_d4, pin_d5, pin_d6, pin_d7,
  pin_xclk, pin_pclk, pin_vsync, pin_href, pin_sccb_sda, pin_sccb_scl, pin_pwdn, pin_reset, xclk_freq_hz,
  pixel_format, frame_size, fb_count, fb_location, grab_mode; };
struct sensor_id_t { uint16_t PID, VER, MIDL, MIDH; };
struct sensor_t { sensor_id_t id; int (*set_whitebal)(sensor_t *, int); int (*set_awb_gain)(sensor_t *, int); int (*set_exposure_ctrl)(sensor_t *, int);
  int (*set_gain_ctrl)(sensor_t *, int); int (*set_saturation)(sensor_t *, int); int (*set_vflip)(sensor_t *, int); int (*set_hmirror)(sensor_t *, int); };
struct camera_fb_t { uint8_t *buf; size_t len; int width, height; };
inline esp_err_t esp_camera_init(const camera_config_t *) { return ESP_OK; }
inline sensor_t *esp_camera_sensor_get() { return nullptr; }
inline camera_fb_t *esp_camera_fb_get() { return nullptr; }
inline void esp_camera_fb_return(camera_fb_t *) {}
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) (void)(x)
#define portEXIT_CRITICAL(x) (void)(x)
inline void vTaskDelay(int) {}
inline int xTaskCreatePinnedToCore(void (*)(void *), const char *, int, void *, int, void *, int) { return 1; }

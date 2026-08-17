#include "Vision.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <math.h>

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

// Static helper functions for color conversions
static inline float srgbToLinear(float c) {
  return (c <= 0.04045f) ? (c / 12.92f) : powf((c + 0.055f) / 1.055f, 2.4f);
}

static inline float labF(float t) {
  return (t > 0.008856f) ? cbrtf(t) : (7.787f * t + 16.0f / 116.0f);
}

static inline bool inThreshold(int L, int A, int B, const LabThreshold &t) {
  return L >= t.l_min && L <= t.l_max &&
         A >= t.a_min && A <= t.a_max &&
         B >= t.b_min && B <= t.b_max;
}

Vision::Vision()
  : tracking_roi({0, 0, MAX_W, MAX_H}),
    target_locked(false),
    mask_buf(nullptr),
    stack_buf(nullptr) {
  memset(threshold_lut, 0, sizeof(threshold_lut));
}

Vision::~Vision() {
  if (mask_buf) {
    heap_caps_free(mask_buf);
  }
  if (stack_buf) {
    heap_caps_free(stack_buf);
  }
}

inline bool Vision::getMaskBit(int idx) {
  return (mask_buf[idx >> 3] >> (idx & 7)) & 1;
}

inline void Vision::clearMaskBit(int idx) {
  mask_buf[idx >> 3] &= ~(1 << (idx & 7));
}

inline void Vision::setMaskBit(int idx) {
  mask_buf[idx >> 3] |= (1 << (idx & 7));
}

inline uint8_t Vision::getThresholdLUT(uint16_t px) {
  return (threshold_lut[px >> 3] >> (px & 7)) & 0x01;
}

void Vision::initThresholdLUT() {
  memset(threshold_lut, 0, sizeof(threshold_lut));

  float r_lin[32], g_lin[64], b_lin[32];
  for (int r = 0; r < 32; r++) r_lin[r] = srgbToLinear(r / 31.0f);
  for (int g = 0; g < 64; g++) g_lin[g] = srgbToLinear(g / 63.0f);
  for (int b = 0; b < 32; b++) b_lin[b] = srgbToLinear(b / 31.0f);

  for (uint32_t px = 0; px < 65536; px++) {
    uint16_t native_px = __builtin_bswap16((uint16_t)px);

    uint8_t r5 = (native_px >> 11) & 0x1F;
    uint8_t g6 = (native_px >> 5) & 0x3F;
    uint8_t b5 = native_px & 0x1F;

    float r = r_lin[r5];
    float g = g_lin[g6];
    float b = b_lin[b5];

    float X = (r * 0.4124f + g * 0.3576f + b * 0.1805f) / 0.95047f;
    float Y = (r * 0.2126f + g * 0.7152f + b * 0.0722f);
    float Z = (r * 0.0193f + g * 0.1192f + b * 0.9505f) / 1.08883f;

    float fx = labF(X), fy = labF(Y), fz = labF(Z);

    int L = (int)(116.0f * fy - 16.0f);
    int A = (int)(500.0f * (fx - fy));
    int B = (int)(200.0f * (fy - fz));

    if (inThreshold(L, A, B, THRESHOLD_BLIMP)) {
      threshold_lut[px >> 3] |= (1 << (px & 7));
    }
  }
}

IRAM_ATTR void Vision::generateSramMaskFast(const camera_fb_t *fb, Rect roi) {
  const uint16_t *buf = (const uint16_t *)fb->buf;
  int stride = fb->width;

  for (int ry = 0; ry < roi.h; ry++) {
    int row_pixel_offset = (roi.y + ry) * stride + roi.x;
    const uint16_t *src = &buf[row_pixel_offset];
    int rx = 0;

    while (rx < roi.w && ((row_pixel_offset + rx) & 7) != 0) {
      int bit_offset = row_pixel_offset + rx;
      if (getThresholdLUT(src[rx])) setMaskBit(bit_offset);
      else clearMaskBit(bit_offset);
      rx++;
    }

    for (; rx <= roi.w - 8; rx += 8) {
      uint8_t b = 0;
      b |= (getThresholdLUT(src[rx + 0]) << 0);
      b |= (getThresholdLUT(src[rx + 1]) << 1);
      b |= (getThresholdLUT(src[rx + 2]) << 2);
      b |= (getThresholdLUT(src[rx + 3]) << 3);
      b |= (getThresholdLUT(src[rx + 4]) << 4);
      b |= (getThresholdLUT(src[rx + 5]) << 5);
      b |= (getThresholdLUT(src[rx + 6]) << 6);
      b |= (getThresholdLUT(src[rx + 7]) << 7);

      int bit_offset = row_pixel_offset + rx;
      mask_buf[bit_offset >> 3] = b;
    }

    for (; rx < roi.w; rx++) {
      int bit_offset = row_pixel_offset + rx;
      if (getThresholdLUT(src[rx])) setMaskBit(bit_offset);
      else clearMaskBit(bit_offset);
    }
  }
}

IRAM_ATTR Blob Vision::findLargestBlob(camera_fb_t *fb, Rect roi, uint32_t area_threshold) {
  int stride = fb->width;
  int rw = roi.w, rh = roi.h;

  generateSramMaskFast(fb, roi);

  Blob best = {0, 0, 0, 0, 0, 0, 0};

  for (int ry = 0; ry < rh; ry += SCAN_STEP) {
    int img_y = roi.y + ry;
    int row_idx = img_y * stride;

    for (int rx = 0; rx < rw; rx += SCAN_STEP) {
      int img_x = roi.x + rx;
      int idx = row_idx + img_x;

      if ((idx & 7) == 0 && rx <= rw - 8 && mask_buf[idx >> 3] == 0x00) {
        rx += 7;
        continue;
      }

      if (!getMaskBit(idx)) continue;

      int stack_size = 0;
      stack_buf[stack_size++] = ((uint32_t)ry << 16) | (uint16_t)rx;

      long sumX = 0, sumY = 0;
      uint32_t count = 0;
      int minX = rx, maxX = rx, minY = ry, maxY = ry;

      while (stack_size > 0) {
        uint32_t pos = stack_buf[--stack_size];
        int seed_x = pos & 0xFFFF;
        int seed_y = pos >> 16;

        int row_offset = (roi.y + seed_y) * stride + roi.x;
        int seed_idx = row_offset + seed_x;
        if (!getMaskBit(seed_idx)) continue;

        int lx = seed_x;
        while (lx > 0) {
          int test_idx = row_offset + lx - 1;
          if ((test_idx & 7) == 7 && lx >= 8 && mask_buf[(test_idx - 7) >> 3] == 0xFF) {
            lx -= 8;
          } else if (getMaskBit(test_idx)) {
            lx--;
          } else {
            break;
          }
        }

        int rx_span = seed_x;
        while (rx_span < rw - 1) {
          int test_idx = row_offset + rx_span + 1;
          if ((test_idx & 7) == 0 && rx_span <= rw - 9 && mask_buf[test_idx >> 3] == 0xFF) {
            rx_span += 8;
          } else if (getMaskBit(test_idx)) {
            rx_span++;
          } else {
            break;
          }
        }

        for (int x = lx; x <= rx_span; x++) {
          int bit_idx = row_offset + x;

          if ((bit_idx & 7) == 0 && x + 7 <= rx_span && mask_buf[bit_idx >> 3] == 0xFF) {
            mask_buf[bit_idx >> 3] = 0x00;
            count += 8;
            sumX += (x * 8 + 28);
            sumY += seed_y * 8;
            x += 7;
          } else {
            if (getMaskBit(bit_idx)) {
              clearMaskBit(bit_idx);
              count++;
              sumX += x;
              sumY += seed_y;
            }
          }
        }

        if (lx < minX) minX = lx;
        if (rx_span > maxX) maxX = rx_span;
        if (seed_y < minY) minY = seed_y;
        if (seed_y > maxY) maxY = seed_y;

        const int dys[2] = {-1, 1};
        for (int i = 0; i < 2; i++) {
          int ny = seed_y + dys[i];
          if (ny < 0 || ny >= rh) continue;

          int n_row_offset = (roi.y + ny) * stride + roi.x;
          bool in_span = false;

          for (int x = lx; x <= rx_span; x++) {
            int n_idx = n_row_offset + x;

            if ((n_idx & 7) == 0 && x + 7 <= rx_span && mask_buf[n_idx >> 3] == 0x00) {
              in_span = false;
              x += 7;
              continue;
            }

            if (getMaskBit(n_idx)) {
              if (!in_span) {
                if (stack_size < MAX_STACK_SIZE) {
                  stack_buf[stack_size++] = ((uint32_t)ny << 16) | (uint16_t)x;
                }
                in_span = true;
              }
            } else {
              in_span = false;
            }
          }
        }
      }

      if (count >= area_threshold && count > best.pixels) {
        best.pixels = count;
        best.x = roi.x + minX;
        best.y = roi.y + minY;
        best.w = maxX - minX + 1;
        best.h = maxY - minY + 1;
        best.cx = roi.x + (int)(sumX / count);
        best.cy = roi.y + (int)(sumY / count);
      }
    }
  }
  return best;
}

static inline bool inBounds(int x, int y) {
  return x >= 0 && x < MAX_W && y >= 0 && y < MAX_H;
}

void Vision::errorLoop() {
  while (true) {
    digitalWrite(LED_GPIO_NUM, LOW);
    delay(100);
    digitalWrite(LED_GPIO_NUM, HIGH);
    delay(100);
  }
}

void Vision::setupCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size   = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_RGB565;
  config.grab_mode    = CAMERA_GRAB_LATEST;
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.fb_count     = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    errorLoop();
  }
}

FrameResult Vision::processFrame() {
  FrameResult result = {};
  result.valid = false;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    return result;
  }

  result.camera_fb = fb;
  result.work_fb = *fb;

  if (target_locked) {
    result.blob = findLargestBlob(&result.work_fb, tracking_roi, AREA_THRESHOLD_LOCKED);
  } else {
    Rect full = {0, 0, MAX_W, MAX_H};
    result.blob = findLargestBlob(&result.work_fb, full, AREA_THRESHOLD_SEARCH);
  }
  esp_camera_fb_return(result.camera_fb);  // Release frame buffer
  result.valid = true;
  return result;
}

VisionData Vision::buildVisionData(const Blob &largest) {
  VisionData vdata = {};

  if (largest.pixels > 0) {
    int roi_x = MAX(0, largest.x - ROI_PADDING);
    int roi_y = MAX(0, largest.y - ROI_PADDING);
    int roi_w = MIN(MAX_W - roi_x, largest.w + 2 * ROI_PADDING);
    int roi_h = MIN(MAX_H - roi_y, largest.h + 2 * ROI_PADDING);

    tracking_roi = {roi_x, roi_y, roi_w, roi_h};

    target_locked = true;

    // NOTE: cx/cy report the blob's true centroid (from findLargestBlob),
    // not the padded tracking ROI's top-left corner. w/h remain the ROI's
    // padded box dimensions, used by the base station for display only.
    vdata.cx = (uint16_t)largest.cx;
    vdata.cy = (uint16_t)largest.cy;
    vdata.w  = (uint16_t)roi_w;
    vdata.h  = (uint16_t)roi_h;
  } else {
    target_locked = false;
    tracking_roi = {0, 0, MAX_W, MAX_H};

    vdata.cx = 0;
    vdata.cy = 0;
    vdata.w  = 0;
    vdata.h  = 0;
  }
  return vdata;
}

void Vision::setup() {
  pinMode(LED_GPIO_NUM, OUTPUT);
  digitalWrite(LED_GPIO_NUM, LOW);

  setupCamera();

  size_t total_pixels = MAX_W * MAX_H;
  mask_buf  = (uint8_t *)heap_caps_malloc((total_pixels + 7) / 8, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  stack_buf = (uint32_t *)heap_caps_malloc(MAX_STACK_SIZE * sizeof(uint32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

  if (!mask_buf || !stack_buf) {
    errorLoop();
  }

  initThresholdLUT();

  digitalWrite(LED_GPIO_NUM, HIGH);
}

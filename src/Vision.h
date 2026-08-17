#ifndef VISION_H
#define VISION_H

#include "Arduino.h"
#include "esp_camera.h"

// --- CONFIGURATION & HARDWARE DEFINITIONS ---
#define LED_GPIO_NUM   21

#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39

#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

#define MAX_W          320
#define MAX_H          240
#define MAX_STACK_SIZE 8192

// --- DATA STRUCTURES ---
struct LabThreshold {
  int l_min, l_max;
  int a_min, a_max;
  int b_min, b_max;
};

struct Rect {
  int x, y, w, h;
};

struct Blob {
  int x, y, w, h;
  int cx, cy;
  uint32_t pixels;
};

typedef struct __attribute__((packed)) {
  uint16_t cx, cy, w, h;  // Bounding box / ROI tracking parameters
} VisionData;

typedef struct {
  camera_fb_t *camera_fb; // Raw camera buffer pointer
  camera_fb_t work_fb;    // Working copy of frame metadata/buffer
  Blob blob;              // Detected target blob
  bool valid;             // Frame grab success flag
} FrameResult;

class Vision {
public:
  Vision();
  ~Vision();

  void setup();
  FrameResult processFrame();
  VisionData buildVisionData(const Blob &largest);
  void drawOverlays(camera_fb_t *fb, Rect roi, Blob blob, bool locked);

private:
  void setupCamera();
  void errorLoop();
  void initThresholdLUT();

  void generateSramMaskFast(const camera_fb_t *fb, Rect roi);
  Blob findLargestBlob(camera_fb_t *fb, Rect roi, uint32_t area_threshold);

  inline bool getMaskBit(int idx);
  inline void clearMaskBit(int idx);
  inline void setMaskBit(int idx);
  inline uint8_t getThresholdLUT(uint16_t px);

  Rect tracking_roi;
  bool target_locked;

  uint8_t threshold_lut[8192];
  uint8_t *mask_buf;
  uint32_t *stack_buf;

  static constexpr LabThreshold THRESHOLD_BLIMP = {20, 60, 10, 50, 1, 30};
  static constexpr uint32_t AREA_THRESHOLD_LOCKED = 50;
  static constexpr uint32_t AREA_THRESHOLD_SEARCH = 100;
  static constexpr int ROI_PADDING = 30;
  static constexpr int SCAN_STEP = 1;
};

#endif // VISION_H

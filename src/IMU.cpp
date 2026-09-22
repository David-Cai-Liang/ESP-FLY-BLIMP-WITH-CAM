#include "IMU.h"

#include <esp_timer.h>
#include <math.h>

// Guards the state shared between the sampler task and loop(). File-scope
// because there is only ever one IMU instance, and because a portMUX_TYPE
// can't be brace-initialized portably as a class member.
static portMUX_TYPE imuLock = portMUX_INITIALIZER_UNLOCKED;

IMU::IMU() {}

void IMU::errorLoop() {
  while (true) {
    delay(100);
  }
}

void IMU::setup() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  if (!mpu.begin(0x68, &Wire)) {
    errorLoop();
  }

  // Set the sensor up explicitly rather than inheriting library defaults.
  // +/-250 deg/s is the finest gyro resolution that still covers a blimp's
  // yaw rate, and the 21 Hz DLPF keeps prop vibration from aliasing into the
  // rate estimate. With the DLPF engaged the gyro output rate is 1 kHz, so a
  // divisor of 4 gives 1000/(1+4) = 200 Hz, matching IMU_SAMPLE_HZ.
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setSampleRateDivisor(4);

  calibrateBias();

  // Pinned to core 0: the Arduino loop and the camera driver both live on
  // core 1, and keeping the sampler off that core is what makes its period
  // independent of frame processing time.
  xTaskCreatePinnedToCore(sampleTask, "imu_sampler", 4096, this, 3, nullptr, 0);
}

// Measures the gyro-Z zero offset while the board is held still. Rejects an
// attempt whose spread is too wide to have come from a stationary board (i.e.
// somebody moved it), and retries. If every attempt is noisy the least-bad
// mean is still used — a contaminated estimate beats assuming zero — but
// calibrated_ is left false so the condition is visible in telemetry.
void IMU::calibrateBias() {
  Serial.println("[IMU] Calibrating gyro bias — hold the blimp still...");

  float bestMean = 0;
  float bestStdDev = INFINITY;

  for (int attempt = 1; attempt <= IMU_CAL_ATTEMPTS; attempt++) {
    double sum = 0, sumSq = 0;

    for (uint32_t i = 0; i < IMU_CAL_SAMPLES; i++) {
      sensors_event_t accel, gyro, temp;
      mpu.getEvent(&accel, &gyro, &temp);
      sum += gyro.gyro.z;
      sumSq += (double)gyro.gyro.z * gyro.gyro.z;
      delay(IMU_SAMPLE_PERIOD_MS);
    }

    float mean = (float)(sum / IMU_CAL_SAMPLES);
    float variance = (float)(sumSq / IMU_CAL_SAMPLES - (double)mean * mean);
    float stdDev = variance > 0 ? sqrtf(variance) : 0;

    if (stdDev < bestStdDev) {
      bestStdDev = stdDev;
      bestMean = mean;
    }

    Serial.printf("[IMU] Attempt %d/%d: bias %.4f rad/s (%.2f deg/s), stddev %.4f\n",
                  attempt, IMU_CAL_ATTEMPTS, mean, mean * 180.0f / (float)PI, stdDev);

    if (stdDev <= IMU_CAL_MAX_STDDEV) {
      portENTER_CRITICAL(&imuLock);
      biasRad_ = mean;
      calibrated_ = true;
      portEXIT_CRITICAL(&imuLock);
      Serial.println("[IMU] Bias calibration OK.");
      return;
    }

    Serial.println("[IMU] Too much motion during calibration, retrying.");
  }

  portENTER_CRITICAL(&imuLock);
  biasRad_ = bestMean;
  calibrated_ = false;
  portEXIT_CRITICAL(&imuLock);
  Serial.printf("[IMU] WARNING: calibration never settled. Using best-effort "
                "bias %.4f rad/s; yaw will drift. ZUPT will re-trim once idle.\n",
                bestMean);
}

void IMU::sampleTask(void *arg) {
  static_cast<IMU *>(arg)->sampleLoop();
}

void IMU::sampleLoop() {
  // pdMS_TO_TICKS can floor to 0 if the tick rate is coarser than the sample
  // period, which would turn this into a busy loop.
  TickType_t period = pdMS_TO_TICKS(IMU_SAMPLE_PERIOD_MS);
  if (period == 0) period = 1;

  TickType_t lastWake = xTaskGetTickCount();
  lastSampleUs_ = esp_timer_get_time();
  prevRateRad_ = 0;

  for (;;) {
    // I2C first, outside any critical section — portENTER_CRITICAL masks
    // interrupts on this core and the Wire driver needs them.
    sensors_event_t accel, gyro, temp;
    mpu.getEvent(&accel, &gyro, &temp);

    int64_t nowUs = esp_timer_get_time();
    float dt = (float)(nowUs - lastSampleUs_) / 1e6f;
    lastSampleUs_ = nowUs;

    // A scheduling stall shouldn't be able to inject an arbitrarily large
    // step into the integral; cap it at 5x the nominal period.
    const float dtMax = (IMU_SAMPLE_PERIOD_MS * 5) / 1000.0f;
    if (dt < 0) dt = 0;
    if (dt > dtMax) dt = dtMax;

    portENTER_CRITICAL(&imuLock);
    float bias = biasRad_;
    portEXIT_CRITICAL(&imuLock);

    float rate = gyro.gyro.z - bias;

    // Trapezoidal integration — at 200 Hz the difference from rectangular is
    // small, but it costs nothing and removes a systematic lag.
    float yawStep = 0.5f * (rate + prevRateRad_) * dt;
    prevRateRad_ = rate;

    // ZUPT: while the props are idle and the residual is small, average it
    // over a window and fold a fraction back into the bias estimate.
    bool trimmed = false;
    float newBias = bias;
    if (motorsIdle_ && fabsf(rate) < IMU_ZUPT_RATE_LIMIT) {
      if (zuptCount_ == 0) zuptStartUs_ = nowUs;
      zuptSum_ += rate;
      zuptCount_++;

      if (nowUs - zuptStartUs_ >= (int64_t)IMU_ZUPT_WINDOW_MS * 1000) {
        newBias = bias + IMU_ZUPT_GAIN * (zuptSum_ / zuptCount_);
        trimmed = true;
        zuptSum_ = 0;
        zuptCount_ = 0;
      }
    } else {
      zuptSum_ = 0;
      zuptCount_ = 0;
    }

    portENTER_CRITICAL(&imuLock);
    latest_.ax = accel.acceleration.x;
    latest_.ay = accel.acceleration.y;
    latest_.az = accel.acceleration.z;
    // Telemetry reports the bias-corrected rate, i.e. what the controller
    // actually acts on. Still rad/s, same as before.
    latest_.tz = rate;
    yawRad_ += yawStep;
    rateRad_ = rate;
    if (trimmed) {
      biasRad_ = newBias;
      calibrated_ = true;
    }
    portEXIT_CRITICAL(&imuLock);

    vTaskDelayUntil(&lastWake, period);
  }
}

IMUData IMU::readData() {
  IMUData copy;
  portENTER_CRITICAL(&imuLock);
  copy = latest_;
  portEXIT_CRITICAL(&imuLock);
  return copy;
}

YawState IMU::yawState() {
  YawState s;
  portENTER_CRITICAL(&imuLock);
  s.yawRad = yawRad_;
  s.rateRad = rateRad_;
  s.biasRad = biasRad_;
  s.calibrated = calibrated_;
  portEXIT_CRITICAL(&imuLock);
  return s;
}

void IMU::setMotorsIdle(bool idle) {
  motorsIdle_ = idle;
}

#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PIN_SDA 5
#define PIN_SCL 6

// === Gyro sampling / yaw integration =======================================
// Yaw is integrated in a dedicated task rather than in loop(), because the
// main loop's period is set by the camera pipeline (QVGA capture + blob
// tracking) and is both slow and jittery. Integrating a rate at that cadence
// is the single largest avoidable source of turn error.
#define IMU_SAMPLE_HZ        200
#define IMU_SAMPLE_PERIOD_MS (1000 / IMU_SAMPLE_HZ)

// Boot bias calibration. The MPU6050's gyro-Z zero offset is typically 1-3
// deg/s untrimmed and varies per chip, which integrates into ~10 deg of error
// over a single 5 s turn — so it is measured at startup instead of assumed.
// The blimp must be held still while this runs.
#define IMU_CAL_SAMPLES      400     // 400 @ 200 Hz = 2 s per attempt
#define IMU_CAL_ATTEMPTS     3       // retried if the board was clearly moving
#define IMU_CAL_MAX_STDDEV   0.02f   // rad/s; above this the samples are rejected

// Zero-rate update (ZUPT): re-trims the bias mid-flight, since the MPU6050's
// offset walks with temperature. Only applied while the props are idle and the
// measured rate already sits near the current estimate, so a genuine slow
// rotation can't be mistaken for bias.
#define IMU_ZUPT_RATE_LIMIT  0.05f   // rad/s (~3 deg/s) residual still called "still"
#define IMU_ZUPT_WINDOW_MS   1000    // continuous quiet time before a trim is applied
#define IMU_ZUPT_GAIN        0.25f   // fraction of the observed residual folded in

typedef struct __attribute__((packed)) {
  float ax, ay, az, tz;   // MPU6050 Accelerometer (X, Y, Z) & Gyro Z
} IMUData;

// Gyro-derived yaw estimate. Deliberately separate from IMUData: IMUData is
// the wire format embedded in TelemetryPacket, so its layout can't change
// without breaking base_station.py's struct.unpack.
typedef struct {
  float yawRad;      // integrated heading since boot, bias-corrected, unwrapped
  float rateRad;     // bias-corrected yaw rate, rad/s
  float biasRad;     // current bias estimate, rad/s
  bool  calibrated;  // false if boot calibration never met IMU_CAL_MAX_STDDEV
} YawState;

class IMU {
public:
  IMU();

  void setup();

  // Most recent sample, served from the sampler's cache — this does not touch
  // the I2C bus, so it is safe to call at any rate from loop().
  IMUData readData();

  // Latest integrated yaw / bias-corrected rate.
  YawState yawState();

  // Whether the props are effectively off. This gates in-flight bias
  // re-trimming; call it once per control loop with the commanded outputs.
  void setMotorsIdle(bool idle);

private:
  void errorLoop();
  void calibrateBias();
  void sampleLoop();
  static void sampleTask(void *arg);

  Adafruit_MPU6050 mpu;

  // Written by the sampler task, read by loop() — all access is guarded by the
  // spinlock in IMU.cpp.
  IMUData latest_ = {};
  float yawRad_ = 0;
  float rateRad_ = 0;
  float biasRad_ = 0;
  bool calibrated_ = false;

  volatile bool motorsIdle_ = true;

  // Sampler-task-local integration state.
  int64_t lastSampleUs_ = 0;
  float prevRateRad_ = 0;   // previous corrected rate, for trapezoidal integration

  // ZUPT accumulator (sampler-task-local).
  float zuptSum_ = 0;
  uint32_t zuptCount_ = 0;
  int64_t zuptStartUs_ = 0;
};

#endif // IMU_H

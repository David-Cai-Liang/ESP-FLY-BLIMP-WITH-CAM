#include "IMU.h"

IMU::IMU() {}

void IMU::errorLoop() {
  while (true) {
    delay(100);
  }
}

void IMU::setup() {
  Wire.begin(PIN_SDA, PIN_SCL);
  if (!mpu.begin(0x68, &Wire)) {
    errorLoop();
  }
}

IMUData IMU::readData() {
  IMUData data = {};
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  data.ax = accel.acceleration.x;
  data.ay = accel.acceleration.y;
  data.az = accel.acceleration.z;
  data.tz = gyro.gyro.z;

  return data;
}

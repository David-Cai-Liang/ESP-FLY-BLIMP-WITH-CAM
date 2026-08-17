#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#define PIN_SDA 5
#define PIN_SCL 6

typedef struct __attribute__((packed)) {
  float ax, ay, az, tz;   // MPU6050 Accelerometer (X, Y, Z) & Gyro Z
} IMUData;

class IMU {
public:
  IMU();

  void setup();
  IMUData readData();

private:
  void errorLoop();
  Adafruit_MPU6050 mpu;
};

#endif // IMU_H

#include "motor.h"

MotorData buildMotorData(int16_t m1, int16_t m2, int16_t m3, int16_t m4) {
  MotorData data;
  data.m1 = m1;
  data.m2 = m2;
  data.m3 = m3;
  data.m4 = m4;
  return data;
}

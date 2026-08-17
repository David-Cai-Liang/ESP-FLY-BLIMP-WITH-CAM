#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

// Actual, post-constrain motor outputs for the current loop iteration.
// Filled in by blimp.ino once it has computed m1..m4 (whichever control mode
// is active) and packed into every TelemetryPacket sent to the base station.
// This lets anything downstream on the PC side (e.g. an autonomous
// controller, a logger, base_station.py's live readout) see what the motors
// are *actually* doing right now, as opposed to what was last commanded —
// useful since MODE_PROPORTIONAL and the watchdog can both override or zero
// the commanded values before they ever reach analogWrite().
typedef struct __attribute__((packed)) {
  int16_t m1; // Front Right
  int16_t m2; // Rear Right
  int16_t m3; // Rear Left
  int16_t m4; // Front Left
} MotorData;

// Small convenience constructor, mirroring Vision.cpp's buildVisionData().
MotorData buildMotorData(int16_t m1, int16_t m2, int16_t m3, int16_t m4);

#endif

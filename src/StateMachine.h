#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>
#include <Vision.h>
#include <IMU.h>
#include "Motor.h"

// === Autonomous Sub-State (reported in telemetry) ===========================
// STATE_MANUAL is set by the caller whenever the blimp is under direct
// (MODE_MANUAL) control; the StateMachine class below only ever produces
// STATE_TRACKING, STATE_TURNING, or STATE_SEARCHING.
#define STATE_MANUAL    0
#define STATE_TRACKING  1
#define STATE_TURNING   2
#define STATE_SEARCHING 3

// === State Machine Configuration ============================================
namespace StateMachineConfig {

  // Base thrust while in proportional/autonomous mode
  const int DEFAULT_FORWARD_POWER = 50;
  const int DEFAULT_UPWARD_POWER = 20;

  // Camera-derived tracking parameters, in degrees
  const float YAW_DEADZONE_HALF_DEG = 2;
  const float YAW_GAIN_PER_DEG = 10;                    // motor power per degree of error
  const int TURNING_AREA = 29500;                       // vision blob area that triggers a waypoint turn

  // IMU-derived turning parameters, in radians
  const float TURN_KD = 5;
  const float TURN_KP = 150;
  const float TURN_RATE_SETTLE = PI / 10;
  const float TURN_DEADBAND_RAD = PI / 10;
  const float TURN_MAX_POWER = 255;                     // mirrors MOTOR_MAX in blimp.ino
  const float GYRO_BIAS_RAD_PER_SEC = 0;

  // Waypoint turn sequence (radians to turn at each successive waypoint)
  const float WAYPOINT_LIST[] = {PI / 2, PI / 2, PI / 2, PI / 2};
  const int WAYPOINT_COUNT = sizeof(WAYPOINT_LIST) / sizeof(WAYPOINT_LIST[0]);

  // Wiggle-search tuning (currently unused — see StateMachine.cpp)
  const unsigned long WIGGLE_RAMP_MS = 5000;      // ms of searching to reach full amplitude
  const unsigned long WIGGLE_PERIOD_MS = 2000;    // ms per full left-right sweep cycle
  const int WIGGLE_MIN_AMPLITUDE = 10;            // starting sweep amplitude (PWM units)
  const int WIGGLE_MAX_AMPLITUDE = 255;           // amplitude stops growing past this (mirrors MOTOR_MAX)

}  // namespace StateMachineConfig

// Encapsulates the blimp's autonomous (MODE_AUTONOMOUS) behavior: tracking
// a vision target, turning to the next waypoint once close enough, and
// (eventually) searching when the target is lost. Call update() once per
// loop() iteration while in proportional mode; the caller is responsible for
// only invoking it when the control link is fresh (not stale) and for
// falling back to STATE_MANUAL handling itself.
class StateMachine {
public:
  // Runs one step of the autonomy state machine.
  //   vData:    latest vision blob data (cx, cy, w, h)
  //   iData:    latest IMU data
  //   yawError: degrees of yaw needed to center the target
  //             (+ => target is right of center)
  // Returns the motor outputs for this step (pre-constrain values; the
  // caller is expected to clamp to [0, MOTOR_MAX]) and updates currentState().
  MotorData update(const VisionData &vData, const IMUData &iData, float yawError);

  // The sub-state resulting from the most recent update() call.
  uint8_t currentState() const { return state_; }

private:
  uint8_t state_ = STATE_SEARCHING;

  // Waypoint turn state (persists across update() calls)
  int waypointIndex_ = 0;
  float turnedSoFar_ = 0;
  bool turnInProgress_ = false;
  unsigned long lastTurnStepMs_ = 0;

  // Wiggle-search state (persists across update() calls; currently unused)
  bool wiggleSearchActive_ = false;
  unsigned long wiggleSearchStartMs_ = 0;
  unsigned long wigglePhaseOffsetMs_ = 0;
};

#endif  // STATE_MACHINE_H

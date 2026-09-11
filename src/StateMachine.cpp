#include "StateMachine.h"

using namespace StateMachineConfig;

// Wraps an angle in radians to the range [-PI, PI).
static float wrapPI(float angleRad) {
  while (angleRad >= PI) angleRad -= 2 * PI;
  while (angleRad < -PI) angleRad += 2 * PI;
  return angleRad;
}

// Applies `correction` to `primary`. If that would push `primary` past
// TURN_MAX_POWER (mirrors MOTOR_MAX in blimp.ino), `primary` is clamped and
// the leftover ("overflow") correction is bled into `secondary` (decreasing
// it) instead of being discarded. This keeps the differential response
// roughly linear over a wider range before it fully saturates, rather than
// flattening out the instant one motor hits its cap.
//
// Takes/returns plain values (not references) because MotorData's fields
// live in a packed struct — you can't bind a non-const reference directly
// to a packed field.
struct DifferentialCorrection {
  int primary;
  int secondary;
};

static DifferentialCorrection applyDifferentialCorrection(int primary, int secondary, int correction) {
 primary += correction;
 if (primary > TURN_MAX_POWER) {
   int overflow = primary - (int)TURN_MAX_POWER;
   primary = (int)TURN_MAX_POWER;
   secondary -= overflow;
 }
 return {primary, secondary};
}


MotorData StateMachine::update(const VisionData &vData, const IMUData &iData, float yawError, float pitchError) {
  MotorData out;
  out.m1 = out.m4 = DEFAULT_FORWARD_POWER;
  out.m2 = DEFAULT_UPWARD_POWER;
  state_ = STATE_SEARCHING;

  if (turnInProgress_) {
    state_ = STATE_TURNING;

    unsigned long nowMs = millis();
    unsigned long dtMs = nowMs - lastTurnStepMs_;
    if (dtMs > 200) dtMs = 200;
    lastTurnStepMs_ = nowMs;

    float rate = iData.tz - GYRO_BIAS_RAD_PER_SEC;
    turnedSoFar_ += rate * (dtMs / 1000.0f);

    float yawError = wrapPI(WAYPOINT_LIST[waypointIndex_] - turnedSoFar_);
    int correction = (int)((fabs(yawError) - TURN_DEADBAND_RAD) * TURN_KP);

    if (yawError >= 0) {
      DifferentialCorrection r = applyDifferentialCorrection(out.m4, out.m1, correction);
      out.m4 = r.primary;
      out.m1 = r.secondary;
    } else {
      DifferentialCorrection r = applyDifferentialCorrection(out.m1, out.m4, correction);
      out.m1 = r.primary;
      out.m4 = r.secondary;
    }
    out.m1 -= TURN_KD * iData.tz;
    out.m4 += TURN_KD * iData.tz;

    if (fabs(yawError) <= TURN_DEADBAND_RAD && fabs(iData.tz) <= TURN_RATE_SETTLE) {
      turnInProgress_ = false;
      waypointIndex_ = (waypointIndex_ + 1) % WAYPOINT_COUNT;
    }

  } else {
    bool targetVisible = (vData.w > 0 && vData.h > 0);
    bool closeEnough = (vData.w * vData.h > TURNING_AREA);

    // Require CLOSE_ENOUGH_FRAME_CONFIRM consecutive close-enough frames
    // before actually committing to a turn, so a single oversized/noisy
    // detection can't trigger one. Any frame that drops back below
    // TURNING_AREA resets the streak.
    closeEnoughFrameCount_ = closeEnough ? (closeEnoughFrameCount_ + 1) : 0;
    bool turnConfirmed = closeEnough && (closeEnoughFrameCount_ >= CLOSE_ENOUGH_FRAME_CONFIRM);

    if (!turnConfirmed && targetVisible) {
      state_ = STATE_TRACKING;
      wiggleSearchActive_ = false;

      // Yaw Control
      if (fabs(yawError) > YAW_DEADZONE_HALF_DEG) {
        int correction = (int)((fabs(yawError) - YAW_DEADZONE_HALF_DEG) * YAW_GAIN_PER_DEG);
        if (yawError > 0) {
          DifferentialCorrection r = applyDifferentialCorrection(out.m4, out.m1, correction);
          out.m4 = r.primary;
          out.m1 = r.secondary;
        } else {
          DifferentialCorrection r = applyDifferentialCorrection(out.m1, out.m4, correction);
          out.m1 = r.primary;
          out.m4 = r.secondary;
        }
      }

      // Gyro-rate correction always applied while tracking
      out.m1 -= STRAIGHT_KD * iData.tz;
      out.m4 += STRAIGHT_KD * iData.tz;


      // Pitch Control
      if (fabs(pitchError) > PITCH_DEADZONE_HALF_DEG) {
        int correction = (int)((fabs(pitchError) - PITCH_DEADZONE_HALF_DEG) * PITCH_GAIN_PER_DEG);
        out.m2 = max(0, DEFAULT_UPWARD_POWER + correction);
        out.m3 = max(0, -(DEFAULT_UPWARD_POWER + correction));
      }

    } else if (turnConfirmed) {
      state_ = STATE_TURNING;
      wiggleSearchActive_ = false;
      turnInProgress_ = true;
      turnedSoFar_ = 0;
      lastTurnStepMs_ = millis();
      closeEnoughFrameCount_ = 0;
    }
    // else {
    //   state_ = STATE_SEARCHING;
    //
    //   if (!wiggleSearchActive_) {
    //     wiggleSearchActive_ = true;
    //     wiggleSearchStartMs_ = millis();
    //     wigglePhaseOffsetMs_ = random(0, WIGGLE_PERIOD_MS);
    //   }
    //
    //   unsigned long searchElapsedMs = millis() - wiggleSearchStartMs_;
    //   float rampFraction = (float)searchElapsedMs / (float)WIGGLE_RAMP_MS;
    //   rampFraction = constrain(rampFraction, 0.0, 1.0);
    //
    //   int amplitude = WIGGLE_MIN_AMPLITUDE +
    //                   (int)(rampFraction * (WIGGLE_MAX_AMPLITUDE - WIGGLE_MIN_AMPLITUDE));
    //
    //   unsigned long phaseMs = (searchElapsedMs + wigglePhaseOffsetMs_) % WIGGLE_PERIOD_MS;
    //   float phase = 2.0 * PI * (float)phaseMs / (float)WIGGLE_PERIOD_MS;
    //   int sweep = (int)(amplitude * sin(phase));
    //
    //   if (sweep >= 0) {
    //     out.m1 -= sweep;
    //   } else {
    //     out.m4 -= (-sweep);
    //   }
    // }
  }

  return out;
}

float StateMachine::getTurnYawErrorDeg() const {
  return wrapPI(WAYPOINT_LIST[waypointIndex_] - turnedSoFar_) * (180.0f / (float)PI);
}

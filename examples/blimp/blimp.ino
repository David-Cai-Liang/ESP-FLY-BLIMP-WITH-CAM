// === Camera compile-time switch =============================================
// use_camera = 1: current behavior — camera is initialized and polled every
//                  loop, and telemetry carries real VisionData from the sensor.
// use_camera = 0: all camera code is bypassed (no Vision object, no setup(),
//                  no processFrame()/buildVisionData() calls); vData is left
//                  zero-initialized, so telemetry reports [0,0,0,0] for
//                  cx, cy, w, h every loop.
#define use_camera 1

#include <Vision.h>
#define sensor_t adafruit_sensor_t
#include <IMU.h>
#undef sensor_t
#include "motor.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Corrected Motor GPIO Pin Mapping from ESP-FLY Wiring Diagram
const int MOTOR_M1_FR = 7; // Front Right (M1) -> Pin 7 (Purple Wire)
const int MOTOR_M2_RR = 4; // Rear Right  (M2) -> Pin 4 (Green Wire)
const int MOTOR_M3_RL = 3; // Rear Left   (M3) -> Pin 3 (Blue Wire)
const int MOTOR_M4_FL = 1; // Front Left  (M4) -> Pin 1 (Orange Wire)

// === Control Mode (runtime select, driven by the base station) =============
// MODE_MANUAL       - motors driven directly by ControlPacket.motors from the base station
// MODE_PROPORTIONAL - motors driven by a P controller on vision blob yaw error,
//                      incoming manual stick input is ignored
// The active mode is no longer a compile-time #define: it's carried in every
// ControlPacket as `mode`, so the pilot can flip it live from base_station.py
// (the 'M' key) without reflashing. currentMode defaults to MODE_MANUAL until
// the first packet arrives, matching the watchdog's fail-safe-to-zero posture.
#define MODE_MANUAL       0
#define MODE_PROPORTIONAL 1
uint8_t currentMode = MODE_MANUAL;

const int MOTOR_MAX = 255;               // analogWrite() PWM ceiling (8-bit default)
const int DEFAULT_FORWARD_POWER = 20;
const int DEFAULT_UPWARD_POWER = 50;

// Camera geometry: degrees of yaw needed to center a target on the x-axis.
// Simple flat model — every pixel subtends an equal slice of the horizontal
// FOV. A more accurate model (accounting for lens distortion) can replace
// this later without changing the telemetry layout.
const float HORIZONTAL_FOV_DEG = 57.4;                    // camera's horizontal field of view
const float DEG_PER_PIXEL = HORIZONTAL_FOV_DEG / MAX_W;   // MAX_W (320) comes from Vision.h
const float YAW_DEADZONE_HALF_DEG = 5;
const float YAW_GAIN_PER_DEG = 3;                  // motor power per degree of error
const float TURN_KD = 25;
// const float TURN_RATE_SETTLE
// const float TURN_DEADBAND_DEG
// const float TURN_MAX_POWER 
// const float TURN_KP

// Wiggle-search tuning (used in MODE_PROPORTIONAL when the target isn't
// visible). Noise amplitude ramps up the longer the search has been running,
// so early wiggles are gentle and later ones sweep harder in case the target
// has drifted far off-frame (or the blimp has drifted far from the last
// known bearing).
// const unsigned long WIGGLE_RAMP_MS = 5000;      // ms of searching to reach full amplitude
// const unsigned long WIGGLE_PERIOD_MS = 2000;    // ms per full left-right sweep cycle
// const int WIGGLE_MIN_AMPLITUDE     = 10;        // starting sweep amplitude (PWM units)
// const int WIGGLE_MAX_AMPLITUDE     = MOTOR_MAX; // amplitude stops growing past this

// REPLACE WITH YOUR BASE STATION MAC ADDRESS
// {0x30, 0x30, 0xF9, 0x17, 0xFB, 0x8C}
// {0x30, 0x30, 0xf9, 0x16, 0xa1, 0x0c}
// {0xdc, 0xda, 0x0c, 0x57, 0x56, 0x0c}
uint8_t baseStationAddress[] = {0x30, 0x30, 0xF9, 0x17, 0xFB, 0x8C};

// Telemetry sent from Blimp to Base Station
typedef struct __attribute__((packed)) {
  VisionData vision; // cx, cy, w, h (4 x uint16_t)
  IMUData imu;       // ax, ay, az, tz (4 x float)
  MotorData motors;  // actual, post-constrain M1-M4 outputs (4 x int16_t)
  float yawError;    // degrees of yaw needed to center the target (+ => target is right of center)
} TelemetryPacket;

// Motor control commands received from Base Station
typedef struct __attribute__((packed)) {
  int16_t motors[4]; // Motor 1, 2, 3, 4 speed/direction inputs
  uint8_t mode;      // MODE_MANUAL or MODE_PROPORTIONAL, set live by base_station.py
} ControlPacket;

esp_now_peer_info_t peerInfo;
#if use_camera
Vision vision;
#endif
IMU imu;

ControlPacket incomingControl = {{0, 0, 0, 0}};
volatile bool newControlAvailable = false;

volatile unsigned long lastRecvTime = 0;
const unsigned long CONTROL_TIMEOUT_MS = 1000;

// Callback when telemetry is sent to Base Station
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[ESP-NOW] Telemetry send failed (no ACK from base station)");
  }
}

// Callback when motor controls are received from Base Station
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControlPacket)) {
    memcpy(&incomingControl, incomingData, sizeof(ControlPacket));
    if (incomingControl.mode == MODE_MANUAL || incomingControl.mode == MODE_PROPORTIONAL) {
      currentMode = incomingControl.mode;
    }
    newControlAvailable = true;
    lastRecvTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  randomSeed(esp_random()); // hardware RNG seed, used by the wiggle search
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register ESP-NOW Callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Register Base Station as Peer
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, baseStationAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add Base Station peer");
    return;
  }

  // Initialize Sensors and Camera Hardware
  imu.setup();
#if use_camera
  vision.setup();
#endif
}

void loop() {
  // 1. Process Vision & IMU Telemetry
  VisionData vData = {}; // zero-initialized: all 0

#if use_camera
  FrameResult result = vision.processFrame();
  if (result.valid) {
    vData = vision.buildVisionData(result.blob);
  }
#endif
  // use_camera == 0: camera is bypassed entirely; vData stays [0,0,0,0].

  IMUData iData = imu.readData();

  // Degrees of yaw needed to bring the target's blob center to the middle
  // of the frame. Computed every loop (not just in PROPORTIONAL mode) so
  // it's always available in telemetry for monitoring/tuning.
  float yawError = ((float)vData.cx - (MAX_W / 2.0f)) * DEG_PER_PIXEL;

  bool stale = (millis() - lastRecvTime > CONTROL_TIMEOUT_MS);

  // 2. Compute Motor Outputs for the active control mode
  int16_t m1 = 0, m2 = 0, m3 = 0, m4 = 0;

  if (currentMode == MODE_MANUAL) {
    // Drive motors directly from the base station's ControlPacket.
    // Watchdog: if no packet has arrived within CONTROL_TIMEOUT_MS, force zero.
    newControlAvailable = false;
    m1 = stale ? 0 : incomingControl.motors[0] - TURN_KD * iData.tz;
    m2 = stale ? 0 : incomingControl.motors[1];
    m3 = stale ? 0 : incomingControl.motors[2];
    m4 = stale ? 0 : incomingControl.motors[3] + TURN_KD * iData.tz;

  } else { // MODE_PROPORTIONAL
    // Manual stick input is ignored in this mode.
    newControlAvailable = false;
    
    // float[] waypoint_list = ...; // the values are turning angles for each waypoint
    // int waypoint_index = ...;

    // Same watchdog as manual mode: if the base station link itself has gone
    // stale, stay at zero rather than continuing to fly blind.
    if (!stale) {
      // Fly forward by default; turning is done by decreasing power to one
      // of the two rear motors, not by adding power to the other.
      m1 = m4 = DEFAULT_FORWARD_POWER;
      m2 = DEFAULT_UPWARD_POWER;
      // TODO: update closeEnough to reasonable values; maybe create a guidance.cpp/.h
      bool target_visible = (vData.w > 0 && vData.h > 0);

      bool closeEnough = (vData.w > 180 && vData.h > 180);

      if (!closeEnough && target_visible) {
        // yawError (degrees, computed above) replaces the old pixel-space
        // center_x/error_x calc — same P controller, just working in
        // degrees instead of pixels.
        if (fabs(yawError) > YAW_DEADZONE_HALF_DEG) {
          int correction = (int)((fabs(yawError) - YAW_DEADZONE_HALF_DEG) * YAW_GAIN_PER_DEG);
          if (yawError > 0) {
            m1 -= correction; // target right of center -> yaw right by cutting M1 (Front Right)
            m1 -= TURN_KD * iData.tz;
            m1 = constrain(m1, 0, MOTOR_MAX);
          } else {
            m4 -= correction; // target left of center  -> yaw left  by cutting M4 (Front Left)
            m4 += TURN_KD * iData.tz;
            m4 = constrain(m4, 0, MOTOR_MAX);
          }
        }
      }
      /*
      else if (closeEnough) {
        // Go into IMU-Based Waypoint Mode 
        Turn using the rotation data for next waypoint: waypoint_list[next]
        next=(next+1)%waypoint_num
        float turnedSoFar = 0;
        while (!stale && abs(turnedSoFar - waypoint_list[next]) > TURN_DEADBAND_DEG) {
          stale = (millis() - lastRecvTime > CONTROL_TIMEOUT_MS);
          float rate = imu.readData().tz - gyroBiasDegPerSec;
          float error = wrap180(waypoint_list[next] - turnedSoFar);

            // Preliminary PID
            float turnPower = TURN_KP * error - TURN_KD * yawRate;
            turnPower = constrain(turnPower, -TURN_MAX_POWER, TURN_MAX_POWER);

            if (turnPower >= 0) {        // need to yaw right: m1 up, m4 down
              m1 = constrain((int)turnPower, 0, MOTOR_MAX);
              m4 = 0;
            } else {                     // need to yaw left: m4 up, m1 down
              m4 = constrain((int)-turnPower, 0, MOTOR_MAX);
              m1 = 0;
            }
            m2 = 0; m3 = 0;  // no lift/forward thrust during a turn — minimizes drift off-station

          if (abs(error) <= TURN_DEADBAND_DEG && abs(rate) <= TURN_RATE_SETTLE) {
            closeEnough = false;
            distanceTraveled = 0;
        }
        }
      }
      else {
        // Wiggle search: target isn't visible, so sweep yaw with a smooth
        // sine wave while continuing to fly forward. The sweep amplitude
        // ramps up the longer the search has been running (i.e. the longer
        // we've gone without a fix), so we start with small nudges and
        // escalate to wider sweeps the farther/longer we've been searching
        // blind. A randomized phase offset keeps successive searches from
        // sweeping the exact same way every time.
        if (!wiggleSearchActive) {
          wiggleSearchActive = true;
          wiggleSearchStartMs = millis();
          wigglePhaseOffsetMs = random(0, WIGGLE_PERIOD_MS);
        }

        unsigned long searchElapsedMs = millis() - wiggleSearchStartMs;
        float rampFraction = (float)searchElapsedMs / (float)WIGGLE_RAMP_MS;
        rampFraction = constrain(rampFraction, 0.0, 1.0);

        int amplitude = WIGGLE_MIN_AMPLITUDE +
                        (int)(rampFraction * (WIGGLE_MAX_AMPLITUDE - WIGGLE_MIN_AMPLITUDE));

        // Phase advances steadily with elapsed time, one full sweep every
        // WIGGLE_PERIOD_MS; sin() gives a smooth back-and-forth rather than
        // the jitter of fresh-random-every-loop noise.
        unsigned long phaseMs = (searchElapsedMs + wigglePhaseOffsetMs) % WIGGLE_PERIOD_MS;
        float phase = 2.0 * PI * (float)phaseMs / (float)WIGGLE_PERIOD_MS;
        int sweep = (int)(amplitude * sin(phase));

        if (sweep >= 0) {
          m1 -= sweep; // yaw right
        } else {
          m4 -= (-sweep); // yaw left
        }
      }
      */
  }
  }

  m1 = constrain(m1, 0, MOTOR_MAX);
  m2 = constrain(m2, 0, MOTOR_MAX);
  m3 = constrain(m3, 0, MOTOR_MAX);
  m4 = constrain(m4, 0, MOTOR_MAX);

  // 3. Transmit Telemetry Packet to Base Station, now that the actual
  // (post-constrain) motor outputs for this loop iteration are known.
  TelemetryPacket telemetry;
  telemetry.vision = vData;
  telemetry.imu = iData;
  telemetry.motors = buildMotorData(m1, m2, m3, m4);
  telemetry.yawError = yawError;

  esp_err_t sendResult = esp_now_send(baseStationAddress, (uint8_t *)&telemetry, sizeof(telemetry));
  if (sendResult != ESP_OK) {
    Serial.printf("[ESP-NOW] Telemetry send failed to enqueue, err=%d\n", sendResult);
  }

  Serial.printf("[MODE] %s | [MOTORS] M1: %d | M2: %d | M3: %d | M4: %d\n",
                currentMode == MODE_MANUAL ? "MANUAL" : "PROPORTIONAL", m1, m2, m3, m4);

  analogWrite(MOTOR_M1_FR, m1);
  analogWrite(MOTOR_M2_RR, m2); 
  analogWrite(MOTOR_M3_RL, m3);
  analogWrite(MOTOR_M4_FL, m4);

  vTaskDelay(1);
}
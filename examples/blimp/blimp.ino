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

// Yaw controller tuning
// Deadzone is a 40x40 px box centered on the frame; only the x-extent (+/-20px)
// is used since this controller only corrects yaw (left/right).
const int YAW_DEADZONE_HALF_PX = 40;     // half-width of the 40px-wide deadzone
const int YAW_GAIN = 1;                  // motor power added per pixel of x error
const int MOTOR_MAX = 255;               // analogWrite() PWM ceiling (8-bit default)
const int DEFAULT_FORWARD_POWER = 80;
const int DEFAULT_UPWARD_POWER = 20;

// const float TURN_RATE_SETTLE
// const float TURN_DEADBAND_DEG
// const float TURN_MAX_POWER
// const float TURN_KD
// const float TURN_KP

// REPLACE WITH YOUR BASE STATION MAC ADDRESS
uint8_t baseStationAddress[] = {0x30, 0x30, 0xF9, 0x17, 0xFB, 0x8C};

// Telemetry sent from Blimp to Base Station
typedef struct __attribute__((packed)) {
  VisionData vision; // cx, cy, w, h (4 x uint16_t)
  IMUData imu;       // ax, ay, az, tz (4 x float)
  MotorData motors;  // actual, post-constrain M1-M4 outputs (4 x int16_t)
} TelemetryPacket;

// Motor control commands received from Base Station
typedef struct __attribute__((packed)) {
  int16_t motors[4]; // Motor 1, 2, 3, 4 speed/direction inputs
  uint8_t mode;      // MODE_MANUAL or MODE_PROPORTIONAL, set live by base_station.py
} ControlPacket;

esp_now_peer_info_t peerInfo;
Vision vision;
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

  // Set device in Wi-Fi Station Mode
  WiFi.mode(WIFI_STA);

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
  vision.setup();
}

void loop() {
  // 1. Process Vision & IMU Telemetry
  FrameResult result = vision.processFrame();
  VisionData vData = {};

  if (result.valid) {
    vData = vision.buildVisionData(result.blob);
  }

  IMUData iData = imu.readData();

  bool stale = (millis() - lastRecvTime > CONTROL_TIMEOUT_MS);

  // 2. Compute Motor Outputs for the active control mode
  int16_t m1 = 0, m2 = 0, m3 = 0, m4 = 0;

  if (currentMode == MODE_MANUAL) {
    // Drive motors directly from the base station's ControlPacket.
    // Watchdog: if no packet has arrived within CONTROL_TIMEOUT_MS, force zero.
    newControlAvailable = false;
    m1 = stale ? 0 : incomingControl.motors[0];
    m2 = stale ? 0 : incomingControl.motors[1];
    m3 = stale ? 0 : incomingControl.motors[2];
    m4 = stale ? 0 : incomingControl.motors[3];

  } else { // MODE_PROPORTIONAL
    // Manual stick input is ignored in this mode.
    newControlAvailable = false;
    
    // waypoint[] waypoint_list = ...;
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

      // bool closeEnough = (vData.w > 180 && vData.h > 180);

      if (!closeEnough && target_visible) {
        int center_x = MAX_W / 2;                 // 320 / 2 = 160
        int error_x  = (int)vData.cx - center_x;   // + => target is right of center

        if (abs(error_x) > YAW_DEADZONE_HALF_PX) {
          int correction = (abs(error_x) - YAW_DEADZONE_HALF_PX) * YAW_GAIN;
          if (error_x > 0) {
            m4 -= correction; // target right of center -> yaw right by cutting M3 (Rear Left)
          } else {
            m1 -= correction; // target left of center  -> yaw left  by cutting M2 (Rear Right)
          }
        }
      }
      /*Go into IMU-Based Waypoint Mode 
      else if (closeEnough) {
        Turn using the rotation data for next waypoint: waypoint_list[next]
        next=(next+1)%waypoint_num
        float turnedSoFar = 0;
        while (!stale && abs(turnedSoFar - waypoint_list[next].angle) > TURN_DEADBAND_DEG) {
          stale = (millis() - lastRecvTime > CONTROL_TIMEOUT_MS);
          float rate = imu.readData().tz - gyroBiasDegPerSec;
          float error = wrap180(waypoint_list[next].angle - turnedSoFar);

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
        // may add wiggle search if tracking is not good enough
        // As distance traveled from last waypoint increases, the wiggle angle should increase proportionally.
      }
    }
    */
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
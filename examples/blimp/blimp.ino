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
#include <Motor.h> 
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "BatteryMonitor.h"

// Corrected Motor GPIO Pin Mapping from ESP-FLY Wiring Diagram
const int MOTOR_M1_FR = 7; // Front Right (M1) -> Pin 7 (Purple Wire)
const int MOTOR_M2_RR = 4; // Rear Right  (M2) -> Pin 4 (Green Wire)
const int MOTOR_M3_RL = 3; // Rear Left   (M3) -> Pin 3 (Blue Wire)
const int MOTOR_M4_FL = 1; // Front Left  (M4) -> Pin 1 (Orange Wire)

// === Control Mode (runtime select, driven by the base station) =============
#define MODE_MANUAL       0
#define MODE_PROPORTIONAL 1
volatile uint8_t currentMode = MODE_MANUAL;

// === Autonomous Sub-State (reported in telemetry) ===========================
#define STATE_MANUAL    0
#define STATE_TRACKING  1
#define STATE_TURNING   2
#define STATE_SEARCHING 3
uint8_t currentState = STATE_MANUAL;

const int MOTOR_MAX = 255;               // analogWrite() PWM ceiling (8-bit default)
const int DEFAULT_FORWARD_POWER = 50;
const int DEFAULT_UPWARD_POWER = 20;

// Camera Parameters, In Degrees
const float HORIZONTAL_FOV_DEG = 57.4;                    // camera's horizontal field of view
const float DEG_PER_PIXEL = HORIZONTAL_FOV_DEG / MAX_W;   // MAX_W (320) comes from Vision.h
const float YAW_DEADZONE_HALF_DEG = 2;
const float YAW_GAIN_PER_DEG = 10;                        // motor power per degree of error
const int TURNING_AREA = 29500;

//IMU Parameters, In Radians
const float STRAIGHT_KD = 25;
const float TURN_KD = 5;   
const float TURN_KP = 150;                           
const float TURN_RATE_SETTLE = PI/10;
const float TURN_DEADBAND_RAD = PI/10;
const float TURN_MAX_POWER = MOTOR_MAX;
const float gyroBiasRadPerSec = 0;

// Wiggle-search tuning
const unsigned long WIGGLE_RAMP_MS = 5000;      // ms of searching to reach full amplitude
const unsigned long WIGGLE_PERIOD_MS = 2000;    // ms per full left-right sweep cycle
const int WIGGLE_MIN_AMPLITUDE     = 10;        // starting sweep amplitude (PWM units)
const int WIGGLE_MAX_AMPLITUDE     = MOTOR_MAX; // amplitude stops growing past this

// === Battery Monitor =========================================================
const unsigned long BATTERY_READ_INTERVAL_MS = 500;
BatteryMonitor battMonitor;
unsigned long lastBattReadMs = 0;

// REPLACE WITH YOUR BASE STATION MAC ADDRESS
// {0x30, 0x30, 0xF9, 0x17, 0xFB, 0x8C}
// {0x30, 0x30, 0xf9, 0x16, 0xa1, 0x0c}
// {0xdc, 0xda, 0x0c, 0x57, 0x56, 0x0c}
// {0x30, 0x30, 0xF9, 0x17, 0xFD, 0x40}
// {0x34, 0x85, 0x18, 0xab, 0xed, xc0}
uint8_t baseStationAddress[] = {0x30, 0x30, 0xF9, 0x17, 0xFB, 0x8C};

// Telemetry sent from Blimp to Base Station
typedef struct __attribute__((packed)) {
  VisionData vision; // cx, cy, w, h (4 x uint16_t)
  IMUData imu;       // ax, ay, az, tz (4 x float)
  MotorData motors;  // actual, post-constrain M1-M4 outputs (4 x int16_t)
  float yawError;    // degrees of yaw needed to center the target (+ => target is right of center)
  float battVoltage; // battery voltage in volts, from BatteryMonitor
  uint8_t state;     // STATE_MANUAL / STATE_TRACKING / STATE_TURNING / STATE_SEARCHING
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

volatile ControlPacket incomingControl = {{0, 0, 0, 0}};

volatile unsigned long lastRecvTime = 0;
const unsigned long CONTROL_TIMEOUT_MS = 1000;

// === Waypoint Turn State (persisted across loop() iterations) ==============
const float waypoint_list[] = {PI/2, PI/2, PI/2, PI/2};
const int waypoint_count = sizeof(waypoint_list) / sizeof(waypoint_list[0]);
int waypoint_index = 0;
float turnedSoFar = 0;
bool turnInProgress = false;
unsigned long lastTurnStepMs = 0;

// === Wiggle-Search State (persisted across loop() iterations) ==============
bool wiggleSearchActive = false;
unsigned long wiggleSearchStartMs = 0;
unsigned long wigglePhaseOffsetMs = 0;

// Wraps an angle in radians to the range [-PI, PI).
float wrapPI(float angleRad) {
  while (angleRad >= PI) angleRad -= 2*PI;
  while (angleRad < -PI) angleRad += 2*PI;
  return angleRad;
}

void sendTelemetry(const VisionData &vData, const IMUData &iData,
                    int16_t m1, int16_t m2, int16_t m3, int16_t m4,
                    float yawError, uint8_t state) {
  TelemetryPacket telemetry;
  telemetry.vision = vData;
  telemetry.imu = iData;
  telemetry.motors = buildMotorData(m1, m2, m3, m4);
  telemetry.yawError = yawError;
  telemetry.battVoltage = battMonitor.getVoltage();
  telemetry.state = state;

  esp_err_t sendResult = esp_now_send(baseStationAddress, (uint8_t *)&telemetry, sizeof(telemetry));
  if (sendResult != ESP_OK) {
    Serial.printf("[ESP-NOW] Telemetry send failed to enqueue, err=%d\n", sendResult);
  }

  const char *stateNames[] = {"MANUAL", "TRACKING", "TURNING", "SEARCHING"};
  Serial.printf("[MODE] %s | [STATE] %s | [MOTORS] M1: %d | M2: %d | M3: %d | M4: %d\n",
                currentMode == MODE_MANUAL ? "MANUAL" : "PROPORTIONAL",
                stateNames[state], m1, m2, m3, m4);
}

// Callback when telemetry is sent to Base Station
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[ESP-NOW] Telemetry send failed (no ACK from base station)");
  }
}

// Callback when motor controls are received from Base Station
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(ControlPacket)) {
    memcpy((void*)&incomingControl, incomingData, sizeof(ControlPacket));
    if (incomingControl.mode == MODE_MANUAL || incomingControl.mode == MODE_PROPORTIONAL) {
      currentMode = incomingControl.mode;
    }
    lastRecvTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  randomSeed(esp_random());
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, baseStationAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add Base Station peer");
    return;
  }

  imu.setup();
#if use_camera
  vision.setup();
#endif
  battMonitor.begin();
}

void loop() {
  // 1. Process Vision & IMU Telemetry
  VisionData vData = {};

#if use_camera
  FrameResult result = vision.processFrame();
  if (result.valid) {
    vData = vision.buildVisionData(result.blob);
  }
#endif

  IMUData iData = imu.readData();

  if (millis() - lastBattReadMs >= BATTERY_READ_INTERVAL_MS) {
    lastBattReadMs = millis();
    battMonitor.update();
  }

  float yawError = ((float)vData.cx - (MAX_W / 2.0f)) * DEG_PER_PIXEL;

  bool stale = (millis() - lastRecvTime > CONTROL_TIMEOUT_MS);

  // 2. Compute Motor Outputs for the active control mode
  int16_t m1 = 0, m2 = 0, m3 = 0, m4 = 0;

  if (currentMode == MODE_MANUAL) {
    currentState = STATE_MANUAL;
    m1 = stale ? 0 : incomingControl.motors[0] - STRAIGHT_KD * iData.tz;
    m2 = stale ? 0 : incomingControl.motors[1];
    m3 = stale ? 0 : incomingControl.motors[2];
    m4 = stale ? 0 : incomingControl.motors[3] + STRAIGHT_KD * iData.tz;

  } else { // MODE_PROPORTIONAL
    currentState = STATE_SEARCHING;

    if (!stale) {
      m1 = m4 = DEFAULT_FORWARD_POWER;
      m2 = DEFAULT_UPWARD_POWER;

      if (turnInProgress) {
        currentState = STATE_TURNING;

        unsigned long nowMs = millis();
        unsigned long dtMs = nowMs - lastTurnStepMs;
        if (dtMs > 200) dtMs = 200;
        lastTurnStepMs = nowMs;

        float rate = iData.tz - gyroBiasRadPerSec;
        turnedSoFar += rate * (dtMs / 1000.0f);

        float error = wrapPI(waypoint_list[waypoint_index] - turnedSoFar);
        int correction = (int)((fabs(error) - TURN_DEADBAND_RAD) * TURN_KP);

        if (error >= 0) {      
          m1 += correction;
        } else {                    
          m4 += correction;
        }
        m1 -= TURN_KD * iData.tz;
        m4 += TURN_KD * iData.tz;

        if (abs(error) <= TURN_DEADBAND_RAD && abs(iData.tz) <= TURN_RATE_SETTLE) {
          turnInProgress = false;
          waypoint_index = (waypoint_index + 1) % waypoint_count;
        }

      } else {
        bool target_visible = (vData.w > 0 && vData.h > 0);
        bool closeEnough = (vData.w * vData.h > TURNING_AREA);

        if (!closeEnough && target_visible) {
          currentState = STATE_TRACKING;
          wiggleSearchActive = false;

          if (fabs(yawError) > YAW_DEADZONE_HALF_DEG) {
            int correction = (int)((fabs(yawError) - YAW_DEADZONE_HALF_DEG) * YAW_GAIN_PER_DEG);
            if (yawError > 0) {
              m4 += correction;
            } else {
              m1 += correction;
            }
          }

          // Gyro-rate correction always applied while tracking
          m1 -= TURN_KD * iData.tz;
          m4 += TURN_KD * iData.tz;
        }
        else if (closeEnough) {
          currentState = STATE_TURNING;
          wiggleSearchActive = false;
          turnInProgress = true;
          turnedSoFar = 0;
          lastTurnStepMs = millis();
        }
        // else {
        //   currentState = STATE_SEARCHING;

        //   if (!wiggleSearchActive) {
        //     wiggleSearchActive = true;
        //     wiggleSearchStartMs = millis();
        //     wigglePhaseOffsetMs = random(0, WIGGLE_PERIOD_MS);
        //   }

        //   unsigned long searchElapsedMs = millis() - wiggleSearchStartMs;
        //   float rampFraction = (float)searchElapsedMs / (float)WIGGLE_RAMP_MS;
        //   rampFraction = constrain(rampFraction, 0.0, 1.0);

        //   int amplitude = WIGGLE_MIN_AMPLITUDE +
        //                   (int)(rampFraction * (WIGGLE_MAX_AMPLITUDE - WIGGLE_MIN_AMPLITUDE));

        //   unsigned long phaseMs = (searchElapsedMs + wigglePhaseOffsetMs) % WIGGLE_PERIOD_MS;
        //   float phase = 2.0 * PI * (float)phaseMs / (float)WIGGLE_PERIOD_MS;
        //   int sweep = (int)(amplitude * sin(phase));

        //   if (sweep >= 0) {
        //     m1 -= sweep;
        //   } else {
        //     m4 -= (-sweep);
        //   }
        // }
      }
    }
  }

  m1 = constrain(m1, 0, MOTOR_MAX);
  m2 = constrain(m2, 0, MOTOR_MAX);
  m3 = constrain(m3, 0, MOTOR_MAX);
  m4 = constrain(m4, 0, MOTOR_MAX);

  // 3. Transmit Telemetry Packet to Base Station
  sendTelemetry(vData, iData, m1, m2, m3, m4, yawError, currentState);

  analogWrite(MOTOR_M1_FR, m1);
  analogWrite(MOTOR_M2_RR, m2); 
  analogWrite(MOTOR_M3_RL, m3);
  analogWrite(MOTOR_M4_FL, m4);

  vTaskDelay(1);
}
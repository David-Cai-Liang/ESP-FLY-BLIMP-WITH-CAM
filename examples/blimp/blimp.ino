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
#include <BatteryMonitor.h>
#include <StateMachine.h>

// Corrected Motor GPIO Pin Mapping from ESP-FLY Wiring Diagram
const int MOTOR_M1_FR = 7; // Front Right (M1) -> Pin 7 (Purple Wire)
const int MOTOR_M2_RR = 4; // Rear Right  (M2) -> Pin 4 (Green Wire)
const int MOTOR_M3_RL = 3; // Rear Left   (M3) -> Pin 3 (Blue Wire)
const int MOTOR_M4_FL = 1; // Front Left  (M4) -> Pin 1 (Orange Wire)

// === Control Mode (runtime select, driven by the base station) =============
#define MODE_MANUAL     0
#define MODE_AUTONOMOUS 1
volatile uint8_t currentMode = MODE_MANUAL;

// Reported in telemetry; STATE_MANUAL/TRACKING/TURNING/SEARCHING are defined
// in StateMachine.h. Only STATE_MANUAL is set here directly — the rest come
// from StateMachine::currentState().
uint8_t currentState = STATE_MANUAL;

const int MOTOR_MAX = 255;               // analogWrite() PWM ceiling (8-bit default)

// Camera Parameters, In Degrees
const float HORIZONTAL_FOV_DEG = 57.4;                    // camera's horizontal field of view
const float HORIZONTAL_DEG_PER_PIXEL = HORIZONTAL_FOV_DEG / MAX_W;   // MAX_W (320) comes from Vision.h
const float VERTICAL_FOV_DEG = 44.6;                    // camera's horizontal field of view
const float VERTICAL_DEG_PER_PIXEL = VERTICAL_FOV_DEG / MAX_W;   // MAX_W (320) comes from Vision.h

// IMU Parameter, In Radians — used only for MODE_MANUAL yaw damping below.
// (The autonomous turning/tracking gains live in StateMachine.h.)
const float STRAIGHT_KD = 25;

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
  VisionData vision; // cx, cy, w, h, pixels (4 x uint16_t, 1 x uin32_t)
  IMUData imu;       // ax, ay, az, tz (4 x float)
  MotorData motors;  // actual, post-constrain M1-M4 outputs (4 x int16_t)
  float yawError;    // degrees of yaw needed to center the target (+ => target is right of center)
  float battVoltage; // battery voltage in volts, from BatteryMonitor
  uint8_t state;     // STATE_MANUAL / STATE_TRACKING / STATE_TURNING / STATE_SEARCHING
} TelemetryPacket;

// Motor control commands received from Base Station
typedef struct __attribute__((packed)) {
  int16_t motors[4]; // Motor 1, 2, 3, 4 speed/direction inputs
  uint8_t mode;      // MODE_MANUAL or MODE_AUTONOMOUS, set live by base_station.py
} ControlPacket;

esp_now_peer_info_t peerInfo;
#if use_camera
Vision vision;
#endif
IMU imu;

StateMachine stateMachine;

volatile ControlPacket incomingControl = {{0, 0, 0, 0}};

volatile unsigned long lastRecvTime = 0;
const unsigned long CONTROL_TIMEOUT_MS = 1000;

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
                currentMode == MODE_MANUAL ? "MANUAL" : "AUTONOMOUS",
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
    if (incomingControl.mode == MODE_MANUAL || incomingControl.mode == MODE_AUTONOMOUS) {
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
yaw
  float Error = ((float)vData.cx - (MAX_W / 2.0f)) * HORIZONTAL_DEG_PER_PIXEL;
  float pitchError = ((float)vData.cy - (MAX_H / 2.0f)) * VERTICAL_DEG_PER_PIXEL;

  bool stale = (millis() - lastRecvTime > CONTROL_TIMEOUT_MS);

  // 2. Compute Motor Outputs for the active control mode
  int16_t m1 = 0, m2 = 0, m3 = 0, m4 = 0;

  if (currentMode == MODE_MANUAL) {
    currentState = STATE_MANUAL;
    m1 = stale ? 0 : incomingControl.motors[0] - STRAIGHT_KD * iData.tz;
    m2 = stale ? 0 : incomingControl.motors[1];
    m3 = stale ? 0 : incomingControl.motors[2];
    m4 = stale ? 0 : incomingControl.motors[3] + STRAIGHT_KD * iData.tz;

  } else { // MODE_AUTONOMOUS
    currentState = STATE_SEARCHING;

    if (!stale) {
      MotorData out = stateMachine.update(vData, iData, yawError, pitchError);
      m1 = out.m1;
      m2 = out.m2;
      m3 = out.m3;
      m4 = out.m4;
      currentState = stateMachine.currentState();
    }
  } 

  // Override yawError with IMU yaw error when turning
  if (currentState == STATE_TURNING) {
    yawError = stateMachine.getTurnYawErrorDeg();
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

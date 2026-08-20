#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// REPLACE WITH YOUR BLIMP MAC ADDRESS
// {0x68, 0xee, 0x8f, 0x50, 0x28, 0x58}
// {0x68, 0xee, 0x8f, 0x50, 0x26, 0xd0} 
// {0x68, 0xee, 0x8f, 0x50, 0x28, 0xbc}
// {0x68, 0xee, 0x8f, 0x50, 0x28, 0x94}
// {0x68, 0xee, 0x8f, 0x50, 0x2a, 0x2c} // No back propellers  -  
// {0x80, 0xb5, 0x4e, 0xf1, 0x10, 0x30}
uint8_t blimpAddress[] = {0x68, 0xee, 0x8f, 0x50, 0x28, 0x94};

typedef struct __attribute__((packed)) {
  int16_t motors[4];
  uint8_t mode; // MODE_MANUAL / MODE_PROPORTIONAL, opaque to the base station — just relayed
} ControlPacket;

typedef struct __attribute__((packed)) {
  uint16_t cx, cy, w, h;
  float ax, ay, az, tz;
  int16_t m1, m2, m3, m4; // actual, post-constrain motor outputs from the blimp
} TelemetryPacket;

// Framed Protocol Markers
const uint8_t TELE_HEADER[4] = {0x00, 0xAA, 0x55, 0xFF};
const uint8_t TELE_FOOTER[2] = {0xEE, 0xFF};
const uint8_t CTRL_HEADER[4] = {0x00, 0xBB, 0x66, 0xFF};

esp_now_peer_info_t peerInfo;

TelemetryPacket latestTelemetry;
volatile bool newTelemetryReceived = false;

// Store packet in buffer instead of writing directly inside callback ISR
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(TelemetryPacket)) {
    memcpy(&latestTelemetry, incomingData, sizeof(TelemetryPacket));
    newTelemetryReceived = true;
  }
}

// Callback when a relayed ControlPacket is sent to the Blimp
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[ESP-NOW] Control send failed (no ACK from blimp)");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  memset(&peerInfo, 0, sizeof(peerInfo));

  memcpy(peerInfo.peer_addr, blimpAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add Base Station peer");
    return;
  }
}

void loop() {
  // 1. Safely stream telemetry out over Serial from main loop thread
  if (newTelemetryReceived) {
    newTelemetryReceived = false;
    Serial.write(TELE_HEADER, 4);
    Serial.write((uint8_t *)&latestTelemetry, sizeof(TelemetryPacket));
    Serial.write(TELE_FOOTER, 2);
  }

  // 2. Read incoming motor controls from Python
  const size_t expected_size = 4 + sizeof(ControlPacket);
  if (Serial.available() >= expected_size) {
    if (Serial.peek() == CTRL_HEADER[0]) {
      uint8_t header[4];
      Serial.readBytes(header, 4);

      if (memcmp(header, CTRL_HEADER, 4) == 0) {
        ControlPacket control;
        Serial.readBytes((char *)&control, sizeof(ControlPacket));
        esp_err_t sendResult = esp_now_send(blimpAddress, (uint8_t *)&control, sizeof(control));
        if (sendResult != ESP_OK) {
          Serial.printf("[ESP-NOW] Control send failed to enqueue, err=%d\n", sendResult);
        }
      }
    } else {
      Serial.read(); // Discard unaligned byte
    }
  }
}
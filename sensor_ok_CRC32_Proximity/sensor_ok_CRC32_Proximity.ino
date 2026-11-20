// Future BOX FB12 + ESP-01S. Keep ConfigManager.h beside this sketch.
// #define DEBUG
#include <ESP8266WiFi.h>
#include <espnow.h>
#include "ConfigManager.h"

constexpr uint8_t CONFIG_BUTTON_PIN = 0;
constexpr uint8_t REED_SWITCH_PIN = 3;
constexpr uint8_t STATUS_LED_PIN = 2;  // Active LOW
constexpr uint32_t TRIGGER_CONFIRM_MS = 5000;

struct __attribute__((packed)) sensor_message {
  uint8_t sensor_id;
  bool switch_status;  // true = normal, false = confirmed alarm
  uint32_t timestamp;
  uint32_t checksum;
};
static_assert(sizeof(sensor_message) == 10, "Gateway packet size mismatch");

ConfigManager cfgManager;
DeviceConfig myConfig;
bool stateKnown = false;
bool confirmedNormal = true;
bool timingTrigger = false;
uint32_t triggerStart = 0;
uint32_t lastSend = 0;
uint32_t heartbeatInterval = 0;
volatile int8_t sendResult = -1;
bool ledOn = false;
uint32_t ledStart = 0;
uint32_t ledDuration = 0;

uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFu;
  while (length--) {
    crc ^= uint32_t(*data++) << 24;
    for (uint8_t i = 0; i < 8; ++i) {
      crc = (crc & 0x80000000u) ? (crc << 1) ^ 0x04C11DB7u : crc << 1;
    }
  }
  return crc;
}

bool updateSwitchState(uint32_t now) {
  const bool pinHigh = digitalRead(REED_SWITCH_PIN) == HIGH;
  const bool normal = pinHigh == myConfig.proximity_type;
  if (normal) {
    timingTrigger = false;
  } else {
    if (stateKnown && !confirmedNormal) return false;
    if (!timingTrigger) {
      timingTrigger = true;
      triggerStart = now;
    }
    if (now - triggerStart < TRIGGER_CONFIRM_MS) return false;
    timingTrigger = false;
  }

  if (stateKnown && confirmedNormal == normal) return false;
  stateKnown = true;
  confirmedNormal = normal;
  return true;
}

void onDataSent(uint8_t *mac, uint8_t status) {
  sendResult = (status == 0) ? 0 : 1;
}

void sendSensorData(bool heartbeat) {
  sensor_message msg{};
  msg.sensor_id = myConfig.deviceID;
  msg.switch_status = confirmedNormal;
  msg.timestamp = millis();
  msg.checksum = calculateCRC32(reinterpret_cast<const uint8_t*>(&msg),
                                sizeof(msg) - sizeof(msg.checksum));
  int result = esp_now_send(myConfig.gatewayMAC,
                            reinterpret_cast<uint8_t*>(&msg), sizeof(msg));
  if (result != 0) sendResult = 1;
#if defined(DEBUG)
  Serial.printf("%s ID=%u normal=%u queueResult=%d\n",
                heartbeat ? "Heartbeat" : "State", unsigned(msg.sensor_id),
                unsigned(msg.switch_status), result);
#endif
}

void updateLED(uint32_t now) {
  int8_t result = sendResult;
  if (result >= 0) {
    sendResult = -1;
    ledDuration = (result == 0) ? 100 : 400;
    ledStart = now;
    ledOn = true;
    digitalWrite(STATUS_LED_PIN, LOW);
  } else if (ledOn && now - ledStart >= ledDuration) {
    ledOn = false;
    digitalWrite(STATUS_LED_PIN, HIGH);
  }
}

void restartAfterError() {
  for (uint8_t i = 0; i < 10; ++i) {
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
  }
  ESP.restart();
}

void setup() {
#if defined(DEBUG)
  Serial.begin(115200, SERIAL_8N1, SERIAL_TX_ONLY);
#endif
  pinMode(REED_SWITCH_PIN, INPUT);
  pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);
  cfgManager.begin(CONFIG_BUTTON_PIN);
  myConfig = cfgManager.getConfig();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (!wifi_set_channel(myConfig.channel) || esp_now_init() != 0 ||
      esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER) != 0 ||
      esp_now_register_send_cb(onDataSent) != 0 ||
      esp_now_add_peer(myConfig.gatewayMAC, ESP_NOW_ROLE_SLAVE,
                       myConfig.channel, nullptr, 0) != 0) {
    restartAfterError();
  }
}

void loop() {
  uint32_t now = millis();
  bool changed = updateSwitchState(now);
  // While qualifying a new alarm, heartbeat carries the last confirmed state.
  // On cold start, wait for a known state before sending the first packet.
  if (changed || (stateKnown && now - lastSend >= heartbeatInterval)) {
    sendSensorData(!changed);
    lastSend = now;
    heartbeatInterval = random(500, 5001);
  }
  updateLED(now);
  yield();
}

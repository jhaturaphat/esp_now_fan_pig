#include "ConfigManager.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

// #define DEBUG  // เอาคอมเมนต์ออกเมื่ออยู่ในโหมด DEBUG

// ESP32 Relay x2 Module Pin Configuration
#define RELAY1_PIN 17      // output Active HIGH
#define RELAY2_PIN 16      // output Active HIGH
#define LED_STATUS 23      // output Active HIGH
#define CONFIG_PIN 0       // input pullup
#define AC220_LOSS 34      // input เดิม 27
#define RXD2 21  
#define TXD2 19  
// #define RXD2 15  
// #define TXD2 2  

#define COMMUNICATION_TIMEOUT 30001 // 30 วินาที timeout 
#define SIREN_DURATION 10000        // 10 วินาที

// Global Dynamic Flags
volatile bool handleAlarm = false; 
volatile bool reload = true;
volatile unsigned retry_count = 0;

// LED Status
volatile unsigned long led_blink_start = 0;
const unsigned long LED_BLINK_DURATION = 100; // กระพริบ 100ms
String static_mac_address = "";

// Shared Hardware Inputs
volatile bool ac220_loss = false; 

// โครงสร้างข้อมูลที่รับจาก Sensor (Packed Structure)
typedef struct __attribute__((packed)) sensor_message {
  uint8_t sensor_id;      // ID ของ sensor (1-10)
  bool switch_status;     // สถานะ switch (true=ปกติ, false=แจ้งเตือน)
  uint32_t timestamp;     // timestamp
  uint32_t checksum;      // ใช้ตรวจสอบข้อมูล (CRC32)
} sensor_message;

// เก็บสถานะของแต่ละ sensor
struct sensor_storage {  
  bool is_online;           
  bool switch_state;        
  unsigned long last_seen;  
  uint8_t mac[6];           
};

// Global Shared Counters
volatile int offline_count = 0; 
volatile int alarm_count = 0;

// Config Objects & Dynamic Storage
ConfigManager cfgManager;
DeviceConfig myConfig;
sensor_storage* sensors_storage = nullptr; 
int MAX_SENSORS = 10; 

// Mutex Handle สำหรับคุม Thread Safety ระหว่าง Core 0 และ Core 1
SemaphoreHandle_t sensorMutex = NULL;

// Forward Declarations
void TaskSensorIngestion(void *pvParameters);
void TaskGatewayLogic(void *pvParameters);
void sendSensorsData(bool testRun);
uint32_t calculateCRC32(const uint8_t *data, size_t length);

// คำนวณ CRC32 ตรวจสอบความถูกต้องข้อมูล
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) bit = !bit;
      crc <<= 1;
      if (bit) crc ^= 0x04C11DB7;
    }
  }
  return crc;
}

// Callback เมื่อรับข้อมูล ESP-NOW (ประมวลผลบน Core 0)
void onDataReceive(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  if (len < sizeof(sensor_message)) return;

  sensor_message msg;
  memcpy(&msg, incomingData, sizeof(msg));

  // 1. ตรวจสอบความถูกต้องของ CRC32
  size_t dataSizeToCalculate = sizeof(msg) - sizeof(msg.checksum);
  if (calculateCRC32((uint8_t*)&msg, dataSizeToCalculate) != msg.checksum) {
    #if defined(DEBUG)
      Serial.println(F("❌ CRC Mismatch! Packet dropped."));
    #endif
    return; 
  }

  // 2. ติดไฟ LED Status และบันทึกเวลาที่รับสัญญาณ
  digitalWrite(LED_STATUS, HIGH);
  led_blink_start = millis();

  // 3. Thread-safe Update ไปยัง Memory
  if (msg.sensor_id >= 1 && msg.sensor_id <= MAX_SENSORS) {
    int index = msg.sensor_id - 1;
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(5))) {
      sensors_storage[index].is_online = true;
      sensors_storage[index].last_seen = millis();
      sensors_storage[index].switch_state = msg.switch_status; 
      memcpy(sensors_storage[index].mac, recv_info->src_addr, 6); 
      xSemaphoreGive(sensorMutex);
    }
  }
}

void setup() {
  #if defined(DEBUG)
    Serial.begin(115200);
  #endif
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  pinMode(AC220_LOSS, INPUT_PULLUP);
  // Active HIGH: ตั้งค่า OFF ก่อนเปิดโหมด Output
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED_STATUS, OUTPUT); 

  // สำตรวจสอบ Relay เมื่อเปิดเครื่องครั้งแรก
  digitalWrite(RELAY1_PIN, HIGH );
  digitalWrite(RELAY2_PIN, HIGH );
  delay(500);
  digitalWrite(RELAY1_PIN, LOW );
  digitalWrite(RELAY2_PIN, LOW );

  // 1. สร้าง Mutex ควบคุมข้อมูลข้าม Core
  sensorMutex = xSemaphoreCreateMutex();

  cfgManager.begin(CONFIG_PIN); 
  myConfig = cfgManager.getConfig();

  WiFi.mode(WIFI_STA);
  esp_wifi_set_mac(WIFI_IF_STA, myConfig.virtualMac);
  WiFi.setSleep(false);  
  WiFi.macAddress(myConfig.virtualMac); 
    
  MAX_SENSORS = myConfig.sensor;

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(myConfig.channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK) {
    #if defined(DEBUG)
      Serial.println(F("Error initializing ESP-NOW"));
    #endif
    return;
  }
  
  static_mac_address = WiFi.macAddress(); 
  
  // จัดสรรพื้นที่ Memory ให้ Struct
  sensors_storage = new sensor_storage[MAX_SENSORS];
  for (int i = 0; i < MAX_SENSORS; i++) {
    sensors_storage[i].is_online = false;
    sensors_storage[i].switch_state = true;  // false คือ เซ็นเซอร์พัดลมดับ
    sensors_storage[i].last_seen = 0;
    memset(sensors_storage[i].mac, 0, 6);
  }

  // ลงทะเบียน Callback ระบบไร้สาย
  esp_now_register_recv_cb(onDataReceive);

  // Relay คง OFF ระหว่างเริ่มระบบ จนกว่า TaskGatewayLogic จะตรวจพบ Alarm

  // 2. ประกาศสร้าง Task บน CORE 0 (รับ Sensor Data & เฝ้าระวัง LED Status)
  xTaskCreatePinnedToCore(
    TaskSensorIngestion,    
    "TaskSensorIngestion",  
    3072,                 
    NULL,                 
    1,                    
    NULL,                 
    0                     // Core 0
  );

  // 3. ประกาศสร้าง Task บน CORE 1 (ประมวลผล Logic, Relay, JSON, Serial Output)
  xTaskCreatePinnedToCore(
    TaskGatewayLogic,    
    "TaskGatewayLogic",  
    8192,                 
    NULL,                 
    1,                    
    NULL,                 
    1                     // Core 1
  );
}

// =========================================================================
// CORE 0 TASK: รับผิดชอบการเคลียร์ LED และดูแลงาน Background ฝั่ง Radio
// =========================================================================
void TaskSensorIngestion(void *pvParameters) {
  for (;;) {
    // ดับ LED Status เมื่อครบเวลา
    if (led_blink_start > 0 && (millis() - led_blink_start >= LED_BLINK_DURATION)) {
      digitalWrite(LED_STATUS, LOW);
      led_blink_start = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(10)); // คืน CPU Time ให้ Core 0 รันระบบ ESP-NOW
  }
}

// =========================================================================
// CORE 1 TASK: ประมวลผลสถานะระบบ, เช็ก Timeout, คุม Relay และส่ง Serial2
// =========================================================================
#define NORMAL_SEND_INTERVAL 10000UL  // ปกติ ส่งทุก 10 วินาที
#define ALARM_SEND_INTERVAL   5000UL  // ผิดปกติ ส่งทุก 5 วินาที

void TaskGatewayLogic(void *pvParameters) {

    unsigned long lastSendTime = 0;

    bool lastAlarmState = false;
    bool firstRun = true;

    for (;;) {

        unsigned long currentMillis = millis();

        // ========================================
        // 1. อ่านสถานะ Hardware
        // ========================================
        ac220_loss = (digitalRead(AC220_LOSS) == HIGH);

        // ========================================
        // 2. ตรวจสอบ Sensor Timeout และ Alarm
        // ========================================
        if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10))) {

            int temp_offline = 0;
            int temp_alarm = 0;

            for (int i = 0; i < MAX_SENSORS; i++) {

                // ตรวจสอบการขาดการติดต่อ
                if (sensors_storage[i].last_seen > 0) {

                    if (currentMillis -
                        sensors_storage[i].last_seen >=
                        COMMUNICATION_TIMEOUT) {

                        temp_offline++;

                        if (sensors_storage[i].is_online) {
                            sensors_storage[i].is_online = false;
                            sensors_storage[i].switch_state = HIGH;
                        }
                    }
                }

                // ตรวจสอบสถานะ Alarm
                if (sensors_storage[i].is_online &&
                    sensors_storage[i].last_seen > 0) {

                    if (sensors_storage[i].switch_state == LOW) {
                        temp_alarm++;
                    }
                }
            }

            offline_count = temp_offline;
            alarm_count = temp_alarm;

            xSemaphoreGive(sensorMutex);
        }

        // ========================================
        // 3. ตรวจสอบสถานะระบบ
        // ========================================
        bool isAlarm =
            (alarm_count > 0) ||
            (offline_count > 0) ||
            ac220_loss;

        handleAlarm = isAlarm;

        // ========================================
        // 4. ควบคุม Relay / Siren
        // ========================================
        // Relay Active HIGH: Alarm -> ON (HIGH), Normal -> OFF (LOW)
        // สั่งทุกครั้งในรอบ Task ไม่ต้องรอรอบส่ง JSON
        digitalWrite(RELAY1_PIN, isAlarm ? HIGH : LOW);
        digitalWrite(RELAY2_PIN, isAlarm ? HIGH : LOW);

        // ========================================
        // 5. กำหนดช่วงเวลาส่ง JSON
        // ========================================
        unsigned long sendInterval = isAlarm
            ? ALARM_SEND_INTERVAL
            : NORMAL_SEND_INTERVAL;

        // ตรวจสอบการเปลี่ยนสถานะ
        bool stateChanged = (isAlarm != lastAlarmState);

        // ========================================
        // 6. ส่งข้อมูล JSON
        // ========================================
        if (firstRun ||
            stateChanged ||
            (currentMillis - lastSendTime >= sendInterval)) {

            // ส่ง testRun=true เพียงครั้งเดียวเมื่อ ALARM -> NORMAL
            // รอบอื่นทั้งหมดส่ง testRun=false ตามเดิม
            bool alarmRecovered = (!firstRun && lastAlarmState && !isAlarm);

            lastSendTime = currentMillis;
            lastAlarmState = isAlarm;
            firstRun = false;

            sendSensorsData(alarmRecovered);

            #if defined(DEBUG)
                Serial.printf(
                    "[Gateway] Status: %s | Alarm: %d | Offline: %d | Interval: %lu ms\n",
                    isAlarm ? "ALARM" : "NORMAL",
                    alarm_count,
                    offline_count,
                    sendInterval
                );
            #endif
        }

        // ========================================
        // 7. FreeRTOS Delay
        // ========================================
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =========================================================================
// MAIN LOOP: คืนพื้นที่ Memory ลบ Loop หลักทิ้ง (ย้ายงานไปทำที่ Task ครบแล้ว)
// =========================================================================
void loop() {
  vTaskDelete(NULL);
}

// =========================================================================
// JSON Serialization & Serial Output (ทำบน Core 1 เท่านั้น)
// =========================================================================
void sendSensorsData(bool testRun) {
  JsonDocument doc; 
  JsonArray sensors = doc.createNestedArray("sensors");
  // unsigned long current_time_seconds = millis() / 1000;
  // อ่านข้อมูล Array แบบ Thread-Safe
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(20))) {
    for (int i = 0; i < MAX_SENSORS; i++) {
      JsonObject obj = sensors.createNestedObject();
      obj["id"] = i + 1;
      obj["online"] = sensors_storage[i].is_online;
      obj["switch"] = sensors_storage[i].switch_state;
      obj["uptime"] = sensors_storage[i].last_seen  / 1000;    
    }
    xSemaphoreGive(sensorMutex);
  }
  
  if(testRun) {   
    doc["testRun"] = true;     
  }else{
    doc["testRun"] = false; 
  }
  doc["offline_count"] = offline_count;  
  doc["ac_loss"] = ac220_loss;   
  doc["alarm_count"] = alarm_count;  
  doc["mac"] = static_mac_address;
  doc["ch"] = WiFi.channel();
  
  serializeJson(doc, Serial2);
  Serial2.print("\n");
}


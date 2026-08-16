#include "ConfigManager.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

// #define DEBUG  // เอาคอมเมนต์ออกเมื่ออยู่ในโหมด DEBUG

// ESP32 Relay x2 Module Pin Configuration
#define RELAY1_PIN 17      // output Active LOW
#define RELAY2_PIN 16      // output Active LOW
#define LED_STATUS 23      // output Active HIGH
#define CONFIG_PIN 0       // input pullup
#define TEST_PIN 32        // input pullup
#define TEST_PIN_SERIAL 23 // input pullup
#define DISABLE_SIREN 25   // input pullup
#define AC220_LOSS 27      // input
#define RXD2 21  
#define TXD2 19  

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
volatile bool disableSiren = false;
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
void sendSensorsData();
void test_gw();
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

  pinMode(AC220_LOSS, INPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED_STATUS, OUTPUT); 
  pinMode(TEST_PIN, INPUT_PULLUP);
  pinMode(TEST_PIN_SERIAL, INPUT_PULLUP);
  pinMode(DISABLE_SIREN, INPUT_PULLUP);

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
    sensors_storage[i].switch_state = true;
    sensors_storage[i].last_seen = 0;
    memset(sensors_storage[i].mac, 0, 6);
  }

  // ลงทะเบียน Callback ระบบไร้สาย
  esp_now_register_recv_cb(onDataReceive);

  // สั่ง Relay แสดงสถานะพร้อมทำงานเริ่มต้น
  digitalWrite(RELAY1_PIN, HIGH);  
  digitalWrite(RELAY2_PIN, HIGH);
  delay(100);
  digitalWrite(RELAY1_PIN, LOW); 
  digitalWrite(RELAY2_PIN, LOW);  

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
void TaskGatewayLogic(void *pvParameters) {
  unsigned long PERIOD_SIREN = 0; 
  unsigned long PERIOD_SIREN2 = 0;

  for (;;) {
    disableSiren = (digitalRead(DISABLE_SIREN) == LOW);
    ac220_loss = (digitalRead(AC220_LOSS) == HIGH);

    // --- 1. ตรวจจับ Sensor Timeout และนับจำนวน Alarm (Thread-Safe) ---
    if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(10))) {
      int temp_offline = 0;
      int temp_alarm = 0;
      unsigned long current_time = millis();

      for (int i = 0; i < MAX_SENSORS; i++) {
        if (sensors_storage[i].last_seen > 0) {
          if (current_time - sensors_storage[i].last_seen > COMMUNICATION_TIMEOUT) {
            temp_offline++;
            if (sensors_storage[i].is_online) {
              sensors_storage[i].is_online = !sensors_storage[i].is_online;
              sensors_storage[i].switch_state = true;
            }
          }
        }
        if (sensors_storage[i].is_online && (sensors_storage[i].last_seen > 0)) {
          if (sensors_storage[i].switch_state == LOW) {
            temp_alarm++;
          }
        }
      }
      offline_count = temp_offline;
      alarm_count = temp_alarm;

      xSemaphoreGive(sensorMutex);
    }

    // --- 2. ควบคุม Relay / Siren Logic ---
    if ((alarm_count > 0) || (offline_count > 0) || ac220_loss) {
      if (disableSiren) {
        digitalWrite(RELAY1_PIN, LOW);
        digitalWrite(RELAY2_PIN, LOW);
      } else {
        digitalWrite(RELAY1_PIN, HIGH);
        digitalWrite(RELAY2_PIN, HIGH);
      }

      if (millis() - PERIOD_SIREN >= SIREN_DURATION) {      
        PERIOD_SIREN = millis();      
        handleAlarm = true;
        sendSensorsData(); 
      }
    } else {
      if (handleAlarm && (alarm_count == 0)) { 
        if (millis() - PERIOD_SIREN2 >= 15000) {  
          PERIOD_SIREN2 = millis();
          sendSensorsData(); 
          if (retry_count >= 1) {
            handleAlarm = false;
            retry_count = 0;
          } else {
            retry_count++;       
          }
        }
      } 

      if (reload) {
        reload = false;
        sendSensorsData(); 
      }
    
      digitalWrite(RELAY1_PIN, LOW);     
      digitalWrite(RELAY2_PIN, LOW);     
    }

    // --- 3. ตรวจสอบปุ่มกดทดสอบระบบ ---
    test_gw();

    vTaskDelay(pdMS_TO_TICKS(20)); // ความถี่ประมวลผลประมาณ 50Hz
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
void sendSensorsData() {
  JsonDocument doc; 
  JsonArray sensors = doc.createNestedArray("sensors");
  
  // อ่านข้อมูล Array แบบ Thread-Safe
  if (xSemaphoreTake(sensorMutex, pdMS_TO_TICKS(20))) {
    for (int i = 0; i < MAX_SENSORS; i++) {
      JsonObject obj = sensors.createNestedObject();
      obj["id"] = i + 1;
      obj["online"] = sensors_storage[i].is_online;
      obj["switch"] = sensors_storage[i].switch_state;
      obj["uptime"] = (sensors_storage[i].last_seen > 0) ? ((millis() - sensors_storage[i].last_seen) / 1000) : 0;    
    }
    xSemaphoreGive(sensorMutex);
  }
  
  doc["ac_loss"] = ac220_loss;   
  doc["alarm_count"] = alarm_count;
  doc["offline_count"] = offline_count;
  doc["mac"] = static_mac_address;
  doc["ch"] = WiFi.channel();
  
  serializeJson(doc, Serial2);
  Serial2.print("\n");
}

void test_gw() {
  if ((digitalRead(TEST_PIN) == LOW) || (digitalRead(TEST_PIN_SERIAL) == LOW)) {
    sendSensorsData(); 
    vTaskDelay(pdMS_TO_TICKS(200)); // Debounce ป้องกันการส่งข้อมูลซ้ำ
  }
}
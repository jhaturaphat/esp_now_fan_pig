#include "ConfigManager.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

// #define DEBUG  // เอาคอมเมนต์ออกเมิ่ออยู่ในโหมด DEBUG

#define MAX_SENSORS 10

#define SECURITY 34 //สำหรับใช้ป้องกันการ COPY

#define KID_PIN 34 //สำหรับป้องกันโปรแกรม

#define RELAY1_PIN 16  //out put
#define RELAY2_PIN 17  //out put
#define LED_STATUS 18  //out put
#define CONFIG_PIN 19  //input pulll up
#define TEST_PIN 23 //input pulll up
#define DISABLE_SIREN 25  //input pulll up

#define RXD2 32
#define TXD2 33

#define COMMUNICATION_TIMEOUT 30000  // 30 วินาที timeout 
#define TIMEOUT_SIREN 10000 // 10 วินาที timeout

// Delay
int period = 5000; // 10 วินาที
unsigned long time_now = 0;
const unsigned long SIREN_DURATION = 5000; // 5 วินาที
unsigned long PERIOD_LOSS = 0; // ตัวแปรสำหรับเก็บเวลาที่ทำการอัปเดตสถานะ LED ครั้งล่าสุด
const unsigned long LOSS_DURATION = 5000;  // 5 วินาที
unsigned long PERIOD_SIREN = 0; // ตัวแปรสำหรับเก็บเวลาที่ทำการอัปเดตสถานะ LED ครั้งล่าสุด

// เก็บสถานะของแต่ละ sensor
struct sensor_storage {  
  bool is_online;           // สถานะการเชื่อมต่อ
  bool switch_state;        // สถานะ switch
  unsigned long last_seen;  // เวลาที่รับสัญญาณครั้งล่าสุด
  uint8_t mac[6];          // MAC address ของ sensor
} sensors_storage[MAX_SENSORS];

// โครงสร้างข้อมูลที่รับจาก Sensor
typedef struct sensor_message {
  uint8_t sensor_id;      // ID ของ sensor (1-7)
  bool switch_status;     // สถานะ switch (true=ปกติ, false=แจ้งเตือน)
  uint32_t timestamp;     // timestamp
} sensor_message;

// เก็บรายการ sensor ที่มีปัญหา
String offline_sensors = "";
bool sensor_has_alarm = false;
int offline_count = 0; 
int alarm_count = 0;
// ตัวแปร Timeout
unsigned long sirenStartTime = 0;
bool sirenOn  = false; // สถานะไซเรนว่ากำลังดังอยู่หรือไม่

// 1. สร้าง Object ของ ConfigManager
ConfigManager cfgManager;
// 2. ตัวแปรสำหรับเก็บค่าที่ดึงมาจาก ConfigManager
DeviceConfig myConfig;

// Callback สำหรับรับข้อมูล (สำหรับ ESP32 Arduino Core 3.x)
void onDataReceive(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len){
  sensor_message msg;
  memcpy(&msg, incomingData, sizeof(msg));

  //  หาก sensor ไม่ใช้ 0 และ จำนวนเซ็นเซอร์เกินจำนวนที่ระบบบน MAX_SENSORS
  if(msg.sensor_id >= 1 && msg.sensor_id <= MAX_SENSORS){
    int index = msg.sensor_id - 1;
    // อัพเดทสถานะเซ็นเซอร์ ลงไปยัง sensors_storage    
    sensors_storage[index].is_online = true;
    sensors_storage[index].switch_state = msg.switch_status;
    sensors_storage[index].last_seen = millis();
    memcpy(sensors_storage[index].mac, recv_info->src_addr, 6);     
  }
} //End

void checkSensorsCommunication(){
  offline_count = 0;  
  for(int i = 0; i < MAX_SENSORS; i++){
    // ตรวจสอบการสื่อสาร
    if(sensors_storage[i].last_seen > 0){ // sensor เคยส่งข้อมูลมาแล้ว
      if(millis() - sensors_storage[i].last_seen > COMMUNICATION_TIMEOUT){
        offline_count++;
          if(sensors_storage[i].is_online){
          // ตรวจสอบอีกว่า เคย Online มาก่อนหน้านี้ก็กำหนดสถานะใหม่เป็น Offline โดยใช้ !sensors_storage[i].is_online
          sensors_storage[i].is_online = !sensors_storage[i].is_online;
          sensors_storage[i].switch_state = true;
          #if defined(DEBUG)
          Serial.printf("⚠️ WARNING: Lost communication with Sensor %d\n", i + 1);
          #endif          
        }
      }
    }
  }
}

// สร้าง Array ตามจำนวน MAXSENSOR เพื่อเก็บค่า แต้ละเซ็นเซอร์ไว้ที่ตำแหน่งต่างๆตาม index
void initializeSensorStorage() {
  for (int i = 0; i < MAX_SENSORS; i++) {
    sensors_storage[i].is_online = false;
    sensors_storage[i].switch_state = true;  // เริ่มต้นเป็น closed
    sensors_storage[i].last_seen = 0;
    memset(sensors_storage[i].mac, 0, 6);
  }
}

void CheckHashAlarm(){
  alarm_count = 0;
  for(int i = 0; i < MAX_SENSORS; i++){
    if(sensors_storage[i].is_online && (sensors_storage[i].last_seen > 0)){
      if(sensors_storage[i].switch_state == LOW){
        alarm_count++;
      }
    }
  }
}

#if defined(DEBUG)
void printDebugSensorStatus(){
  yield();
  if (millis() >= time_now + period){
    time_now += period;
 
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║           SYSTEM STATUS              ║");
    Serial.println("╚══════════════════════════════════════╝");
    for (int i = 0; i < MAX_SENSORS; i++) {
      String status_icon = sensors_storage[i].is_online ? "🟢" : sensors_storage[i].last_seen > 0  ? "🔴" : "⚫" ;
      String switch_icon = sensors_storage[i].switch_state ? "☃️" : "🚨";
      String connection = sensors_storage[i].is_online ? "ONLINE " : "OFFLINE";
      String switch_status = sensors_storage[i].switch_state ? "ปกติ" : "ฉุกเฉิน  ";
      unsigned long time_since_last = sensors_storage[i].last_seen > 0 ? (millis() - sensors_storage[i].last_seen)  : 0;

      Serial.printf("Sensor %d: %s %s │ %s %s │ Last: %lu sec ago\n",
                    i + 1,
                    status_icon.c_str(), connection.c_str(),
                    switch_icon.c_str(), switch_status.c_str(),
                    time_since_last);  

    }
  }
}
#endif

void sendSensorsData(){
  StaticJsonDocument<1024> doc;
  JsonArray arr = doc.to<JsonArray>();

  for(int i = 0; i < MAX_SENSORS; i++){
    JsonObject obj = arr.createNestedObject();
    obj["id"] = i + 1;
    obj["online"] = sensors_storage[i].is_online;
    obj["switch"] = sensors_storage[i].switch_state;
    obj["uptime"] = sensors_storage[i].last_seen / 1000;
  }
  // ส่งผ่าน Serial พร้อม delimiter
  serializeJson(doc, Serial2);
  Serial2.print("\n"); // ใช้ newline เป็น delimiter
}

void blinkLED(){
  for(int i = 0; i < 5; i++){
    digitalWrite(LED_STATUS, HIGH);
    delay(100);
    digitalWrite(LED_STATUS, LOW);
    delay(100);
  }
}

void setup() {
  #if defined(DEBUG)
  Serial.begin(115200);
  #endif
  // Serial2.begin(9600, SERIAL_8N1, RX, TX);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  pinMode(KID_PIN, INPUT_PULLUP);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED_STATUS, OUTPUT); 
  pinMode(CONFIG_PIN, INPUT_PULLUP);
  pinMode(TEST_PIN, INPUT_PULLUP);

  cfgManager.begin(CONFIG_PIN); 
  #if defined(DEBUG)
  Serial.println("เมื่อโค้ดมาถึงตรงนี้ หมายความว่า ESP ได้เข้าสู่ Normal Operation Mode แล้ว");
  #endif
  myConfig = cfgManager.getConfig();

  // ตั้งค่า WiFi Mode
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // ป้องกัน WiFi sleep สำหรับ ESP-NOW

  // กำหนด Channel
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(myConfig.channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // เริ่มต้น ESP-NOW
  if (esp_now_init() != ESP_OK) {
    #if defined(DEBUG)
    Serial.println("Error initializing ESP-NOW");
    #endif
    return;
  }

  // ลงทะเบียน callback สำหรับรับข้อมูล
  esp_now_register_recv_cb(onDataReceive);

  #if defined(DEBUG)
  Serial.println("\n✅ ESP32 Gateway Ready - 24x7 Mode");
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║            GATEWAY INFO              ║");
  Serial.println("╚══════════════════════════════════════╝");
  
  // แสดงข้อมูล hardware
  Serial.printf("🔧 Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("🔧 Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("🔧 CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  
  // แสดงข้อมูล WiFi/ESP-NOW
  Serial.printf("📡 MAC Address: %s\n", WiFi.macAddress().c_str());
  Serial.printf("📶 WiFi Channel: %d\n", WiFi.channel());
  Serial.printf("🔧 Expected message size: %d bytes\n", sizeof(sensor_message));
  
  Serial.println("════════════════════════════════════════");
  Serial.println("🔍 Waiting for ESP-01 sensors...");
  Serial.println("   Copy this MAC to ESP-01 code:");
  // แปลง MAC address ให้ถูกต้อง
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("   {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}\n", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println();
  #endif
  // สร้าง Array ตามจำนวน MAXSENSOR เพื่อเก็บค่า แต่ละเซ็นเซอร์ไว้ที่ตำแหน่งต่างๆตาม index
  initializeSensorStorage();

  if(digitalRead(KID_PIN) != LOW){  
    #if defined(DEBUG)  
    Serial.print("ป้องกัน Code Protection ต่อขานี้ลงกร์าว");    
    #endif
    return;
  }

  blinkLED();

  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  delay(100);
  digitalWrite(RELAY1_PIN, HIGH); 
  digitalWrite(RELAY2_PIN, HIGH);  

}



void loop() {
  // blinkLED();
  digitalWrite(LED_STATUS, LOW);
  checkSensorsCommunication();
  CheckHashAlarm();
  if((alarm_count > 0) || (offline_count >= MAX_SENSORS)){
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    // digitalWrite(LED_STATUS, HIGH);
    if (millis() - PERIOD_SIREN >= SIREN_DURATION) {      
      PERIOD_SIREN = millis();      
      #if defined(DEBUG)
      Serial.printf("แจ้งเตือน %d 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨\n", alarm_count);
      #endif  
      sendSensorsData();    
    }
  }else{
    digitalWrite(RELAY1_PIN, HIGH);     
  }
  // Relay 2 มีเซ็นเซอร์บางตัว offline ใช้สัญญาณไฟ
  if(offline_count > 0 ){  
    digitalWrite(RELAY2_PIN, LOW);
    if (millis() - PERIOD_LOSS >= LOSS_DURATION) {
      PERIOD_LOSS = millis();
      sendSensorsData();
      #if defined(DEBUG)
      Serial.printf("มีเซ็นเซอร์จำนวน %d Offline 📵📵📵📵📵📵📵📵📵📵📵📵📵📵📵📵📵📵📵\n" ,offline_count);
      #endif      
    }    
  }else{
    digitalWrite(RELAY2_PIN, HIGH);    
  }
  #if defined(DEBUG)
  printDebugSensorStatus();
  #endif  
  digitalWrite(LED_STATUS, HIGH);
}

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#define DEBUG  // เอาคอมเมนต์ออกเมิ่ออยู่ในโหมด DEBUG

#define MAX_SENSORS 10
#define ACTIVE_SIREN 16 
#define LED_STATUS 17
#define RELAY_PIN 18
#define CONFIG_PIN 19

#define COMMUNICATION_TIMEOUT 30000  // 30 วินาที timeout 
#define TIMEOUT_SIREN 10000 // 10 วินาที timeout

// ตัวแปร Timeout
unsigned long sirenStartTime = 0;
bool sirenOn  = false; // สถานะไซเรนว่ากำลังดังอยู่หรือไม่
const unsigned long SIREN_DURATION = 10000; // 10 วินาที
const unsigned long SILENCE_DURATION = 5000;  // 5 วินาที

// ตัวแปรที่ต้องกำหนดจาก config
#define CHANNEL 1

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

    if(!msg.switch_status){
      sensor_has_alarm = ture;
      #if defined(DEBUG)
      Serial.printf("Sensor ID %d มีการแจ้งเตือน 🚨 \n", msg.sensor_id);
      #endif
    }
  }
}

void checkSensorsCommunication(){
  offline_count = 0;
  for(int i - 0; i < MAX_SENSORS; i++){
    // ตรวจสอบการสื่อสาร
    if(sensors_storage[i].last_seen > 0){ // sensor เคยส่งข้อมูลมาแล้ว
      if(millis() - sensors_storage[i].last_seen > COMMUNICATION_TIMEOUT){
        offline_count++;
          if(sensor_storage[i].is_online){
          // ตรวจสอบอีกว่า เคย Online มาก่อนหน้านี้ก็กำหนดสถานะใหม่เป็น Offline โดยใช้ !sensors_storage[i].is_online
          sensors_storage[i].is_online = !sensors_storage[i].is_online;
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
  if(sensor_has_alarm){
    unsigned long currentMillis = millis();   
     
    if(sirenOn){
      // ตรวจสอบว่าเปิดมานานเกิน 10 วินาทีหรือยัง
      if (currentMillis - lastStateChangeTime >= SIREN_DURATION) {
        sirenOn = false;             // เปลี่ยนสถานะเป็นปิด
        digitalWrite(ACTIVE_SIREN, HIGH); // ปิดไซเรน
        lastStateChangeTime = currentMillis; // บันทึกเวลาที่เปลี่ยนสถานะ     
        #if defined(DEBUG)
        Serial.println("Siren OFF (Silence 5s)");
        #endif
      }
    }else{
      // ตรวจสอบว่าถึงเวลาต้องหยุดไซเรนหรือยัง
      if (currentMillis - lastStateChangeTime >= SILENCE_DURATION){
        sirenOn = true;              // เปลี่ยนสถานะเป็นเปิด
        digitalWrite(ACTIVE_SIREN, LOW); // เปิดไซเรน
        lastStateChangeTime = currentMillis; // บันทึกเวลาที่เปลี่ยนสถานะ
        #if defined(DEBUG)
        Serial.println("Siren ON (Active 10s)");
        #endif
      }
    }    

  }else{
    // ถ้าไม่มี alarm แล้ว ให้ปิดไซเรนทันทีและรีเซ็ตสถานะ
    if (sirenOn) {
      digitalWrite(ACTIVE_SIREN, HIGH);
      sirenOn = false;
      #if defined(DEBUG)
      Serial.println("Alarm cleared. Siren OFF now.");
      #endif
    }
  }
}

void setup() {
  #if defined(DEBUG)
  Serial.begin(115200);
  #endif

  pinMode(ACTIVE_SIREN, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(CONFIG_PIN, INPUT_PULLUP);

  // ตั้งค่า WiFi Mode
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // ป้องกัน WiFi sleep สำหรับ ESP-NOW

  // กำหนด Channel
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE);
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

}



void loop() {
  checkSensorsCommunication();
  CheckHashAlarm();

}

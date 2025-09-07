/*
 * ESP32 Gateway Code - รับสัญญาณจาก ESP-01 Sensors จำนวน 7 ตัว
 * ใช้ ESP-NOW Protocol สำหรับการสื่อสาร
 * Enhanced version - แสดงรายละเอียด sensor ที่ขาดการติดต่อและ switch ที่เปิด
 */

#include <esp_now.h>
#include <WiFi.h>

// กำหนด PIN
#define BUZZER_PIN  18       // Buzzer สำหรับแจ้งเตือนขาดการสื่อสาร
#define SIREN_PIN 19        // Siren สำหรับแจ้งเตือนหลัก
#define LED_STATUS_PIN 2    // LED แสดงสถานะ

// จำนวน Sensor
#define MAX_SENSORS 7
#define COMMUNICATION_TIMEOUT 30000  // 30 วินาที timeout 
#define SIREN_TIMEOUT 60000 // 60 วินาที timeout

unsigned long lastSensorCheck = 0;
unsigned long lastComunication = 0;

// โครงสร้างข้อมูลที่รับจาก Sensor
typedef struct sensor_message {
  uint8_t sensor_id;      // ID ของ sensor (1-7)
  bool switch_status;     // สถานะ switch (true=closed, false=open)
  uint32_t timestamp;     // timestamp
} sensor_message;

// เก็บสถานะของแต่ละ sensor
struct sensor_status {
  bool is_online;           // สถานะการเชื่อมต่อ
  bool switch_state;        // สถานะ switch
  unsigned long last_seen;  // เวลาที่รับสัญญาณครั้งล่าสุด
  uint8_t mac[6];          // MAC address ของ sensor
};

sensor_status sensors[MAX_SENSORS];
bool siren_active = false;
bool buzzer_active = false;
unsigned long last_check_time = 0;
unsigned long last_status_print = 0;
unsigned long siren_start_time = 0;
unsigned long buzzer_start_time = 0;

// เก็บรายการ sensor ที่มีปัญหา
String offline_sensors = "";
String open_switches = "";
String system_status = "";


void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); 
  // กำหนด PIN Mode
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SIREN_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  
  // เริ่มต้น WiFi ในโหมด Station
  WiFi.mode(WIFI_STA);
  // Serial.println("MAC Address: " + WiFi.macAddress());
  
  // เริ่มต้น ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

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
  
  // ลงทะเบียน callback function (ESP32 Arduino Core 3.x)
  esp_now_register_recv_cb(onDataReceive);
  
  // เริ่มต้นสถานะ sensor
  initializeSensors();
  
  Serial.println("ESP32 Gateway Ready");
  Serial.println("Waiting for sensors...");
  
  // ทดสอบระบบเสียง
  testSounds();
}

void loop() {
  unsigned long currentMillis = millis();
  
  if(currentMillis - lastSensorCheck >= SIREN_TIMEOUT){ //ทำงานทุกๆ 30 วินาที
    lastSensorCheck = currentMillis;   
    checkSensorCommunication();
    handleAlarms();
    updateStatusLED();
  }
  
  // แสดงสถานะระบบทุก 10 วินาที
  if (currentMillis - last_status_print >= 10000) {
    last_status_print = currentMillis;
    printSystemStatus();
  }

  // ตรวจสอบ Comunication Loss
  if(currentMillis - lastComunication >= SIREN_TIMEOUT){
    lastComunication = currentMillis;
    handleComunicationAlarms();
  }
}

// Callback สำหรับรับข้อมูล (สำหรับ ESP32 Arduino Core 3.x)
void onDataReceive(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  sensor_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  
  if (msg.sensor_id >= 1 && msg.sensor_id <= MAX_SENSORS) {
    int index = msg.sensor_id - 1;
    
    // อัพเดทสถานะ sensor
    sensors[index].is_online = true;
    sensors[index].switch_state = msg.switch_status;
    sensors[index].last_seen = millis();
    memcpy(sensors[index].mac, recv_info->src_addr, 6); 
    
    // ตรวจสอบสถานะ switch
    if (!msg.switch_status) {  // switch เปิด (แม่เหล็กออกจากกัน)
      triggerSiren();
      // Serial.printf("🚨ALERT: Sensor %d detected intrusion!\n", msg.sensor_id);
    }
  }
}

void initializeSensors() {
  for (int i = 0; i < MAX_SENSORS; i++) {
    sensors[i].is_online = false;
    sensors[i].switch_state = true;  // เริ่มต้นเป็น closed
    sensors[i].last_seen = 0;
    memset(sensors[i].mac, 0, 6);
  }
}

void checkSensorCommunication() {
  if (millis() - last_check_time < 5000) return;  // ตรวจสอบทุก 5 วินาที
  
  last_check_time = millis();
  int offline_count = 0;
  int open_switch_count = 0;
  
  // รีเซ็ตรายการ
  offline_sensors = "";
  open_switches = "";
  
  for (int i = 0; i < MAX_SENSORS; i++) {
    // ตรวจสอบการสื่อสาร
    if (sensors[i].last_seen > 0) {  // sensor เคยส่งข้อมูลมาแล้ว
      if (millis() - sensors[i].last_seen > COMMUNICATION_TIMEOUT) {
        if (sensors[i].is_online) {
          Serial.printf("⚠️ WARNING: Lost communication with Sensor %d\n", i + 1);
          sensors[i].is_online = false;
        }
        offline_count++;        
        // เพิ่ม sensor ID ที่ขาดการติดต่อ
        if (offline_sensors.length() > 0) offline_sensors += ", ";
        offline_sensors += String(i + 1);
      }
    }
    
    // ตรวจสอบ switch ที่เปิด
    if (sensors[i].is_online && !sensors[i].switch_state) {
      open_switch_count++;
      
      // เพิ่ม sensor ID ที่ switch เปิด
      if (open_switches.length() > 0) open_switches += ", ";
      open_switches += String(i + 1);
    }
  }
  
  // แสดงรายงานสถานะ
  if (offline_count > 0) {
    Serial.printf("📡 COMMUNICATION LOST: Sensors [%s] (%d/%d sensors)\n", 
                  offline_sensors.c_str(), offline_count, MAX_SENSORS);
  }
  
  if (open_switch_count > 0) {
    Serial.printf("🚨 SWITCHES OPEN: Sensors [%s] (%d switches open)\n", 
                  open_switches.c_str(), open_switch_count);
  }
  
  // จัดการเสียงเตือนตามจำนวน sensor ที่ขาดการสื่อสาร
  if (offline_count == MAX_SENSORS) {
    // ขาดการสื่อสารทั้งหมด -> เปิดไซเรน
    triggerSiren();
    Serial.printf("🚨 CRITICAL: Lost communication with ALL sensors! [%s]\n", offline_sensors.c_str());
  } else if (offline_count > 0 && offline_count < MAX_SENSORS) {  
    // ขาดการสื่อสารบางตัว -> เปิด buzzer
    triggerBuzzer();
    Serial.printf("⚠️ WARNING: Partial communication loss - Sensors [%s] offline\n", offline_sensors.c_str());
  } else {
    // ทุก sensor เชื่อมต่อปกติ -> ปิดเสียงเตือน
    if (buzzer_active && !siren_active) {
      stopBuzzer();
      Serial.println("✅ All sensors back online - Buzzer deactivated");
    }
  }
}

//🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
void triggerSiren() {  
  if (!siren_active) {
    siren_active = true;
    siren_start_time = millis();
    digitalWrite(SIREN_PIN, HIGH);
    // Serial.println("🚨 SIREN ACTIVATED! 🚨");
    //ส่งข้อมูลไปยัง ESP ตัวที่ 2
    Serial2.println(getSystemStatus());   
    // ปิด buzzer เมื่อเปิดไซเรน
    if (buzzer_active) {
      stopBuzzer();
    }
  }
}

void triggerBuzzer() {
  // Serial.println("📢buzzer_active:"+(String)buzzer_active+"📢siren_active:"+(String)siren_active);
  if (!buzzer_active && !siren_active) {   
    
    buzzer_active = true;
    buzzer_start_time = millis();
    Serial.println("🎉 BUZZER ACTIVATED! 🎉");
  }
}

void stopSiren() {
  if (siren_active) {
    siren_active = false;
    digitalWrite(SIREN_PIN, LOW);
    Serial.println("✅ SIREN DEACTIVATED");
  }
}

void stopBuzzer() {
  if (buzzer_active) {
    buzzer_active = false;
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("✅ BUZZER DEACTIVATED");
  }
}

void handleAlarms() {
  // จัดการไซเรน (เปิดไว้ 10 วินาที)    
  if (siren_active && ((millis() - siren_start_time > SIREN_TIMEOUT))) {
    stopSiren();    
  }
}

// จัดการ buzzer BUZZER_PIN = 18
//📢📢📢📢📢📢📢📢📢📢📢
void handleComunicationAlarms(){    
  if (buzzer_active && !siren_active) {    
    Serial2.println(getSystemStatus());     
    digitalWrite(BUZZER_PIN, HIGH);    
  }else{
    digitalWrite(BUZZER_PIN, LOW);
  }
}

void updateStatusLED() {
  // LED กระพริบแสดงสถานะ
  if (siren_active) {
    // กระพริบเร็วเมื่อไซเรนเปิด
    digitalWrite(LED_STATUS_PIN, (millis() / 100) % 2);
  } else if (buzzer_active) {
    // กระพริบช้าเมื่อ buzzer เปิด
    digitalWrite(LED_STATUS_PIN, (millis() / 300) % 2);
  } else {
    // เปิดค้างเมื่อทำงานปกติ
    digitalWrite(LED_STATUS_PIN, HIGH);
  }
}

void testSounds() {
  Serial.println("Testing buzzer...");
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
  
  Serial.println("Testing siren...");
  digitalWrite(SIREN_PIN, HIGH);
  delay(200);
  digitalWrite(SIREN_PIN, LOW);
  
  Serial.println("Sound test complete");
}

String getSystemStatus(){
  system_status = "START\n";  // เริ่มด้วย marker;
  for (int i = 0; i < MAX_SENSORS; i++) {
    // String status_icon = sensors[i].is_online ? "🟢" : "🔴";
    String status_icon = sensors[i].is_online ? "🟢" : sensors[i].last_seen > 0 ? "🔴" : "⚫" ;
    String switch_icon = sensors[i].switch_state ? "🔒" : "🚨";
    String connection = sensors[i].is_online ? "ONLINE " : "OFFLINE";
    String switch_status = sensors[i].switch_state ? "CLOSED" : "OPEN  ";
    
    unsigned long time_since_last = sensors[i].last_seen > 0 ? (millis() - sensors[i].last_seen) / 1000 : 0;
    
    system_status += "Sensor " + String(i + 1) + ": " + 
                status_icon.c_str() + " " + connection.c_str() + " │ " + 
                switch_icon.c_str() + " " + switch_status.c_str() + " │ Last: " + 
                String(time_since_last) + " sec ago\n";
  }
  
  return system_status += "END\n";  // จบด้วย marker;
}

void printSystemStatus() {
  
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║           SYSTEM STATUS              ║");
  Serial.println("╚══════════════════════════════════════╝");
  
  int online_count = 0;
  int offline_count = 0;
  int open_count = 0;
  
  for (int i = 0; i < MAX_SENSORS; i++) {
    // String status_icon = sensors[i].is_online ? "🟢" : "🔴";
    String status_icon = sensors[i].is_online ? "🟢" : sensors[i].last_seen > 0  ? "🔴" : "⚫" ;
    String switch_icon = sensors[i].switch_state ? "🔒" : "🚨";
    String connection = sensors[i].is_online ? "ONLINE " : "OFFLINE";
    String switch_status = sensors[i].switch_state ? "CLOSED" : "OPEN  ";
    
    unsigned long time_since_last = sensors[i].last_seen > 0 ? (millis() - sensors[i].last_seen) / 1000 : 0;
    
    Serial.printf("Sensor %d: %s %s │ %s %s │ Last: %lu sec ago\n",
                  i + 1,
                  status_icon.c_str(), connection.c_str(),
                  switch_icon.c_str(), switch_status.c_str(),
                  time_since_last);   

    if (sensors[i].is_online) online_count++;
    else offline_count++;
    
    if (!sensors[i].switch_state) open_count++;
  } 
  
  
  Serial.println("─────────────────────────────────────────");
  Serial.printf("📊 Summary: %d Online │ %d Offline │ %d Switches Open\n", 
                online_count, offline_count, open_count);
  
  if (offline_count > 0) {
    Serial.printf("📡 Offline Sensors: [%s]\n", offline_sensors.c_str());
  }
  
  if (open_count > 0) {
    Serial.printf("🚨 Open Switches: [%s]\n", open_switches.c_str());
  }
  
  Serial.printf("🔊 Alarms: %s%s\n", 
                siren_active ? "SIREN " : "",
                buzzer_active ? "BUZZER " : "");
  
  Serial.println("═════════════════════════════════════════\n");
}
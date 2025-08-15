/*
 * ESP32 Gateway Code - รับสัญญาณจาก ESP-01 Sensors จำนวน 7 ตัว
 * ใช้ ESP-NOW Protocol สำหรับการสื่อสาร
 */

#include <esp_now.h>
#include <WiFi.h>

// กำหนด PIN
#define BUZZER_PIN 18       // Buzzer สำหรับแจ้งเตือนขาดการสื่อสาร
#define SIREN_PIN 19        // Siren สำหรับแจ้งเตือนหลัก
#define LED_STATUS_PIN 2    // LED แสดงสถานะ

// จำนวน Sensor
#define MAX_SENSORS 7
#define COMMUNICATION_TIMEOUT 30000  // 30 วินาที timeout

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
unsigned long siren_start_time = 0;
unsigned long buzzer_start_time = 0;

void setup() {
  Serial.begin(115200);
  
  // กำหนด PIN Mode
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SIREN_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  
  // เริ่มต้น WiFi ในโหมด Station
  WiFi.mode(WIFI_STA);
  Serial.println("MAC Address: " + WiFi.macAddress());
  
  // เริ่มต้น ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
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
  checkSensorCommunication();
  handleAlarms();
  updateStatusLED();
  delay(1000);
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
    
    Serial.printf("🟢Sensor %d: Switch=%s, RSSI=%d\n", 
                  msg.sensor_id, 
                  msg.switch_status ? "CLOSED" : "OPEN",
                  recv_info->rx_ctrl->rssi);
    
    // ตรวจสอบสถานะ switch
    if (!msg.switch_status) {  // switch เปิด (แม่เหล็กออกจากกัน)
      triggerSiren();
      Serial.printf("ALERT: Sensor %d detected intrusion!\n", msg.sensor_id);
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
  if (millis() - last_check_time < 10000) return;  // ตรวจสอบทุก 10 วินาที
  
  last_check_time = millis();
  int offline_count = 0;
  
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (sensors[i].last_seen > 0) {  // sensor เคยส่งข้อมูลมาแล้ว
      if (millis() - sensors[i].last_seen > COMMUNICATION_TIMEOUT) {
        if (sensors[i].is_online) {
          Serial.printf("⛔WARNING: Lost communication with Sensor %d\n", i + 1);
          sensors[i].is_online = false;
        }
        offline_count++;
      }
    }
  }
  
  // จัดการเสียงเตือนตามจำนวน sensor ที่ขาดการสื่อสาร
  if (offline_count == MAX_SENSORS) {
    // ขาดการสื่อสารทั้งหมด -> เปิดไซเรน
    triggerSiren();
    Serial.println("CRITICAL: Lost communication with ALL sensors!");
  } else if (offline_count > 0 && offline_count < MAX_SENSORS) {
    // ขาดการสื่อสารบางตัว -> เปิด buzzer
    triggerBuzzer();
    Serial.printf("📵WARNING: %d sensors offline\n", offline_count);
  } else {
    // ทุก sensor เชื่อมต่อปกติ -> ปิดเสียงเตือน
    if (buzzer_active && !siren_active) {
      stopBuzzer();
    }
  }
}

void triggerSiren() {
  if (!siren_active) {
    siren_active = true;
    siren_start_time = millis();
    digitalWrite(SIREN_PIN, HIGH);
    Serial.println("SIREN ACTIVATED!");
    
    // ปิด buzzer เมื่อเปิดไซเรน
    if (buzzer_active) {
      stopBuzzer();
    }
  }
}

void triggerBuzzer() {
  if (!buzzer_active && !siren_active) {
    buzzer_active = true;
    buzzer_start_time = millis();
    Serial.println("BUZZER ACTIVATED!");
  }
}

void stopSiren() {
  if (siren_active) {
    siren_active = false;
    digitalWrite(SIREN_PIN, LOW);
    Serial.println("SIREN DEACTIVATED");
  }
}

void stopBuzzer() {
  if (buzzer_active) {
    buzzer_active = false;
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("BUZZER DEACTIVATED");
  }
}

void handleAlarms() {
  // จัดการไซเรน (เปิดไว้ 10 วินาที)
  if (siren_active && millis() - siren_start_time > 10000) {
    stopSiren();
  }
  
  // จัดการ buzzer (เสียงสั่น)
  if (buzzer_active && !siren_active) {
    if ((millis() / 500) % 2) {  // เสียงสั่นทุก 500ms
      digitalWrite(BUZZER_PIN, HIGH);
    } else {
      digitalWrite(BUZZER_PIN, LOW);
    }
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

void printSystemStatus() {
  Serial.println("\n=== SYSTEM STATUS ===");
  for (int i = 0; i < MAX_SENSORS; i++) {
    Serial.printf("🟢🟢Sensor %d: %s, Switch: %s, Last seen: %lu ms ago\n",
                  i + 1,
                  sensors[i].is_online ? "ONLINE" : "OFFLINE",
                  sensors[i].switch_state ? "CLOSED" : "OPEN",
                  sensors[i].last_seen > 0 ? millis() - sensors[i].last_seen : 0);
  }
  Serial.println("==================\n");
}
/*
 * ESP32 Gateway Code - รับสัญญาณจาก ESP-01 Sensors จำนวน 7 ตัว
 * ใช้ ESP-NOW Protocol สำหรับการสื่อสาร
 * Fixed Version
 */

// กำหนด PIN
#define BUZZER_PIN 18       // Buzzer สำหรับแจ้งเตือนขาดการสื่อสาร
#define SIREN_PIN 19        // Siren สำหรับแจ้งเตือนหลัก
#define LED_STATUS_PIN 2    // LED แสดงสถานะ

// จำนวน Sensor
#define MAX_SENSORS 7
#define COMMUNICATION_TIMEOUT 30000  // 30 วินาที timeout

#define BLYNK_PRINT Serial
#define BLYNK_TEMPLATE_ID "TMPL6ngbPI81S"
#define BLYNK_TEMPLATE_NAME "test01"
#define BLYNK_AUTH_TOKEN "eKOKJX72HVSjWGsNVasTbh0aSUIazDU-"

#include <BlynkSimpleEsp32.h>
#include <esp_now.h>
#include <WiFi.h>

unsigned long lastSensorCheck = 0;
unsigned long lastLedUpdate = 0;

char ssid[] = "kid_2.4GHz";
char pass[] = "xx3xx3xx";

// ⚠️ หากยังเชื่อมต่อไม่ได้ ให้ลองเปลี่ยนข้อมูล WiFi ตรงนี้
// char ssid[] = "ชื่อ_WiFi_ที่แน่ใจ";
// char pass[] = "รหัสผ่าน_ที่แน่ใจ";

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
  delay(1000);
  Serial.println("Starting ESP32 Gateway...");

  // กำหนด PIN Mode ก่อน
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(SIREN_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);
  
  // ปิดเสียงเตือนทั้งหมดก่อน
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(SIREN_PIN, LOW);
  digitalWrite(LED_STATUS_PIN, LOW);
  
  // เริ่มต้นสถานะ sensor
  initializeSensors();
  
  // เริ่มต้น WiFi ในโหมด Station พร้อม ESP-NOW
  WiFi.mode(WIFI_AP_STA);  // ✅ เปลี่ยนเป็น AP+STA เพื่อรองรับ ESP-NOW
  Serial.println("📡 ESP32 MAC Address: " + WiFi.macAddress());
  Serial.println("📡 ESP32 AP MAC Address: " + WiFi.softAPmacAddress());
  
  // เริ่มต้น ESP-NOW ก่อน WiFi
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ Error initializing ESP-NOW");
    return;
  }
  Serial.println("✅ ESP-NOW initialized successfully");
  
  // ลงทะเบียน callback function
  esp_now_register_recv_cb(onDataReceive);
  Serial.println("✅ ESP-NOW callback registered");
  Serial.println("🎯 Gateway is ready to receive from any ESP-01 sensor");
  Serial.println("📋 Expected sensor IDs: 1, 2, 3, 4, 5, 6, 7");
  
  // เชื่อมต่อ WiFi สำหรับ Blynk
  Serial.println("🔍 Scanning for WiFi networks...");
  int n = WiFi.scanNetworks();
  Serial.printf("Found %d networks:\n", n);
  for (int i = 0; i < n; i++) {
    Serial.printf("  %d: %s (RSSI: %d) %s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), 
                  WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "[OPEN]" : "[SECURED]");
  }
  Serial.println();
  
  // ตั้งค่า WiFi แบบละเอียด
  WiFi.disconnect(true);  // ปิดการเชื่อมต่อเก่า
  delay(1000);
  WiFi.mode(WIFI_AP_STA);    // ตั้งเป็น AP+STA mode
  WiFi.setHostname("ESP32-Gateway");  // ตั้งชื่อ device
  
  Serial.printf("🔗 Connecting to WiFi: %s\n", ssid);
  WiFi.begin(ssid, pass);
  
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 30) {
    delay(1000);
    wifi_attempts++;
    Serial.printf("Attempt %d/30 - Status: %d\n", wifi_attempts, WiFi.status());
    
    // ลองเชื่อมต่อใหม่ทุก 10 ครั้ง
    if (wifi_attempts % 10 == 0) {
      Serial.println("🔄 Retrying WiFi connection...");
      WiFi.disconnect();
      delay(1000);
      WiFi.begin(ssid, pass);
    }
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi connected successfully!");
    Serial.printf("📶 SSID: %s\n", WiFi.SSID());
    Serial.printf("📡 IP address: %s\n", WiFi.localIP().toString());
    Serial.printf("📶 Signal strength: %d dBm\n", WiFi.RSSI());
    Serial.printf("🔐 MAC Address: %s\n", WiFi.macAddress());
    
    // เริ่มต้น Blynk
    Serial.println("🔗 Connecting to Blynk...");
    Blynk.config(BLYNK_AUTH_TOKEN);
    if (Blynk.connect()) {
      Serial.println("✅ Blynk connected");
    } else {
      Serial.println("❌ Blynk connection failed");
    }
  } else {
    Serial.println("\n❌ WiFi connection failed!");
    Serial.printf("Final WiFi status: %d\n", WiFi.status());
    Serial.println("📋 WiFi Status Codes:");
    Serial.println("  0 = WL_IDLE_STATUS");
    Serial.println("  1 = WL_NO_SSID_AVAIL (Network not found)");
    Serial.println("  2 = WL_SCAN_COMPLETED");
    Serial.println("  3 = WL_CONNECTED");
    Serial.println("  4 = WL_CONNECT_FAILED (Wrong password)");
    Serial.println("  5 = WL_CONNECTION_LOST");
    Serial.println("  6 = WL_DISCONNECTED");
    Serial.println("⚠️ Running without Blynk (ESP-NOW only mode)");
  }
  
  // ทดสอบระบบเสียง
  testSounds();
  
  Serial.println("\n🚀 ESP32 Gateway Ready");
  Serial.println("📡 Listening for ESP-NOW messages on all channels...");
  Serial.println("🔍 Waiting for sensors to register...");
  
  // แสดงสถานะเริ่มต้น
  printSystemStatus();
}

void loop() {
  unsigned long currentMillis = millis();

  // ตรวจสอบ Sensor ทุก 1 วินาที
  if (currentMillis - lastSensorCheck >= 1000) {
    lastSensorCheck = currentMillis;
    checkSensorCommunication();
    handleAlarms();  // ✅ เพิ่มการจัดการเสียงเตือน
  }

  // อัพเดท LED ทุก 200 ms
  if (currentMillis - lastLedUpdate >= 200) {
    lastLedUpdate = currentMillis;
    updateStatusLED();
  }

  // Blynk ต้องรันตลอด (เฉพาะเมื่อ WiFi เชื่อมต่อ)
  if (WiFi.status() == WL_CONNECTED) {
    Blynk.run();
  }
  
  // แสดงสถานะระบบทุก 10 วินาที (ลดลงเพื่อ debug)
  static unsigned long lastStatusPrint = 0;
  if (currentMillis - lastStatusPrint >= 10000) {
    lastStatusPrint = currentMillis;
    printSystemStatus();
  }
}

// ✅ แก้ไข Callback สำหรับ ESP32 Arduino Core ใหม่
void onDataReceive(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
  Serial.println("📨 ESP-NOW message received!");
  Serial.printf("   From MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
                recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5]);
  Serial.printf("   Data length: %d bytes\n", len);
  
  if (len != sizeof(sensor_message)) {
    Serial.printf("❌ Invalid message size: %d bytes (expected: %d)\n", len, sizeof(sensor_message));
    Serial.print("Raw data: ");
    for (int i = 0; i < len; i++) {
      Serial.printf("%02X ", incomingData[i]);
    }
    Serial.println();
    return;
  }
  
  sensor_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  
  Serial.printf("   Parsed - Sensor ID: %d, Switch: %s, Timestamp: %lu\n", 
                msg.sensor_id, msg.switch_status ? "CLOSED" : "OPEN", msg.timestamp);
  
  if (msg.sensor_id < 1 || msg.sensor_id > MAX_SENSORS) {
    Serial.printf("❌ Invalid sensor ID: %d (valid range: 1-%d)\n", msg.sensor_id, MAX_SENSORS);
    return;
  }
  
  int index = msg.sensor_id - 1;
  
  // อัพเดทสถานะ sensor
  sensors[index].is_online = true;
  sensors[index].switch_state = msg.switch_status;
  sensors[index].last_seen = millis();
  memcpy(sensors[index].mac, recv_info->src_addr, 6);
  
  Serial.printf("✅ Sensor %d registered successfully! Switch=%s\n", 
                msg.sensor_id, msg.switch_status ? "CLOSED" : "OPEN");
  
  // ✅ ส่งสถานะขึ้น Blynk (เมื่อเชื่อมต่อ)
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    Blynk.virtualWrite(index, msg.switch_status ? 1 : 0);  // V0..V6
    Serial.printf("📤 Sent to Blynk V%d: %d\n", index, msg.switch_status ? 1 : 0);
  }
  
  // ตรวจสอบสถานะ switch
  if (!msg.switch_status) {  // switch เปิด (แม่เหล็กออกจากกัน)
    triggerSiren();
    Serial.printf("🚨 ALERT: Sensor %d detected intrusion!\n", msg.sensor_id);
  }
}

void initializeSensors() {
  for (int i = 0; i < MAX_SENSORS; i++) {
    sensors[i].is_online = false;
    sensors[i].switch_state = true;  // เริ่มต้นเป็น closed
    sensors[i].last_seen = 0;
    memset(sensors[i].mac, 0, 6);
  }
  Serial.println("✅ Sensors initialized");
}

void checkSensorCommunication() {
  if (millis() - last_check_time < 10000) return;  // ตรวจสอบทุก 10 วินาที
  
  last_check_time = millis();
  int offline_count = 0;
  int registered_count = 0;  // นับ sensor ที่เคยส่งข้อมูล
  
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (sensors[i].last_seen > 0) {  // sensor เคยส่งข้อมูลมาแล้ว
      registered_count++;
      if (millis() - sensors[i].last_seen > COMMUNICATION_TIMEOUT) {
        if (sensors[i].is_online) {
          Serial.printf("⛔ WARNING: Lost communication with Sensor %d\n", i + 1);
          sensors[i].is_online = false;
          
          // แจ้ง OFFLINE บน Blynk
          if (WiFi.status() == WL_CONNECTED) {
            Blynk.virtualWrite(i, 0);
          }
        }
        offline_count++;
      }
    }
  }
  
  // จัดการเสียงเตือนเฉพาะ sensor ที่ลงทะเบียนแล้ว
  if (registered_count == 0) {
    // ยังไม่มี sensor ใดลงทะเบียน -> ไม่เตือน
    return;
  }
  
  if (offline_count == registered_count) {
    // ขาดการสื่อสารทั้งหมด -> เปิดไซเรน
    triggerSiren();
    Serial.printf("🚨 CRITICAL: Lost communication with ALL %d sensors!\n", registered_count);
  } else if (offline_count > 0) {
    // ขาดการสื่อสารบางตัว -> เปิด buzzer
    triggerBuzzer();
    Serial.printf("📵 WARNING: %d/%d sensors offline\n", offline_count, registered_count);
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
    Serial.println("🚨 SIREN ACTIVATED!");
    
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
    Serial.println("🔔 BUZZER ACTIVATED!");
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
  Serial.println("🔊 Testing buzzer...");
  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);
  delay(200);
  
  Serial.println("🔊 Testing siren...");
  digitalWrite(SIREN_PIN, HIGH);
  delay(200);
  digitalWrite(SIREN_PIN, LOW);
  
  Serial.println("✅ Sound test complete");
}

void printSystemStatus() {
  Serial.println("\n=== 📊 SYSTEM STATUS ===");
  Serial.printf("WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
  Serial.printf("Blynk: %s\n", Blynk.connected() ? "Connected" : "Disconnected");
  Serial.printf("ESP-NOW: Active\n");
  
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (sensors[i].last_seen > 0) {
      Serial.printf("Sensor %d: %s, Switch: %s, Last seen: %lu ms ago\n",
                    i + 1,
                    sensors[i].is_online ? "🟢 ONLINE" : "🔴 OFFLINE",
                    sensors[i].switch_state ? "CLOSED" : "OPEN",
                    millis() - sensors[i].last_seen);
    } else {
      Serial.printf("Sensor %d: ⚪ Never registered\n", i + 1);
    }
  }
  Serial.println("========================\n");
}

// ✅ Blynk Virtual Pin handlers
BLYNK_CONNECTED() {
  Serial.println("✅ Connected to Blynk Cloud");
}

BLYNK_DISCONNECTED() {
  Serial.println("❌ Disconnected from Blynk Cloud");
}
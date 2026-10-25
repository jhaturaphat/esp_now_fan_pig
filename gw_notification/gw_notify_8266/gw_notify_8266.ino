#include <Arduino.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "WifiManager.h"
#include "Notifying.h"
#include "debug.h"


// ตั้งค่า NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600, 60000); // UTC+7 สำหรับประเทศไทย
// สร้าง Object ของ Server ที่พอร์ต 80 รอไว้
AsyncWebServer server(80);
WifiManager wifiManager(&server);
Notifying notifier;

String inputBuffer = "";

const unsigned long NOTIFY_INTERVAL = 30000UL;
unsigned long lastNotifyTime = 0;

unsigned long LED_INTERVAL = 1000;
unsigned long lastLedTime = 0;
bool LED_BLANKING = LOW;

unsigned int NOTIFICATION_INTERVAL_HOURS = 1; 
bool scheduled_time = false;  //ส่งแจ้งเือนตามเวลา
int lastNotifiedHour = -1; // ตัวแปรสำหรับเก็บชั่วโมงล่าสุดที่มีการแจ้งเตือน

void setup() {
    #if defined(DEBUG)
    Serial.begin(115200);
    #else
    Serial.begin(9600);
    #endif
    pinMode(LED_PIN, OUTPUT);
    // เรียกฟังก์ชันจัดการ Wi-Fi (ถ้าต่อไม่ได้ ระบบจะเปิดเว็บ Config หน้า 192.168.4.1 ค้างไว้รอเรา)
    wifiManager.begin(false);
    #if defined(DEBUG)
    Serial.println("[Main] System is running normally.");
    #endif
}

// สร้าง Mock JSON และส่งแจ้งเตือน
void sendMockSensorsData() {

  // ArduinoJson v6
  StaticJsonDocument<1024> doc;

  JsonArray sensors = doc.createNestedArray("sensors");

  for (int i = 0; i < 7; i++) {

    JsonObject obj = sensors.createNestedObject();

    obj["id"] = i + 1;
    obj["online"] = (i != 2);
    obj["switch"] = (i % 2 == 0);
    obj["uptime"] = (i == 2) ? 0 : millis() / 1000UL;
  }

  doc["testRun"] = true;
  doc["offline_count"] = 1;
  doc["ac_loss"] = false;
  doc["alarm_count"] = 0;
  doc["mac"] = WiFi.macAddress();
  doc["ch"] = WiFi.channel();

  String jsonString;
  jsonString.reserve(measureJson(doc) + 1);
  serializeJson(doc, jsonString);

  // ส่ง JSON ผ่าน Notifying
  bool result = notifier.send(
    jsonString,
    wifiManager._config
  );

  #if defined(DEBUG)
  Serial.println("[MOCK] Sending JSON:");
  Serial.println(jsonString);
  Serial.println(result ? "[MOCK] SEND OK" : "[MOCK] SEND FAILED");
  Serial.printf("[HEAP] Free: %u bytes\n", ESP.getFreeHeap());
  #endif
}


void receiveData() {
    #if defined(debug)
        while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n') {
        // ได้ข้อมูลครบแล้ว
        Serial.println(inputBuffer);
        inputBuffer = "";  // Clear buffer
        } else {
        inputBuffer += c;
        }
    }
    #else
    while (Serial.available()) {
        char c = Serial.read();
        
        if (c == '\n') {
        // ได้ข้อมูลครบแล้ว
        processJson(inputBuffer);
        inputBuffer = "";  // Clear buffer
        } else {
        inputBuffer += c;
        }
    }
    #endif
}

// 1. ส่งค่าแบบอ้างอิง (&) ลดการสำเนาข้อความในแรม
void processJson(const String& jsonString) {   
  // 2. ใช้ JsonDocument ตัวใหม่ (ArduinoJson V7) เบาและเร็วกว่า StaticJsonDocument ตัวเก่ามาก
  JsonDocument doc;   
  #if defined(DEBUG)
    Serial.println(jsonString);
  #endif  
  DeserializationError error = deserializeJson(doc, jsonString);  
  if (error) {    
    LED_INTERVAL = 80;
    #if defined(DEBUG)
      Serial.println("JSON parse failed!"); // ใช้ F() มาร์โครเพื่อประหยัดแรม RAM
    #endif
    return;
  }
  LED_INTERVAL = 1000;
  
  #if defined(DEBUG)
    Serial.println(F("📨 Data received!"));
  #endif
  // 3. ดึงค่ามาเก็บในตัวแปรครั้งเดียว ลดการเข้าถึง Object ใน JSON ซ้ำซ้อน (ทำงานเร็วขึ้นชัดเจน)
  bool ac_loss = (bool)doc["ac_loss"];
  int alarm_count = (int)doc["alarm_count"];
  int offline_count = (int)doc["offline_count"];
  
  if(alarm_count > 0 || offline_count > 0 || scheduled_time == true || ac_loss == true){
    if(notifier.send(jsonString, wifiManager._config)){
      scheduled_time = false;
    }
  }
  
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastNotifyTime >= NOTIFY_INTERVAL) {
    lastNotifyTime = currentMillis;
    timeClient.update();
    scheduled_time = false;
    #if defined(DEBUG)
    if (WiFi.status() == WL_CONNECTED) {
      // sendMockSensorsData();
    }
    #endif
  }

  // ตรวจสอบว่าเวลาผ่านไปจนถึงกำหนดหรือยัง
if (currentMillis - lastLedTime >= LED_INTERVAL) {
  // บันทึกเวลาล่าสุดที่ไฟเปลี่ยนสถานะ
  lastLedTime = currentMillis;  
  // สลับสถานะตัวแปรไฟกระพริบ (จากติดเป็นดับ / จากดับเป็นติด)
  LED_BLANKING = !LED_BLANKING;  
  // สั่งให้ขา LED ทำงานตามสถานะใหม่
  digitalWrite(LED_PIN, LED_BLANKING);
}


  int currentHour = timeClient.getHours();
  if (currentHour != lastNotifiedHour) {
    if ((currentHour % NOTIFICATION_INTERVAL_HOURS) == 0) {      
      // *** ใส่โค้ดแจ้งเตือนของคุณที่นี่ ***      
      scheduled_time = true;        
      lastNotifiedHour = currentHour;
      sendMockSensorsData();
    }
  }

  yield();
}

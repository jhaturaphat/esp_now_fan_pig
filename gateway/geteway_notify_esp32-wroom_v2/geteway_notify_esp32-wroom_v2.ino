// 1 = ntfy, 2 = telegram, 3 = discord
// #include <HardwareSerial.h>
#include "ConfigManager.h"
#include <WiFi.h>
// #include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "Notifier.h"
#include "PinConfig.h"


// ตั้งค่า NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 7 * 3600, 60000); // UTC+7 สำหรับประเทศไทย
// ตัวแปรสำหรับเก็บชั่วโมงล่าสุดที่มีการแจ้งเตือน
int lastNotifiedHour = -1;
unsigned int NOTIFICATION_INTERVAL_HOURS = 2; 
// --- ส่วนเพิ่มเติมสำหรับ Non-blocking ---
unsigned long previousMillisUpdate = 0;
// กำหนดช่วงเวลาที่จะอัปเดตเวลาจาก NTP (ทุก 1 นาที = 60000 มิลลิวินาที)
const long intervalUpdate = 60000; 
// --------------------------------------------------------------
// LED Status
int ledState = LOW;             // ledState used to set the LED
long previousMillisLED = 0;        // will store last time LED was updated
long intervalLED = 500;  

// ----------------------------------------------------
ConfigManager configManager;
ConfigWiFi configWiFi;
ConfigNotify configNotify; // การตั้งค่าเลือกการแจ้งเตือน

// สร้าง object
Notifier notifier;

String inputBuffer = "";

void receiveData() {
  while (Serial2.available()) {
    char c = Serial2.read();
    
    if (c == '\n') {
      // ได้ข้อมูลครบแล้ว
      processJson(inputBuffer);
      inputBuffer = "";  // Clear buffer
    } else {
      inputBuffer += c;
    }
  }
}

void processJson(const String& jsonString) { // 1. ส่งค่าแบบอ้างอิง (&) ลดการสำเนาข้อความในแรม
  digitalWrite(TEST_PIN_SERIAL, HIGH);
  
  // 2. ใช้ JsonDocument ตัวใหม่ (ArduinoJson V7) เบาและเร็วกว่า StaticJsonDocument ตัวเก่ามาก
  JsonDocument doc; 
  
  #if defined(DEBUG)
    Serial.println(jsonString);
  #endif
  
  DeserializationError error = deserializeJson(doc, jsonString);
  
  if (error) {
    digitalWrite(RELAY1_PIN, HIGH);
    digitalWrite(RELAY2_PIN, HIGH);
    #if defined(DEBUG)
      Serial.println(F("JSON parse failed!")); // ใช้ F() มาร์โครเพื่อประหยัดแรม RAM
    #endif
    return;
  }
  
  #if defined(DEBUG)
    Serial.println(F("📨 Data received!"));
  #endif

  notifier.sendAll(jsonString);

  // 3. ดึงค่ามาเก็บในตัวแปรครั้งเดียว ลดการเข้าถึง Object ใน JSON ซ้ำซ้อน (ทำงานเร็วขึ้นชัดเจน)
  int alarm_count = doc["alarm_count"];
  int offline_count = doc["offline_count"];
  bool is_siren_disabled = dsiable_siren();

  #if defined(DEBUG)
    Serial.print(F("***Alert count = "));
    Serial.print(alarm_count);
    Serial.print(F(" | Offline count = "));
    Serial.println(offline_count);
    Serial.println(is_siren_disabled ? F("🔕") : F("🔔"));
  #endif

  // 4. ยุบและจัดกลุ่มเงื่อนไข Logic (ผลลัพธ์เหมือนเดิม 100% แต่สั้นลงและประมวลผลไวขึ้น)
  if ((alarm_count > 0 || offline_count > 0) && !is_siren_disabled) {
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    #if defined(DEBUG)
      Serial.println(F("🔔🔔"));
    #endif
  } else {
    // บล็อกนี้จะทำงานเมื่อ: ไม่มี alarm/offline เลย หรือ มีแต่โดนปิดไซเรนอยู่ (dsiable_siren เป็น true)
    digitalWrite(RELAY1_PIN, HIGH);
    digitalWrite(RELAY2_PIN, HIGH);
    #if defined(DEBUG)
      if (alarm_count > 0 || offline_count > 0) Serial.println(F("🔕🔕"));
    #endif
  }
}


void setup() {
  #if defined(DEBUG) 
  Serial.begin(115200);
  #endif
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
   
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  // pinMode(CONFIG_PIN, INPUT_PULLUP);  
  pinMode(TEST_PIN_SERIAL, OUTPUT);
  // สำหรับ ESP32 PCL
  // pinMode(TEST_PIN, INPUT);
  // pinMode(DISABLE_SIREN, INPUT);
  // สำหรับ ESP32 Devkit
  pinMode(TEST_PIN, INPUT_PULLUP);
  pinMode(DISABLE_SIREN, INPUT_PULLUP);
  
  if(!configManager.begin(CONFIG_PIN)){
    // ❌ ถ้าโหลดไม่สำเร็จ
    #if defined(DEBUG)    
    Serial.println("Failed to load notify config!");
    #endif
    return;
  }
  #if defined(DEBUG)
  Serial.println("เมื่อโค้ดมาถึงตรงนี้ หมายความว่า ESP ได้เข้าสู่ Normal Operation Mode แล้ว");
  #endif
  
  configWiFi = configManager.getConfigWiFi();
  #if defined(DEBUG)
  // ใช้ Serial.print() เพื่อดูค่า
  Serial.println("--- Wifi Values ---");
  Serial.print("SSID: ");
  Serial.println(configWiFi.ssid); // พิมพ์ char*
  Serial.print("PASSWORD: ");
  Serial.println(configWiFi.password); // พิมพ์ char*
  Serial.println("---------------------------");
  #endif
  
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(configWiFi.ssid, configWiFi.password);
  while (WiFi.status() != WL_CONNECTED) {
    // LED_STATUS
    digitalWrite(LED_STATUS, LOW);
    delay(50);
    digitalWrite(LED_STATUS, HIGH);
    delay(50);
    #if defined(DEBUG)
    Serial.println("Connecting to WiFi...");
    #endif
  }
  #if defined(DEBUG)
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  #endif
  
  configNotify = configManager.getConfigNotify();    
  #if defined(DEBUG)      
  Serial.println("---------------------------");     
    Serial.printf("URL: %s\n", (configNotify.url != nullptr) ? configNotify.url : "(NULL)");    
    Serial.printf("Channel: %s\n", (configNotify.channel != nullptr) ? configNotify.channel : "(NULL)");
    Serial.printf("Type: %d\n", configNotify.type);
    Serial.printf("Interval: %d\n", configNotify.interval);
    Serial.println("---------------------------");     
  #endif
  NOTIFICATION_INTERVAL_HOURS = configNotify.interval;
  notifier.setLocation(configNotify.location);
  // ตั้งค่า Notifier
  // 1 = ntfy, 2 = telegram, 3 = discord
  switch(configNotify.type){
    case 1:
      notifier.setupNtfy(configNotify.url);
      notifier.enableNtfy(true);
    break;
    case 2:
      notifier.setupTelegram(configNotify.url, configNotify.channel);
      notifier.enableTelegram(true);
    break;
    case 3:
      notifier.setupDiscord(configNotify.url);
      notifier.enableDiscord(true);
    break;
  }


  // เริ่มต้น NTP Client
  timeClient.begin();
  digitalWrite(TEST_PIN_SERIAL, LOW);

}

void test_gw(){
  // TEST_PIN
  if((digitalRead(TEST_PIN) == LOW)) {
    digitalWrite(TEST_PIN_SERIAL, LOW); 
    delay(100);  
  }else{
    digitalWrite(TEST_PIN_SERIAL, HIGH);   
  }
}

bool dsiable_siren(){  
  if(digitalRead(DISABLE_SIREN) == LOW){    
    return true;
  }else{
    return false;
  }
}

void loop() { 
  test_gw(); 
  unsigned long currentMillis = millis();
  
  // ตรวจสอบเวลาด้วย millis() ว่าถึงรอบต้องอัปเดตเวลา NTP หรือยัง
  if (currentMillis - previousMillisUpdate >= intervalUpdate) {
    previousMillisUpdate = currentMillis;
    timeClient.update();
    digitalWrite(TEST_PIN_SERIAL, HIGH); 
    // digitalWrite(RELAY1_PIN, HIGH);
    // digitalWrite(RELAY2_PIN, HIGH);
  }
  int currentHour = timeClient.getHours();
  if (currentHour != lastNotifiedHour) {
    if ((currentHour % NOTIFICATION_INTERVAL_HOURS) == 0) {      
      // *** ใส่โค้ดแจ้งเตือนของคุณที่นี่ ***      
      digitalWrite(TEST_PIN_SERIAL, LOW);     
      lastNotifiedHour = currentHour;
    }
  }

// LED Blink
  unsigned long currentMillisLED = millis();
  if(currentMillisLED - previousMillisLED > intervalLED) {
     previousMillisLED = currentMillisLED;
     if (ledState == LOW){
      ledState = HIGH;
     }else{
      ledState = LOW;
     }     
    digitalWrite(LED_STATUS, ledState);  
  }

  delay(100);
  receiveData();
}
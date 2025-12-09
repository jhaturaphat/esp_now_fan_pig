// 1 = ntfy, 2 = telegram, 3 = discord

#include "ConfigManager.h"
#include <WiFi.h>
// #include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "Notifier.h"


#define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา

#define KID_BUG_PIN 34 //สำหรับป้องกันโปรแกรม

#define RELAY1_PIN 16  //out put
#define RELAY2_PIN 17  //out put
#define LED_STATUS 18  //out put
#define CONFIG_PIN 19  //input pulll up
#define TEST_PIN 23 //input pulll up
#define TEST_PIN_SERIAL 26 //input pulll up
#define DISABLE_SIREN 25  //input pulll up

// กำหนดค่าให้สลับกันกับ ตัวส่ง 
#define RXD2 33
#define TXD2 32


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

void processJson(String jsonString) {
  digitalWrite(TEST_PIN_SERIAL, HIGH);
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, jsonString);
  
  if (error) {
    digitalWrite(TEST_PIN_SERIAL, LOW);
    Serial.println("JSON parse failed!");
    return;
  }
  
  Serial.println("📨 Data received!");
  notifier.sendAll(jsonString);
  // ประมวลผลข้อมูล
  JsonArray sensors = doc["sensors"];
  for (JsonObject sensor : sensors) {
    int id = sensor["id"];
    bool online = sensor["online"];
    // ... ทำอะไรต่อตามต้องการ
  }
}

void setup() {
  #if defined(DEBUG) 
  Serial.begin(115200);
  #endif
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
   
  pinMode(KID_BUG_PIN, INPUT_PULLUP);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(TEST_PIN, INPUT);
  pinMode(TEST_PIN_SERIAL, OUTPUT);
  pinMode(DISABLE_SIREN, INPUT);
  
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
    delay(200);
    digitalWrite(LED_STATUS, HIGH);
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
    

  
  if(digitalRead(KID_BUG_PIN) != LOW){      
    #if defined(DEBUG)  
    Serial.print("ป้องกัน Code Protection ต่อขานี้ลงกร์าว");    
    #endif
    // return;
    ESP.restart();
  }

  // เริ่มต้น NTP Client
  timeClient.begin();

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

void loop() { 
  test_gw(); 
  unsigned long currentMillis = millis();
  
  // ตรวจสอบเวลาด้วย millis() ว่าถึงรอบต้องอัปเดตเวลา NTP หรือยัง
  if (currentMillis - previousMillisUpdate >= intervalUpdate) {
    previousMillisUpdate = currentMillis;
    timeClient.update();
  }
  int currentHour = timeClient.getHours();
  if (currentHour != lastNotifiedHour) {
    if (currentHour % NOTIFICATION_INTERVAL_HOURS == 0) {      
      // *** ใส่โค้ดแจ้งเตือนของคุณที่นี่ ***      
      digitalWrite(TEST_PIN_SERIAL, LOW);     
      lastNotifiedHour = currentHour;
    }
  }

  delay(100);
  receiveData();
}
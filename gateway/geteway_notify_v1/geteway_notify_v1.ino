// 1 = ntfy, 2 = telegram, 3 = discord

#include "ConfigManager.h"
#include <WiFi.h>
#include <ArduinoJson.h>


#define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา

#define KID_PIN 34 //สำหรับป้องกันโปรแกรม

#define RELAY1_PIN 16  //out put
#define RELAY2_PIN 17  //out put
#define LED_STATUS 18  //out put
#define CONFIG_PIN 19  //input pulll up
#define TEST_PIN 23 //input pulll up
#define DISABLE_SIREN 25  //input pulll up

const char* ssid = "kid_2.4GHz";
const char* password = "xx3xx3xx";

ConfigManager configManager;
ConfigWiFi configWiFi;
ConfigNotify configNotify;

void setup() {
  Serial.begin(115200);
   
  pinMode(KID_PIN, INPUT_PULLUP);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED_STATUS, OUTPUT);
  pinMode(TEST_PIN, INPUT);
  pinMode(DISABLE_SIREN, INPUT);

  configManager.begin(CONFIG_PIN);
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
  
  WiFi.begin(configWiFi.ssid, configWiFi.password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
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
  // ใช้ Serial.print() เพื่อดูค่า
  Serial.println("--- ConfigNotify Values ---");
  Serial.print("URL: ");
  Serial.println(configNotify.url); // พิมพ์ char*
  
  Serial.print("Channel: ");
  Serial.println(configNotify.channel); // พิมพ์ char*
  
  Serial.print("Type: ");
  Serial.println(configNotify.type); // พิมพ์ char*
  
  Serial.print("Interval: ");
  Serial.println(configNotify.interval); // พิมพ์ uint8_t
  Serial.println("---------------------------");
  #endif
  
  if(digitalRead(KID_PIN) != LOW){  
    #if defined(DEBUG)  
    Serial.print("ป้องกัน Code Protection ต่อขานี้ลงกร์าว");    
    #endif
    return;
  }

}

void loop() {
  delay(100);
}
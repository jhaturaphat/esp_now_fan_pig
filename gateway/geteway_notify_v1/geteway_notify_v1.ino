#include "ConfigManager.h"
#include <WiFi.h>
#include <ArduinoJson.h>


#define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา

#define RELAY1_PIN 16  //out put
#define RELAY2_PIN 17  //out put
#define LED_STATUS 18  //out put
#define CONFIG_PIN 19  //input pulll up
#define TEST_PIN 23 //input pulll up
#define DISABLE_SIREN 25  //input pulll up

const char* ssid = "kid_2.4GHz";
const char* password = "xx3xx3xx";

ConfigManager configManager;

void setup() {
  Serial.begin(115200);

  configManager.begin(CONFIG_PIN);
  #if defined(DEBUG)
  Serial.println("เมื่อโค้ดมาถึงตรงนี้ หมายความว่า ESP ได้เข้าสู่ Normal Operation Mode แล้ว");
  #endif
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  delay(100);
}
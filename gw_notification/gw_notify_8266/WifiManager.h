
#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H


#include <ESP8266WiFi.h>
#include <ArduinoJson.h> 
#include <ESPAsyncWebServer.h>
#include <ESPAsyncTCP.h>
// #include <DNSServer.h>
// #include <ESP8266mDNS.h> // เพิ่มไลบรารี mDNS
#include <LittleFS.h>
#include "debug.h"



#define LED_PIN 2

// โครงสร้างข้อมูลสำหรับเก็บบันทึกแจ้งเตือน
struct NotifyConfig {
  String ntfy_topic;
  String telegram_token;
  String telegram_channel;
  String discord_webhook;
  int type;
  int interval;
  String location;
};


class WifiManager {
  
  public:

    NotifyConfig _config;
    const char* _configFile = "/notifying.json";
    // กำหนดให้ส่งตัวแปร Pointer ของ AsyncWebserver เข้ามาจัดการร่วมกัน
    WifiManager(AsyncWebServer* server);
     // ฟังก์ชันเริ่มต้นระบบ (อ่านค่า และ พยายามเชื่อมต่อ)
    void begin(bool mode);  //true = เข้าสู่โหมดตั้งค่า 
    // รีเซ็ตล้างค่า Wi-Fi ที่เคยบันทึกไว้ (เผื่ออยากสั่งล้างค่าเพื่อตั้งใหม่)
    void resetSettings();

    bool loadConfigNotify();
    bool saveConfigNotify();


  private:
    AsyncWebServer* _server;
    // DNSServer _dnsServer;
    // const byte _DNS_PORT = 53;
    bool read_config = false;
    String _ssid = "";
    String _password = "";
    const unsigned long _timeoutMs = 15000;         // รอเชื่อมต่อ Wi-Fi บ้าน 15 วินาที

    // ตัวแปรสำหรับจัดการการกระพริบ LED แบบ non-blocking
    unsigned long previousMillis = 0;
    const long interval = 100; // ช่วงเวลา 300 มิลลิวินาที
    bool ledState = LOW;

    bool loadConfigWifi();                              // โหลดค่าจาก LittleFS
    bool saveConfigWifi(String ssid, String password);  // บันทึกค่าลง LittleFS
    void startConfigMode();                         // เปิดโหมด Config (AP)
    void setupConfigServer();                       // ตั้งค่าหน้าเว็บสำหรับรับค่าจากมือถือ 
    String processor(const String& var);
    String chipID();
};

#endif // WIFI_MANAGER_H
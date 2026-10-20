
#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H


#include <ESP8266WiFi.h>
#include <ArduinoJson.h> 
#include <ESPAsyncWebServer.h>
#include <ESPAsyncTCP.h>
#include <LittleFS.h>


#define DEBUG

class WifiManager {
  
  public:
    // กำหนดให้ส่งตัวแปร Pointer ของ AsyncWebserver เข้ามาจัดการร่วมกัน
    WifiManager(AsyncWebServer* server);
     // ฟังก์ชันเริ่มต้นระบบ (อ่านค่า และ พยายามเชื่อมต่อ)
    void begin();
    // รีเซ็ตล้างค่า Wi-Fi ที่เคยบันทึกไว้ (เผื่ออยากสั่งล้างค่าเพื่อตั้งใหม่)
    void resetSettings();


  private:
    AsyncWebServer* _server;
    String _ssid = "";
    String _password = "";
    const unsigned long _timeoutMs = 15000;         // รอเชื่อมต่อ Wi-Fi บ้าน 15 วินาที

    bool loadConfig();                              // โหลดค่าจาก LittleFS
    bool saveConfig(String ssid, String password);  // บันทึกค่าลง LittleFS
    void startConfigMode();                         // เปิดโหมด Config (AP)
    void setupConfigServer();                       // ตั้งค่าหน้าเว็บสำหรับรับค่าจากมือถือ 
    String processor(const String& var);
    String chipID();
};

#endif // WIFI_MANAGER_H
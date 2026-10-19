#include "WifiManager.h"
#include "html.h"

WifiManager::WifiManager(AsyncWebServer* server) {
  _server = server;
}

void WifiManager::begin(){
  // เริ่มต้นใช้งานระบบไฟล์ LittleFS
  if(!LittleFS.begin()){
    #if defined(DEBUG)
    Serial.println("[FS] LittleFS Mount Failed. Formating...");
    #endif
    LittleFS.format();
    LittleFS.begin();
  }
  // ถ้านำค่า Wi-Fi ที่บันทึกไว้มาโหลดสำเร็จ ให้ลองเชื่อมต่อดู
  if(loadConfig() && _ssid.length() > 0) {
    #if defined(DEBUG)
    Serial.println("\n[WiFi] Found saved credentials. Connecting to: " + _ssid);
    #endif
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid.c_str(), _password.c_str());

    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < _timeoutMs) {
      delay(500);
      Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
      #if defined(DEBUG)
      Serial.println("\n[WiFi] Connected successfully!");
      Serial.print("[WiFi] IP Address: ");
      Serial.println(WiFi.localIP());
      #endif
      return; // เชื่อมต่อสำเร็จ จบการทำงานฟังก์ชันนี้
    }
    #if defined(DEBUG)
    Serial.println("\n[WiFi] Connection failed with saved credentials.");
    #endif
  }else{
    #if defined(DEBUG)
    Serial.println("[WiFi] No saved Wi-Fi configuration found.");
    #endif
  }

  // ถ้าไม่มีข้อมูล หรือต่อไม่ติด -> เข้าโหมด Config ให้มือถือเข้ามาตั้งค่า
    startConfigMode();
}


void WifiManager::startConfigMode() {
    #if defined(DEBUG)
    Serial.println("[WiFi] Entering Configuration Mode...");
    #endif
    WiFi.mode(WIFI_AP);
    
    // ตั้งชื่อสัญญาณ Wi-Fi ที่ ESP-01 ปล่อยออกมา (ไม่มีรหัสผ่าน)
    const char* apSsid = "ESP01_Config";
    WiFi.softAP(apSsid);
    #if defined(DEBUG)
    Serial.print("[WiFi] Connect to: ");
    Serial.println(apSsid);
    Serial.print("[WiFi] Open browser and go to IP: ");
    Serial.println(WiFi.softAPIP());
    #endif
    // เรียกฟังก์ชันเปิดหน้าเว็บสำหรับการตั้งค่า
    setupConfigServer();
}

bool WifiManager::loadConfig() {
    if (!LittleFS.exists("/wifi.txt")) return false;

    File f = LittleFS.open("/wifi.txt", "r");
    if (!f) return false;

    _ssid = f.readStringUntil('\n');
    _password = f.readStringUntil('\n');
    
    // ลบช่องว่างหรืออักขระพิเศษส่วนเกินออก
    _ssid.trim();
    _password.trim();
    
    f.close();
    return true;
}

void WifiManager::setupConfigServer() {  
    // 1. หน้าแรก แสดงฟอร์มกรอก Wi-Fi
    _server->on("/", HTTP_GET, [this](AsyncWebServerRequest *request){             
        request->send_P(200, "text/html", config_html, [this](const String& var){
          return this->processor(var);
        });
    });

    // 2. Path รับข้อมูล POST จากฟอร์มเมื่อกดปุ่ม "บันทึก"
    _server->on("/save", HTTP_POST, [this](AsyncWebServerRequest *request){    
        String reqSsid = "";
        String reqPass = ""; 
        if (request->hasArg("ssid")) {
            reqSsid = request->arg("ssid");
        }
        if (request->hasArg("password")) {
            reqPass = request->arg("password");
        }

        if (reqSsid.length() > 0) {
            // บันทึกค่าลงหน่วยความจำ
            saveConfig(reqSsid, reqPass);
            
            // ส่งข้อความบอกมือถือว่ากำลังรีสตาร์ท
            request->send(200, "text/html", "<h3>Save Success! ESP-01 is restarting to connect to your Wi-Fi...</h3>");
            
            // รอสักครู่แล้วสั่งรีสตาร์ทบอร์ดเพื่อเริ่มทำงานใหม่
            delay(2000);
            ESP.restart();
        } else {
            request->send(400, "text/plain", "SSID cannot be empty.");
        }
    });

    _server->begin();
}

String WifiManager::processor(const String& var){
  String ssid = "";
  String pass = "";
  if(LittleFS.exists("/wifi.txt")){
      File f = LittleFS.open("/wifi.txt", "r");
      if (f){
        ssid = f.readStringUntil('\n');
        pass = f.readStringUntil('\n');    
        if(var == "SSID"){
          return ssid;
        }else if(var == "PASS"){
          return pass;
        }
      }else{
        return "";
      }
    }  
    return "";
}


bool WifiManager::saveConfig(String ssid, String pass) {
    File f = LittleFS.open("/wifi.txt", "w");
    if (!f) return false;

    f.println(ssid);
    f.println(pass);
    f.close();
    #if defined(DEBUG)
    Serial.println("[FS] Wi-Fi configuration saved to LittleFS.");
    #endif
    return true;
}

void WifiManager::resetSettings() {
    if (LittleFS.exists("/wifi.txt")) {
        LittleFS.remove("/wifi.txt");
        #if defined(DEBUG)
        Serial.println("[FS] Reset completed. WiFi settings deleted.");
        #endif
    }
}



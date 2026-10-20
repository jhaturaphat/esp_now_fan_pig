#include "WifiManager.h"
#include "html.h"
#include "style.h"
#include "script.h"

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
    const String apName = "ESP_Config_" + chipID(); 
    WiFi.softAP(apName.c_str());
    #if defined(DEBUG)
    Serial.print("[WiFi] Connect to: ");
    Serial.println(apName);
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

    // 2. โหลด Style css
    _server->on("/styles.css",HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200,"text/css", style);
    });
    // 3. โหลด Javascript
    _server->on("/script.js",HTTP_GET, [](AsyncWebServerRequest *request){
      request->send_P(200,"text/javascript", script);
    });

    // 4. Path รับข้อมูล POST จากฟอร์มเมื่อกดปุ่ม "บันทึก"
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

      // 5. Scan network
      _server->on("/getWifiList", HTTP_GET, [](AsyncWebServerRequest *request) {
      // ดึงจำนวนเครือข่ายที่ระบบสแกนเจอในรอบล่าสุด (ไม่ต้องเริ่มสแกนใหม่)
        int n = WiFi.scanComplete(); 

        JsonDocument doc; // สำหรับ ArduinoJson v7 (ถ้า v6 ให้ใช้ DynamicJsonDocument doc(2048);)
        JsonArray root = doc.to<JsonArray>();

        // กรณีสแกนเสร็จเรียบร้อยและเจอสัญญาณ
        if (n > 0) {
          for (int i = 0; i < n; ++i) {
            JsonObject network = root.add<JsonObject>(); 
            network["id"] = i + 1;
            network["ssid"] = WiFi.SSID(i);
            network["rssi"] = WiFi.RSSI(i);
            network["encryption"] = (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? "open" : "secured";
          }
        }

        String jsonResponse;
        serializeJson(doc, jsonResponse);

        // ส่งข้อมูล JSON กลับไปทันที (ไม่มีการบล็อกการทำงาน)
        request->send(200, "application/json", jsonResponse);

        // สั่งให้ระบบเริ่มสแกนรอบถัดไปแบบอัตโนมัติ (Async) เพื่อเตรียมข้อมูลสำหรับครั้งหน้า
        // พารามิเตอร์แรก 'true' หมายถึงให้ทำงานแบบ Async / เบื้องหลัง
        WiFi.scanNetworks(true, false);
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

String WifiManager::chipID(){
  String chipIDStr = String(ESP.getChipId(), HEX);
  chipIDStr.toUpperCase();
  return chipIDStr;
}

void WifiManager::resetSettings() {
    if (LittleFS.exists("/wifi.txt")) {
        LittleFS.remove("/wifi.txt");
        #if defined(DEBUG)
        Serial.println("[FS] Reset completed. WiFi settings deleted.");
        #endif
    }
}



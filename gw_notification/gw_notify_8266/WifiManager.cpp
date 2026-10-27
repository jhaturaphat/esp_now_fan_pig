#include "WifiManager.h"
#include "html.h"
#include "style.h"
#include "script.h"

WifiManager::WifiManager(AsyncWebServer* server) {
  _server = server;
}

void WifiManager::begin(bool mode){
  // เริ่มต้นใช้งานระบบไฟล์ LittleFS
  if(!LittleFS.begin()){
    #if defined(DEBUG)
    Serial.println("[FS] LittleFS Mount Failed. Formating...");
    #endif
    LittleFS.format();
    LittleFS.begin();
  }
  // ถ้านำค่า Wi-Fi ที่บันทึกไว้มาโหลดสำเร็จ ให้ลองเชื่อมต่อดู
  if(loadConfigWifi() && _ssid.length() > 0) {
    #if defined(DEBUG)
    Serial.println("\n[WiFi] Found saved credentials. Connecting to: " + _ssid);
    #endif
    
    if(!mode){
      //เข้าโหมด Config ให้มือถือเข้ามาตั้งค่า
      startConfigMode();
    }
    // หากไม่ใช่โหมดการตั้งค่าก็ทำงานในโหมด Client Station
    WiFi.mode(WIFI_STA);
    WiFi.begin(_ssid.c_str(), _password.c_str());
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startTime < _timeoutMs) {
      delay(500);
      #if defined(DEBUG)
      Serial.print(".");
      #endif
    }

    if (WiFi.status() == WL_CONNECTED) {
      #if defined(DEBUG)
      Serial.println("\n[WiFi] Connected successfully!");
      Serial.print("[WiFi] IP Address: ");
      Serial.println(WiFi.localIP());
      #endif
      loadConfigNotify();  // หากเชื่อมต่สำเร็จให้โหลด ConfigFileNotify
      return; // เชื่อมต่อสำเร็จ จบการทำงานฟังก์ชันนี้
    }
    #if defined(DEBUG)
    Serial.println("\n[WiFi] Connection failed with saved credentials.");
    #endif
  }else{    
    #if defined(DEBUG)
    Serial.println("[WiFi] No saved Wi-Fi configuration found.");
    #endif
    startConfigMode();
  }

  
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

    // เริ่ม DNSServer และตั้งค่า Captive Portal
    /*_dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    _dnsServer.start(_DNS_PORT, "*", WiFi.softAPIP());
    // เริ่ม mDNS
    if (MDNS.begin("่esp8266")) {
        MDNS.addService("http", "tcp", 80); // ประกาศว่าเป็นเซิร์ฟเวอร์ HTTP บนพอร์ต 80
    }*/

    // เรียกฟังก์ชันเปิดหน้าเว็บสำหรับการตั้งค่า
    setupConfigServer();
}

bool WifiManager::loadConfigWifi() {
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

bool WifiManager::loadConfigNotify(){
  if (!LittleFS.exists(_configFile)) {
    Serial.println("[WifiManager] Config file not found. Using defaults.");
    return false;
  }

  File configFile = LittleFS.open(_configFile, "r");
  if (!configFile) return false;

  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, configFile);
  configFile.close();

  if (error) {
    #if defined(DEBUG)
    Serial.println("[WifiManager] Failed to parse config JSON");
    #endif
    return false;
  }
  _config.ntfy_topic       = doc["ntfy_topic"] | "";
  _config.telegram_token   = doc["telegram_token"] | "";
  _config.telegram_channel = doc["telegram_channel"] | "";
  _config.discord_webhook  = doc["discord_webhook"] | "";
  _config.type             = doc["type"] | 1;
  _config.interval         = doc["interval"] | 1;
  _config.location         = doc["location"] | "โปรดระบุ";

  #if defined(DEBUG)
  Serial.println("[WifiManager] Config loaded successfully!");
  #endif
  return true;
}

// ฟังก์ชันบันทึกคอนฟิกลงไฟล์ JSON
bool WifiManager::saveConfigNotify() {
  File configFile = LittleFS.open(_configFile, "w");
  if (!configFile) return false;

  StaticJsonDocument<512> doc;
  doc["ntfy_topic"]       = _config.ntfy_topic;
  doc["telegram_token"]   = _config.telegram_token;
  doc["telegram_channel"] = _config.telegram_channel;
  doc["discord_webhook"]   = _config.discord_webhook;
  doc["type"]             = _config.type;
  doc["interval"]         = _config.interval;
  doc["location"]         = _config.location;

  if (serializeJson(doc, configFile) == 0) {
    configFile.close();
    return false;
  }

  configFile.close();
  Serial.println("[WifiManager] Config saved to LittleFS!");
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
    _server->on("/save_wifi", HTTP_POST, [](AsyncWebServerRequest *request){
      // ส่ง response ทันที
        request->send(200, "application/json", "{\"status\":\"processing\"}");
      }, NULL, [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
      
      
      #if defined(DEBUG)
      // แปลงข้อมูลเป็น String
      String jsonString = String((char*)data).substring(0, len);
      Serial.println("Received JSON: " + jsonString);
      #endif

      // Parse JSON
      StaticJsonDocument<64> doc;
      DeserializationError error = deserializeJson(doc, data);

      if (error) {
        Serial.println("JSON parse failed");
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
        return;
      }
      String ssid = doc["ssid"];
      String password = doc["password"];
      if(saveConfigWifi(ssid, password)){
        request->send(200, "application/json", "{\"status\":\"success\",\"message\":\"WiFi config saved\"}");
      }else{
        request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to open file\"}");
      }
      
    });

    _server->on("/save_notify", HTTP_POST, 
      [](AsyncWebServerRequest *request) {}, 
      NULL, 
      [this](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      
      // วนลูปเก็บข้อความจากบัฟเฟอร์อย่างปลอดภัย ป้องกันบอร์ด Crash
      #if defined(DEBUG)
      // แปลงข้อมูลเป็น String
      String jsonString = String((char*)data).substring(0, len);
      Serial.println("Received JSON: " + jsonString);
      #endif

      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, data);

      if (error) {
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
        return;
      }

      // อัปเดตค่าเข้าสู่ตัวแปร Private Config ของคลาส
      _config.ntfy_topic       = doc["ntfy_topic"] | "";
      _config.telegram_token   = doc["telegram_token"] | "";
      _config.telegram_channel = doc["telegram_channel"] | "";
      _config.discord_webhook   = doc["discord_webhook"] | "";
      _config.type             = doc["type"] | 1;
      _config.interval         = doc["interval"] | 1;
      _config.location         = doc["location"] | "โปรดระบุ";

      // บันทึกลง Flash
      if (saveConfigNotify()) {
        request->send(200, "application/json", "{\"status\":\"success\",\"message\":\"Configuration saved\"}");
      } else {
        request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to write file\"}");
      }
    }
  );

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

    // 6. Restart device 
    _server->on("/reset", HTTP_GET,[](AsyncWebServerRequest *request){
      ESP.restart();
    });

    // สำหรับ iOS Captive Portal
    _server->on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request) { 
        // request->send(200, "text/html", index_html, [](const String& var) { return String(); });
        request->redirect("http://" + WiFi.softAPIP().toString());
    });

    // หาก url ไม่ตรงกับ route
    /*_server->onNotFound([this](AsyncWebServerRequest *request) {
        request->redirect("http://" + WiFi.softAPIP().toString());
    });*/


    _server->begin();
    
    
    // Loop for blinking LED during config mode
    while(true) {
        yield();
        unsigned long currentMillis = millis();
        // ตรวจสอบและกระพริบ LED ทุก 300 มิลลิวินาที
        if (currentMillis - previousMillis >= interval) {
          previousMillis = currentMillis;  
          ledState = !ledState;          
          digitalWrite(LED_PIN, ledState);            
        }
        // ประมวลผลคำขอ DNS และ mDNS
        // _dnsServer.processNextRequest();
        // MDNS.update();
    }
}

String WifiManager::processor(const String& var){
  String ssid = "";
  String pass = "";
  if(LittleFS.exists("/wifi.txt") && (var == "SSID" || var == "PASS")){
    File f = LittleFS.open("/wifi.txt", "r");
    if (f){
      ssid = f.readStringUntil('\n');
      pass = f.readStringUntil('\n');    
      if(var == "SSID"){
        return ssid;
      }else if(var == "PASS"){
        return pass;
      }
    }  
    return "";
  }else if(LittleFS.exists(_configFile)){
    if(!read_config){          
      read_config = loadConfigNotify();   
    }      
    if(var == "LOCALTION" && read_config){
      return _config.location;
    }else if(var == "NTFY_TOPIC" && read_config){
      return _config.ntfy_topic;
    }else if(var == "TEL_TOKEN" && read_config){
      return _config.telegram_token;
    }else if(var == "TEL_CH" && read_config){
      return _config.telegram_channel;
    }else if(var == "DISC_URL" && read_config){
      return _config.discord_webhook;
    }
  }else{
    return "";
  }
  return "";
}    



bool WifiManager::saveConfigWifi(String ssid, String pass) {
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



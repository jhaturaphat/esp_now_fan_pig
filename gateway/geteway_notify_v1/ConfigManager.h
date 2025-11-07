#include <WiFi.h>
#include <FS.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>

#define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา

struct ConfigWiFi {
  char* ssid;
  char* password;
};

struct ConfigNotify {  
  char* url;
  char* channel;
  uint8_t type;
  uint8_t interval = 4;
};

class ConfigManager {
  private: 
  // กำหนดขาใช้งาน
  const int LED_STATUS = 18;

  // web server object
  AsyncWebServer server;
  DNSServer dnsServer;
  const byte DNS_PORT = 53;

  ConfigWiFi configWiFi;
  // ParamNtfy paramNtfy;
  ConfigNotify configNotify;

  // ตัวแปรสำหรับจัดการการกระพริบ LED แบบ non-blocking
  unsigned long previousMillis = 0;
  const long interval = 300; // ช่วงเวลา 300 มิลลิวินาที
  bool ledState = LOW;

  String chipID() {
    uint64_t chipMac = ESP.getEfuseMac();  // ได้ MAC address เป็นเลข 64-bit
    // ใช้เฉพาะ 6 ตัวอักษรท้าย (3 ไบต์สุดท้าย) เพื่อให้เหมือน ESP8266 chipId
    uint32_t chipId32 = (uint32_t)(chipMac >> 16); 
    String chipIDStr = String(chipId32, HEX);
    chipIDStr.toUpperCase();
    return chipIDStr;
  }

  bool loadConfigWiFi(){
    File file = LittleFS.open("/wifi_config.json", "r");
    if (!file) {
      #if defined(DEBUG)
      Serial.println("Failed to open config file");
      #endif
      return false;
    }

    StaticJsonDocument<64> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
      #if defined(DEBUG)
      Serial.println("Failed to parse config file");
      #endif
      return false;
    }

    // จัดการหน่วยความจำให้เหมาะกับ char*
    const char* ssid = doc["ssid"];
    const char* password = doc["password"];

    // ปล่อยหน่วยความจำเก่า (ถ้ามี)
    if (configWiFi.ssid) free(configWiFi.ssid);
    if (configWiFi.password) free(configWiFi.password);
    
    // ทำ strdup เพื่อ copy string ไปยัง heap
    configWiFi.ssid = strdup(ssid);
    configWiFi.password = strdup(password);
    #if defined(DEBUG)
    Serial.printf("Loaded SSID: %s\n", configWiFi.ssid);
    Serial.printf("Loaded Password: %s\n", configWiFi.password);
    #endif
    return true;
  }

  bool loadConfigNotify(){    
    File file = LittleFS.open("/notify_config.json", "r");
    if(!file){
      #if defined(DEBUG)
      Serial.println("Failed to open config file");
      #endif
      return false;
    }

    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
      #if defined(DEBUG)
      Serial.println("Failed to parse config file");
      #endif
      return false;
    }

    // จัดการหน่วยความจำให้เหมาะกับ char*
    const char* url = doc["url"];
    const char* channel = doc["channel"];
    const uint8_t type = doc["type"];
    const uint8_t interval = doc["interval"];
    // ปล่อยหน่วยความจำเก่า (ถ้ามี)
    // if (configNotify.url) free(configNotify.url);
    // if (configNotify.channel) free(configNotify.channel);
    // if (configNotify.type) free(configNotify.type);
  // ควรเปลี่ยนเป็น ปล่อยหน่วยความจำเก่า (ถ้ามี)
    if (configNotify.url != nullptr) {
      free(configNotify.url);
      configNotify.url = nullptr;
    }
    if (configNotify.url != nullptr) {
      free(configNotify.channel);
      configNotify.channel = nullptr;
    }

    // ทำ strdup เพื่อ copy string ไปยัง heap
    configNotify.url = strdup(url);
    configNotify.channel = strdup(channel);
    configNotify.type = type;
    configNotify.interval = interval;

    #if defined(DEBUG)
    Serial.printf("Loaded URL: %s\n", configNotify.url);
    Serial.printf("Loaded CHANNEL: %s\n", configNotify.channel);
    Serial.printf("Loaded TYPE: %d\n", configNotify.type);
    Serial.printf("Loaded INTERVAL: %d\n", configNotify.interval);
    #endif
    return true;
  }

  // Start Web Server
  void enterConfigMode() {
    // Initialize LittleFS
   
    WiFi.setSleep(0);
    WiFi.mode(WIFI_AP);
    String apName = "🛜SETUP_GW-"+ chipID();
    WiFi.softAP(apName.c_str(),"", 1, 0, 4); // ช่อง 1, จำกัด 4 อุปกรณ์
    delay(50); //หน่วงเวลา 100 ms เพื่อให้ AP เริ่มทำงาน

    // เริ่ม DNSServer และตั้งค่า Captive Portal
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

    // เริ่ม mDNS
    if (MDNS.begin("่esp32")) {
        MDNS.addService("http", "tcp", 80); // ประกาศว่าเป็นเซิร์ฟเวอร์ HTTP บนพอร์ต 80            
    }else{
        #if defined(DEBUG)
        Serial.println("Error starting mDNS");
        #endif
    }

    server.on("/", HTTP_GET,[](AsyncWebServerRequest * request) {
      request->send(LittleFS, "/index.html", "text/html");
    });

    // Route to load style.css file
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(LittleFS, "/style.css", "text/css");
    });
    // Route to load script.js file
    server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request){
      request->send(LittleFS, "/script.js", "text/javascript");
    });

  // รับ JSON ผ่าน POST
  server.on("/save_wifi", HTTP_POST, [](AsyncWebServerRequest *request){
    // ส่ง response ทันที
      request->send(200, "application/json", "{\"status\":\"processing\"}");
    }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    
    
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

    // เปิดไฟล์เพื่อเขียน
    File file = LittleFS.open("/wifi_config.json", "w");
    if(!file){
      #if defined(DEBUG)
      Serial.println("Failed to open file for writing");
      #endif
      request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to open file\"}");
      return;
    }
    // เขียน JSON ลงไฟล์
    if (serializeJson(doc, file) == 0) {
      #if defined(DEBUG)
      Serial.println("Failed to write JSON to file");
      #endif
      file.close();
      request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to write file\"}");
      return;
    }

    #if defined(DEBUG)
    Serial.println("WiFi config saved successfully");
    #endif

    if (LittleFS.exists("/wifi_config.json")) {
      #if defined(DEBUG)
      Serial.println("✅ wifi_config.json exists");
      #endif
      file.close();
      request->send(200, "application/json", "{\"status\":\"success\",\"message\":\"WiFi config saved\"}");
    } else {
      #if defined(DEBUG)
      Serial.println("⚠️ wifi_config.json not found");
      #endif
      file.close();
      request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to write file\"}");      
    }

  });

  // รับ JSON ผ่าน POST
  server.on("/save_notify", HTTP_POST,[](AsyncWebServerRequest *request){
    request->send(200, "application/json", "{\"status\":\"processing\"}");
    }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    #if defined(DEBUG)
     // แปลงข้อมูลเป็น String
    String jsonString = String((char*)data).substring(0, len);
    Serial.println("Received JSON: " + jsonString);
    #endif
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, data);
    if (error) {
      #if defined(DEBUG)
      Serial.println("JSON parse failed");
      #endif
      request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
      return;
    }

    // เปิดไฟล์เพื่อเขียน
    File file = LittleFS.open("/notify_config.json", "w");
    if (!file) {
      #if defined(DEBUG)
      Serial.println("Failed to open file for writing");
      #endif
      request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to open file\"}");
      return;
    }

    // เขียน JSON ลงไฟล์
    if (serializeJson(doc, file) == 0) {
      #if defined(DEBUG)
      Serial.println("Failed to write JSON to file");
      #endif
      file.close();
      request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to write file\"}");
      return;
    }

    if (LittleFS.exists("/notify_config.json")) {
      #if defined(DEBUG)
      Serial.println("✅ notify_config.json exists");
      #endif
      file.close();
      request->send(200, "application/json", "{\"status\":\"success\",\"message\":\"notify_config saved\"}");
    } else {
      #if defined(DEBUG)
      Serial.println("⚠️ notify_config.json not found");
      #endif
      file.close();
      request->send(500, "application/json", "{\"status\":\"error\",\"message\":\"Failed to write file\"}");
      return;
    }
  });

// Start Server
    server.begin();

    while(true) {
      yield();
      unsigned long currentMillis = millis();
      // ตรวจสอบและกระพริบ LED ทุก 300 มิลลิวินาที
      if (currentMillis - previousMillis >= interval) {
          previousMillis = currentMillis;
          ledState = !ledState;
          digitalWrite(LED_STATUS, ledState);
      } 
      // ประมวลผลคำขอ DNS และ mDNS
      dnsServer.processNextRequest();    
    }

  }
  //End Web Server

  public:
    // เรียก Contructor
    ConfigManager() : server(80) {}

    bool begin(int BUNTTON_PUSH){
      if(!LittleFS.begin()){
        #if defined(DEBUG)
        Serial.println("An Error has occurred while mounting LittleFS");
        #endif
        return false;
      }
      pinMode(BUNTTON_PUSH, INPUT_PULLUP); // GPIO0 must be HIGH for normal boot        
      pinMode(LED_STATUS, OUTPUT);
      digitalWrite(LED_STATUS, LOW);

      if(digitalRead(BUNTTON_PUSH) == LOW){        
        enterConfigMode();        
      }
      #if defined(DEBUG)
      Serial.println("✨✨✨ Load config JSON ✨✨✨");
      #endif      
      loadConfigWiFi();
      loadConfigNotify();

      // ถอดขา GPIO0 ออกจาก INPUT_PULLUP เมื่อเข้าสู่โหมดทำงานปกติ
      pinMode(BUNTTON_PUSH, INPUT); // GPIO0 must be HIGH for normal boot    
      return true; // สำเร็จ (เข้าสู่ Normal Mode)

    }

    ConfigWiFi getConfigWiFi() const {
      return configWiFi;
    }
    
    ConfigNotify getConfigNotify() const {
      return configNotify;
    }



};
#include "HardwareSerial.h"
#include <WiFi.h>
#include <FS.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>

// #define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา

struct ConfigWiFi {
  char* ssid = nullptr;
  char* password = nullptr;
};

/*
ืntfy = URLtopic
discord = Webhook URL
telegram = Token + chat_id
*/
// 1 = ntfy, 2 = telegram, 3 = discord
struct ConfigNotify {  
  char* url = nullptr;
  char* channel = nullptr;
  uint8_t type = 1;
  uint8_t interval = 6;
  char* location = nullptr;
};

class ConfigManager {
  private: 
  // กำหนดขาใช้งาน
  const int LED_STATUS = 23;

  // web server object
  AsyncWebServer server;
  DNSServer dnsServer;
  const byte DNS_PORT = 53;

  mutable ConfigWiFi configWiFi;  //mutable
  // ParamNtfy paramNtfy;
  mutable ConfigNotify configNotify;

  // ตัวแปรสำหรับจัดการการกระพริบ LED แบบ non-blocking
  unsigned long previousMillis = 0;
  const long interval = 100; // ช่วงเวลา 300 มิลลิวินาที
  bool ledState = LOW;

  String chipID() {
    uint64_t chipMac = ESP.getEfuseMac();  // ได้ MAC address เป็นเลข 64-bit
    // ใช้เฉพาะ 6 ตัวอักษรท้าย (3 ไบต์สุดท้าย) เพื่อให้เหมือน ESP8266 chipId
    uint32_t chipId32 = (uint32_t)(chipMac >> 16); 
    String chipIDStr = String(chipId32, HEX);
    chipIDStr.toUpperCase();
    return chipIDStr;
  }

  bool loadConfigWiFi() const{
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

bool loadConfigNotify() const{    
    File file = LittleFS.open("/notify_config.json", "r");
    if(!file){
      #if defined(DEBUG)
      Serial.println("Failed to open config file");
      #endif
      return false;
    }

    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, file);
    file.close();

    if (error) {
      #if defined(DEBUG)
      Serial.println("Failed to parse config file");
      #endif
      return false;
    }

    // ✅ ตรวจสอบว่ามี key ที่จำเป็นหรือไม่
    if (!doc.containsKey("url") || !doc.containsKey("channel")) {
      #if defined(DEBUG)
      Serial.println("Missing required fields url and channel in config");
      #endif
      return false;
    }

    const char* _url = doc["url"];
    const char* _channel = doc["channel"];
    const uint8_t _type = doc["type"] | 0;
    const uint8_t _interval = doc["interval"] | 0;
    const char* _location = doc["location"];

    //✅ ตรวจสอบว่าไม่เป็น NULL และไม่ใช่ string ว่าง
    if (_url == nullptr || _channel == nullptr) {
      #if defined(DEBUG)
      Serial.println("URL or Channel is null or empty");
      #endif
      return false;
    }

    if((strlen(_url) == 0 || strlen(_channel) == 0) && _type == 2){
      #if defined(DEBUG)
      Serial.println("Telegram Channel is null or empty");
      #endif
      return false;
    }

    

    // ✅ ปล่อยหน่วยความจำเก่า
    if (configNotify.url != nullptr) {
      free(configNotify.url);
      configNotify.url = nullptr;
    }
    if (configNotify.channel != nullptr) { 
      free(configNotify.channel);
      configNotify.channel = nullptr;
    }
    if (configNotify.location != nullptr) { 
      free(configNotify.location);
      configNotify.location = nullptr;
    }

    // ทำ strdup เพื่อ copy string ไปยัง heap
    configNotify.url = strdup(_url);
    configNotify.channel = strdup(_channel);
    configNotify.location = strdup(_location);

     #if defined(DEBUG)
    Serial.println("========copy string ไปยัง heap=======");
    Serial.printf("%s\n", configNotify.url);
    Serial.println("========================");
    #endif
    
    // ✅ ตรวจสอบว่า strdup สำเร็จหรือไม่ 1 = ntfy, 2 = telegram, 3 = discord
    if ((configNotify.url == nullptr || configNotify.channel == nullptr) && configNotify.type == 2) {
      #if defined(DEBUG)
      Serial.println("Failed to allocate memory for config");
      #endif
      return false;
    }
    
    configNotify.type = _type;
    configNotify.interval = _interval;

    #if defined(DEBUG)
    Serial.printf("Location: %s\n", configNotify.location);
    Serial.printf("Loaded URL: %s\n", configNotify.url);
    Serial.printf("Loaded CHANNEL: %s\n", configNotify.channel);
    Serial.printf("Loaded TYPE: %d\n", configNotify.type);
    Serial.printf("Loaded INTERVAL: %d\n", configNotify.interval);
    #endif
    
    return true;
}

    // ฟังก์ชันสำหรับอ่านไฟล์และพิมพ์เนื้อหาออกทาง Serial
#if defined(DEBUG)
void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path, "r"); // เปิดไฟล์ในโหมดอ่าน ("r")
  if (!file || file.isDirectory()) {
    Serial.println("- Failed to open file for reading");
    return;
  }

  Serial.println("- Read from file:");
  // อ่านข้อมูลทีละไบต์จนกว่าจะหมดไฟล์และพิมพ์ออกทาง Serial
  while (file.available()) {
    Serial.write(file.read()); 
  }
  
  // ขึ้นบรรทัดใหม่หลังอ่านจบ
  Serial.println(); 

  // ปิดไฟล์หลังจากอ่านเสร็จ
  file.close();
}
#endif

  // Start Web Server
  void enterConfigMode() {
    // Initialize LittleFS
   
    WiFi.setSleep(0);
    WiFi.mode(WIFI_AP);
    String apName = "📲SETUP_GW-"+ chipID();
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

    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");

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

      if (LittleFS.exists("/wifi_config.json")){
        //  ลบไฟล์เก่า
        LittleFS.remove("/wifi_config.json");
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
      StaticJsonDocument<1024> doc;
      DeserializationError error = deserializeJson(doc, data);
      if (error) {
        #if defined(DEBUG)
        Serial.println("JSON parse failed");
        #endif
        request->send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
        return;
      }

      if (LittleFS.exists("/notify_config.json")){
        //  ลบไฟล์เก่า
        LittleFS.remove("/notify_config.json");  
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

    server.on("/getWifiList", HTTP_GET, [](AsyncWebServerRequest *request) {

        
        // สร้างตัวแปรส่งคำตอบกลับในรูปแบบ JSON
        AsyncJsonResponse *response = new AsyncJsonResponse(true); // true = เป็น Array
        JsonArray root = response->getRoot().as<JsonArray>();

        // เริ่มทำการสแกน WiFi
        int n = WiFi.scanNetworks(false, false, false, 300);

        if (n > 0) {
          for (int i = 0; i < n; ++i) {
            JsonObject network = root.add<JsonObject>();
            network["id"] = i + 1;
            network["ssid"] = WiFi.SSID(i);
            network["rssi"] = WiFi.RSSI(i);
            network["encryption"] = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "open" : "secured";
          }
        }

        // ลบผลการสแกนออกจาก Memory
        WiFi.scanDelete();

        // ส่งคำตอบกลับไปหา Client
        response->setLength();
        request->send(response);
    });

    // ส่งค่าการตั้งค่าWiFi
    server.on("/getWifi", HTTP_GET, [](AsyncWebServerRequest *request){      
      if (LittleFS.exists("/wifi_config.json")) {
        request->send(LittleFS, "/wifi_config.json", "application/json");
      } else {
          // หากไม่มีไฟล์ ให้ส่ง JSON เปล่าๆ กลับไปแทน
          request->send(200, "application/json", "{}");
      }
    });
    // ส่งค่าการตั้งค่าการแจ้งเตือน
    server.on("/getNotify", HTTP_GET, [](AsyncWebServerRequest *request){
      if (LittleFS.exists("/notify_config.json")) {
        request->send(LittleFS, "/notify_config.json", "application/json");
      } else {
          // หากไม่มีไฟล์ ให้ส่ง JSON เปล่าๆ กลับไปแทน
          request->send(200, "application/json", "{}");
      }
    });

    server.on("/reset", HTTP_GET,[](AsyncWebServerRequest *request){
      ESP.restart();
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
          delay(10);
          if(!LittleFS.begin()){
            #if defined(DEBUG)
            Serial.println("An Error has occurred while mounting LittleFS");
            #endif
            return false;
          }
          pinMode(BUNTTON_PUSH, INPUT_PULLUP); 
          delay(5000); 
          pinMode(LED_STATUS, OUTPUT);
          digitalWrite(LED_STATUS, LOW); 
          if(digitalRead(BUNTTON_PUSH) == LOW){        
            enterConfigMode();        
          }
          
          #if defined(DEBUG)
          Serial.println("✨✨✨ Load config JSON ✨✨✨");
          readFile(LittleFS, "/notify_config.json");
          #endif 
          
          return true; // สำเร็จ (เข้าสู่ Normal Mode)

        }

        ConfigWiFi getConfigWiFi() const {
          loadConfigWiFi();
          return configWiFi;
        }
        
        ConfigNotify getConfigNotify() const {
          loadConfigNotify();
          return configNotify;
        }



    };
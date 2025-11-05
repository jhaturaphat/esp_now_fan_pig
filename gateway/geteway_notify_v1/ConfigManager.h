#include <WiFi.h>
#include <FS.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>



struct ParamWiFi {
  char* ssid;
  char* password;
};

struct ParamNtfy {
  char* topic; 
};

struct ParamDiscord {
  char* webhooks;
};

struct ParamTelegram {
  char* token;
  char channel;
};

class ConfigManager {
  private: 
  // กำหนดขาใช้งาน
  const int LED_STATUS = 18;

  // web server object
  AsyncWebServer server;
  DNSServer dnsServer;
  const byte DNS_PORT = 53;

  ParamWiFi paramWiFi;
  ParamNtfy paramNtfy;
  ParamTelegram paramTelegram;

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

  // Start Web Server
  void enterConfigMode() {
    // Initialize LittleFS
    if(!LittleFS.begin()){
      #if defined(DEBUG)
      Serial.println("An Error has occurred while mounting LittleFS");
      #endif
      return;
    }
    WiFi.setSleep(0);
    WiFi.mode(WIFI_AP);
    String apName = "SETUP_GW-"+ chipID();
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
  server.on("/json", HTTP_POST, [](AsyncWebServerRequest *request){
    // ส่ง response ทันที
      request->send(200, "application/json", "{\"status\":\"processing\"}");
    }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    
    // แปลงข้อมูลเป็น String
    String jsonString = String((char*)data).substring(0, len);
     #if defined(DEBUG)
    Serial.println("Received JSON: " + jsonString);
    #endif

    // Parse JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonString);

    if (!error) {
      // อ่านค่าจาก JSON
      if (doc.containsKey("name")) {
        String name = doc["name"];
         #if defined(DEBUG)
        Serial.println("Name: " + name);
        #endif
      }
      if (doc.containsKey("value")) {
        int value = doc["value"];
         #if defined(DEBUG)
        Serial.println("Value: " + String(value));
        #endif
      }
      } else {
         #if defined(DEBUG)
        Serial.println("JSON parse error");
        #endif
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
      pinMode(BUNTTON_PUSH, INPUT_PULLUP); // GPIO0 must be HIGH for normal boot        
      pinMode(LED_STATUS, OUTPUT);
      digitalWrite(LED_STATUS, LOW);

      bool switchPressed = (digitalRead(BUNTTON_PUSH) == LOW);

      if(switchPressed){
        enterConfigMode();
      }

      // ถอดขา GPIO0 ออกจาก INPUT_PULLUP เมื่อเข้าสู่โหมดทำงานปกติ
      pinMode(BUNTTON_PUSH, INPUT); // GPIO0 must be HIGH for normal boot    
      return true; // สำเร็จ (เข้าสู่ Normal Mode)

    }


};
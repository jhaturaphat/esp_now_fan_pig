#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <DNSServer.h>
#include "esp_mac.h"

#define MAX_CH 13
// #define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา
// --- โครงสร้างข้อมูลสำหรับเก็บการตั้งค่า ---
struct DeviceConfig {    
    uint8_t channel = 1;    
};

// --- HTML Template (อยู่นอกคลาส) ---
const char index_html[] PROGMEM = R"rawliteral(
 	<!DOCTYPE HTML><html lang="th"><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>ESP Config</title><style>
    body{font-family:Arial;margin:0;padding:0;text-align:center;background:#f9f9f9}
    h1{margin:20px;font-size:2em}
    .container{max-width:500px;margin:20px auto;padding:20px;background:#fff;border:1px solid #ccc;border-radius:8px;box-shadow:0 2px 5px rgba(0,0,0,0.1)}
    form{display:flex;flex-direction:column}
    label{text-align:left;margin-top:10px;font-weight:bold}
    input[type=number],input[type=text]{padding:10px;margin-top:5px;border:1px solid #ccc;border-radius:4px;font-size:16px}
    .btn{background:#4CAF50;color:#fff;padding:12px;margin-top:20px;border:none;border-radius:5px;font-size:16px;cursor:pointer}
    .btn:hover{background:#45a049}
    .message{margin-top:15px;font-size:16px}
    .error{color:red}.success{color:green}
    @media(max-width:600px){h1{font-size:1.5em}.container{margin:10px;padding:15px}.btn{padding:10px;font-size:15px}}
    </style></head><body>
    <h1>Reciver Gateway Config</h1>
    <h3>การกำหนดค่าเกตเวย์ตัวรับ</h3>
    <div class="container">
    <form id="configForm" onsubmit="submitForm(event)">
    <label for="ch">Channel ID (1-13):</label>
    <input type="number" id="channel" name="channel" min="1" max="13" value="%CH_VALUE%" placeholder="1-13" required>    
    <input type="submit" value="Save & Restart" class="btn">
    </form>
    <div id="message" class="message"></div>
    <hr><div>%MAC_ADDR%</div><hr>
    <div>Copyright by Mr.Jaturapat Siriboon</div>
    </div>
    <script>
    function trimInputs(){    
    let ch=document.getElementById('ch');
    }
    async function submitForm(e){e.preventDefault();
    trimInputs();
    let f=document.getElementById('configForm');
    let m=document.getElementById('message');
    let d=new FormData(f);
    try{let r=await fetch('/save',{method:'POST',body:d});
    if(r.ok){m.textContent='Saved successfully!';m.className='message success';}
    else{let t=await r.text();m.textContent='Error: '+t;m.className='message error';}
    }catch(e){m.textContent='Error: Failed to connect to server';m.className='message error';}}
    </script></body></html>

)rawliteral";

class ConfigManager {
private:
    
    const int LED_PIN = 18;           // GPIO2: Output (LED status)
    const int EEPROM_SIZE = sizeof(DeviceConfig);

    // --- Web Server Objects ---    
    AsyncWebServer server;
    DNSServer dnsServer;
    const byte DNS_PORT = 53;
    
    const char* PARAM_ID = "id";
    const char* PARAM_CH = "channel";
    const char* PARAM_MAC = "mac";

    // --- ตัวแปรสำหรับเก็บค่า ---
    DeviceConfig currentConfig;
    
    // ตัวแปรสำหรับจัดการการกระพริบ LED แบบ non-blocking
    unsigned long previousMillis = 0;
    const long interval = 300; // ช่วงเวลา 300 มิลลิวินาที
    bool ledState = LOW;

    // --- Private Methods ---

    // 1. บันทึกค่าลง EEPROM
    void saveConfig() {
        EEPROM.put(0, currentConfig);
        EEPROM.commit();
        //Serial.println("ConfigManager: Configuration Saved.");
    }

    // 3. ตัวประมวลผล HTML (แทนที่ค่าใน Template)
    String processor(const String& var){     

        if(var == "CH_VALUE"){
            return String(currentConfig.channel);
        }else if(var == "MAC_ADDR"){
            return getInterfaceMacAddress(ESP_MAC_WIFI_STA);
        }      
        return String();
    }

    String chipID() {
      uint64_t chipMac = ESP.getEfuseMac();  // ได้ MAC address เป็นเลข 64-bit
      // ใช้เฉพาะ 6 ตัวอักษรท้าย (3 ไบต์สุดท้าย) เพื่อให้เหมือน ESP8266 chipId
      uint32_t chipId32 = (uint32_t)(chipMac >> 16); 
      String chipIDStr = String(chipId32, HEX);
      chipIDStr.toUpperCase();
      return chipIDStr;
    }

    String getInterfaceMacAddress(esp_mac_type_t interface) {

        String mac = "";
        unsigned char mac_base[6] = {0};
        if (esp_read_mac(mac_base, interface) == ESP_OK) {
            char buffer[18];  // 6*2 characters for hex + 5 characters for colons + 1 character for null terminator
            sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
            mac = buffer;
        }
        return mac;
    }

    // 4. โหมดตั้งค่า (รัน Web Server)
    void enterConfigMode() {   
           // Serial.println("\n*** ConfigManager: Entering Configuration Mode ***");        
        WiFi.setSleep(false);
        WiFi.mode(WIFI_AP);        
        String apName = "📲GW_NOW-" + chipID();          
        WiFi.softAP(apName.c_str(),"", 1, 0, 4); // ช่อง 1, จำกัด 4 อุปกรณ์
       
        delay(50); //หน่วงเวลา 100 ms เพื่อให้ AP เริ่มทำงาน
        // Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
        
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

        // --- Web Server Routes ---
        // หน้าหลัก (แสดงฟอร์ม)
      server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request){
        String html = index_html; // ดึงจาก PROGMEM

        if(currentConfig.channel > 13){
            html.replace("%CH_VALUE%", "");
        }else{
            html.replace("%CH_VALUE%", String(currentConfig.channel));
        }
            html.replace("%MAC_ADDR%", getInterfaceMacAddress(ESP_MAC_WIFI_STA));

            request->send(200, "text/html", html);
        });

    
    // รับข้อมูลจากฟอร์มและบันทึก
    server.on("/save", HTTP_POST, [this](AsyncWebServerRequest *request){
        String msg = "Configuration Saved! Device will restart...";
        bool valid = true;    
        // รับ CH
        if(request->hasParam(PARAM_CH, true)){
            String chStr = request->getParam(PARAM_CH, true)->value();
            uint16_t ch = chStr.toInt();
            if(ch >= 1 && ch <= MAX_CH) {
                this->currentConfig.channel = (uint8_t)ch;
            } else {
                msg = "Invalid ID! Must be 1-"+MAX_CH; 
                valid = false;
            }
        } else { valid = false; }
       

        // บันทึกและตอบกลับ
        if (valid) {
            this->saveConfig();
            request->send(200, "text/plain", "OK. " + msg);
            // Serial.println("ConfigManager: Saving and Restarting...");
            delay(1000); 
            //ESP.restart();
        } else {
            request->send(400, "text/plain", "ERROR. " + msg);
        }
    });

        // สำหรับ iOS Captive Portal
    server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request) { 
        // request->send(200, "text/html", index_html, [](const String& var) { return String(); });
        request->redirect("http://" + WiFi.softAPIP().toString());
    });
    // สำหรับ iOS Captive Portal
    server.on("/fwlink", HTTP_GET, [](AsyncWebServerRequest *request){
        request->redirect("http://" + WiFi.softAPIP().toString());
    });
    // สำหรับ iOS Captive Portal
    server.onNotFound([this](AsyncWebServerRequest *request) {
        request->redirect("http://" + WiFi.softAPIP().toString());
    });

    server.begin();
    
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
        dnsServer.processNextRequest();       
      }
    }

public:
    // Constructor
    ConfigManager() : server(80) {}
    
    // Public Method: โหลดค่าและตัดสินใจเข้าโหมดตั้งค่า
    bool begin(int PIN_CONFIG) {
        EEPROM.begin(EEPROM_SIZE);
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, LOW);
        
        // 1. ตรวจสอบการกดสวิตช์เพื่อเข้าโหมดตั้งค่า
        pinMode(PIN_CONFIG, INPUT_PULLUP); // GPIO0 must be HIGH for normal boot
        delay(500); 

        bool switchPressed = (digitalRead(PIN_CONFIG) == LOW);
        
        // 2. โหลดค่า
        EEPROM.get(0, currentConfig);

        if (switchPressed) {
            // เข้าโหมดตั้งค่าถ้า: (1) สวิตช์ถูกกด หรือ (2) ค่าที่โหลดมาไม่ถูกต้อง
            #if defined(DEBUG)
            if (switchPressed) Serial.println("ConfigManager: Switch pressed. Entering Config Mode.");            
            #endif
            
            enterConfigMode(); 
            // โค้ดจะหยุดที่นี่ (ใน enterConfigMode) จนกว่าจะมีการรีสตาร์ท
        }

        // 4. โหมดทำงานปกติ
        #if defined(DEBUG)
        Serial.print("ConfigManager: Normal Operation (ID: "); 
        Serial.print(currentConfig.channel);
        Serial.println(")");
        #endif
        
        // ถอดขา GPIO0 ออกจาก INPUT_PULLUP เมื่อเข้าสู่โหมดทำงานปกติ
        pinMode(PIN_CONFIG, INPUT); 
        
        return true; // สำเร็จ (เข้าสู่ Normal Mode)
    }

    // Public Method: ดึงค่า Config ที่โหลดแล้ว
    DeviceConfig getConfig() const {
        return currentConfig;
    }
};
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>
#include <DNSServer.h>
#include <ESP8266mDNS.h> // เพิ่มไลบรารี mDNS

#define MAX_ID 10
#define MAX_CH 13
// #define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา
// --- โครงสร้างข้อมูลสำหรับเก็บการตั้งค่า ---
struct DeviceConfig {
    uint8_t deviceID = 0;
    uint8_t channel = 1;
    uint8_t gatewayMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};    
};

// --- HTML Template (อยู่นอกคลาส) ---
const char index_html[] PROGMEM = R"rawliteral(
 	<!DOCTYPE HTML><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>ESP Config</title><style>
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
    <h1>Sensor Config</h1>
    <div class="container">
    <form id="configForm" onsubmit="submitForm(event)">
    <label for="id">Sensor ID (1-10):</label>
    <input type="number" id="id" name="id" min="1" max="10" value="%ID_VALUE%" placeholder="1-10" required>
    <label for="ch">Channel ID (1-13):</label>
    <input type="number" id="channel" name="channel" min="1" max="13" value="%CH_VALUE%" placeholder="1-13" required>
    <label for="mac">Gateway MAC (1F:2F:3F:4F:5F:6F):</label>
    <input type="text" id="mac" name="mac" pattern="[0-9A-Fa-f]{2}(:[0-9A-Fa-f]{2}){5}" value="%MAC_VALUE%" required>
    <input type="submit" value="Save & Restart" class="btn">
    </form>
    <div id="message" class="message"></div>
    <div>Copyright by Mr.Jaturapat Siriboon</div>
    </div>
    <script>
    function trimInputs(){
    let id=document.getElementById('id');
    let ch=document.getElementById('ch');
    let mac=document.getElementById('mac');    
    if(mac)mac.value=mac.value.trim().toUpperCase();
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
    
    const int LED_PIN = 2;           // GPIO2: Output (LED status)
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
        if(var == "ID_VALUE"){
            return String(currentConfig.deviceID);
        }

        if(var == "CH_VALUE"){
            return String(currentConfig.channel);
        }

        if(var == "MAC_VALUE"){
            char macStr[18];
            sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X", 
                    currentConfig.gatewayMAC[0], currentConfig.gatewayMAC[1], currentConfig.gatewayMAC[2], 
                    currentConfig.gatewayMAC[3], currentConfig.gatewayMAC[4], currentConfig.gatewayMAC[5]);
            return String(macStr);
        }
        return String();
    }

    String chipID(){
      uint32_t chipId = ESP.getChipId();
      String chipIDStr = String(chipId, HEX);
      chipIDStr.toUpperCase();
      return chipIDStr;
    }

    // 4. โหมดตั้งค่า (รัน Web Server)
    void enterConfigMode() {
        // Serial.println("\n*** ConfigManager: Entering Configuration Mode ***");
        WiFi.setOutputPower(20.5);
        WiFi.setSleep(false);
        system_update_cpu_freq(160);

        String apName = "🛜SETUP-" + chipID();         
        
        // WiFi.mode(WIFI_AP_STA);
        WiFi.softAP(apName.c_str(),"", 1, 0, 4); // ช่อง 1, จำกัด 4 อุปกรณ์
        // WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
        delay(50); //หน่วงเวลา 100 ms เพื่อให้ AP เริ่มทำงาน
        // Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
        
        // เริ่ม DNSServer และตั้งค่า Captive Portal
        dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
        dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

        // เริ่ม mDNS
        if (MDNS.begin("่esp8266")) {
            MDNS.addService("http", "tcp", 80); // ประกาศว่าเป็นเซิร์ฟเวอร์ HTTP บนพอร์ต 80
        }

        // --- Web Server Routes ---
        // หน้าหลัก (แสดงฟอร์ม)
      server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request){
        String html = index_html; // ดึงจาก PROGMEM
        if(currentConfig.deviceID > 10){
            html.replace("%ID_VALUE%", "");
        }else{
            html.replace("%ID_VALUE%", String(currentConfig.deviceID));
        }

        if(currentConfig.channel > 13){
            html.replace("%CH_VALUE%", "");
        }else{
            html.replace("%CH_VALUE%", String(currentConfig.channel));
        }
        

        char macStr[18];
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                currentConfig.gatewayMAC[0], currentConfig.gatewayMAC[1], currentConfig.gatewayMAC[2],
                currentConfig.gatewayMAC[3], currentConfig.gatewayMAC[4], currentConfig.gatewayMAC[5]);
        html.replace("%MAC_VALUE%", String(macStr));

        request->send(200, "text/html", html);
        });

        // สำหรับ iOS Captive Portal
        server.on("/generate_204", HTTP_GET, [](AsyncWebServerRequest *request) { 
            // request->send(200, "text/html", index_html, [](const String& var) { return String(); });
            request->redirect("http://" + WiFi.softAPIP().toString());
        });

        // รับข้อมูลจากฟอร์มและบันทึก
        server.on("/save", HTTP_POST, [this](AsyncWebServerRequest *request){
            String msg = "Configuration Saved! Device will restart...";
            bool valid = true;
            
            // รับ ID
            if(request->hasParam(PARAM_ID, true)){
                String idStr = request->getParam(PARAM_ID, true)->value();
                uint16_t id = idStr.toInt();
                if(id >= 1 && id <= MAX_ID) {
                    this->currentConfig.deviceID = (uint8_t)id;
                } else {
                    msg = "Invalid ID! Must be 1-"+MAX_ID; 
                    valid = false;
                }
            } else { valid = false; }

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
            
            // รับ MAC
            if(request->hasParam(PARAM_MAC, true) && valid){
                String macStr = request->getParam(PARAM_MAC, true)->value();
                if (sscanf(macStr.c_str(), "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
                            &this->currentConfig.gatewayMAC[0], &this->currentConfig.gatewayMAC[1], &this->currentConfig.gatewayMAC[2], 
                            &this->currentConfig.gatewayMAC[3], &this->currentConfig.gatewayMAC[4], &this->currentConfig.gatewayMAC[5]) != 6) {
                    msg = "Invalid MAC Address format!"; valid = false;
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
        // หาก url ไม่ตรงกับ route
        server.onNotFound([this](AsyncWebServerRequest *request) {
            request->redirect("http://" + WiFi.softAPIP().toString());
        });
        // รัน server
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
            MDNS.update();
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
        delay(100); 

        bool switchPressed = (digitalRead(PIN_CONFIG) == LOW);
        
        // 2. โหลดค่า
        EEPROM.get(0, currentConfig);
        
        // 3. ตรวจสอบเงื่อนไขการทำงาน
        bool configValid = (currentConfig.deviceID >= 1 && currentConfig.deviceID <= MAX_ID);

        if (switchPressed || !configValid) {
            // เข้าโหมดตั้งค่าถ้า: (1) สวิตช์ถูกกด หรือ (2) ค่าที่โหลดมาไม่ถูกต้อง
            #if defined(DEBUG)
            if (switchPressed) Serial.println("ConfigManager: Switch pressed. Entering Config Mode.");
            if (!configValid) Serial.println("ConfigManager: Invalid config found. Entering Config Mode.");
            #endif
            
            enterConfigMode(); 
            // โค้ดจะหยุดที่นี่ (ใน enterConfigMode) จนกว่าจะมีการรีสตาร์ท
        }

        // 4. โหมดทำงานปกติ
        #if defined(DEBUG)
        Serial.print("ConfigManager: Normal Operation (ID: "); 
        Serial.print(currentConfig.deviceID);
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
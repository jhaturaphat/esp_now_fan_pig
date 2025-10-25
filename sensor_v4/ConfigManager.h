#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>

#define MAX_ID 20

// --- โครงสร้างข้อมูลสำหรับเก็บการตั้งค่า ---
struct DeviceConfig {
    uint8_t deviceID = 0;
    uint8_t gatewayMAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
};

// --- HTML Template (อยู่นอกคลาส) ---
const char index_html[] PROGMEM = R"rawliteral(
  <!DOCTYPE HTML>
  <html>
  <head>
    <title>Sensor Config</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; }
      .btn { background-color: #4CAF50; color: white; padding: 10px; border: none; cursor: pointer; border-radius: 5px; }
      input[type=number], input[type=text] {
        width: 90%; padding: 10px; margin: 8px 0; border: 1px solid #ccc;
        border-radius: 4px; box-sizing: border-box;
      }
      .container { margin: 20px; padding: 20px; border: 1px solid #ccc; border-radius: 5px; }
      .message { margin-top: 10px; color: #333; }
      .error { color: red; }
      .success { color: green; }
    </style>
  </head>
  <body>
  <h1>ESP Config</h1>
  <div class="container">
    <form id="configForm" onsubmit="submitForm(event)">
      <label for="id">Device ID (1-99):</label><br>
      <input type="number" id="id" name="id" min="1" max="99" value="%ID_VALUE%" required><br>
      <label for="mac">Gateway MAC (XX:XX:XX:XX:XX:XX):</label><br>
      <input type="text" id="mac" name="mac"
        pattern="[0-9A-Fa-f]{2}(:[0-9A-Fa-f]{2}){5}" value="%MAC_VALUE%" required><br>
      <input type="submit" value="Save & Restart" class="btn">
    </form>
    <div id="message" class="message"></div>
  </div>
  <script>
    function trimInputs() {
      const idField = document.getElementById('id');
      const macField = document.getElementById('mac');
      if (idField) idField.value = idField.value.trim();
      if (macField) macField.value = macField.value.trim().toUpperCase();
    }

    async function submitForm(event) {
      event.preventDefault(); // ป้องกันการรีเฟรชหน้า
      trimInputs(); // ตัดช่องว่างหน้า-หลัง

      const form = document.getElementById('configForm');
      const messageDiv = document.getElementById('message');
      const formData = new FormData(form);

      try {
        const response = await fetch('/save', {
          method: 'POST',
          body: formData
        });

        if (response.ok) {
          const result = await response.text();
          messageDiv.textContent = 'Saved successfully!';
          messageDiv.className = 'message success';
        } else {
          const error = await response.text();
          messageDiv.textContent = 'Error: ' + error;
          messageDiv.className = 'message error';
        }
      } catch (error) {
        messageDiv.textContent = 'Error: Failed to connect to server';
        messageDiv.className = 'message error';
      }
    }
  </script>
</body>
</html>
)rawliteral";

class ConfigManager {
private:
    // --- ตั้งค่า Hardware และ EEPROM ---
    const int CONFIG_BUTTON_PIN = 0; // GPIO0: Input (LOW = Config Mode)
    const int LED_PIN = 2;           // GPIO2: Output (LED status)
    const int EEPROM_SIZE = sizeof(DeviceConfig);

    // --- Web Server Objects ---    
    AsyncWebServer server(80);
    // const char* ap_ssid = "ESP_SETUP";
    const char* PARAM_ID = "id";
    const char* PARAM_MAC = "mac";

    // --- ตัวแปรสำหรับเก็บค่า ---
    DeviceConfig currentConfig;

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
        
        String apName = "AP-SETUP-" + chipID();
        // Setup AP        
        WiFi.softAP(apName.c_str(), "", 6, 0, 4); // จำกัด 4 อุปกรณ์
        delay(120); //หน่วงเวลา 100 ms เพื่อให้ AP เริ่มทำงาน
        // Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
        
        // --- Web Server Routes ---

            
        // หน้าหลัก (แสดงฟอร์ม)
      server.on("/", HTTP_GET, [this](AsyncWebServerRequest *request){
        String html = index_html; // ดึงจาก PROGMEM
        html.replace("%ID_VALUE%", String(currentConfig.deviceID));

        char macStr[18];
        sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
                currentConfig.gatewayMAC[0], currentConfig.gatewayMAC[1], currentConfig.gatewayMAC[2],
                currentConfig.gatewayMAC[3], currentConfig.gatewayMAC[4], currentConfig.gatewayMAC[5]);
        html.replace("%MAC_VALUE%", String(macStr));

        request->send(200, "text/html", html);
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
                ESP.restart();
            } else {
                request->send(400, "text/plain", "ERROR. " + msg);
            }
        });

        server.begin();
        
        // Loop for blinking LED during config mode
        while(true) {
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(300);
        }
    }

public:
    // Constructor
    ConfigManager() : server(80) {}

    // Public Method: โหลดค่าและตัดสินใจเข้าโหมดตั้งค่า
    bool begin() {
        EEPROM.begin(EEPROM_SIZE);
        pinMode(LED_PIN, OUTPUT);
        digitalWrite(LED_PIN, LOW);
        
        // 1. ตรวจสอบการกดสวิตช์เพื่อเข้าโหมดตั้งค่า
        pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP); // GPIO0 must be HIGH for normal boot
        delay(100); 

        bool switchPressed = (digitalRead(CONFIG_BUTTON_PIN) == LOW);
        
        // 2. โหลดค่า
        EEPROM.get(0, currentConfig);
        
        // 3. ตรวจสอบเงื่อนไขการทำงาน
        bool configValid = (currentConfig.deviceID >= 1 && currentConfig.deviceID <= 99);

        if (switchPressed || !configValid) {
            // เข้าโหมดตั้งค่าถ้า: (1) สวิตช์ถูกกด หรือ (2) ค่าที่โหลดมาไม่ถูกต้อง
            // if (switchPressed) Serial.println("ConfigManager: Switch pressed. Entering Config Mode.");
            // if (!configValid) Serial.println("ConfigManager: Invalid config found. Entering Config Mode.");
            
            enterConfigMode(); 
            // โค้ดจะหยุดที่นี่ (ใน enterConfigMode) จนกว่าจะมีการรีสตาร์ท
        }

        // 4. โหมดทำงานปกติ
        // Serial.print("ConfigManager: Normal Operation (ID: "); 
        // Serial.print(currentConfig.deviceID);
        // Serial.println(")");
        
        // ถอดขา GPIO0 ออกจาก INPUT_PULLUP เมื่อเข้าสู่โหมดทำงานปกติ
        pinMode(CONFIG_BUTTON_PIN, INPUT); 
        
        return true; // สำเร็จ (เข้าสู่ Normal Mode)
    }

    // Public Method: ดึงค่า Config ที่โหลดแล้ว
    DeviceConfig getConfig() const {
        return currentConfig;
    }
};
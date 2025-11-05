#include <WiFi.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "kid_2.4GHz";
const char* password = "xx3xx3xx";

AsyncWebServer server(80);

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  // รับ JSON ผ่าน POST
  server.on("/json", HTTP_POST, [](AsyncWebServerRequest *request){
    // ส่ง response ทันที
    request->send(200, "application/json", "{\"status\":\"processing\"}");
  }, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    
    // แปลงข้อมูลเป็น String
    String jsonString = String((char*)data).substring(0, len);
    
    Serial.println("Received JSON: " + jsonString);

    // Parse JSON
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, jsonString);

    if (!error) {
      // อ่านค่าจาก JSON
      if (doc.containsKey("name")) {
        String name = doc["name"];
        Serial.println("Name: " + name);
      }
      if (doc.containsKey("value")) {
        int value = doc["value"];
        Serial.println("Value: " + String(value));
      }
    } else {
      Serial.println("JSON parse error");
    }
  });

  server.begin();
}

void loop() {
  delay(100);
}
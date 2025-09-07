#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

const char* ssid = "TEST";
const char* password = ""; // ใส่รหัสผ่าน WiFi ถ้ามี
const char* webhookUrl = "https://discord.com/api/webhooks/1344992087879188550/vGrbJbJQUkaf_ZmxpzSvm9_1gcwHrEylA-VOQCTrlKsu2gq8mq9tTycQWKT7I8jlQb4n";

unsigned long lastMessage = 0;
const unsigned long interval = 60000 * 2; // ส่งทุก 60 วินาที

String buffer = "";  // Buffer สำหรับสะสม message
bool lastWasNewline = false;  // ตรวจสอบ \n ก่อนหน้า

// ตั้งค่า NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 25200, 60000); // UTC+7 สำหรับประเทศไทย

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); 
  
  // เชื่อมต่อ Wi-Fi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("Connected to WiFi!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    
    // ทดสอบส่งข้อความเมื่อเชื่อมต่อสำเร็จ
    sendMessageToDiscord("ESP32 เชื่อมต่อสำเร็จแล้ว! 🎉");
  } else {
    Serial.println();
    Serial.println("Failed to connect to WiFi");
  }

  // เริ่มต้น NTP Client
  timeClient.begin();


}

void loop() {
  
  unsigned long currentMillis = millis();
    
  static unsigned long lastDataTime = 0;
  const unsigned long dataTimeout = 1000; // รอ 1 วินาที หลังจากไม่มีข้อมูลเพิ่ม
  
  // อ่านข้อมูลที่มีอยู่
  while (Serial2.available()) {
    char c = Serial2.read();
    buffer += c;
    // Debug: แสดงทุกตัวอักษรที่ได้รับ
    // Serial.print("Received char: ");
    // if (c == '\n') {
    //   Serial.println("[NEWLINE]");
    // } else if (c == '\r') {
    //   Serial.println("[CARRIAGE RETURN]");
    // } else {
    //   Serial.println(c);
    // }

    // ตรวจสอบ end marker
    if (buffer.endsWith("END")) {
      buffer.replace("END", "");  // ลบ marker
      buffer.trim();
      if (buffer.length() > 0) {
        Serial.println("Full message received:");
        Serial.println("=== START ===");
        Serial.println(buffer);
        Serial.println("=== END ===");
        sendMessageToDiscord("🎉แจ้งเตือนพัดลม ครับ");        
        sendMessageToDiscord(buffer);
      } else {
        Serial.println("Empty buffer after trim!");
      }
      buffer = "";
    }
  }

  delay(10);  // ป้องกัน loop เร็วเกิน
  
  
  // ส่งข้อความทดสอบทุก 60 วินาที (ถ้าต้องการ)
  // if (currentMillis - lastMessage > interval) {
  //   if (WiFi.status() == WL_CONNECTED) {
  //     String testMessage = "ข้อความทดสอบจาก ESP32 - เวลา: " + String(currentMillis/1000) + " วินาที";
  //     sendMessageToDiscord(testMessage);
  //     lastMessage = currentMillis;
  //   }
  // }
  
  // ตรวจสอบการเชื่อมต่อ WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected, attempting to reconnect...");
    WiFi.reconnect();
    delay(5000);
  }

  timeClient.update();
  // ดึงเวลาปัจจุบัน
  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();
  int currentSecond = timeClient.getSeconds();

  // ตัวอย่าง: ส่งข้อมูลเวลา 8:00 และ 20:00
  if ((currentHour == 8 || currentHour == 12 || currentHour == 16 || 
       currentHour == 20 || currentHour == 0 || currentHour == 4) && 
       currentMinute == 0 && currentSecond == 0) {
    String testMessage = "🤖 ข้อความทดสอบจากระบบ 🐷🐓 - เวลา: " + String(currentMillis/1000) + " วินาที";
    sendMessageToDiscord(testMessage);
    delay(1000); // รอ 1 วินาทีเพื่อป้องกันการส่งซ้ำ
  }

}

void sendMessageToDiscord(String msg) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected - cannot send message");
    return;
  }
  
  WiFiClientSecure client;
  client.setInsecure(); // ไม่ตรวจสอบ SSL certificate (ใช้เฉพาะการทดสอบ)
  
  HTTPClient http;
  
  Serial.println("Attempting to send message to Discord...");
  
  if (http.begin(client, webhookUrl)) {
    http.addHeader("Content-Type", "application/json");
    http.addHeader("User-Agent", "ESP32-Discord-Bot");
    
    // สร้าง JSON payload และ escape อักขระพิเศษ
    String escapedMsg = msg;
    escapedMsg.replace("\"", "\\\""); // escape เครื่องหมาย quote
    escapedMsg.replace("\n", "\\n");  // escape newline
    
    
    String jsonPayload = "{\"content\": \"" + escapedMsg + "\"}";
    
    Serial.println("Payload: " + jsonPayload);
    
    // ส่ง POST request
    int httpResponseCode = http.POST(jsonPayload);
    
    if (httpResponseCode > 0) {
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
      
      if (httpResponseCode == 200 || httpResponseCode == 204) {
        Serial.println("Message sent successfully!");
      } else {
        String response = http.getString();
        Serial.println("Response: " + response);
      }
    } else {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
      Serial.println("Error: " + http.errorToString(httpResponseCode));
    }
    
    http.end();
  } else {
    Serial.println("Failed to begin HTTP connection");
  }
  
  delay(100); // หน่วงเวลาเล็กน้อย
}
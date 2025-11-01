#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

// #define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา

#define BUZZER_PIN 18  //Buzzer สำหรับขาดการสือสาร

unsigned long previousMillis = 0;
#define WIFI_TIMEOUT 30000 

const char* ssid = "TEST";
const char* password = ""; // ใส่รหัสผ่าน WiFi ถ้ามี
const char* webhookUrl = "https://discord.com/api/webhooks/1344992087879188550/vGrbJbJQUkaf_ZmxpzSvm9_1gcwHrEylA-VOQCTrlKsu2gq8mq9tTycQWKT7I8jlQb4n";

unsigned long lastMessage = 0;
const unsigned long interval = 60000; // ส่งทุก 60 วินาที

String buffer = "";  // Buffer สำหรับสะสม message
bool lastWasNewline = false;  // ตรวจสอบ \n ก่อนหน้า
bool receiving = false;  // Track ว่ากำลังรับ message หรือไม่

// ตั้งค่า NTP Client
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 25200, 60000); // UTC+7 สำหรับประเทศไทย

HardwareSerial mySerial(2);

void setup() {
  #if defined(DEBUG) 
  Serial.begin(115200);
  #endif

  mySerial.begin(9600, SERIAL_8N1, 16, 17);  // TX=17, RX=16
  
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // เชื่อมต่อ Wi-Fi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);  // ป้องกัน WiFi sleep
  #if defined(DEBUG) 
  Serial.println("Connecting to WiFi...");
  #endif
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000);
    #if defined(DEBUG) 
    Serial.print(".");
    #endif
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    #if defined(DEBUG) 
    Serial.println();
    Serial.println("Connected to WiFi!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    #endif
    String ip = String(WiFi.localIP());
    // ทดสอบส่งข้อความเมื่อเชื่อมต่อสำเร็จ
    // sendMessageToDiscord("Gateway เชื่อมต่อสำเร็จแล้ว! 🎉");
    sendMessageToDiscord("📶 เชื่อมต่อสำเร็จแล้ว! 🎉 \n IP:"+ip);  
  } else {
    #if defined(DEBUG) 
    Serial.println();
    Serial.println("Failed to connect to WiFi");
    #endif
  }

  // เริ่มต้น NTP Client
  timeClient.begin();


}

void loop() {
  
  unsigned long currentMillis = millis();
    
  static unsigned long lastDataTime = 0;
  const unsigned long dataTimeout = 1000; // รอ 1 วินาที หลังจากไม่มีข้อมูลเพิ่ม
  
  // อ่านข้อมูลที่มีอยู่
  while (mySerial.available()) {
    char c = mySerial.read();
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

    // ตรวจสอบ start marker
    if (buffer.endsWith("START\n")) {
      buffer = "";  // ล้าง buffer เริ่มใหม่
      receiving = true;
      #if defined(DEBUG) 
      Serial.println("Started new message");
      #endif
    }else if (receiving && buffer.endsWith("END\n")) { // ตรวจสอบ end marker
      buffer.replace("START\n", "");  // ลบ start marker
      buffer.replace("END\n", "");    // ลบ end marker
      buffer.trim();
      if (buffer.length() > 0) {
        #if defined(DEBUG) 
        Serial.println("Full message received:");
        Serial.println("=== START ===");
        Serial.println(buffer);
        Serial.println("=== END ===");
        #endif
        sendMessageToDiscord("🎉แจ้งเตือนพัดลม ครับ\n"+buffer);        
        // sendMessageToDiscord(buffer);
      } else {
        #if defined(DEBUG) 
        Serial.println("Empty buffer after trim!");
        #endif
      }
      buffer = "";
      receiving = false;
    }
  }

  delay(10);  // ป้องกัน loop เร็วเกิน
  
  
  // ส่งข้อความทดสอบทุก 60 วินาที (ถ้าต้องการ)
  // if (currentMillis - lastMessage >= interval) {
  //   if (WiFi.status() == WL_CONNECTED) {
  //     String testMessage = "ข้อความทดสอบจาก ESP32 - เวลา: " + String(currentMillis/1000) + " วินาที";
  //     // sendMessageToDiscord(testMessage);
  //     Serial.println(testMessage);      
  //     sendData("READ");     
  //     lastMessage = currentMillis;
  //   }
  // }
  
  
  // ตรวจสอบการเชื่อมต่อ WiFi
  if ((WiFi.status() != WL_CONNECTED)) {
    #if defined(DEBUG) 
    Serial.println("WiFi disconnected, attempting to reconnect...");
    #endif
    WiFi.disconnect();
    WiFi.reconnect();
    previousMillis = currentMillis;
    digitalWrite(BUZZER_PIN, HIGH);    
    delay(1000);
  }else{
    digitalWrite(BUZZER_PIN, LOW);     
  }
  // ส่งแจ้งเตือนตามเวลา
  systemStatus();
}

void sendData(String msg) {
  byte checksum = 0;
  for (int i = 0; i < msg.length(); i++) {
    checksum += (byte)msg.charAt(i);
  }
  checksum %= 256;

  mySerial.print('$');
  mySerial.print(msg);
  mySerial.print((char)checksum);
  mySerial.println();
  mySerial.flush();
  #if defined(DEBUG) 
  Serial.println("Sent: " + msg);  // Debug
  Serial.print((char)checksum);
  Serial.println(checksum);  // Debug
  #endif
}

void systemStatus(){
  timeClient.update();
  // ดึงเวลาปัจจุบัน
  int currentHour = timeClient.getHours();
  int currentMinute = timeClient.getMinutes();
  int currentSecond = timeClient.getSeconds();

  // ตัวอย่าง: ส่งข้อมูลเวลา 8:00 และ 20:00
  if ((currentHour == 8 || currentHour == 12 || currentHour == 16 || 
       currentHour == 20 || currentHour == 0 || currentHour == 4) && 
       currentMinute == 0 && currentSecond == 0) {
    String testMessage = "🤖 ข้อความทดสอบจากฟาร์ม 🐷🐓 - เวลา: " + String(currentHour) + ":"+String(currentMinute)+" นาที";
    sendMessageToDiscord(testMessage);    
    delay(1000); // รอ 1 วินาทีเพื่อป้องกันการส่งซ้ำ
    sendData("READ"); 
  }
}

void sendMessageToDiscord(String msg) {
  if (WiFi.status() != WL_CONNECTED) {
    #if defined(DEBUG) 
    Serial.println("WiFi not connected - cannot send message");
    #endif
    return;
  }
  
  WiFiClientSecure client;
  client.setInsecure(); // ไม่ตรวจสอบ SSL certificate (ใช้เฉพาะการทดสอบ)
  
  HTTPClient http;
  #if defined(DEBUG) 
  Serial.println("Attempting to send message to Discord...");
  #endif
  
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
    
    #if defined(DEBUG) 
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
    #endif
    
    http.end();
  } else {
    #if defined(DEBUG) 
    Serial.println("Failed to begin HTTP connection");
    #endif
  }
  
  delay(100); // หน่วงเวลาเล็กน้อย
}
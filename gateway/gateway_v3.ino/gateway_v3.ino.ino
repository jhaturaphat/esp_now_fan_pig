/*
 * ESP32 Gateway Code - รับสัญญาณจาก ESP-01 Sensors จำนวน 7 ตัว
 * ใช้ ESP-NOW Protocol สำหรับการสื่อสาร
 * Enhanced version - แสดงรายละเอียด sensor ที่ขาดการติดต่อและ switch ที่เปิด
 */

#include <esp_now.h>  //สำหรับ ESP32
#include <WiFi.h>
#include <HardwareSerial.h>

// กำหนด PIN
#define BUZZER_PIN18  18       // Buzzer สำหรับแจ้งเตือนขาดการสื่อสาร
#define SIREN_PIN19 19        // Siren สำหรับแจ้งเตือนหลัก
#define LED_STATUS_PIN2 5    // LED แสดงสถานะ

// จำนวน Sensor
#define MAX_SENSORS 7
#define COMMUNICATION_TIMEOUT 30000  // 30 วินาที timeout 
#define SIREN_TIMEOUT 120000 // 120 วินาที timeout
#define CHECK_TIMEOUT 5000 // 5 วินาที timeout

unsigned long lastSensorCheck = 0;
unsigned long lastComunication = 0;


// โครงสร้างข้อมูลที่รับจาก Sensor
typedef struct sensor_message {
  uint8_t sensor_id;      // ID ของ sensor (1-7)
  bool switch_status;     // สถานะ switch (true=closed, false=open)
  uint32_t timestamp;     // timestamp
} sensor_message;

// เก็บสถานะของแต่ละ sensor
struct sensor_status {
  bool is_online;           // สถานะการเชื่อมต่อ
  bool switch_state;        // สถานะ switch
  unsigned long last_seen;  // เวลาที่รับสัญญาณครั้งล่าสุด
  uint8_t mac[6];          // MAC address ของ sensor
};

sensor_status sensors[MAX_SENSORS];
bool siren_active = false;
bool buzzer_active = false;
unsigned long last_check_time = 0;
unsigned long last_status_print = 0;
unsigned long siren_start_time = 0;
unsigned long buzzer_start_time = 0;

// เก็บรายการ sensor ที่มีปัญหา
String offline_sensors = "";
String open_switches = "";
String system_status = "";

HardwareSerial mySerial(2);

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, 16, 17);  // TX=17, RX=16
  // กำหนด PIN Mode
  pinMode(BUZZER_PIN18, OUTPUT);
  pinMode(SIREN_PIN19, OUTPUT);
  pinMode(LED_STATUS_PIN2, OUTPUT);
  
  // เริ่มต้น WiFi ในโหมด Station
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // ป้องกัน WiFi sleep สำหรับ ESP-NOW
  // Serial.println("MAC Address: " + WiFi.macAddress());
  
  // เริ่มต้น ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.println("\n✅ ESP32 Gateway Ready - 24x7 Mode");
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║            GATEWAY INFO              ║");
  Serial.println("╚══════════════════════════════════════╝");
  
  // แสดงข้อมูล hardware
  Serial.printf("🔧 Chip Model: %s\n", ESP.getChipModel());
  Serial.printf("🔧 Chip Revision: %d\n", ESP.getChipRevision());
  Serial.printf("🔧 CPU Frequency: %d MHz\n", ESP.getCpuFreqMHz());
  
  // แสดงข้อมูล WiFi/ESP-NOW
  Serial.printf("📡 MAC Address: %s\n", WiFi.macAddress().c_str());
  Serial.printf("📶 WiFi Channel: %d\n", WiFi.channel());
  Serial.printf("🔧 Expected message size: %d bytes\n", sizeof(sensor_message));
  
  Serial.println("════════════════════════════════════════");
  Serial.println("🔍 Waiting for ESP-01 sensors...");
  Serial.println("   Copy this MAC to ESP-01 code:");
  // แปลง MAC address ให้ถูกต้อง
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.printf("   {0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}\n", 
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println();
  
  // ลงทะเบียน callback function (ESP32 Arduino Core 3.x)
  esp_now_register_recv_cb(onDataReceive);
  
  // เริ่มต้นสถานะ sensor
  initializeSensors();
  
  Serial.println("ESP32 Gateway Ready");
  Serial.println("Waiting for sensors...");
  
  // ทดสอบระบบเสียง
  testSounds();
}

void loop() {
  unsigned long currentMillis = millis();

  String buffer = "";
  while(mySerial.available()){    
    receiveAndRespond();
  }
  checkSensorCommunication();
  if(currentMillis - lastSensorCheck >= CHECK_TIMEOUT){  //ตรวจสอบทุกๆ 5 วินาที 
    lastSensorCheck = currentMillis;
    handleAlarms();
    updateStatusLED();
  }
  
  // แสดงสถานะระบบทุก 10 วินาที
  if (currentMillis - last_status_print >= 10000) {
    last_status_print = currentMillis;
    printSystemStatus();
  }

  // ตรวจสอบ Comunication Loss
  if(currentMillis - lastComunication >= COMMUNICATION_TIMEOUT){
    lastComunication = currentMillis;
    handleComunicationAlarms();
  }
}

// Callback สำหรับรับข้อมูล (สำหรับ ESP32 Arduino Core 3.x)
void onDataReceive(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
  sensor_message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  
  if (msg.sensor_id >= 1 && msg.sensor_id <= MAX_SENSORS) {
    int index = msg.sensor_id - 1;
    
    // อัพเดทสถานะ sensor
    sensors[index].is_online = true;
    sensors[index].switch_state = msg.switch_status;
    sensors[index].last_seen = millis();
    memcpy(sensors[index].mac, recv_info->src_addr, 6); 
    
    // ตรวจสอบสถานะ switch
    if (!msg.switch_status) {  // switch เปิด (แม่เหล็กออกจากกัน)
      triggerSiren();
      Serial.printf("🚨🚨🚨🚨ALERT: Sensor %d detected intrusion!\n", msg.sensor_id);
    }else{      
      stopSiren();
    }
  }
}

// สร้าง Array ตามจำนวน MAXSENSOR เพื่อเก็บค่า แต้ละเซ็นเซอร์ไว้ที่ตำแหน่งต่างๆตาม index
void initializeSensors() {
  for (int i = 0; i < MAX_SENSORS; i++) {
    sensors[i].is_online = false;
    sensors[i].switch_state = true;  // เริ่มต้นเป็น closed
    sensors[i].last_seen = 0;
    memset(sensors[i].mac, 0, 6);
  }
}
// ตรวจสอบการเชื่อมต่อ
void checkSensorCommunication() {
  if (millis() - last_check_time < 5000) return;  // ตรวจสอบทุก 5 วินาที
  
  last_check_time = millis();
  int offline_count = 0;
  int open_switch_count = 0;
  
  // รีเซ็ตรายการ
  offline_sensors = "";
  open_switches = "";
  
  for (int i = 0; i < MAX_SENSORS; i++) {
    // ตรวจสอบการสื่อสาร
    if (sensors[i].last_seen > 0) {  // sensor เคยส่งข้อมูลมาแล้ว
      if (millis() - sensors[i].last_seen > COMMUNICATION_TIMEOUT) {
        if (sensors[i].is_online) {
          Serial.printf("⚠️ WARNING: Lost communication with Sensor %d\n", i + 1);
          sensors[i].is_online = false;
        }
        offline_count++;        
        // เพิ่ม sensor ID ที่ขาดการติดต่อ
        if (offline_sensors.length() > 0) offline_sensors += ", ";
        offline_sensors += String(i + 1);
      }
    }
    
    // ตรวจสอบ switch ที่เปิด
    if (sensors[i].is_online && !sensors[i].switch_state) {
      open_switch_count++;
      
      // เพิ่ม sensor ID ที่ switch เปิด
      if (open_switches.length() > 0) open_switches += ", ";
      open_switches += String(i + 1);
    }
  }
  
  // แสดงรายงานสถานะ
  if (offline_count > 0) {
    Serial.printf("📡 COMMUNICATION LOST: Sensors [%s] (%d/%d sensors)\n", 
                  offline_sensors.c_str(), offline_count, MAX_SENSORS);
  }
  
  if (open_switch_count > 0) {
    Serial.printf("🚨 SWITCHES OPEN: Sensors [%s] (%d switches open)\n", 
                  open_switches.c_str(), open_switch_count);
  }
  
  // จัดการเสียงเตือนตามจำนวน sensor ที่ขาดการสื่อสาร
  if (offline_count == MAX_SENSORS) {
    // ขาดการสื่อสารทั้งหมด -> เปิดไซเรน
    triggerSiren();
    Serial.printf("🚨 CRITICAL: Lost communication with ALL sensors! [%s]\n", offline_sensors.c_str());
  } else if (offline_count > 0 && offline_count < MAX_SENSORS) {  
    // ขาดการสื่อสารบางตัว -> เปิด buzzer    
    triggerSiren();
    // triggerBuzzer(); //อันเดิมใช้ตัวนี้
    Serial.printf("⚠️ WARNING: Partial communication loss - Sensors [%s] offline\n", offline_sensors.c_str());
  } else {
    // ทุก sensor เชื่อมต่อปกติ -> ปิดเสียงเตือน
    if (buzzer_active && !siren_active) {
      stopBuzzer();
      stopSiren();
      Serial.println("✅ All sensors back online - Buzzer deactivated");
    }
  }
}

//🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨
void triggerSiren() {  
  // if (!siren_active) {
    siren_active = true;
    siren_start_time = millis();
    digitalWrite(SIREN_PIN19, LOW); //แจ้งเตือน Siren Active LOW
    Serial.println("🚨 SIREN ACTIVATED! 🚨");
    //ส่งข้อมูลไปยัง ESP ตัวที่ 2
    mySerial.println(getSystemStatus());   
    // ปิด buzzer เมื่อเปิดไซเรน
    if (buzzer_active) {
      stopBuzzer();
    }
  // }
}

void stopSiren() {
  if (siren_active) {
    siren_active = false;
    digitalWrite(SIREN_PIN19, HIGH);
    mySerial.println(getSystemStatus()); 
    Serial.println("✅ SIREN DEACTIVATED");
  }
}

void triggerBuzzer() {
  Serial.println("📢buzzer_active:"+(String)buzzer_active+"📢siren_active:"+(String)siren_active);
  if (!buzzer_active && !siren_active) {       
    buzzer_active = true;
    buzzer_start_time = millis();
    Serial.println("🎉 BUZZER ACTIVATED! 🎉");
  }
}

void stopBuzzer() {
  if (buzzer_active) {
    buzzer_active = false;
    digitalWrite(BUZZER_PIN18, LOW);
    mySerial.println(getSystemStatus()); 
    Serial.println("✅ BUZZER DEACTIVATED");
  }
}

void handleAlarms() {
  // จัดการไซเรน (เปิดไว้ 10 วินาที)    
  if (siren_active && ((millis() - siren_start_time > SIREN_TIMEOUT))) {    
    stopSiren();     
  }
}

// จัดการ buzzer BUZZER_PIN18 = 18
//📢📢📢📢📢📢📢📢📢📢📢
void handleComunicationAlarms(){    
  if (buzzer_active && !siren_active) {    
    mySerial.println(getSystemStatus());   
    digitalWrite(SIREN_PIN19, LOW);  //แจ้งเตือน Siren Active LOW
    // digitalWrite(BUZZER_PIN18, HIGH);  
    Serial.println("🚨 BUZZER ACTIVATED! 🚨");  
  }else{
    digitalWrite(SIREN_PIN19, HIGH);
    // digitalWrite(BUZZER_PIN18, LOW);
  }
}

void updateStatusLED() {
  // LED กระพริบแสดงสถานะ
  if (siren_active) {
    // กระพริบเร็วเมื่อไซเรนเปิด
    digitalWrite(LED_STATUS_PIN2, (millis() / 100) % 2);
  } else if (buzzer_active) {
    // กระพริบช้าเมื่อ buzzer เปิด
    digitalWrite(LED_STATUS_PIN2, (millis() / 300) % 2);
  } else {
    // เปิดค้างเมื่อทำงานปกติ
    digitalWrite(LED_STATUS_PIN2, HIGH);
  }
}

void testSounds() {
  Serial.println("Testing buzzer...");
  digitalWrite(BUZZER_PIN18, LOW);
  delay(200);
  digitalWrite(BUZZER_PIN18, HIGH);
  delay(200);
  
  Serial.println("Testing siren...");
  digitalWrite(SIREN_PIN19, LOW);
  delay(200);
  digitalWrite(SIREN_PIN19, HIGH);
  
  Serial.println("Sound test complete");
}

void receiveAndRespond() {
  if (mySerial.peek() == '$') {  // เช็ค header
    mySerial.read();  // ทิ้ง header
    String msg = mySerial.readStringUntil('\n');
    if (msg.length() > 0) {
      // Extract checksum
      char receivedChecksum = msg.charAt(msg.length() - 1);
      msg = msg.substring(0, msg.length() - 1);  // ตัด checksum

      // คำนวณ checksum
      byte calcChecksum = 0;
      for (int i = 0; i < msg.length(); i++) {
        calcChecksum += (byte)msg.charAt(i);
      }
      calcChecksum %= 256;   

      // if (calcChecksum == (byte)receivedChecksum) {
        if(msg.startsWith("READ")){
        Serial.println("Received valid: " + msg);  // Debug

        // ส่งอะไรก็ได้กลับ (กำหนดเอง)
        String response = getSystemStatus();  // ตัวอย่าง: ยืนยันด้วย "ACK:" + ข้อความที่ได้รับ
        // หรือรวม ESP-NOW: String response = "DATA:" + String(espNowData);
        // หรือคำสั่ง: String response = "TURN_ON_LED";
        sendData(response);
      } else {
        Serial.println("Checksum error!");
        sendData("RETRY");  // ขอให้ Slave ส่งซ้ำ
      }
    }
  } else {
    // Clear buffer ถ้าไม่มี header
    while (mySerial.available()) mySerial.read();
  }
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
}

String getSystemStatus(){
  system_status = "START\n";  // เริ่มด้วย marker;
  for (int i = 0; i < MAX_SENSORS; i++) {
    // String status_icon = sensors[i].is_online ? "🟢" : "🔴";
    String status_icon = sensors[i].is_online ? "🟢" : sensors[i].last_seen > 0 ? "🔴" : "⚫" ;
    String switch_icon = sensors[i].switch_state ? "☃️" : "🚨";
    String connection = sensors[i].is_online ? "ONLINE " : "OFFLINE";
    String switch_status = sensors[i].switch_state ? "ปกติ" : "ฉุกเฉิน  ";
    
    unsigned long time_since_last = sensors[i].last_seen > 0 ? (millis() - sensors[i].last_seen) / 1000 : 0;
    
    system_status += "Sensor " + String(i + 1) + ": " + 
                status_icon.c_str() + " " + connection.c_str() + " │ " + 
                switch_icon.c_str() + " " + switch_status.c_str() + " │ Last: " + 
                String(time_since_last) + " sec ago\n";
  }
  
  return system_status += "END\n";  // จบด้วย marker;
}

void printSystemStatus() {
  
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║           SYSTEM STATUS              ║");
  Serial.println("╚══════════════════════════════════════╝");
  
  int online_count = 0;
  int offline_count = 0;
  int open_count = 0;
  
  for (int i = 0; i < MAX_SENSORS; i++) {
    // String status_icon = sensors[i].is_online ? "🟢" : "🔴";
    String status_icon = sensors[i].is_online ? "🟢" : sensors[i].last_seen > 0  ? "🔴" : "⚫" ;
    String switch_icon = sensors[i].switch_state ? "☃️" : "🚨";
    String connection = sensors[i].is_online ? "ONLINE " : "OFFLINE";
    String switch_status = sensors[i].switch_state ? "ปกติ" : "ฉุกเฉิน  ";
    
    unsigned long time_since_last = sensors[i].last_seen > 0 ? (millis() - sensors[i].last_seen) / 1000 : 0;
    
    Serial.printf("Sensor %d: %s %s │ %s %s │ Last: %lu sec ago\n",
                  i + 1,
                  status_icon.c_str(), connection.c_str(),
                  switch_icon.c_str(), switch_status.c_str(),
                  time_since_last);   

    if (sensors[i].is_online) online_count++;
    else offline_count++;
    
    if (!sensors[i].switch_state) open_count++;
  } 
  
  
  Serial.println("─────────────────────────────────────────");
  Serial.printf("📊 Summary: %d Online │ %d Offline │ %d Switches Open\n", 
                online_count, offline_count, open_count);
  
  if (offline_count > 0) {
    Serial.printf("📡 Offline Sensors: [%s]\n", offline_sensors.c_str());
  }
  
  if (open_count > 0) {
    Serial.printf("🚨 Open Switches: [%s]\n", open_switches.c_str());
  }
  
  Serial.printf("🔊 Alarms: %s%s\n", 
                siren_active ? "SIREN " : "",
                buzzer_active ? "BUZZER " : "");
  
  Serial.println("═════════════════════════════════════════\n");
}
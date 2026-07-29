/*
Future BOX FB12 + ESP-01s
https://easyeda.com/editor#id=c7245d4e303c4946be1aa8df7e19e6da|8ff0f60a614a4b55ace3a2a8d3f7e9fd|8b1b3ca6d1164febb14547686424542b|36900463ee184503a31e06a342c9d34d
 */
  #include "ConfigManager.h"
  #include <ESP8266WiFi.h> 
  #include <espnow.h>   //สำหรับ ESP8266

// #define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา

#define CONFIG_BUTTON_PIN  0 // GPIO0: Input (LOW = Config Mode)
#define REED_SWITCH_PIN 3   // GPIO3 (RXD) สำหรับ reed switch
#define STATUS_LED_PIN 2    // GPIO2 สำหรับ LED สถานะ

// โครงสร้างข้อมูลที่ส่งไป Gateway
// 2. เพิ่มฟิลด์ checksum ไว้ท้ายสุดของ struct
typedef struct __attribute__((packed)) sensor_message {
  uint8_t sensor_id;      
  bool switch_status;     
  uint32_t timestamp;     
  uint32_t checksum;      // ใช้ตรวจสอบข้อมูล
} sensor_message;

// ตัวแปรสำหรับจัดการสถานะ
bool current_switch_state = true;
bool last_switch_state = true;
bool confirmed_switch_state = true;
unsigned long last_send_time = 0;
unsigned long last_heartbeat = 0;
unsigned long state_change_time = 0;
// ค่าตั้งเวลา
// const unsigned long SEND_INTERVAL = 1000;      // ส่งทุก 1 วินาที เมื่อมีการเปลี่ยนแปลง
const unsigned long HEARTBEAT_INTERVAL = random(5000, 10001); // ส่ง heartbeat ทุก 5-10 วินาที
const unsigned long DEBOUNCE_DELAY = 100;        // รอ 100ms เพื่อยืนยันสถานะ
const unsigned long CONFIRMATION_DELAY = 200;    // รอ 200ms ก่อนส่งข้อมูล

// ประกาศตัวแปร global สำหรับ LED Status
unsigned long led_blink_start = 0;
const unsigned long LED_BLINK_DURATION = 100; // กระพริบ 100ms

// เพิ่มในส่วนกำหนดตัวแปรด้านบน
#define RANDOM_DELAY_MIN 100    // หน่วงเวลาขั้นต่ำ (10ms)
#define RANDOM_DELAY_MAX 300  // หน่วงเวลาสูงสุด (30ms)

// 1. สร้าง Object ของ ConfigManager
ConfigManager cfgManager;
// 2. ตัวแปรสำหรับเก็บค่าที่ดึงมาจาก ConfigManager
DeviceConfig myConfig;

void setup() {
  #if defined(DEBUG)
  Serial.begin(115200);  
  #endif
  // เรียกใช้เมธอด begin()
  // begin() จะตรวจสอบสวิตช์และค่าใน EEPROM
  // ถ้าจำเป็น มันจะรัน Web Server และไม่กลับมาจนกว่าจะรีสตาร์ท  
  cfgManager.begin(CONFIG_BUTTON_PIN);  
  // เมื่อโค้ดมาถึงตรงนี้ หมายความว่า ESP ได้เข้าสู่ Normal Operation Mode แล้ว
  #if defined(DEBUG)
  Serial.println("เมื่อโค้ดมาถึงตรงนี้ หมายความว่า ESP ได้เข้าสู่ Normal Operation Mode แล้ว");
  #endif
  myConfig = cfgManager.getConfig();

  // *** สำคัญ: ห้ามใช้ Serial.begin() เพราะใช้ RXD เป็น GPIO ***
  
  // กำหนด PIN Mode
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);  // GPIO3 (RXD) สำหรับ reed switch
  pinMode(STATUS_LED_PIN, OUTPUT);         // GPIO2 สำหรับ LED
  pinMode(CONFIG_BUTTON_PIN, INPUT_PULLUP);
  
  // แสดงสถานะเริ่มต้น
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // เริ่มต้น WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // Serial.printf("Sensor ID: %d\n", SENSOR_ID);
  // Serial.println("MAC Address: " + WiFi.macAddress());
  // *** สำคัญ: ตั้งค่า Channel ให้เป็น Channel ที่กำหนดไว้ ***  
  // กำหนด Channel
  wifi_set_channel(myConfig.channel);
  // เริ่มต้น ESP-NOW
  if (esp_now_init() != 0) {
    #if defined(DEBUG)
    Serial.println("Error initializing ESP-NOW");
    #endif
    errorBlink();
    delay(5000);
    ESP.restart();
  }
  
  // กำหนดบทบาทเป็น Controller
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  // ลงทะเบียน callback สำหรับการส่งข้อมูล
  esp_now_register_send_cb(onDataSent);

  #if defined(DEBUG)
  Serial.print("ID = ");
  Serial.println(myConfig.deviceID);
  Serial.print("CHANNEL = ");
  Serial.println(myConfig.channel);
  Serial.print("MAC GW= ");
  Serial.printf("{0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X}\n", 
                myConfig.gatewayMAC[0], myConfig.gatewayMAC[1], myConfig.gatewayMAC[2], myConfig.gatewayMAC[3], myConfig.gatewayMAC[4], myConfig.gatewayMAC[5]);
  Serial.println();
  #endif

  
    // เพิ่ม Gateway เป็น Peer  
  if (esp_now_add_peer(myConfig.gatewayMAC, ESP_NOW_ROLE_SLAVE, myConfig.channel, NULL, 0) != 0) {
    #if defined(DEBUG)
    Serial.println("Failed to add Gateway as peer");
    #endif
    errorBlink();
  }
  
  // Serial.println("ESP-01 Sensor Ready");
  
  // อ่านสถานะเริ่มต้น
  last_switch_state = digitalRead(REED_SWITCH_PIN);
  
  // ส่งข้อมูลเริ่มต้น
  sendSensorData(true);
  
  // แสดงสถานะพร้อมทำงาน
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop() {

  bool switchPressed = (digitalRead(CONFIG_BUTTON_PIN) == LOW);
  if(switchPressed) ESP.restart(); // สั่งให้ ESP รีสตาร์ท;

  // อ่านสถานะ switch
  current_switch_state = digitalRead(REED_SWITCH_PIN);
  unsigned long current_time = millis();
  
   // ตรวจสอบการเปลี่ยนแปลงสถานะ
  if (current_switch_state != last_switch_state) {
    // บันทึกเวลาที่สถานะเปลี่ยน
    state_change_time = current_time;
    last_switch_state = current_switch_state;
  }

  // ตรวจสอบว่าสถานะคงที่มานานพอแล้วและผ่านช่วง debounce
  if (current_time - state_change_time > DEBOUNCE_DELAY) {
    // ถ้าสถานะที่ยืนยันแล้วไม่ตรงกับสถานะปัจจุบัน (หลัง debounce)
    if (confirmed_switch_state != last_switch_state) {
      // รอเพิ่มเติมเพื่อยืนยันว่าสถานะเปลี่ยนจริงๆ
      if (current_time - state_change_time > CONFIRMATION_DELAY) {
        // ส่งข้อมูลเมื่อยืนยันแล้วว่าสถานะเปลี่ยนจริงๆ
        confirmed_switch_state = last_switch_state;
        sendSensorData(true);
        last_send_time = current_time;        
        // กระพริบ LED เมื่อส่งข้อมูล
        // blinkStatusLED();
      }
    }
  }
  
  // สุ่มครั้งเดียวต่อ heartbeat
  if (current_time - last_heartbeat > HEARTBEAT_INTERVAL){
  // คำนวณ random() ทุก loop
  // if (current_time - last_heartbeat > random(RANDOM_DELAY_MIN, RANDOM_DELAY_MAX)) {
    
      sendSensorData(true);
    
    last_heartbeat = current_time;
    // กระพริบ LED เมื่อส่งข้อมูล
    // blinkStatusLED();
  }
  // ตรวจสอบว่าถึงเวลาปิด LED หรือยัง
  if(led_blink_start > 0 && millis() - led_blink_start >= LED_BLINK_DURATION) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    led_blink_start = 0; // reset
  }
  // เข้าสู่โหมดประหยัดไฟ
  //delay(100);
}

// 1. เพิ่มฟังก์ชันคำนวณ CRC32 (วางไว้ก่อนฟังก์ชัน sendSensorData)
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) bit = !bit;
      crc <<= 1;
      if (bit) crc ^= 0x04C11DB7;
    }
  }
  return crc;
}

void sendSensorData(bool is_heartbeat) {
  sensor_message msg;
  
  #if defined(DEBUG)
  msg.sensor_id = random(1, 11);
  #else
  msg.sensor_id = myConfig.deviceID; 
  #endif

  msg.switch_status = last_switch_state; //digitalRead(REED_SWITCH_PIN);
  msg.timestamp = millis();
  
  // 3. คำนวณ CRC32 จากเนื้อข้อมูลทั้งหมด (ยกเว้นช่อง checksum ตัวเอง)
  size_t dataSizeToCalculate = sizeof(msg) - sizeof(msg.checksum);
  msg.checksum = calculateCRC32((uint8_t*)&msg, dataSizeToCalculate);

  // ส่งข้อมูลขนาดเต็มที่มี checksum พ่วงไปด้วย
  esp_now_send(myConfig.gatewayMAC, (uint8_t *) &msg, sizeof(msg));
  
  #if defined(DEBUG)
  if (is_heartbeat) {
    Serial.printf("Heartbeat sent - Switch: %s\n", msg.switch_status ? " ✅ OPEN" : "🚨 CLOSED");
  } else {
    Serial.printf("Alert sent - Switch: %s\n", msg.switch_status ? " ✅ OPEN" : "🚨 CLOSED");
  }
  #endif
}

// Callback เมื่อส่งข้อมูลเสร็จ
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  // เปิด LED และบันทึกเวลา
  digitalWrite(STATUS_LED_PIN, LOW);
  led_blink_start = millis();

  #if defined(DEBUG)
  char macStr[18]; // array สำหรับเก็บสตริง MAC address
  
  // แปลง MAC address จากรูปแบบ uint8_t* ให้เป็นสตริงที่อ่านได้
  sprintf(macStr, "%02x:%02x:%02x:%02x:%02x:%02x",
          mac_addr[0], mac_addr[1], mac_addr[2], 
          mac_addr[3], mac_addr[4], mac_addr[5]);
  #endif
  if (sendStatus == 0) {
    #if defined(DEBUG)
    Serial.printf("Data sent successfully to MAC: %s\n", macStr);
    #endif
  } else {
    #if defined(DEBUG)
    Serial.println("Error sending data");
    #endif
    errorBlink();
  }
}

void blinkStatusLED() {  
  if(!confirmed_switch_state){
    digitalWrite(STATUS_LED_PIN, LOW);
  }
  for (int i = 0; i < 4; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(200);    
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(200);
  }
}

void errorBlink() {
  // กระพริบเร็วแสดงข้อผิดพลาด
  for (int i = 0; i < 10; i++) {
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
  }
}
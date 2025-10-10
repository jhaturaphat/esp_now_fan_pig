/*
 * ESP-01 Sensor Code - ส่งสัญญาณสถานะ Switch แม่เหล็กไปยัง Gateway
 * ใช้ ESP-NOW Protocol สำหรับการสื่อสาร
 * 
 * การต่อวงจร ESP-01 (วิธีที่ 4 - ใช้ RXD):
 * - GPIO3 (RXD): Reed Switch (Pull-up ภายใน)
 * - GPIO2: LED สถานะ (Optional)
 * - GPIO0: ปล่อยว่าง หรือ Pull-up 10kΩ ไป VCC (เพื่อ boot ปกติ)
 * - VCC: 3.3V
 * - GND: Ground
 * 
 * วงจร Reed Switch:
 * GPIO3 (RXD) ---|Reed Switch|--- GND
 * 
 * *** สำคัญ: 
 * 1. ต้องเปลี่ยน SENSOR_ID ในแต่ละตัวให้ต่างกัน (1-7)
 * 2. ปิด Serial.begin() เพราะใช้ RXD เป็น GPIO
 * 3. ห้ามใช้ Serial.print() ใดๆ
 * 4. RXD สะอาดกว่า TXD (ไม่มี boot message)
 */

#include <ESP8266WiFi.h>
#include <espnow.h>   //สำหรับ ESP8266

// กำหนดค่าสำคัญ - *** เปลี่ยนในแต่ละ sensor ***
#define SENSOR_ID 1  // เปลี่ยนเป็น 1,2,3,4,5,6,7 ในแต่ละตัว

// MAC Address ของ Gateway ESP32 - *** ต้องใส่ MAC จริงของ Gateway ***
uint8_t gateway_mac[] = {0x24, 0xD7, 0xEB, 0x0E, 0xF1, 0xFC}; // เปลี่ยนเป็น MAC จริง

// กำหนด PIN - ใช้ RXD (GPIO3)
#define REED_SWITCH_PIN 3   // GPIO3 (RXD) สำหรับ reed switch
#define STATUS_LED_PIN 2    // GPIO2 สำหรับ LED สถานะ

// โครงสร้างข้อมูลที่ส่งไป Gateway
typedef struct sensor_message {
  uint8_t sensor_id;      // ID ของ sensor
  bool switch_status;     // สถานะ switch (true=closed, false=open)  
  uint32_t timestamp;     // timestamp
} sensor_message;

// ตัวแปรสำหรับจัดการสถานะ
bool last_switch_state = true;
bool confirmed_switch_state = true;
unsigned long last_send_time = 0;
unsigned long last_heartbeat = 0;
unsigned long state_change_time = 0;

// ค่าตั้งเวลา
const unsigned long DEBOUNCE_DELAY = 200;        // รอ 200ms เพื่อยืนยันสถานะ
const unsigned long SEND_INTERVAL = 1000;        // ส่งซ้ำได้ทุก 1 วินาที
const unsigned long HEARTBEAT_INTERVAL = 10000;  // ส่ง heartbeat ทุก 10 วินาที
const unsigned long CONFIRMATION_DELAY = 500;    // รอ 500ms ก่อนส่งข้อมูล

void setup() {
  // *** สำคัญ: ห้ามใช้ Serial.begin() เพราะใช้ RXD เป็น GPIO ***
  
  // กำหนด PIN Mode
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);  // GPIO3 (RXD) สำหรับ reed switch
  pinMode(STATUS_LED_PIN, OUTPUT);         // GPIO2 สำหรับ LED
  
  // แสดงสถานะเริ่มต้น
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // เริ่มต้น WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // เริ่มต้น ESP-NOW
  if (esp_now_init() != 0) {
    errorBlink();
    ESP.restart();
  }
  
  // กำหนดบทบาทเป็น Controller
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  // ลงทะเบียน callback สำหรับการส่งข้อมูล
  esp_now_register_send_cb(onDataSent);
  
  // เพิ่ม Gateway เป็น Peer
  if (esp_now_add_peer(gateway_mac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != 0) {
    errorBlink();
  }
  
  // อ่านสถานะเริ่มต้น
  last_switch_state = digitalRead(REED_SWITCH_PIN);
  confirmed_switch_state = last_switch_state;
  
  // ส่งข้อมูลเริ่มต้นหลังจากรอสักครู่
  delay(1000);
  sendSensorData(true);
  
  // แสดงสถานะพร้อมทำงาน
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop() {
  // อ่านสถานะ switch
  bool current_switch_state = digitalRead(REED_SWITCH_PIN);
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
        sendSensorData(false);
        last_send_time = current_time;
        
        // กระพริบ LED เมื่อส่งข้อมูล
        blinkStatusLED();
      }
    }
  }
  
  // ส่ง Heartbeat ทุก 10 วินาที
  if (current_time - last_heartbeat > HEARTBEAT_INTERVAL) {
    sendSensorData(true);
    last_heartbeat = current_time;
    blinkStatusLED();
  }
  
  // เข้าสู่โหมดประหยัดไฟ
  delay(100);
}

void sendSensorData(bool is_heartbeat) {
  sensor_message msg;
  msg.sensor_id = SENSOR_ID;
  msg.switch_status = confirmed_switch_state;  // ใช้สถานะที่ยืนยันแล้ว
  msg.timestamp = millis();
  
  // ส่งข้อมูล
  esp_now_send(gateway_mac, (uint8_t *) &msg, sizeof(msg));
}

// Callback เมื่อส่งข้อมูลเสร็จ
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus != 0) {
    errorBlink();
  }
}

void blinkStatusLED() {
  for (int i = 0; i < 3; i++) {
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
  }
}

void errorBlink() {
  // กระพริบเร็วแสดงข้อผิดพลาด
  for (int i = 0; i < 10; i++) {
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(50);
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(50);
  }
}

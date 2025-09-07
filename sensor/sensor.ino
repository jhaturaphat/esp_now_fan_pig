/*
 * ESP-01 Sensor Code - ส่งสัญญาณสถานะ Switch แม่เหล็กไปยัง Gateway
 * ใช้ ESP-NOW Protocol สำหรับการสื่อสาร
 * 
 * การต่อวงจร ESP-01:
 * - GPIO0: Reed Switch (Pull-up ภายใน)
 * - GPIO2: LED สถานะ (Optional)
 * - VCC: 3.3V
 * - GND: Ground
 * 
 * *** สำคัญ: ต้องเปลี่ยน SENSOR_ID ในแต่ละตัวให้ต่างกัน (1-7) ***
 */

#include <ESP8266WiFi.h>
#include <espnow.h>

// กำหนดค่าสำคัญ - *** เปลี่ยนในแต่ละ sensor ***
#define SENSOR_ID 4  // เปลี่ยนเป็น 1,2,3,4,5,6,7 ในแต่ละตัว

// MAC Address ของ Gateway ESP32 - *** ต้องใส่ MAC จริงของ Gateway *** 24:d7:eb:0e:f1:fc
uint8_t gateway_mac[] = {0x24, 0xD7, 0xEB, 0x0E, 0xF1, 0xFC}; // เปลี่ยนเป็น MAC จริง

// กำหนด PIN
#define REED_SWITCH_PIN 0   // GPIO0 สำหรับ reed switch
#define STATUS_LED_PIN 2    // GPIO2 สำหรับ LED สถานะ

// โครงสร้างข้อมูลที่ส่งไป Gateway
typedef struct sensor_message {
  uint8_t sensor_id;      // ID ของ sensor
  bool switch_status;     // สถานะ switch (true=closed, false=open)  
  uint32_t timestamp;     // timestamp
} sensor_message;

// ตัวแปรสำหรับจัดการสถานะ
bool last_switch_state = true;
unsigned long last_send_time = 0;
unsigned long last_heartbeat = 0;
const unsigned long SEND_INTERVAL = 1000;      // ส่งทุก 1 วินาที เมื่อมีการเปลี่ยนแปลง
const unsigned long HEARTBEAT_INTERVAL = 10000; // ส่ง heartbeat ทุก 10 วินาที

void setup() {
  // Serial.begin(115200);
  
  // กำหนด PIN Mode
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);  // ใช้ Pull-up ภายใน
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // แสดงสถานะเริ่มต้น
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // เริ่มต้น WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  // Serial.printf("Sensor ID: %d\n", SENSOR_ID);
  // Serial.println("MAC Address: " + WiFi.macAddress());
  
  // เริ่มต้น ESP-NOW
  if (esp_now_init() != 0) {
    // Serial.println("Error initializing ESP-NOW");
    errorBlink();
    ESP.restart();
  }
  
  // กำหนดบทบาทเป็น Controller
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  
  // ลงทะเบียน callback สำหรับการส่งข้อมูล
  esp_now_register_send_cb(onDataSent);
  
  // เพิ่ม Gateway เป็น Peer
  if (esp_now_add_peer(gateway_mac, ESP_NOW_ROLE_SLAVE, 1, NULL, 0) != 0) {
    // Serial.println("Failed to add Gateway as peer");
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
  // อ่านสถานะ switch
  bool current_switch_state = digitalRead(REED_SWITCH_PIN);
  unsigned long current_time = millis();
  
  // ตรวจสอบการเปลี่ยนแปลงสถานะ
  if (current_switch_state != last_switch_state) {
    // Serial.printf("Switch state changed: %s\n", 
    //               current_switch_state ? "CLOSED" : "OPEN");
    
    // ส่งข้อมูลทันทีเมื่อมีการเปลี่ยนแปลง
    sendSensorData(false);
    last_switch_state = current_switch_state;
    last_send_time = current_time;
    
    // กระพริบ LED เมื่อส่งข้อมูล
    blinkStatusLED();
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
  msg.switch_status = digitalRead(REED_SWITCH_PIN);
  msg.timestamp = millis();
  
  // ส่งข้อมูล
  esp_now_send(gateway_mac, (uint8_t *) &msg, sizeof(msg));
  
  // if (is_heartbeat) {
  //   Serial.printf("Heartbeat sent - Switch: %s\n", 
  //                msg.switch_status ? "CLOSED" : "OPEN");
  // } else {
  //   Serial.printf("Alert sent - Switch: %s\n", 
  //                 msg.switch_status ? "CLOSED" : "OPEN");
  // }
}

// Callback เมื่อส่งข้อมูลเสร็จ
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  if (sendStatus == 0) {
    //Serial.println("Data sent successfully");
  } else {
    //Serial.println("Error sending data");
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
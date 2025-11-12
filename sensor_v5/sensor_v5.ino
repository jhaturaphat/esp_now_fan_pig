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
  #include "ConfigManager.h"
  #include <ESP8266WiFi.h> 
  #include <espnow.h>   //สำหรับ ESP8266

// #define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา

#define CONFIG_BUTTON_PIN  0 // GPIO0: Input (LOW = Config Mode)
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
const unsigned long SEND_INTERVAL = 1000;      // ส่งทุก 1 วินาที เมื่อมีการเปลี่ยนแปลง
const unsigned long HEARTBEAT_INTERVAL = random(500, 10000); // ส่ง heartbeat ทุก 5-10 วินาที
const unsigned long DEBOUNCE_DELAY = 200;        // รอ 200ms เพื่อยืนยันสถานะ
const unsigned long CONFIRMATION_DELAY = 500;    // รอ 500ms ก่อนส่งข้อมูล

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
  delay(5000); //รอผู้ใช้กดปุ่มเพื่อตั้งค่า
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
  // if (current_time - last_heartbeat > HEARTBEAT_INTERVAL)
  if (current_time - last_heartbeat > random(RANDOM_DELAY_MIN, RANDOM_DELAY_MAX)) {
    sendSensorData(true);
    last_heartbeat = current_time;
    // กระพริบ LED เมื่อส่งข้อมูล
    blinkStatusLED();
  }
  
  // เข้าสู่โหมดประหยัดไฟ
  delay(100);
}

void sendSensorData(bool is_heartbeat) {
  sensor_message msg;
  // จำลองเซ็นเซอร์ 1-10 ตัว
  #if defined(DEBUG)
  msg.sensor_id = random(1, 11);
  #else
  msg.sensor_id = myConfig.deviceID; 
  #endif

  msg.switch_status = digitalRead(REED_SWITCH_PIN);
  msg.timestamp = millis();
  
  // เพิ่มการหน่วงเวลาแบบสุ่มก่อนส่งข้อมูล
  // delay(random(RANDOM_DELAY_MIN, RANDOM_DELAY_MAX));
  // ส่งข้อมูล
  esp_now_send(myConfig.gatewayMAC, (uint8_t *) &msg, sizeof(msg));
  #if defined(DEBUG)
  if (is_heartbeat) {
    Serial.printf("Heartbeat sent - Switch: %s\n", 
                 msg.switch_status ? " ✅ OPEN" : "🚨 CLOSED");
  } else {
    Serial.printf("Alert sent - Switch: %s\n", 
                  msg.switch_status ? " ✅ OPEN" : "🚨 CLOSED");
  }
  #endif
}

// Callback เมื่อส่งข้อมูลเสร็จ
void onDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
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
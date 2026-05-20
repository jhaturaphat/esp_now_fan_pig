#include "ConfigManager.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

// #define DEBUG  // เอาคอมเมนต์ออกเมิ่ออยู่ในโหมด DEBUG

// ESP32 Relay x2 Module
// #define KID_BUG_PIN 34 //สำหรับป้องกันโปรแกรม
#define RELAY1_PIN 17  //out put Active LOW
#define RELAY2_PIN 16  //out put Active LOW
#define LED_STATUS 23  //out put Active HIGH
#define CONFIG_PIN 0  //input pulll up
#define TEST_PIN 32 //input pulll up
#define TEST_PIN_SERIAL 18 //input pulll up
#define DISABLE_SIREN 25  //input pulll up
#define AC220_LOSS 27  //input pull up
#define RXD2 21  //สีส้ม
#define TXD2 19  //ขาว-ส้ม



#define COMMUNICATION_TIMEOUT 30000  // 30 วินาที timeout 
#define TIMEOUT_SIREN 10000 // 10 วินาที timeout

// Delay
int period = 5000; // 10 วินาที
unsigned long time_now = 0;
const unsigned long SIREN_DURATION = 5000; // 5 วินาที
unsigned long PERIOD_LOSS = 0; // ตัวแปรสำหรับเก็บเวลาที่ทำการอัปเดตสถานะ LED ครั้งล่าสุด
const unsigned long LOSS_DURATION = 5000;  // 5 วินาที
unsigned long PERIOD_SIREN = 0; // ตัวแปรสำหรับเก็บเวลาที่ทำการอัปเดตสถานะ LED ครั้งล่าสุด
unsigned long PERIOD_SIREN2 = 0; 
bool handleAlarm = false; 
bool reload = true;
// ประกาศตัวแปร global สำหรับ LED Status
unsigned long led_blink_start = 0;
const unsigned long LED_BLINK_DURATION = 100; // กระพริบ 100ms
String static_mac_address = "";
StaticJsonDocument<1024> doc; //ประกาศ JsonDocument ไว้ข้างนอก

// --- Switch PIN
bool disableSiren = false;
bool ac220_loss = false;  //สถานะไฟฟ้า 220V อ่านค่ามาจาก relay 220V

// เก็บสถานะของแต่ละ sensor
struct sensor_storage {  
  bool is_online;           // สถานะการเชื่อมต่อ
  bool switch_state;        // สถานะ switch
  unsigned long last_seen;  // เวลาที่รับสัญญาณครั้งล่าสุด
  uint8_t mac[6];          // MAC address ของ sensor
};

// โครงสร้างข้อมูลที่รับจาก Sensor
typedef struct sensor_message {
  uint8_t sensor_id;      // ID ของ sensor (1-7)
  bool switch_status;     // สถานะ switch (true=ปกติ, false=แจ้งเตือน)
  uint32_t timestamp;     // timestamp
} sensor_message;

// เก็บรายการ sensor ที่มีปัญหา
String offline_sensors = "";
bool sensor_has_alarm = false;
int offline_count = 0; 
int alarm_count = 0;
// ตัวแปร Timeout
unsigned long sirenStartTime = 0;
// bool sirenOn  = false; // สถานะไซเรนว่ากำลังดังอยู่หรือไม่

// 1. สร้าง Object ของ ConfigManager
ConfigManager cfgManager;
// 2. ตัวแปรสำหรับเก็บค่าที่ดึงมาจาก ConfigManager
DeviceConfig myConfig;
sensor_storage* sensors_storage = nullptr;  // ใช้ pointer แทน array
int MAX_SENSORS = 10;  // เก็บค่าที่อ่านจาก EEPROM

// Callback สำหรับรับข้อมูล (สำหรับ ESP32 Arduino Core 3.x)
void onDataReceive(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len){
  // เปิด LED และบันทึกเวลา
  digitalWrite(LED_STATUS, HIGH);
  led_blink_start = millis();
  // สร้าง object sensor
  sensor_message msg;
  memcpy(&msg, incomingData, sizeof(msg));

  //  หาก sensor ไม่ใช้ 0 และ จำนวนเซ็นเซอร์เกินจำนวนที่ระบบบน MAX_SENSORS
  if(msg.sensor_id >= 1 && msg.sensor_id <= MAX_SENSORS){
    int index = msg.sensor_id - 1;
    // อัพเดทสถานะเซ็นเซอร์ ลงไปยัง sensors_storage    
    sensors_storage[index].is_online = true;
    sensors_storage[index].switch_state = msg.switch_status;
    sensors_storage[index].last_seen = millis();
    memcpy(sensors_storage[index].mac, recv_info->src_addr, 6); 
  }
  
} //End

void checkSensorsCommunication(){
  offline_count = 0;  
  for(int i = 0; i < MAX_SENSORS; i++){
    // ตรวจสอบการสื่อสาร
    if(sensors_storage[i].last_seen > 0){ // sensor เคยส่งข้อมูลมาแล้ว
      if(millis() - sensors_storage[i].last_seen > COMMUNICATION_TIMEOUT){
        offline_count++;
          if(sensors_storage[i].is_online){
          // ตรวจสอบอีกว่า เคย Online มาก่อนหน้านี้ก็กำหนดสถานะใหม่เป็น Offline โดยใช้ !sensors_storage[i].is_online
          sensors_storage[i].is_online = !sensors_storage[i].is_online;
          sensors_storage[i].switch_state = true;
          #if defined(DEBUG)
          Serial.printf("⚠️ WARNING: Lost communication with Sensor %d\n", i + 1);
          #endif          
        }
      }
    }
  }
}

// สร้าง Array ตามจำนวน MAXSENSOR เพื่อเก็บค่า แต้ละเซ็นเซอร์ไว้ที่ตำแหน่งต่างๆตาม index
void initializeSensorStorage() {
  // Allocate memory
  sensors_storage = new sensor_storage[MAX_SENSORS];

  for (int i = 0; i < MAX_SENSORS; i++) {
    sensors_storage[i].is_online = false;
    sensors_storage[i].switch_state = true;  // เริ่มต้นเป็น closed
    sensors_storage[i].last_seen = 0;
    memset(sensors_storage[i].mac, 0, 6);
  }
}

void CheckHashAlarm(){
  alarm_count = 0;
  for(int i = 0; i < MAX_SENSORS; i++){
    if(sensors_storage[i].is_online && (sensors_storage[i].last_seen > 0)){
      if(sensors_storage[i].switch_state == LOW){
        alarm_count++;
      }
    }
  }
}

/*void sendSensorsData(){
  StaticJsonDocument<1024> doc;  
  // สร้าง array ของ sensors
  JsonArray sensors = doc.createNestedArray("sensors");
  
  for(int i = 0; i < MAX_SENSORS; i++){
    JsonObject obj = sensors.createNestedObject();
    obj["id"] = i + 1;
    obj["online"] = sensors_storage[i].is_online;
    obj["switch"] = sensors_storage[i].switch_state;
    obj["uptime"] = sensors_storage[i].last_seen / 1000;    
  }
  doc["ac_loss"] = ac220_loss;   // ค่า off line = true | on line = false
  doc["alarm_count"] = alarm_count;
  doc["offline_count"] = offline_count;
  doc["mac"] = static_mac_address;
  doc["ch"] = WiFi.channel();
  
  serializeJson(doc, Serial2);
  Serial2.print("\n");
}*/

// Up speed function sendSensorsData 
void sendSensorsData(){
  // เคลียร์ข้อมูลเก่าออก (เร็วมากเพราะไม่ต้องจองพื้นที่เมมโมรี่ใหม่)
  doc.clear(); 
  
  // สร้าง Array ของ sensors
  JsonArray sensors = doc.createNestedArray("sensors");
  
  for(int i = 0; i < MAX_SENSORS; i++){
    JsonObject obj = sensors.createNestedObject();
    obj["id"] = i + 1;
    obj["online"] = sensors_storage[i].is_online;
    obj["switch"] = sensors_storage[i].switch_state;
    // ใช้การเลื่อนบิต (Bit Shift) หรือหารตรงๆ 
    // ถ้าไม่คิดมากการหารด้วย 1000 ปกติก็ยอมรับได้ครับ
    obj["uptime"] = sensors_storage[i].last_seen / 1000;    
  }
  
  doc["ac_loss"] = ac220_loss;   
  doc["alarm_count"] = alarm_count;
  doc["offline_count"] = offline_count;
  
  // ดึงค่า MAC จากตัวแปรที่อ่านไว้แล้ว (จุดนี้จะทำให้โค้ดเร็วขึ้นอย่างเห็นได้ชัด)
  doc["mac"] = static_mac_address;
  doc["ch"] = WiFi.channel();
  
  // ส่งข้อมูลออกทาง Serial2
  serializeJson(doc, Serial2);
  Serial2.print("\n");
}

String getInterfaceMacAddress(esp_mac_type_t interface) {

    String mac = "";
    unsigned char mac_base[6] = {0};
    if (esp_read_mac(mac_base, interface) == ESP_OK) {
        char buffer[18];  // 6*2 characters for hex + 5 characters for colons + 1 character for null terminator
        sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
        mac = buffer;
    }
    return mac;
}

void blinkLED(){
  for(int i = 0; i < 10; i++){
    digitalWrite(LED_STATUS, HIGH);
    delay(100);
    digitalWrite(LED_STATUS, LOW);
    delay(100);
  }
}

#if defined(DEBUG)
void printDebugSensorStatus(){
  yield();
  if (millis() >= time_now + period){
    time_now += period;
 
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║           SYSTEM STATUS              ║");
    Serial.println("╚══════════════════════════════════════╝");
    for (int i = 0; i < MAX_SENSORS; i++) {
      String status_icon = sensors_storage[i].is_online ? "🟢" : sensors_storage[i].last_seen > 0  ? "🔴" : "⚫" ;
      String switch_icon = sensors_storage[i].switch_state ? "☃️" : "🚨";
      String connection = sensors_storage[i].is_online ? "ONLINE " : "OFFLINE";
      String switch_status = sensors_storage[i].switch_state ? "ปกติ" : "ฉุกเฉิน  ";
      unsigned long time_since_last = sensors_storage[i].last_seen > 0 ? (millis() - sensors_storage[i].last_seen)  : 0;

      Serial.printf("Sensor %d: %s %s │ %s %s │ Last: %lu sec ago\n",
                    i + 1,
                    status_icon.c_str(), connection.c_str(),
                    switch_icon.c_str(), switch_status.c_str(),
                    time_since_last);  

    }
  }
}
#endif

void setup() {
  #if defined(DEBUG)
  Serial.begin(115200);
  #endif
  // Serial2.begin(9600, SERIAL_8N1, RX, TX);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  // pinMode(KID_BUG_PIN, INPUT_PULLUP);
  pinMode(AC220_LOSS, INPUT);
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED_STATUS, OUTPUT); 
  // pinMode(CONFIG_PIN, INPUT_PULLUP);
  pinMode(TEST_PIN, INPUT_PULLUP);
  pinMode(TEST_PIN_SERIAL, INPUT_PULLUP);
  pinMode(DISABLE_SIREN, INPUT_PULLUP);

  cfgManager.begin(CONFIG_PIN); 
  #if defined(DEBUG)
  Serial.println("เมื่อโค้ดมาถึงตรงนี้ หมายความว่า ESP ได้เข้าสู่ Normal Operation Mode แล้ว");
  #endif
  myConfig = cfgManager.getConfig();

  // ตั้งค่า WiFi Mode
  WiFi.mode(WIFI_STA);
  // -------------------------------------------------------------------------------
  // เปลี่ยน MAC Address (ต้องทำก่อน esp_now_init)  
  //  esp_wifi_set_mac(WIFI_IF_STA, (uint8_t[]){0x1C, 0x69, 0x20, 0x9B, 0x60, 0x48});
   esp_wifi_set_mac(WIFI_IF_STA, myConfig.virtualMac);
  // -------------------------------------------------------------------------------
  WiFi.setSleep(false);  // ป้องกัน WiFi sleep สำหรับ ESP-NOW
  WiFi.macAddress(myConfig.virtualMac); // กำหนด Mac address ใหม่
  static_mac_address = WiFi.macAddress(); // อ่านค่า MAC
  
  MAX_SENSORS = myConfig.sensor;
  // กำหนด Channel
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(myConfig.channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // เริ่มต้น ESP-NOW
  if (esp_now_init() != ESP_OK) {
    #if defined(DEBUG)
    Serial.println("Error initializing ESP-NOW");
    #endif
    return;
  }

  // ลงทะเบียน callback สำหรับรับข้อมูล
  esp_now_register_recv_cb(onDataReceive);

  #if defined(DEBUG)
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
  #endif
  // สร้าง Array ตามจำนวน MAXSENSOR เพื่อเก็บค่า แต่ละเซ็นเซอร์ไว้ที่ตำแหน่งต่างๆตาม index
  initializeSensorStorage();

  // if(digitalRead(KID_BUG_PIN) != LOW){  
  //   #if defined(DEBUG)  
  //   Serial.print("ป้องกัน Code Protection ต่อขานี้ลงกร์าว");    
  //   #endif
  //   ESP.restart();
  // }

  blinkLED();
  //Relay cctive HIGH
  digitalWrite(RELAY1_PIN, HIGH);  
  digitalWrite(RELAY2_PIN, HIGH);
  delay(100);
  digitalWrite(RELAY1_PIN, LOW); 
  digitalWrite(RELAY2_PIN, LOW);  
  

}

void test_gw(){
  // TEST_PIN
  if((digitalRead(TEST_PIN) == LOW) || (digitalRead(TEST_PIN_SERIAL) == LOW)) {
    sendSensorsData(); 
  }
}

// bool disable_siren(){  
//   if(digitalRead(DISABLE_SIREN) == LOW){
//     delay(10);
//     return true;
//   }else{
//     return false;
//   }
// }


void loop() {  
  //Relay cctive HIGH
  disableSiren  = (digitalRead(DISABLE_SIREN) == LOW);
  ac220_loss = (digitalRead(AC220_LOSS) == LOW); // เก็บค่าจาก relay 200V ac 
  #if defined(DEBUG)
  if(disableSiren){
    Serial.print("Switch Disable Siren State = ");
    Serial.println(disableSiren);
  }
  #endif

  checkSensorsCommunication();
  // ----------------------------------------------------------------------------------
  CheckHashAlarm();
  // ----------------------------------------------------------------------------------
  test_gw();
  // ----------------------------------------------------------------------------------
  // ตรวจสอบว่าถึงเวลาปิด LED หรือยัง
  if(led_blink_start > 0 && millis() - led_blink_start >= LED_BLINK_DURATION) {
    digitalWrite(LED_STATUS, LOW);
    led_blink_start = 0; // reset
  }
  // ----------------------------------------------------------------------------------  
    if((alarm_count > 0) || (offline_count > 0) || ac220_loss){
    if(disableSiren){ //หากต้องการปิด siren
      digitalWrite(RELAY1_PIN, LOW);
      digitalWrite(RELAY2_PIN, LOW);
      #if defined(DEBUG)
      Serial.println("Relay OFF");
      #endif
    }else{
      digitalWrite(RELAY1_PIN, HIGH);
      digitalWrite(RELAY2_PIN, HIGH);
      #if defined(DEBUG)
      Serial.println("Relay Active🚨");
      #endif       
    }

    if (millis() - PERIOD_SIREN >= SIREN_DURATION) {      
      PERIOD_SIREN = millis();      
      #if defined(DEBUG)
      Serial.printf("แจ้งเตือน %d 🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨\n", alarm_count);
      #endif  
      handleAlarm = true;
      sendSensorsData(); 
    }

  }else{
    // ส่งแจ้งสถานะปกติ 1 ครั้ง หลังจากมี Alarm เกิดขึ้น
    if(millis() - PERIOD_SIREN2 >= SIREN_DURATION){
      PERIOD_SIREN2 = millis(); 
      if(handleAlarm && (alarm_count == 0)){
        handleAlarm = !handleAlarm;
        sendSensorsData(); 
      }
      if(reload){
        reload = !reload;
        sendSensorsData(); 
      }
    }
    digitalWrite(RELAY1_PIN, LOW);     
    digitalWrite(RELAY2_PIN, LOW);     
  }

  // ---------------------------------------------------------------------------------
  // Relay 2 มีเซ็นเซอร์บางตัว offline ใช้สัญญาณไฟ
  // if(offline_count > 0 ){      
  //   if (millis() - PERIOD_LOSS >= LOSS_DURATION) {
  //     PERIOD_LOSS = millis();
  //     sendSensorsData();
  //     #if defined(DEBUG)
  //     Serial.printf("มีเซ็นเซอร์จำนวน %d Offline 📵📵📵📵📵📵📵📵📵📵📵📵📵📵📵📵📵📵📵\n" ,offline_count);
  //     #endif      
  //   }    
  // }
  // ---------------------------------------------------------------------------------
  #if defined(DEBUG) 
    printDebugSensorStatus(); 
  #endif  
}

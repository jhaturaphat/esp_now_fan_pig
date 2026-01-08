#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200); // สำหรับ Monitor บนคอมพิวเตอร์
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // สำหรับคุยกับ SIM900A
  
  delay(3000);
  Serial.println("Testing SIM900A...");
  
  // ทดสอบการเชื่อมต่อ
  Serial2.println("ATD0813907061;"); 
}

void loop() {
  // อ่านจาก SIM900A มาโชว์ที่ Serial Monitor
  if (Serial2.available()) {
    Serial.write(Serial2.read());
  }
  // อ่านจาก Serial Monitor ส่งไป SIM900A
  if (Serial.available()) {
    Serial2.write(Serial.read());
  }
}

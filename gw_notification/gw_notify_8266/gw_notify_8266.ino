#include <Arduino.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "WifiManager.h"

// สร้าง Object ของ Server ที่พอร์ต 80 รอไว้
AsyncWebServer server(80);

// ส่ง Address ของ server เข้าไปในคลาส Manager
WifiManager wifiManager(&server);

void setup() {
    Serial.begin(115200);

    // เรียกฟังก์ชันจัดการ Wi-Fi (ถ้าต่อไม่ได้ ระบบจะเปิดเว็บ Config หน้า 192.168.4.1 ค้างไว้รอเรา)
    wifiManager.begin();

    // --- ส่วนนี้คือหน้าเว็บโปรเจกต์ปกติของคุณ (ทำงานเมื่อ ESP-01 ต่อ Wi-Fi บ้านสำเร็จแล้วเท่านั้น) ---
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Welcome to your Main Project Page on ESP-01!");
    });

    // เริ่มทำงาน Server โหมดปกติ
    server.end(); // สั่งปิดระบบของโหมด Config ก่อนหน้า (ถ้ามี)
    server.begin();
    Serial.println("[Main] System is running normally.");
}

void loop() {
    // ว่างไว้ได้เลย
}

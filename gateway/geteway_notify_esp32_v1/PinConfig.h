

// #define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา
//GPIO สำหรับ ESP32-Dev-Module, ESP-32U
#define KID_BUG_PIN 34 //สำหรับป้องกันโปรแกรม
#define RELAY1_PIN 16  //out put Active LOW
#define RELAY2_PIN 17  //out put Active LOW
#define LED_STATUS 19  //out put Active HIGH
// #define CONFIG_PIN 0  //input pulll up
#define CONFIG_PIN 0  //for Board 2xRlay
#define TEST_PIN 23 //input pulll up
#define TEST_PIN_SERIAL 26 //input pulll up
#define DISABLE_SIREN 25  //input pulll up

// GPIO สำหรับ ESP32-S3-WROOM-1U-N8R2
// Board ESP32S3 Dev Module
// #define KID_BUG_PIN 6 //สำหรับป้องกันโปรแกรม
// #define RELAY1_PIN 36  //out put Active LOW
// #define RELAY2_PIN 37  //out put Active LOW
// #define LED_STATUS 39  //out put Active HIGH
// #define CONFIG_PIN 0  //input pulll up
// #define TEST_PIN 1 //input pulll up
// #define TEST_PIN_SERIAL 18 //input pulll up
// #define DISABLE_SIREN 17  //input pulll up

// กำหนดค่าให้สลับกันกับ ตัวส่ง 
#define RXD2 33
#define TXD2 32
// HardwareSerial Serial2(2); 
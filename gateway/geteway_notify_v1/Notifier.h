#ifndef NOTIFIER_H
#define NOTIFIER_H

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>

class Notifier {
private:
  String discord_webhook;
  String telegram_token;
  String telegram_chat_id;
  String ntfy_server;
  String ntfy_topic;
  
  bool discord_enabled;
  bool telegram_enabled;
  bool ntfy_enabled;
  
  String createMessage(String jsonString);
  String createDiscordMessage(String jsonString);
  String createTelegramMessage(String jsonString);
  String createNtfyMessage(String jsonString);
  
public:
  Notifier();
  
  // ตั้งค่า Discord
  void setupDiscord(String webhook);
  
  // ตั้งค่า Telegram
  void setupTelegram(String token, String chat_id);
  
  // ตั้งค่า Ntfy
  void setupNtfy(String server);
  
  // ส่งข้อความไปทั้งหมดที่เปิดใช้งาน
  void sendAll(String jsonString);
  
  // ส่งแยกแต่ละ platform
  bool sendDiscord(String jsonString);
  bool sendTelegram(String jsonString);
  bool sendNtfy(String jsonString);
  
  // เปิด/ปิดการใช้งาน
  void enableDiscord(bool enable);
  void enableTelegram(bool enable);
  void enableNtfy(bool enable);
};

#endif

#ifndef NOTIFYING_H
#define NOTIFYING_H
#include <Arduino.h>
#include "WifiManager.h"  // ใช้ NotifyConfig เดิม ไม่ประกาศซ้ำ
class Notifying {
public:
  bool send(const String& message, const NotifyConfig& cfg);
  String createNtfyMessage(const String& jsonString, const NotifyConfig& cfg);
  String createDiscordMessage(String jsonString, const NotifyConfig& cfg);
  String createTelegramMessage(String jsonString, const NotifyConfig& cfg);
private:
  static String jsonEscape(const String& s);
};
#endif

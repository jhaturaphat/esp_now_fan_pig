#include "Notifying.h"
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>

String Notifying::jsonEscape(const String& s) {
  String out;
  out.reserve(s.length() + 12);
  for (size_t i = 0; i < s.length(); ++i) {
    const char c = s[i];
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if (c == '\n') out += F("\\n");
    else if (c == '\r') out += F("\\r");
    else if (c == '\t') out += F("\\t");
    else if ((uint8_t)c < 0x20) out += ' ';
    else out += c;
  }
  return out;
}


String Notifying::createNtfyMessage(const String& jsonString, const NotifyConfig& cfg) { // 1. ส่งแบบอ้างอิง (&) ลดการสำเนาข้อความในแรม
  // 2. ใช้ JsonDocument (V7) เพื่อความเร็วและประหยัดหน่วยความจำ
  JsonDocument doc;
  deserializeJson(doc, jsonString);
  
  // 3. จองพื้นที่หน่วยความจำล่วงหน้า (ลดขั้นตอนการตัดต่อ/จองแรมใน Loop บ่อยๆ)
  String message;
  message.reserve(1024); 

  message += F("**🐷🐓Sensors Status Report**\n**🏠");
  message += cfg.location;
  message += F("**\n\n");
  
  bool ac_loss = doc["ac_loss"]; // ดึงค่าตรงๆ ไม่ต้อง Cast Type
  message += F("⚡Main Power is ");
  message += ac_loss ? F("🚫Offline\n") : F("🔋Online\n"); // ใช้ F() ช่วยประหยัดแรม
  
  JsonArray sensors = doc["sensors"];
  for (JsonObject sensor : sensors) {
    int id = sensor["id"];
    bool online = sensor["online"];
    bool switch_state = sensor["switch"];
    unsigned long uptime = sensor["uptime"];
    
    message += F("🤖");
    message += id; // String เติมต่อตัวเลขได้เลย ไม่ต้องครอบ String(id) ให้ช้าลง
    message += F(": ");
    
    // 4. แยกการทำงานเชิงตรรกะแบบ Flat ช่วยให้ MCU ประมวลผลเงื่อนไขได้เร็วกว่าซ้อนกันไปมา
    if (online) {
      message += F("🟢ONLINE");
    } else if (uptime > 0) {
      message += F("🔴OFFLINE");
    } else {
      message += F("⚫NONE");
    }
    
    message += F(" | ");
    
    if (uptime > 0) {
      message += switch_state ? F("☃️ปกติ") : F("🚨ฉุกเฉิน");
    } else {
      message += F("🔌ไม่พบ");
    }
    
    message += F(" | Uptime: ");
    message += uptime;
    message += F("s\n");
  }
  
  // 5. ดึงข้อมูลท้ายตารางมาเติมโดยตรง ไม่ต้องผ่าน String()
  message += F("\n⚠️ *Alarms:* ");
  message += (int)doc["alarm_count"];
  message += F("❌ *Offline:* ");
  message += (int)doc["offline_count"];
  message += F("\n🎁 ch=");
  message += String(doc["ch"]);
  message += F(" | mac=");
  message += String(doc["mac"]); // อ่านค่า string จาก JSON ตรงๆ 
  
  return message;
}

// สร้าง messaage discord
String Notifying::createDiscordMessage(String jsonString, const NotifyConfig& cfg) {
  StaticJsonDocument<1024> doc;
  deserializeJson(doc, jsonString);
  
  String message = "\n**🐷🐓Sensors Status Report**\n";
  message +=  "**🏠"+cfg.location;"**\n\n";
  
  JsonArray sensors = doc["sensors"];

  bool ac_loss = (bool)doc["ac_loss"];
  message += "⚡Main Power is "+String(ac_loss ? "🚫Offline\n" : "🔋Online\n");

  for (JsonObject sensor : sensors) {
    int id = sensor["id"];
    bool online = sensor["online"];
    bool switch_state = sensor["switch"];
    unsigned long uptime = sensor["uptime"];   
    
    message += "🤖"+ String(id) + ": ";
    message += online ? "🟢ONLINE" : uptime > 0  ? "🔴OFFLINE" : "⚫NONE" ;
    message += " | " + String(switch_state ? uptime > 0 ? "☃️ปกติ" : "🔌ไม่พบ"  : uptime > 0 ? "🚨ฉุกเฉิน" : "🔌ไม่พบ");
    message += " | Uptime: " + String(uptime) + "s\n";
  }
  
 message += "\n⚠️ *Alarms:* " + String((int)doc["alarm_count"]);
 message += " ❌ *Offline:* " + String((int)doc["offline_count"]);
 message += "\n🎁 ch=" + String(doc["ch"]) +" | mac=" + String(doc["mac"]);
#if defined(DEBUG) 
Serial.print("MAC = ");
Serial.println(String(doc["mac"]));
Serial.print("Channel = ");
Serial.println(String(doc["ch"]));
#endif
  
  return message;
}

// สร้าง Message Telegram
String Notifying::createTelegramMessage(String jsonString, const NotifyConfig& cfg) {
  StaticJsonDocument<1024> doc;
  deserializeJson(doc, jsonString);
  
  String message = "**🐷🐓Sensors Status Report**\n";
  message +=  "**🏠"+cfg.location;+"**\n\n";
  
  JsonArray sensors = doc["sensors"];
  bool ac_loss = (bool)doc["ac_loss"];
  message += "⚡Main Power is "+String(ac_loss ? "🚫Offline\n" : "🔋Online\n");
  for (JsonObject sensor : sensors) {
    int id = sensor["id"];
    bool online = sensor["online"];
    bool switch_state = sensor["switch"];
    unsigned long uptime = sensor["uptime"];
    
    message += "🤖"+ String(id) + ":* ";
    message += online ? "🟢ONLINE" : uptime > 0  ? "🔴OFFLINE" : "⚫NONE";
    message += " | " + String(switch_state ? uptime > 0 ? "☃️ปกติ" : "🔌ไม่พบ"  : uptime > 0 ? "🚨ฉุกเฉิน" : "🔌ไม่พบ");
    message += " | Uptime: " + String(uptime) + "s\n";
  }
  
  message += "\n⚠️ *Alarms:* " + String((int)doc["alarm_count"] );
  message += "❌ *Offline:* " + String((int)doc["offline_count"]);
  message += "\n🎁 ch=" + String(doc["ch"]) +" | mac=" + String(doc["mac"]);

  #if defined(DEBUG) 
  Serial.print("MAC = ");
  Serial.println(String(doc["mac"]));
  #endif
  
  return message;
}


bool Notifying::send(const String& message, const NotifyConfig& cfg) {
  if (WiFi.status() != WL_CONNECTED || message.length() == 0) {
    #if defined(DEBUG)
    Serial.println(F("[Notifying] No WiFi or empty message"));
    #endif
    return false;
  }
  String fullMessage;
  // fullMessage.reserve(cfg.location.length() + message.length() + 4);
  // if (cfg.location.length()) { fullMessage += '['; fullMessage += cfg.location; fullMessage += F("] "); }
  // fullMessage += message;

  if(cfg.type == 1){
    fullMessage = createNtfyMessage(message, cfg);
  }else if(cfg.type == 2){
    fullMessage = createTelegramMessage(message, cfg);
  }else{
    fullMessage = createDiscordMessage(message, cfg);
  }

  String url;
  String payload;
  const char* contentType = "text/plain; charset=utf-8";
  int expected = 200;
  if (cfg.type == 1 && cfg.ntfy_topic.length()) {
    if (cfg.ntfy_topic.startsWith("https://")) url = cfg.ntfy_topic;
    else if (cfg.ntfy_topic.indexOf("/") < 0) url = String(F("https://ntfy.sh/")) + cfg.ntfy_topic;
    else return false;
    payload = fullMessage;
  } else if (cfg.type == 2 && cfg.telegram_token.length() && cfg.telegram_channel.length()) {
    url = String(F("https://api.telegram.org/bot")) + cfg.telegram_token + F("/sendMessage");
    contentType = "application/json";
    payload.reserve(fullMessage.length() + cfg.telegram_channel.length() + 48);
    payload = F("{\"chat_id\":\""); payload += jsonEscape(cfg.telegram_channel);
    payload += F("\",\"text\":\""); payload += jsonEscape(fullMessage); payload += F("\"}");
  } else if (cfg.type == 3 && cfg.discord_webhook.startsWith("https://")) {
    url = cfg.discord_webhook;
    contentType = "application/json";
    expected = 204;
    payload.reserve(fullMessage.length() + 24);
    payload = F("{\"content\":\""); payload += jsonEscape(fullMessage); payload += F("\"}");
  } else {
    #if defined(DEBUG)
    Serial.println(F("[Notifying] Invalid config"));
    #endif
    return false;
  }

  BearSSL::WiFiClientSecure client;
  client.setInsecure(); // ไม่ตรวจสอบใบรับรอง: เสี่ยง MITM; ควรใช้ trust anchor หากทรัพยากรเพียงพอ
  client.setTimeout(5000);
  HTTPClient http;
  http.setTimeout(5000);
  http.setReuse(false);
  if (!http.begin(client, url)) return false;
  http.addHeader(F("Content-Type"), contentType);
  const int code = http.POST(payload);
  const bool ok = (code == expected || (cfg.type == 3 && code == 200));
  #if defined(DEBUG)
  Serial.printf("[Notifying] type=%d HTTP=%d heap=%u\n", cfg.type, code, ESP.getFreeHeap());
  #endif
  http.end();
  return ok;
}

// https://discord.com/api/webhooks/1427287898759368744/pQCXfltKyhnN_rASeL4Jt-G2FGqFim8USSBepLEn0mDP7LAyFVhvV8diZkdH_Pco9ZB0

#include "Notifier.h"

// #define DEBUG  //เปิดใช้งานเมื่ออยู่ในโหมดพัฒนา

Notifier::Notifier() {
  discord_enabled = false;
  telegram_enabled = false;
  ntfy_enabled = false;
  ntfy_server = "https://ntfy.sh";
}

void Notifier::setLocation(String msg){
    location = msg;
}

void Notifier::setupDiscord(String webhook) {
  discord_webhook = webhook;
  discord_enabled = true;
}

void Notifier::setupTelegram(String token, String chat_id) {
  telegram_token = token;
  telegram_chat_id = chat_id;
  telegram_enabled = true;
}

void Notifier::setupNtfy(String server) {
  ntfy_server = server;
  ntfy_enabled = true;
}

void Notifier::enableDiscord(bool enable) {
  discord_enabled = enable;
}

void Notifier::enableTelegram(bool enable) {
  telegram_enabled = enable;
}

void Notifier::enableNtfy(bool enable) {
  ntfy_enabled = enable;
}
// สร้าง messaage discord
String Notifier::createDiscordMessage(String jsonString) {
  StaticJsonDocument<1024> doc;
  deserializeJson(doc, jsonString);
  
  String message = "**🐷🐓Sensors Status Report**\n";
  message +=  "**🏠"+location+"**\n\n";
  
  JsonArray sensors = doc["sensors"];
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
String Notifier::createTelegramMessage(String jsonString) {
  StaticJsonDocument<1024> doc;
  deserializeJson(doc, jsonString);
  
  String message = "**🐷🐓Sensors Status Report**\n";
  message +=  "**🏠"+location+"**\n\n";
  
  JsonArray sensors = doc["sensors"];
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
// สร้าง Message สำหรับ ntfy
String Notifier::createNtfyMessage(String jsonString) {
  StaticJsonDocument<1024> doc;
  deserializeJson(doc, jsonString);
  
  String message = "**🐷🐓Sensors Status Report**\n";
  message +=  "**🏠"+location+"**\n\n";
  
  JsonArray sensors = doc["sensors"];

  bool ac_loss = (bool)doc["ac_loss"];
  message += "⚡Main Power is "+String(ac_loss ? "🚫Offline\n" : "🔋Online\n");

  for (JsonObject sensor : sensors) {
    int id = sensor["id"];
    bool online = sensor["online"];
    bool switch_state = sensor["switch"];
    unsigned long uptime = sensor["uptime"];
    
    message += "🤖"+ String(id) + ": ";
    message += online ? "🟢ONLINE" : uptime > 0  ? "🔴OFFLINE" : "⚫NONE";
    message += " | " + String(switch_state ? uptime > 0 ? "☃️ปกติ" : "🔌ไม่พบ"  : uptime > 0 ? "🚨ฉุกเฉิน" : "🔌ไม่พบ");
    message += " | Uptime: " + String(uptime) + "s\n";
  }
  
  message += "\n⚠️ *Alarms:* " + String((int)doc["alarm_count"]);
  message += "❌ *Offline:* " + String((int)doc["offline_count"]);
  message += "\n🎁 ch=" + String(doc["ch"]) +" | mac=" + String(doc["mac"]);
  
  return message;
}

// ส่งข้อมูลรายงานไปยัง Discord
bool Notifier::sendDiscord(String jsonString) {
  if(!discord_enabled || WiFi.status() != WL_CONNECTED) {
    return false;
  }
  
  bool success = false;
  WiFiClientSecure client;
  client.setInsecure(); 
  HTTPClient http;
  
  String message = createDiscordMessage(jsonString);
  StaticJsonDocument<2048> discordDoc;
  discordDoc["content"] = message;
  
  String payload;
  serializeJson(discordDoc, payload);
  
  if(http.begin(client, discord_webhook)){
    http.addHeader("Content-Type", "application/json");
    http.addHeader("User-Agent", "ESP32-Discord-Bot");
  
    int httpCode = http.POST(payload);   
    
    if(httpCode >= 200 && httpCode < 300) { // เช็คช่วง Success code (ปกติ Discord คือ 204)
      #if defined(DEBUG) 
      Serial.println("✅ Discord: Sent! Code: " + String(httpCode));
      #endif
      success = true;
    } else {
      #if defined(DEBUG) 
      Serial.println("❌ Discord: Failed! Code: " + String(httpCode));
      #endif
      success = false;
    }
    http.end(); // ปิดการเชื่อมต่อตรงนี้เพื่อให้ Resource ถูกคืนค่าแน่นอน
  } else {
    #if defined(DEBUG) 
    Serial.println("Failed to begin HTTP connection");
    #endif
    success = false;
  }

  return success; // คืนค่าครั้งเดียวที่ท้ายฟังก์ชัน
}

bool Notifier::sendTelegram(String jsonString) {
  if(!telegram_enabled || WiFi.status() != WL_CONNECTED) {
    return false;
  }
  
  HTTPClient http;
  
  String message = createTelegramMessage(jsonString);
  
  // URL encode
  message.replace(" ", "%20");
  message.replace("\n", "%0A");
  message.replace("*", "%2A");
  message.replace(":", "%3A");
  
  String url = "https://api.telegram.org/bot" + telegram_token + 
               "/sendMessage?chat_id=" + telegram_chat_id + 
               "&text=" + message + "&parse_mode=Markdown";
  
  http.begin(url);
  int httpCode = http.GET();
  http.end();
  
  if(httpCode > 0) {
    #if defined(DEBUG) 
    Serial.println("✅ Telegram: Sent! Code: " + String(httpCode));
    #endif
    return true;
  } else {
    #if defined(DEBUG) 
    Serial.println("❌ Telegram: Failed! Code: " + String(httpCode));
    #endif
    return false;
  }
}

bool Notifier::sendNtfy(String jsonString) {
  if(!ntfy_enabled || WiFi.status() != WL_CONNECTED) {
    return false;
  }
  
  HTTPClient http;
  
  String message = createNtfyMessage(jsonString);
  String url = ntfy_server;
  
  http.begin(url);
  http.addHeader("Content-Type", "text/plain");
  // http.addHeader("Title", "ESP32 Sensors Report");
  http.addHeader("Priority", "default");
  // http.addHeader("Tags", "computer,sensors");
  
  int httpCode = http.POST(message);
  http.end();
  
  if(httpCode > 0) {
    #if defined(DEBUG) 
    Serial.println("✅ Ntfy: Sent! Code: " + String(httpCode));
    #endif
    return true;
  } else {
    #if defined(DEBUG) 
    Serial.println("❌ Ntfy: Failed! Code: " + String(httpCode));
    #endif
    return false;
  }
}

void Notifier::sendAll(String jsonString) {
  int retry = 0;
  if(discord_enabled) {
    sendDiscord(jsonString);
    delay(1000);
  }
  
  if(telegram_enabled) {
    sendTelegram(jsonString);
    delay(1000);
  }
  
  if(ntfy_enabled) {
    sendNtfy(jsonString);
    delay(1000);
  }
}
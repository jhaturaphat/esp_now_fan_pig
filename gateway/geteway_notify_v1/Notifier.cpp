#include "Notifier.h"

Notifier::Notifier() {
  discord_enabled = false;
  telegram_enabled = false;
  ntfy_enabled = false;
  ntfy_server = "https://ntfy.sh";
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
  
  String message = "**📊 Sensors Status Report**\n\n";
  
  JsonArray sensors = doc["sensors"];
  for (JsonObject sensor : sensors) {
    int id = sensor["id"];
    bool online = sensor["online"];
    bool switch_state = sensor["switch"];
    unsigned long uptime = sensor["uptime"];
    
    message += "**Sensor " + String(id) + ":** ";
    message += online ? "🟢 Online" : "🔴 Offline";
    message += " | Switch: " + String(switch_state ? "ON" : "OFF");
    message += " | Uptime: " + String(uptime) + "s\n";
  }
  
  message += "\n⚠️ **Alarms:** " + String((int)doc["alarm_count"]);
  message += "\n❌ **Offline:** " + String((int)doc["offline_count"]);
  
  return message;
}
// สร้าง Message Telegram
String Notifier::createTelegramMessage(String jsonString) {
  StaticJsonDocument<1024> doc;
  deserializeJson(doc, jsonString);
  
  String message = "📊 *Sensors Status Report*\n\n";
  
  JsonArray sensors = doc["sensors"];
  for (JsonObject sensor : sensors) {
    int id = sensor["id"];
    bool online = sensor["online"];
    bool switch_state = sensor["switch"];
    unsigned long uptime = sensor["uptime"];
    
    message += "*Sensor " + String(id) + ":* ";
    message += online ? "🟢 Online" : "🔴 Offline";
    message += " | Switch: " + String(switch_state ? "ON" : "OFF");
    message += " | Uptime: " + String(uptime) + "s\n";
  }
  
  message += "\n⚠️ *Alarms:* " + String((int)doc["alarm_count"]);
  message += "\n❌ *Offline:* " + String((int)doc["offline_count"]);
  
  return message;
}
// สร้าง Message สำหรับ ntfy
String Notifier::createNtfyMessage(String jsonString) {
  StaticJsonDocument<1024> doc;
  deserializeJson(doc, jsonString);
  
  String message = "Sensors Status Report\n\n";
  
  JsonArray sensors = doc["sensors"];
  for (JsonObject sensor : sensors) {
    int id = sensor["id"];
    bool online = sensor["online"];
    bool switch_state = sensor["switch"];
    unsigned long uptime = sensor["uptime"];
    
    message += "Sensor " + String(id) + ": ";
    message += online ? "🟢 Online" : "🔴 Offline";
    message += " | Switch: " + String(switch_state ? "ON" : "OFF");
    message += " | Uptime: " + String(uptime) + "s\n";
  }
  
  message += "\nAlarms: " + String((int)doc["alarm_count"]);
  message += "\nOffline: " + String((int)doc["offline_count"]);
  
  return message;
}

bool Notifier::sendDiscord(String jsonString) {
  if(!discord_enabled || WiFi.status() != WL_CONNECTED) {
    return false;
  }
  
  HTTPClient http;
  
  String message = createDiscordMessage(jsonString);
  
  StaticJsonDocument<2048> discordDoc;
  discordDoc["content"] = message;
  
  String payload;
  serializeJson(discordDoc, payload);
  
  http.begin(discord_webhook);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.POST(payload);
  http.end();
  
  if(httpCode > 0) {
    Serial.println("✅ Discord: Sent! Code: " + String(httpCode));
    return true;
  } else {
    Serial.println("❌ Discord: Failed! Code: " + String(httpCode));
    return false;
  }
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
    Serial.println("✅ Telegram: Sent! Code: " + String(httpCode));
    return true;
  } else {
    Serial.println("❌ Telegram: Failed! Code: " + String(httpCode));
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
  http.addHeader("Title", "ESP32 Sensors Report");
  http.addHeader("Priority", "default");
  http.addHeader("Tags", "computer,sensors");
  
  int httpCode = http.POST(message);
  http.end();
  
  if(httpCode > 0) {
    Serial.println("✅ Ntfy: Sent! Code: " + String(httpCode));
    return true;
  } else {
    Serial.println("❌ Ntfy: Failed! Code: " + String(httpCode));
    return false;
  }
}

void Notifier::sendAll(String jsonString) {
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
  }
}
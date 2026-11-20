#pragma once
#include <Arduino.h>
#include <LittleFS.h>
struct Settings {
  String fanName;
  String ssid, pass, discord, topic, token, ntp="pool.ntp.org";
  int minutes=15, confirmMs=2000, utcMinutes=420;
  bool useDiscord=false, useNtfy=false, nc=false;
};
Settings cfg;
bool fsReady;
bool validText(const String& s, unsigned maxLen) {
  if (s.length()>maxLen) return false;
  for (unsigned i=0;i<s.length();i++) if ((uint8_t)s[i]<32 || s[i]==127) return false;
  return true;
}
bool valid(const Settings& c) {
  return validText(c.fanName,120) && c.ssid.length() && validText(c.ssid,32) && validText(c.pass,64) &&
    validText(c.discord,300) && (!c.useDiscord || c.discord.startsWith("https://discord.com/api/webhooks/")) &&
    validText(c.topic,300) && (!c.useNtfy || (c.topic.startsWith("https://") && c.topic.lastIndexOf('/')>7 && !c.topic.endsWith("/"))) &&
    validText(c.token,512) && c.ntp.length() && validText(c.ntp,128) &&
    c.minutes>=15 && c.minutes<=1440 && c.confirmMs>=50 && c.confirmMs<=30000 &&
    c.utcMinutes>=-720 && c.utcMinutes<=840;
}
bool writeConfig(const Settings& c) {
  if (!fsReady || !valid(c)) return false;
  String data="FAN2\n"+c.ssid+"\n"+c.pass+"\n"+c.discord+"\n"+c.topic+"\n"+c.token+"\n"+c.ntp+"\n"+
    c.minutes+"\n"+c.confirmMs+"\n"+c.utcMinutes+"\n"+int(c.useDiscord)+"\n"+int(c.useNtfy)+"\n"+int(c.nc)+"\n"+c.fanName+"\nEND\n";
  File f=LittleFS.open("/next","w");
  if (!f) return false;
  bool ok=f.print(data)==data.length();
  f.close();
  return ok && LittleFS.rename("/next","/settings");
}
bool loadConfig() {
  fsReady=LittleFS.begin();
  if (!fsReady) return false;
  File f=LittleFS.open("/settings","r");
  if (!f || f.size()>2000) return false;
  String version=f.readStringUntil('\n');
  if (version!="FAN1" && version!="FAN2") return false;
  String* fields[]={&cfg.ssid,&cfg.pass,&cfg.discord,&cfg.topic,&cfg.token,&cfg.ntp};
  for (auto field:fields) *field=f.readStringUntil('\n');
  cfg.minutes=f.readStringUntil('\n').toInt(); cfg.confirmMs=f.readStringUntil('\n').toInt();
  cfg.utcMinutes=f.readStringUntil('\n').toInt(); cfg.useDiscord=f.readStringUntil('\n')=="1";
  cfg.useNtfy=f.readStringUntil('\n')=="1"; cfg.nc=f.readStringUntil('\n')=="1";
  if (version=="FAN2") cfg.fanName=f.readStringUntil('\n');
  return f.readStringUntil('\n')=="END" && valid(cfg);
}

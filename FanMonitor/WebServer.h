#pragma once
#include <ESP8266WebServer.h>
ESP8266WebServer web(80);
bool restarting=false;
uint32_t restartAt;
String esc(String s) {
  s.replace("&","&amp;"); s.replace("'","&#39;"); s.replace("<","&lt;"); s.replace(">","&gt;");
  return s;
}
String input(const char* label,const char* key,String value,const char* type="text") {
  return String("<label>")+label+"<input name='"+key+"' type='"+type+"' value='"+esc(value)+"'></label>";
}
String check(const char* label,const char* key,bool on) {
  return String("<label><input type=checkbox name='")+key+"'"+(on?" checked":"")+"> "+label+"</label>";
}
void home() {
  String h=F("<!doctype html><html lang=th><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'><title>Fan monitor</title><style>*{box-sizing:border-box}body{font:16px system-ui;max-width:34rem;margin:auto;padding:1rem;background:#f6faf9;color:#193a35}label{display:block;margin:.8rem 0}input:not([type=checkbox]),select,button{width:100%;padding:.6rem;font-size:1rem}button{margin:1rem 0}</style><h1>ตั้งค่าเซ็นเซอร์พัดลม</h1><p>GPIO3 ขณะนี้ <b id=raw></b></p><script>async function readPin(){try{raw.textContent=await(await fetch('/pin',{cache:'no-store'})).text()}catch(e){raw.textContent='Offline'}}setInterval(readPin,700);readPin()</script><form method=post action='/save'><h2>พัดลม</h2>");
  h+=input("ชื่อหรือหมายเลขพัดลม เช่น ตัวที่ 1","fanName",cfg.fanName);
  h+=F("<h2>Wi-Fi</h2>");
  h+=input("SSID","ssid",cfg.ssid)+input("รหัส Wi-Fi (เว้นว่างเพื่อใช้ค่าเดิม)","pw",cfg.pass)+check("ใช้ Wi-Fi แบบเปิด","open",false);
  h+=F("<h2>การแจ้งเตือน</h2>");
  h+=input("Heartbeat (15–1440 นาที)","minutes",String(cfg.minutes),"number");
  h+=check("เปิด Discord","discordOn",cfg.useDiscord)+input("Discord Webhook URL (เว้นว่างเพื่อใช้ค่าเดิม)","discord",cfg.discord);
  h+=check("เปิด ntfy","ntfyOn",cfg.useNtfy)+input("ntfy HTTPS topic URL","topic",cfg.topic)+input("Bearer token (เว้นว่างเพื่อใช้ค่าเดิม)","token",cfg.token)+check("ล้าง token","clearToken",false);
  h+=F("<h2>นาฬิกาและเซ็นเซอร์</h2>");
  h+=input("NTP server","ntp",cfg.ntp)+input("UTC offset (นาที)","offset",String(cfg.utcMinutes),"number");
  h+=String("<label>Proximity<select name='mode'><option value=no>NO — ดับเมื่อ LOW</option><option value=nc")+(cfg.nc?" selected":"")+">NC — ดับเมื่อ HIGH</option></select></label>";
  h+=input("ยืนยันสถานะต่อเนื่อง (50–30000 ms)","confirm",String(cfg.confirmMs),"number");
  h+=F("<button>บันทึกและ Restart</button></form><form method=post action='/restart'><button>Restart โดยใช้ค่าที่บันทึกไว้</button></form></html>");
  web.sendHeader("Cache-Control","no-store");
  web.send(200,"text/html; charset=utf-8",h);
}
void restartDevice() {
  web.send(200,"text/plain; charset=utf-8","กำลัง Restart");
  restartAt=millis();
  restarting=true;
}
void saveForm() {
  Settings s=cfg;
  s.fanName=web.arg("fanName");
  s.fanName.trim();
  if (!validText(s.fanName,120)) { web.send(400,"text/plain; charset=utf-8","ชื่อพัดลมยาวได้สูงสุด 120 ไบต์"); return; }
  s.ssid=web.arg("ssid");
  if (web.hasArg("open")) s.pass="";
  else if (web.arg("pw").length()) s.pass=web.arg("pw");
  if (web.arg("discord").length()) s.discord=web.arg("discord");
  s.topic=web.arg("topic");
  if (web.hasArg("clearToken")) s.token="";
  else if (web.arg("token").length()) s.token=web.arg("token");
  s.useDiscord=web.hasArg("discordOn"); s.useNtfy=web.hasArg("ntfyOn");
  s.ntp=web.arg("ntp"); s.utcMinutes=web.arg("offset").toInt();
  s.nc=web.arg("mode")=="nc";
  s.confirmMs=web.arg("confirm").toInt(); s.minutes=web.arg("minutes").toInt();
  if (!valid(s)) { web.send(400,"text/plain; charset=utf-8","ตรวจ SSID, URL และช่วงตัวเลข แล้วกดย้อนกลับ"); return; }
  if (!writeConfig(s)) { web.send(500,"text/plain","LittleFS write failed"); return; }
  restartDevice();
}
void startWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(("FanSetup-"+String(ESP.getChipId(),HEX)).c_str());
  web.on("/",HTTP_GET,home);
  web.on("/pin",HTTP_GET,[](){web.send(200,"text/plain",digitalRead(3)?"HIGH":"LOW");});
  web.on("/save",HTTP_POST,saveForm); web.on("/restart",HTTP_POST,restartDevice);
  web.begin();
}

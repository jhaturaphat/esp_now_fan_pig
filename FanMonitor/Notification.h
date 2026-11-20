#pragma once
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecureBearSSL.h>
#include "Roots.h"
struct Delivery { uint32_t due=0; bool immediate=true; };
Delivery channel[2];
int lastCode[2]={0,0};
int postState(int i, const String& msg, uint32_t& retryMs) {
  static BearSSL::X509List trust(anchors);
  BearSSL::WiFiClientSecure tls;
  tls.setTrustAnchors(&trust);
  tls.setTimeout(2500);
  HTTPClient http;
  http.setTimeout(2500);
  if (!http.begin(tls,i==0 ? cfg.discord : cfg.topic)) return -1;
  const char* headers[]={"Retry-After"};
  http.collectHeaders(headers,1);
  http.addHeader("Content-Type",i==0 ? "application/json" : "text/plain; charset=utf-8");
  if (i==1 && cfg.token.length()) http.addHeader("Authorization","Bearer "+cfg.token);
  String body=msg;
  if (i==0) {
    body.replace("\\","\\\\");
    body.replace("\"","\\\"");
    body="{\"allowed_mentions\":{\"parse\":[]},\"content\":\""+body+"\"}";
  }
  int code=http.POST(body);
  if (code==429) retryMs=1000UL*constrain(http.header("Retry-After").toInt()+1,10L,86400L);
  http.end();
  return code;
}
void notifyTick() {
  static int previous=-1, next=0;
  if (!known) return;
  if (previous!=int(alarm)) {
    previous=alarm;
    for (auto& c:channel) c.immediate=true;
  }
  if (WiFi.status()!=WL_CONNECTED || !clockReady()) return;
  for (int k=0;k<2;k++) {
    int i=(next+k)%2;
    auto& c=channel[i];
    uint32_t now=millis();
    if (!(i==0 ? cfg.useDiscord : cfg.useNtfy) ||
        (!c.immediate && int32_t(now-c.due)<0)) continue;
    String identity="ESP-"+String(ESP.getChipId(),HEX);
    if (cfg.fanName.length()) identity="พัดลม "+cfg.fanName+" | "+identity;
    String msg=identity+" | "+(alarm?"🚨 ALARM พัดลมดับ":"🟢 Heartbeat พัดลมปกติ")+" | "+localStamp();
    uint32_t interval=alarm ? 10000UL : cfg.minutes*60000UL;
    uint32_t retryMs=10000;
    lastCode[i]=postState(i,msg,retryMs);
    c.due=now+((lastCode[i]>=200 && lastCode[i]<300)?interval:retryMs);
    c.immediate=false;
    next=1-i;
    break;
  }
}

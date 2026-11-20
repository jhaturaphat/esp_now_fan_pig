#pragma once
#include <ESP8266WiFi.h>
void startStation() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(cfg.ssid.c_str(),cfg.pass.c_str());
}
void maintainWifi() {
  static uint32_t last=millis();
  if (WiFi.status()!=WL_CONNECTED && millis()-last>=30000) {
    last=millis();
    WiFi.reconnect();
  }
}

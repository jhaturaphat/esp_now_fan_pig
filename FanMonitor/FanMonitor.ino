#include "Config.h"
#include "Wifi.h"
#include "Ntp.h"
#include "Sensor.h"
#include "Notification.h"
#include "WebServer.h"
enum Mode { BootWindow, Portal, Monitor };
#include "StatusLed.h"
Mode mode=BootWindow;
uint32_t bootAt, buttonAt;
bool pressing=false, configured;
void setup() {
  beginStatusLed();
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  pinMode(0,INPUT_PULLUP);
  beginSensor();
  configured=loadConfig();
  bootAt=millis();
}
void loop() {
  pollSensor();
  uint32_t now=millis();
  updateStatusLed(mode, now);
  if (mode==BootWindow) {
    if (digitalRead(0)==LOW) {
      if (!pressing) buttonAt=now;
      pressing=true;
    } else pressing=false;
    if (!configured || (pressing && now-buttonAt>=400)) {
      mode=Portal; startWeb();
    } else if (now-bootAt>=8000 && !pressing) {
      mode=Monitor; startStation(); startClock();
    }
  } else if (mode==Portal) {
    web.handleClient();
    if (restarting && now-restartAt<0x80000000UL && now-restartAt>=400) ESP.restart();
  } else {
    maintainWifi(); notifyTick();
  }
  yield();
}

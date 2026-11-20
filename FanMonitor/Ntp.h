#pragma once
#include <time.h>
void startClock() { configTime(0,0,cfg.ntp.c_str()); }
bool clockReady() { return time(nullptr)>1704067200; }
String localStamp() {
  time_t t=time(nullptr)+cfg.utcMinutes*60L;
  struct tm dt;
  gmtime_r(&t,&dt);
  char buf[24];
  strftime(buf,sizeof(buf),"%Y-%m-%d %H:%M:%S",&dt);
  return String(buf);
}

#pragma once
volatile uint32_t edgeAt;
void IRAM_ATTR sensorEdge() { edgeAt=micros(); }
bool known=false, alarm=false;
void beginSensor() {
  pinMode(3,INPUT_PULLUP);
  edgeAt=micros();
  attachInterrupt(digitalPinToInterrupt(3),sensorEdge,CHANGE);
}
void pollSensor() {
  noInterrupts();
  bool low=digitalRead(3)==LOW;
  uint32_t elapsed=micros()-edgeAt;
  interrupts();
  if (elapsed>=uint32_t(cfg.confirmMs)*1000) {
    known=true;
    alarm=cfg.nc ? !low : low;
  }
}

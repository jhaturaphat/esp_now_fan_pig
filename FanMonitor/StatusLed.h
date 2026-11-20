#pragma once
constexpr uint8_t statusLedPin = 2;
void beginStatusLed() {
  digitalWrite(statusLedPin, HIGH);
  pinMode(statusLedPin, OUTPUT);
}
void updateStatusLed(Mode currentMode, uint32_t now) {
  bool on;
  if (currentMode == BootWindow) {
    on = true;
  } else if (currentMode == Portal) {
    uint32_t phase = now % 2000;
    on = phase < 120 || (phase >= 300 && phase < 420);
  } else if (known && alarm) {
    on = now % 250 < 125;
  } else if (!known || WiFi.status() != WL_CONNECTED || !clockReady()) {
    on = now % 1000 < 500;
  } else {
    on = now % 3000 < 100;
  }
  digitalWrite(statusLedPin, on ? LOW : HIGH);
}

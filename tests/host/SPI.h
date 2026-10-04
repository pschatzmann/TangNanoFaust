#pragma once
// Minimal SPI API for running the library on a PC (tests only; no SPI).
#include "Arduino.h"
struct SPISettings {
  SPISettings(uint32_t = 0, int = 0, int = 0) {}
};
struct SPIClass {
  void begin() {}
  void beginTransaction(SPISettings) {}
  void endTransaction() {}
  uint8_t transfer(uint8_t) { return 0; }
  void transfer(void *, size_t) {}
};
inline SPIClass SPI;

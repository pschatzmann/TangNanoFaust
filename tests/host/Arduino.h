#pragma once
// Minimal Arduino API for running the library on a PC (tests only).
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>
inline uint32_t millis() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1000 + t.tv_nsec / 1000000; }
inline uint32_t micros() { timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec * 1000000 + t.tv_nsec / 1000; }
inline void delay(uint32_t ms) { usleep(ms * 1000); }
inline void delayMicroseconds(uint32_t us) { usleep(us); }
#define OUTPUT 1
#define HIGH 1
#define LOW 0
#define MSBFIRST 1
#define SPI_MODE0 0
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t *b, size_t n) { size_t i = 0; while (i < n && write(b[i])) i++; return i; }
};
class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
  virtual int peek() { return -1; }
  virtual void flush() {}
  void setTimeout(unsigned long ms) { timeout_ = ms; }
  size_t readBytes(uint8_t *b, size_t n) {  // like Arduino: waits up to the timeout per byte
    size_t i = 0;
    while (i < n) { int c = timedRead(); if (c < 0) break; b[i++] = (uint8_t)c; }
    return i;
  }
  size_t readBytes(char *b, size_t n) { return readBytes((uint8_t *)b, n); }
 protected:
  unsigned long timeout_ = 1000;
  int timedRead() {
    uint32_t start = millis();
    do { int c = read(); if (c >= 0) return c; usleep(100); } while (millis() - start < timeout_);
    return -1;
  }
};

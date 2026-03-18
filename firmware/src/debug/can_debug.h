#pragma once

#include <Arduino.h>

#include "app/app_context.h"

// Consolidated CAN send function with internal CAN FD DLC rounding
void send_can_message(uint8_t message_type, const uint8_t* data, uint8_t data_len = 8);

// Custom CAN Debug Interface - acts like Serial but sends via CAN
class CANDebugInterface : public Print {
private:
  char message_buffer[64];
  int buffer_pos = 0;

public:
  void send_string(const char* msg);
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  void flush_buffer();
};

// Global CAN debug interface instance
extern CANDebugInterface CANDebug;

// CAN Debug Stream wrapper - makes CANDebugInterface compatible with Stream
class CANDebugStream : public Stream {
private:
  CANDebugInterface* can_debug;

public:
  explicit CANDebugStream(CANDebugInterface* debug) : can_debug(debug) {}
  int available() override { return 0; }
  int read() override { return -1; }
  int peek() override { return -1; }
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buffer, size_t size) override;
};


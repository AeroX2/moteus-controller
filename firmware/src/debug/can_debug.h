#pragma once

#include <Arduino.h>

#include "telemetry/messages.h"

extern uint32_t id;
extern long update_frequency;
extern uint8_t init_errors;

// Consolidated CAN send function. payload_len = bytes in data; type byte is added internally.
void send_can_message(telemetry::CanMsgType message_type, const uint8_t* data, uint8_t payload_len = 7);

// Custom CAN Debug Interface — Print output over CAN (debug string frames).
// When not building with -DCAN_DEBUG_ENABLED, `Serial` is redefined to
// this object so existing Serial.print/println calls go out over CAN.
class CANDebugInterface : public Print {
private:
  char message_buffer[64];
  int buffer_pos = 0;

public:
  void send_string(const char* msg);
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *buffer, size_t size) override;
  void flush_buffer();

  // No-op UART API so setup() can keep Serial.begin / setRx / setTx unchanged.
  void begin(unsigned long baud = 0) { (void)baud; }
  void begin(unsigned long baud, uint8_t config) {
    (void)baud;
    (void)config;
  }
  void setRx(uint32_t pin) { (void)pin; }
  void setTx(uint32_t pin) { (void)pin; }
  explicit operator bool() const { return true; }
};

// Global CAN debug interface instance
extern CANDebugInterface CANDebug;

#if CAN_DEBUG_ENABLED
#undef Serial
#define Serial CANDebug
#endif

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


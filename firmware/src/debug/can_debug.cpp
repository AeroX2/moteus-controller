#include "debug/can_debug.h"

#include <cstring>

CANFDMessage CAN_TX_msg;
CANFDMessage CAN_RX_msg;

uint32_t id = 0;
long update_frequency = 100;
uint8_t init_errors = 0;

void send_can_message(uint8_t message_type, const uint8_t* data, uint8_t data_len) {
  if (id == 0) return; // Only send if CAN is initialized

  if (data_len == 0) data_len = 1;

  auto round_dlc = [](uint8_t n)->uint8_t {
    if (n <= 8)  return n;
    if (n <= 12) return 12;
    if (n <= 16) return 16;
    if (n <= 20) return 20;
    if (n <= 24) return 24;
    if (n <= 32) return 32;
    if (n <= 48) return 48;
    return 64;
  };
  uint8_t dlc = round_dlc(data_len);

  CAN_TX_msg.id = id;
  CAN_TX_msg.len = dlc;
  CAN_TX_msg.ext = true;
  CAN_TX_msg.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH;
  CAN_TX_msg.data[0] = message_type;

  int copy_bytes = (data_len > 1) ? (data_len - 1) : 0;
  if (copy_bytes > 63) copy_bytes = 63;
  for (int i = 0; i < copy_bytes; i++) {
    CAN_TX_msg.data[1 + i] = data[i];
  }
  for (int i = copy_bytes; i < (dlc - 1); i++) {
    CAN_TX_msg.data[1 + i] = 0x00;
  }
  fdcan1.tryToSendReturnStatusFD(CAN_TX_msg);
}

CANDebugInterface CANDebug;

void CANDebugInterface::send_string(const char* msg) {
  if (id == 0 || msg == nullptr) return;
  size_t total = strlen(msg);
  size_t offset = 0;
  while (offset < total) {
    size_t chunk = total - offset;
    if (chunk > 63) chunk = 63;
    uint8_t frame[63] = {0};
    if (chunk) memcpy(frame, msg + offset, chunk);
    send_can_message(0x04, frame, static_cast<uint8_t>(chunk + 1));
    offset += chunk;
  }
}

size_t CANDebugInterface::write(uint8_t c) {
  if (c == '\n' || c == '\r') {
    if (buffer_pos > 0) {
      message_buffer[buffer_pos] = '\0';
      send_string(message_buffer);
      buffer_pos = 0;
    }
  } else {
    if (buffer_pos >= 63) {
      message_buffer[63 - 1] = '\0';
      send_string(message_buffer);
      buffer_pos = 0;
    }
    message_buffer[buffer_pos++] = static_cast<char>(c);
  }
  return 1;
}

size_t CANDebugInterface::write(const uint8_t *buffer, size_t size) {
  for (size_t i = 0; i < size; i++) {
    write(buffer[i]);
  }
  return size;
}

void CANDebugInterface::flush_buffer() {
  if (buffer_pos > 0) {
    message_buffer[buffer_pos] = '\0';
    send_string(message_buffer);
    buffer_pos = 0;
  }
}

size_t CANDebugStream::write(uint8_t c) {
  if (can_debug) return can_debug->write(c);
  return 0;
}

size_t CANDebugStream::write(const uint8_t *buffer, size_t size) {
  if (can_debug) return can_debug->write(buffer, size);
  return 0;
}


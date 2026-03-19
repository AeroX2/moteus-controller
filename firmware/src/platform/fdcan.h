// FDCAN firewall: only fdcan.cpp includes ACANFD_STM32.h.
// All other code uses these abstract send/recv interfaces.
#pragma once

#include <ACANFD_STM32_from_cpp.h>
#include <cstdint>

// Initialize FDCAN with the given CAN ID (used for filters and address).
void fdcan_begin(uint32_t id);

// Send a CAN FD frame. No-op if id is 0 (not initialized).
void fdcan_send(const CANFDMessage* msg);

// Receive a frame if available. Returns true and fills *msg when a frame was received.
bool fdcan_recv(CANFDMessage* msg);

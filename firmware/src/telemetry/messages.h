// CAN message type IDs and packed payload definitions
#pragma once

#include <cstdint>
#include <cstring>

namespace telemetry {

enum class CanMsgType : uint8_t {
  MotionData = 0x00,
  DriverStatus = 0x01,
  InitStatus = 0x03,
  DebugString = 0x04,
};

#pragma pack(push, 1)
struct MotionDataPayload {
  float angle;
  float velocity;
  uint8_t reserved[3];
};

struct DriverStatusPayload {
  uint8_t status_reg;
  uint8_t ready;
  uint8_t fault;
  uint8_t nfault_reg;
  uint8_t ready_reg;
  uint8_t reserved[2];
};

struct InitStatusPayload {
  uint8_t init_errors;
  uint8_t motor_status;
  uint8_t reserved[5];
};
#pragma pack(pop)

constexpr uint8_t MOTION_DATA_PAYLOAD_SIZE = sizeof(MotionDataPayload);
constexpr uint8_t DRIVER_STATUS_PAYLOAD_SIZE = sizeof(DriverStatusPayload);
constexpr uint8_t INIT_STATUS_PAYLOAD_SIZE = sizeof(InitStatusPayload);

// Encode helpers: write packed struct bytes into buffer.
inline void motion_data_encode(const MotionDataPayload* p, uint8_t* buf) {
  std::memcpy(buf, p, sizeof(MotionDataPayload));
}

inline void driver_status_encode(const DriverStatusPayload* p, uint8_t* buf) {
  std::memcpy(buf, p, sizeof(DriverStatusPayload));
}

inline void init_status_encode(const InitStatusPayload* p, uint8_t* buf) {
  std::memcpy(buf, p, sizeof(InitStatusPayload));
}

}  // namespace telemetry

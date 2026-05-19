#include "app/can_init.h"

#include <Arduino.h>

#include "debug/can_debug.h"
#include "platform/fdcan.h"

void can_init() {
  const uint32_t uid0 = HAL_GetUIDw0();
  const uint32_t uid1 = HAL_GetUIDw1();
  const uint32_t uid2 = HAL_GetUIDw2();

  // Derive a stable 29-bit extended CAN id from all UID words.
  id = (uid0 ^ (uid1 << 7) ^ (uid2 >> 3)) & 0x1FFFFFFF;
  if (id == 0) {
    // Some boards expose zeroed UID registers; avoid disabling CAN traffic.
    id = 0x56004D;
  }
  fdcan_begin(id);
}


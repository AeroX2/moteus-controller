#include "app/can_init.h"

#include <Arduino.h>

#include "debug/can_debug.h"
#include "platform/fdcan.h"

void can_init() {
  id = HAL_GetUIDw0();
  fdcan_begin(id);
}


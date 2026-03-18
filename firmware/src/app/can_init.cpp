#include "app/can_init.h"

#include <Arduino.h>

#include "app/app_context.h"

// Pull in the config types without defining `fdcan1`.
#include <ACANFD_STM32_Settings.h>
#include <ACANFD_STM32_Filters.h>

void can_init() {
  id = HAL_GetUIDw0();

  // Configure ACANFD_STM32 for FD mode with bit rate switching
  ACANFD_STM32_Settings settings(1000000, DataBitRateFactor::x5);

  ACANFD_STM32_StandardFilters standardFilters;
  standardFilters.addSingle(id, ACANFD_STM32_FilterAction::FIFO0);
  settings.mNonMatchingStandardFrameReception = ACANFD_STM32_FilterAction::REJECT;

  ACANFD_STM32_ExtendedFilters extendedFilters;
  extendedFilters.addSingle(id, ACANFD_STM32_FilterAction::FIFO0);
  settings.mNonMatchingExtendedFrameReception = ACANFD_STM32_FilterAction::REJECT;

  const uint32_t errorCode = fdcan1.beginFD(settings, standardFilters, extendedFilters);
  if (errorCode == 0) {
    Serial.println("CANFD configuration OK");
  } else {
    Serial.print("CANFD configuration error: 0x");
    Serial.println(errorCode, HEX);
  }

  Serial.print("My CAN ID: 0x");
  Serial.println(id, HEX);
}


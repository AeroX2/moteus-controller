// Single translation unit for ACANFD_STM32 - defines fdcan1 and IRQ handlers.
#include "platform/fdcan.h"

#include <Arduino.h>
#include <ACANFD_STM32.h>
#include <ACANFD_STM32_Settings.h>
#include <ACANFD_STM32_Filters.h>

static uint32_t fdcan_id = 0;

void fdcan_begin(uint32_t id) {
  fdcan_id = id;

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

void fdcan_send(const CANFDMessage* msg) {
  if (fdcan_id == 0 || msg == nullptr) return;
  fdcan1.tryToSendReturnStatusFD(*msg);
}

bool fdcan_recv(CANFDMessage* msg) {
  if (msg == nullptr) return false;
  return fdcan1.receiveFD0(*msg);
}

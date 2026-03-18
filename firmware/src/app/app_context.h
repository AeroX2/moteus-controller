// Shared application objects and state.
#pragma once

#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/stspin32g4/STSPIN32G4.h"
// Provides ACANFD_STM32 class + CANFDMessage type, but does NOT define `fdcan1`.
#include <ACANFD_STM32_from_cpp.h>

// Magnetic sensor instance - SPI
extern MagneticSensorSPI magnetic_sensor;
extern SPIClass spi_class;

// Current sense
extern LowsideCurrentSense current_sense;

// Motor + driver
extern STSPIN32G4 driver;
extern BLDCMotor motor;

// CAN
extern CANFDMessage CAN_TX_msg;
extern CANFDMessage CAN_RX_msg;
extern uint32_t id;

extern long update_frequency;
extern uint8_t init_errors;

// Defined by including `ACANFD_STM32.h` in exactly one translation unit (main.cpp).
extern ACANFD_STM32 fdcan1;


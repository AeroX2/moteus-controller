#pragma once

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/stspin32g4/STSPIN32G4.h"

extern MagneticSensorSPI magnetic_sensor;
extern SPIClass spi_class;
extern LowsideCurrentSense current_sense;
extern STSPIN32G4 driver;
extern BLDCMotor motor;

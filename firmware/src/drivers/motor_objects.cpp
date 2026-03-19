#include "drivers/motor_objects.h"

MagneticSensorSPI magnetic_sensor = MagneticSensorSPI(AS5048_SPI, PD2);
SPIClass spi_class(PB5, PB4, PB3);

// (gain values comes from B-G431B-ESC1, multiplied by 10 because that is what worked)
LowsideCurrentSense current_sense = LowsideCurrentSense(0.01f, -64.0f / 7.0f * 10.0f, PB1, _NC, PA2);

STSPIN32G4 driver = STSPIN32G4();
BLDCMotor motor = BLDCMotor(7);

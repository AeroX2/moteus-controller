#include "drivers/motor_objects.h"

// Driver
STSPIN32G4 driver = STSPIN32G4();


// ML5010: phase resistance (Ω), datasheet continuous current (A) — thermal limit, not DC supply current.
constexpr float MOTOR_PHASE_RESISTANCE_OHM = 0.08f; // moteus measured value
constexpr float MOTOR_PHASE_KV = 240.65f; // from calibration (kv)
constexpr float MOTOR_PHASE_INDUCTANCE_Q = 0.03f / 1000.0f; // moteus measured value
constexpr float MOTOR_PHASE_INDUCTANCE_D = 0.02f / 1000.0f; // moteus measured value
BLDCMotor motor = BLDCMotor(7, MOTOR_PHASE_RESISTANCE_OHM, MOTOR_PHASE_KV, MOTOR_PHASE_INDUCTANCE_Q, MOTOR_PHASE_INDUCTANCE_D); //, MOTOR_PHASE_INDUCTANCE_H);

// SPI bus and magnetic sensor
SPIClass spi_class(PB5, PB4, PB3);
MagneticSensorSPI magnetic_sensor = MagneticSensorSPI(AS5048_SPI, PD2);

// Standalone STM32 OPAMP + external R (hw_opamp.cpp). Rf/Rin ≈ 11 k / 1.5 k = 22/3.
LowsideCurrentSense current_sense = LowsideCurrentSense(0.01f, -22.0f / 3.0f, PB1, _NC, PA2);

#include "app/motor_init.h"

#include <Arduino.h>
#include <cmath>

#include "debug/can_debug.h"
#include "drivers/motor_objects.h"
#include "hal/hw_opamp.h"
#include "storage/pid_flash.h"

#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"


void motor_init() {
  Serial.println("Initializing CORDIC");
  if (!SimpleFOC_CORDIC_Config()) {
    Serial.println("CORDIC init failed");
    init_errors |= 0x01;
  }

  magnetic_sensor.init(&spi_class);
  motor.linkSensor(&magnetic_sensor);

  driver.voltage_power_supply = 24.0f;
  if (driver.init() == 0) {
    Serial.println("Driver init failed!");
    init_errors |= 0x02;
  } else {
    Serial.println("Driver init success!");
  }

  motor.linkDriver(&driver);

  Serial.print("Driver fault: ");
  Serial.println(driver.isFault() ? "true" : "false");
  if (driver.isFault()) init_errors |= 0x02;

  motor.voltage_sensor_align = 1.5f;

  opamp_init();
  current_sense.linkDriver(&driver);
  if (current_sense.init()) {
    Serial.println("Current sense init success!");
  } else {
    Serial.println("Current sense init failed!");
    init_errors |= 0x04;
  }
  motor.linkCurrentSense(&current_sense);

  motor.controller = MotionControlType::angle;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  motor.updateVelocityLimit(100.0f);
  motor.updateCurrentLimit(20.0f);

  motor.LPF_current_q.Tf = 0.001;
  motor.LPF_current_d.Tf = 0.001;

  if (load_pid_from_flash()) Serial.println("Loaded PID values from flash.");

  delay(100);
  if (motor.init() == 0) {
    Serial.println("Motor init failed!");
    init_errors |= 0x08;
  } else {
    Serial.println("Motor init success!");
  }

  if (motor.tuneCurrentController(500.0f) == 0) {
    Serial.println("Current controller tuned!");
  } else {
    Serial.println("Current controller tuning failed!");
    init_errors |= 0x10;
  }

  // motor.characteriseMotor(1.0f);
  // Serial.println("Motor characterised!");
  // Serial.print("Motor phase resistance: ");
  // Serial.printf("%f", motor.phase_resistance);
  // Serial.print(" Ω\n");
  // Serial.print("Motor phase inductance: ");
  // Serial.printf("%f", motor.phase_inductance);
  // Serial.print(" H\n");
  // Serial.print("Motor phase KV: ");
  // Serial.printf("%f", motor.KV_rating);
  // Serial.print(" RPM/V\n");
  // return;

  if (motor.initFOC() == 0) {
    Serial.println("Motor FOC init failed!");
    init_errors |= 0x10;
  } else {
    Serial.println("Motor FOC init success!");
  }

  motor.disable();
}


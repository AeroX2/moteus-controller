#include "app/motor_init.h"

#include <Arduino.h>

#include "app/app_context.h"
#include "app/motor_tuning.h"
#include "debug/can_debug.h"
#include "hal/hw_opamp.h"
#include "storage/pid_flash.h"

#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

void motor_init() {
  Serial.println("Initializing CORDIC");
  if (!SimpleFOC_CORDIC_Config()) {
    Serial.println("CORDIC init failed");
    init_errors |= 0x01;
  }

  driver.voltage_power_supply = 24.0f;
  driver.voltage_limit = 24.0f;

  if (driver.init() == 0) {
    Serial.println("Driver init failed!");
    init_errors |= 0x02;
  } else {
    Serial.println("Driver init success!");
  }

  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);

  Serial.print("Driver fault: ");
  Serial.println(driver.isFault() ? "true" : "false");
  if (driver.isFault()) init_errors |= 0x02;

  if (motor.init() == 0) {
    Serial.println("Motor init failed!");
    init_errors |= 0x08;
  } else {
    Serial.println("Motor init success!");
  }

  opamp_init();
  if (current_sense.init()) {
    Serial.println("Current sense init success!");
  } else {
    Serial.println("Current sense init failed!");
    init_errors |= 0x04;
  }
  motor.linkCurrentSense(&current_sense);

  magnetic_sensor.init(&spi_class);
  motor.linkSensor(&magnetic_sensor);

  motor.monitor_downsample = 0;
  motor.monitor_variables = 0;
  motor.useMonitoring(CANDebug);

  motor_apply_tuning_defaults();

  if (load_pid_from_flash()) Serial.println("Loaded PID values from flash.");

  delay(100);
  if (motor.initFOC() == 0) {
    Serial.println("Motor FOC init failed!");
    init_errors |= 0x10;
  } else {
    Serial.println("Motor FOC init success!");
  }

  motor.disable();
}


#include "app/motor_init.h"

#include <Arduino.h>

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

  // Default motor limits, filters, and PID tuning
  motor.current_limit = 3.0;
  motor.voltage_limit = 8.0;
  motor.velocity_limit = 100.0;
  motor.voltage_sensor_align = 1.5;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;

  motor.PID_current_q.P = 0.02;
  motor.PID_current_q.I = 10.0;
  motor.PID_current_q.output_ramp = 1000.0;
  motor.PID_current_d.P = 0.02;
  motor.PID_current_d.I = 10.0;
  motor.PID_current_d.output_ramp = 1000.0;

  motor.LPF_current_q.Tf = 0.001;
  motor.LPF_current_d.Tf = 0.001;
  motor.LPF_velocity.Tf = 0.02;

  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 0.003;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 400.0;

  motor.P_angle.P = 10.0;
  motor.P_angle.limit = 70.0;

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


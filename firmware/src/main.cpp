#include <Arduino.h>
#include <SoftwareSerial.h>
// #include <HardwareSerial.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/stspin32g4/STSPIN32G4.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

// magnetic sensor instance - I2C
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// BLDC motor & driver instance
STSPIN32G4 driver = STSPIN32G4();
BLDCMotor motor = BLDCMotor(7, 0.3, 330);

SoftwareSerial softwareSerial(PC14, PC15);

// commander interface
Commander command = Commander(softwareSerial);
void onMotor(char* cmd) {
  command.motor(&motor, cmd);
}
void doLed(char* cmd) {
  if (*cmd == '0')
    digitalWrite(PC2, LOW);
  else
    digitalWrite(PC2, HIGH);
}
void doStatus(char* cmd) {
  softwareSerial.println("Driver status: ");
  STSPIN32G4Status status = driver.status();
  softwareSerial.print("      Lock: ");
  softwareSerial.println(status.lock ? "LOCKED" : "UNLOCKED");
  softwareSerial.print("  VCC UVLO: ");
  softwareSerial.println(status.vcc_uvlo ? "YES" : "NO");
  softwareSerial.print("     VDS P: ");
  softwareSerial.println(status.vds_p ? "YES" : "NO");
  softwareSerial.print("     RESET: ");
  softwareSerial.println(status.reset ? "YES" : "NO");
  softwareSerial.print("      TSHD: ");
  softwareSerial.println(status.thsd ? "YES" : "NO");
}

TwoWire twoWire(PF0, PC4);

void setup() {
  softwareSerial.begin(115200);
  SimpleFOCDebug::enable(&softwareSerial);

  softwareSerial.println("Initializing CORDIC");
  if (!SimpleFOC_CORDIC_Config())
    softwareSerial.println("CORDIC init failed");

  // initialise magnetic sensor hardware
  sensor.init(&twoWire);
  // link the motor to the sensor
  // motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]

  driver.voltage_power_supply = 24.0f;
  driver.pwm_frequency = 30000;
  driver.init();

  motor.linkDriver(&driver);

  softwareSerial.print("Driver ready: ");
  softwareSerial.println(driver.isReady() ? "true" : "false");
  softwareSerial.print("Driver fault: ");
  softwareSerial.println(driver.isFault() ? "true" : "false");

  // set control loop type to be used

  // motor.PID_velocity.P = 4.0f;
  // motor.PID_velocity.I = 0.0f;
  // motor.PID_velocity.D = 0.2f; 
  // motor.LPF_velocity.Tf = 0.04f; 

  // motor.useMonitoring(softwareSerial);
  motor.current_limit = 2;
  // motor.voltage_limit = driver.voltage_limit / 2.0f;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::velocity_openloop;
  motor.motion_downsample = 4;

  // align encoder and start FOC
  motor.init();
  // motor.initFOC();

  pinMode(PC2, OUTPUT);
  command.add('A', onMotor, "motor");
  command.add('L', doLed, "led control");
  command.add('S', doStatus, "driver status");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  softwareSerial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));

  _delay(1000);
}

// bool flip = false;
// long old_time = 0;
// float target = 5.0f;

void loop() {
  // // iterative setting of the FOC phase voltage
  motor.loopFOC();

  // // iterative function setting the outter loop target
  // // velocity, position or voltage
  // // if target not set in parameter uses motor.target variable
  motor.move();
  // motor.monitor();

  // // user communication
  command.run();

  // if (millis() - old_time > 3000) {
  //   printDriverStatus();
  //   target = 0.0f;
  //   old_time = millis();
  // }
  // delayMicroseconds(100);
}

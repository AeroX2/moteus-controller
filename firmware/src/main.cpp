#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/stspin32g4/STSPIN32G4.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

// magnetic sensor instance - I2C
MagneticSensorSPI sensor = MagneticSensorSPI(AS5048_SPI, PD2);

// BLDC motor & driver instance
STSPIN32G4 driver = STSPIN32G4();
BLDCMotor motor = BLDCMotor(7); //, 0.3);  //, 300);

// STM32_CAN Can(FDCAN1, DEF);

// static CAN_message_t CAN_TX_msg;

float target_velocity = 0;
// commander interface
Commander command = Commander(Serial);
void onStop(char* cmd) {
  motor.disable();
}
void onMotor(char* cmd) {
  // command.motor(&motor, cmd);
  command.scalar(&target_velocity, cmd);
}
void doLed(char* cmd) {
  if (*cmd == '0')
    digitalWrite(PC2, LOW);
  else
    digitalWrite(PC2, HIGH);
}
void doStatus(char* cmd) {
  Serial.println("Driver status: ");
  STSPIN32G4Status status = driver.status();
  Serial.print("      Lock: ");
  Serial.println(status.lock ? "LOCKED" : "UNLOCKED");
  Serial.print("  VCC UVLO: ");
  Serial.println(status.vcc_uvlo ? "YES" : "NO");
  Serial.print("     VDS P: ");
  Serial.println(status.vds_p ? "YES" : "NO");
  Serial.print("     RESET: ");
  Serial.println(status.reset ? "YES" : "NO");
  Serial.print("      TSHD: ");
  Serial.println(status.thsd ? "YES" : "NO");
}

SPIClass spi_class(PB5, PB4, PB3);

void setup() {
  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  Serial.println("Initializing CORDIC");
  if (!SimpleFOC_CORDIC_Config())
    Serial.println("CORDIC init failed");

  // initialise magnetic sensor hardware
  sensor.init(&spi_class);
  // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24.0f;
  // driver.pwm_frequency = 20000;
  // driver.dead_zone = 0.05;
  driver.init();

  motor.linkDriver(&driver);

  Serial.print("Driver ready: ");
  Serial.println(driver.isReady() ? "true" : "false");
  Serial.print("Driver fault: ");
  Serial.println(driver.isFault() ? "true" : "false");

  // motor.PID_velocity.P = 40.0f;
  // motor.PID_velocity.I = 0.0f;
  // motor.PID_velocity.D = 0.001f;
  // motor.LPF_velocity.Tf = 0.04f;
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;
  motor.LPF_velocity.Tf = 0.01;
  motor.PID_velocity.output_ramp = 1000;

  motor.P_angle.P = 20;
  motor.P_angle.I = 0;  // usually only P controller is enough
  motor.P_angle.D = 0;  // usually only P controller is enough
  // acceleration control using output ramp
  // this variable is in rad/s^2 and sets the limit of acceleration
  motor.P_angle.output_ramp = 10000;  // default 1e6 rad/s^2
  // angle low pass filtering
  // default 0 - disabled
  // use only for very noisy position sensors - try to avoid and keep the values very small
  motor.LPF_angle.Tf = 0;  // default 0
  // setting the limits
  //  maximal velocity of the position control
  // motor.velocity_limit = 4;  // rad/s - default 20

  motor.monitor_downsample = 4;
  motor.useMonitoring(Serial);

  // set control loop type to be used
  motor.current_limit = 2;
  motor.voltage_limit = 1;
  motor.voltage_sensor_align = 5;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;

  // align encoder and start FOC
  motor.init();
  motor.initFOC();

  pinMode(PC2, OUTPUT);
  command.add('A', onMotor, "motor");
  command.add('L', doLed, "led control");
  command.add('T', doStatus, "driver status");
  command.add('S', onStop, "stop");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println(F("Motor commands sketch | Initial motion control > torque/voltage : target 2V."));

  // Can.begin();
  // Can.setBaudRate(500000);

  _delay(1000);
}

// bool flip = false;
long old_time = 0;
// float target = 5.0f;

bool ledOn = false;
void loop() {
  // iterative setting of the FOC phase voltage
  motor.loopFOC();

  // // iterative function setting the outter loop target
  // // velocity, position or voltage
  // // if target not set in parameter uses motor.target variable
  motor.move(target_velocity);
  // motor.monitor();

  // // user communication
  command.run();

  // if (millis() - old_time > 100) {
  // ledOn = !ledOn;
  // digitalWrite(PC2, ledOn ? LOW : HIGH);
  // Serial.print("Sensor angle: ");
  // Serial.println(sensor.getAngle());

  // CAN_TX_msg.id = (0x321);
  // CAN_TX_msg.len = 8;
  // CAN_TX_msg.buf[0] =  0x42;
  // CAN_TX_msg.buf[1] =  0xad;
  // CAN_TX_msg.buf[2] =  0x12;
  // CAN_TX_msg.buf[3] =  0x34;
  // CAN_TX_msg.buf[4] =  0x56;
  // CAN_TX_msg.buf[5] =  0x78;
  // CAN_TX_msg.buf[6] =  0x9a;
  // CAN_TX_msg.buf[7] =  0xbc;

  // Can.write(CAN_TX_msg);

  //   old_time = millis();
  // }
  // delayMicroseconds(100);
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

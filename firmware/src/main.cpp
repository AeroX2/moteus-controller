#include <Arduino.h>
#include <SoftwareSerial.h>
// #include <HardwareSerial.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/stspin32g4/STSPIN32G4.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

// magnetic sensor instance - I2C
// MagneticSensorI2C sensor = MagneticSensorSPI(AS5600_I2C);

// BLDC motor & driver instance
STSPIN32G4 driver = STSPIN32G4();
BLDCMotor motor = BLDCMotor(7, 0.3, 330);

SoftwareSerial softwareSerial(PC14, PC15);
STM32_CAN Can(FDCAN1, DEF);

static CAN_message_t CAN_TX_msg;

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
  // sensor.init(&twoWire);
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

  Can.begin();
  // Can.enableLoopBack();
  Can.setBaudRate(500000);

  _delay(1000);
}

// bool flip = false;
long old_time = 0;
// float target = 5.0f;

bool ledOn = false;
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

  if (millis() - old_time > 3000) {
    ledOn = !ledOn;
    digitalWrite(PC2, ledOn ? LOW : HIGH);

    CAN_TX_msg.id = (0x321);
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] =  0x42;
    CAN_TX_msg.buf[1] =  0xad;
    CAN_TX_msg.buf[2] =  0x12;
    CAN_TX_msg.buf[3] =  0x34;
    CAN_TX_msg.buf[4] =  0x56;
    CAN_TX_msg.buf[5] =  0x78;
    CAN_TX_msg.buf[6] =  0x9a;
    CAN_TX_msg.buf[7] =  0xbc;
    // CAN_TX_msg.buf[8] =  0xde;
    // CAN_TX_msg.buf[9] =  0xf1;
    // CAN_TX_msg.buf[10] =  0x12;
    // CAN_TX_msg.buf[11] =  0x34;
    // CAN_TX_msg.buf[12] =  0x56;
    // CAN_TX_msg.buf[13] =  0x78;
    // CAN_TX_msg.buf[14] =  0x9a;
    // CAN_TX_msg.buf[15] =  0xab;

    Can.write(CAN_TX_msg);
    
    old_time = millis();
  }
  // delayMicroseconds(100);
}

void SystemClock_Config(void)
{
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

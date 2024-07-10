#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "STM32_CAN.h"
#include "stm32g4xx_hal_opamp.h"
#include "drivers/stspin32g4/STSPIN32G4.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

// Magnetic sensor instance - I2C
MagneticSensorSPI magnetic_sensor = MagneticSensorSPI(AS5048_SPI, PD2);
SPIClass spi_class(PB5, PB4, PB3);

// Low side current sense sensor
LowsideCurrentSense current_sense = LowsideCurrentSense(0.0005f, 32.0 / 7.0 * 1000.0, PB1, _NC, PA2);

// BLDC motor & driver instance
STSPIN32G4 driver = STSPIN32G4();
BLDCMotor motor = BLDCMotor(7, 0.0946, 330, 2.3e-05);

// CAN communication
STM32_CAN Can(FDCAN1, DEF);
static CAN_message_t CAN_TX_msg;

// Commander interface
Commander command = Commander(Serial);
float target_velocity = 0;

void on_stop(char* cmd) {
  motor.disable();
}

void on_motor(char* cmd) {
  // command.motor(&motor, cmd);
  command.scalar(&target_velocity, cmd);
}

void on_led(char* cmd) {
  digitalWrite(PC2, cmd[0] == '0' ? LOW : HIGH);
}

void on_status(char* cmd) {
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

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

void opamp_init(void) {
  // hopamp->Instance = OPAMPx_Def;
  // hopamp->Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  // hopamp->Init.Mode = OPAMP_PGA_MODE;
  // hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  // hopamp->Init.InternalOutput = DISABLE;
  // hopamp->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  // hopamp->Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  // hopamp->Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
  // hopamp->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp1.Init.Mode = OPAMP_PGA_MODE;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_8_OR_MINUS_7;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp2.Init.Mode = OPAMP_PGA_MODE;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = DISABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_8_OR_MINUS_7;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_HIGHSPEED;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = DISABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_8_OR_MINUS_7;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  if (
      HAL_OPAMP_Init(&hopamp1) != HAL_OK &&
      HAL_OPAMP_Init(&hopamp2) != HAL_OK &&
      HAL_OPAMP_Init(&hopamp3) != HAL_OK) {
    Error_Handler();
  }

  if (
      HAL_OPAMP_Start(&hopamp1) != HAL_OK &&
      HAL_OPAMP_Start(&hopamp2) != HAL_OK &&
      HAL_OPAMP_Start(&hopamp3) != HAL_OK) {
    Error_Handler();
  }
}

void setup() {
  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  Serial.println("Initializing CORDIC");
  if (!SimpleFOC_CORDIC_Config())
    Serial.println("CORDIC init failed");

  driver.voltage_power_supply = 24.0f;
  driver.pwm_frequency = 20000;
  // driver.dead_zone = 0.05;
  driver.init();
  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);

  Serial.print("Driver ready: ");
  Serial.println(driver.isReady() ? "true" : "false");
  Serial.print("Driver fault: ");
  Serial.println(driver.isFault() ? "true" : "false");

  // Initialise the op amps for the current sensing
  opamp_init();
  current_sense.init();
  motor.linkCurrentSense(&current_sense);

  // Initialise the magnetic sensor for position sensing
  magnetic_sensor.init(&spi_class);
  motor.linkSensor(&magnetic_sensor);

  // Setup motor limits
  motor.current_limit = 1;
  motor.voltage_limit = 1;
  // motor.velocity_limit = 20;
  // motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = MotionControlType::velocity_openloop;
  // motor.monitor_downsample = 4;
  // motor.useMonitoring(Serial);

  // Align encoder and start FOC
  motor.init();
  motor.initFOC();

  pinMode(PC2, OUTPUT);
  command.add('A', on_motor, "motor");
  command.add('L', on_led, "led control");
  command.add('T', on_status, "driver status");
  command.add('S', on_stop, "stop");

  Can.begin();
  Can.setBaudRate(500000);

  Serial.println("SimpleFOC ready!");

  _delay(1000);
}

long old_time = 0;

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);
  // motor.monitor();

  command.run();

  // if (millis() - old_time > 50) {
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
  // }
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

#include <Arduino.h>

#include "STM32_CAN.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/stspin32g4/STSPIN32G4.h"
// #include "stm32g4xx_hal_opamp.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"

// Magnetic sensor instance - I2C
MagneticSensorSPI magnetic_sensor = MagneticSensorSPI(AS5048_SPI, PD2);
SPIClass spi_class(PB5, PB4, PB3);

// Low side current sense sensor 
// (gain values comes from B-G431B-ESC1, multiplied by 10 because that is what worked)
LowsideCurrentSense current_sense = LowsideCurrentSense(0.01f, -64.0f / 7.0f * 10.0f, PA2, PB1, _NC);

// BLDC motor & driver instance
STSPIN32G4 driver = STSPIN32G4();
BLDCMotor motor = BLDCMotor(7, 0.04);

// CAN communication
STM32_CAN Can(FDCAN1, DEF);
static CAN_message_t CAN_TX_msg;
static CAN_message_t CAN_RX_msg;

// Commander interface
Commander command = Commander(Serial);

void on_stop(char* cmd) {
  motor.disable();
}

void on_motor(char* cmd) {
  command.motor(&motor, cmd);
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

  Serial.print("Driver ready: ");
  Serial.println(driver.isReady() ? "true" : "false");
  Serial.print("Driver fault: ");
  Serial.println(driver.isFault() ? "true" : "false");
}

void on_reset(char* cmd) {
  driver.disable();
  driver.clearFaults();
  driver.enable();
  Serial.print("Driver ready: ");
  Serial.println(driver.isReady() ? "true" : "false");
  Serial.print("Driver fault: ");
  Serial.println(driver.isFault() ? "true" : "false");
}

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp3;

void opamp_init(void) {
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.InternalOutput = DISABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp3.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.InternalOutput = DISABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  if (
      HAL_OPAMP_Init(&hopamp1) != HAL_OK ||
      HAL_OPAMP_Init(&hopamp3) != HAL_OK) {
    Error_Handler();
  }

  if (
      HAL_OPAMP_Start(&hopamp1) != HAL_OK ||
      HAL_OPAMP_Start(&hopamp3) != HAL_OK) {
    Error_Handler();
  }
}

uint32_t id;

void setup() {
  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  Serial.println("Initializing CORDIC");
  if (!SimpleFOC_CORDIC_Config())
    Serial.println("CORDIC init failed");

  driver.voltage_power_supply = 24.0f;
  driver.voltage_limit = 24.0f;
  // driver.pwm_frequency = 40000;
  // driver.dead_zone = 0.03;

  driver.init();
  motor.linkDriver(&driver);
  current_sense.linkDriver(&driver);

  Serial.print("Driver ready: ");
  Serial.println(driver.isReady() ? "true" : "false");
  Serial.print("Driver fault: ");
  Serial.println(driver.isFault() ? "true" : "false");

  // motor init
  motor.init();

  // Initialise the op amps for the current sensing
  opamp_init();
  if (current_sense.init()) {
    Serial.println("Current sense init success!");
  } else {
    Serial.println("Current sense init failed!");
  }
  motor.linkCurrentSense(&current_sense);

  // Initialise the magnetic sensor for position sensing
  magnetic_sensor.init(&spi_class);
  // motor.sensor_direction = Direction::CW;
  motor.linkSensor(&magnetic_sensor);

  // Setup motor limits
  motor.current_limit = 1;
  // motor.voltage_limit = 0.01;
  motor.voltage_sensor_align = 1;
  motor.velocity_limit = 50;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;
  
  // motor.motion_downsample = 5;
  // motor.monitor_downsample = 1000;
  // motor.monitor_variables = _MON_TARGET | _MON_VEL;
  // motor.useMonitoring(Serial);

  motor.PID_current_q.P = 0.5; 
  motor.PID_current_q.I = 3; 
  motor.PID_current_q.D = 0; 

  motor.PID_current_d.P= 0.5;
  motor.PID_current_d.I = 3;
  motor.PID_current_d.D = 0;

  motor.LPF_current_q.Tf = 0.01; 
  motor.LPF_current_d.Tf = 0.01; 
  
  motor.LPF_velocity.Tf = 0.01; 
  motor.LPF_angle.Tf = 0.01; 

  // Align encoder and start FOC
  delay(100);
  motor.initFOC();

  // Please don't start, wait for commands
  motor.disable();

  pinMode(PC2, OUTPUT);
  command.add('M', on_motor, "motor");
  command.add('L', on_led, "led control");
  command.add('T', on_status, "driver status");
  command.add('R', on_reset, "driver reset");
  command.add('S', on_stop, "stop");

  // Can.setAutoBusOffRecovery(true);
  Can.begin();
  Can.setBaudRate(500000);
  id = HAL_GetUIDw0();

  Serial.println("SimpleFOC ready!");

  _delay(1000);
}

long old_time = 0;

void loop() {
  motor.loopFOC();
  motor.move();

  command.run();
  motor.monitor();

  if (Can.read(CAN_RX_msg)) {
    Serial.println((char*)CAN_RX_msg.buf);
    command.run((char*)CAN_RX_msg.buf);
  }

  if (millis() - old_time > 100) {
    float angle = magnetic_sensor.getAngle();
    float velocity = magnetic_sensor.getVelocity();

    union {
      float f;
      unsigned char b[4];
    } angle_to_bytes;
    angle_to_bytes.f = angle;

    union {
      float f;
      unsigned char b[4];
    } velocity_to_bytes;
    velocity_to_bytes.f = velocity;

    CAN_TX_msg.id = id;
    CAN_TX_msg.len = 8;
    CAN_TX_msg.buf[0] = angle_to_bytes.b[0];
    CAN_TX_msg.buf[1] = angle_to_bytes.b[1];
    CAN_TX_msg.buf[2] = angle_to_bytes.b[2];
    CAN_TX_msg.buf[3] = angle_to_bytes.b[3];
    CAN_TX_msg.buf[4] = velocity_to_bytes.b[0];
    CAN_TX_msg.buf[5] = velocity_to_bytes.b[1];
    CAN_TX_msg.buf[6] = velocity_to_bytes.b[2];
    CAN_TX_msg.buf[7] = velocity_to_bytes.b[3];

    Can.write(CAN_TX_msg);

    old_time = millis();
  }

  // _delay(1);
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** 
   * Initializes the RCC Oscillators according to the specified parameters
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

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

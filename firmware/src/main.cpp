#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/stspin32g4/STSPIN32G4.h"
// #include "stm32g4xx_hal_opamp.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
#include "ACANFD_STM32.h"

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
static CANFDMessage CAN_TX_msg;
static CANFDMessage CAN_RX_msg;
static uint32_t id;
static long update_frequency = 100;

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
  // Send status through CAN instead of Serial
  STSPIN32G4Status status = driver.status();
  
  // Use the same CAN ID as other messages
  CAN_TX_msg.id = id;
  CAN_TX_msg.len = 8;
  CAN_TX_msg.ext = true;
  CAN_TX_msg.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH;
  
  // Pack status information into 8 bytes:
  // Byte 0: Message type (0x01 for status)
  // Byte 1: Status flags (lock, vcc_uvlo, vds_p, reset, thsd)
  // Byte 2: Driver ready flag
  // Byte 3: Driver fault flag
  // Bytes 4-7: Reserved for future use
  
  CAN_TX_msg.data[0] = 0x01; // Message type = status
  
  // Pack status flags into byte 1
  uint8_t status_flags = 0;
  if (status.lock) status_flags |= 0x01;
  if (status.vcc_uvlo) status_flags |= 0x02;
  if (status.vds_p) status_flags |= 0x04;
  if (status.reset) status_flags |= 0x08;
  if (status.thsd) status_flags |= 0x10;
  CAN_TX_msg.data[1] = status_flags;
  
  CAN_TX_msg.data[2] = driver.isReady() ? 0x01 : 0x00;
  CAN_TX_msg.data[3] = driver.isFault() ? 0x01 : 0x00;
  
  // Reserved bytes
  CAN_TX_msg.data[4] = 0x00;
  CAN_TX_msg.data[5] = 0x00;
  CAN_TX_msg.data[6] = 0x00;
  CAN_TX_msg.data[7] = 0x00;
  
  fdcan1.tryToSendReturnStatusFD(CAN_TX_msg);
  
  // Also print to Serial for debugging
  Serial.println("Driver status sent via CAN");
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

void on_update_frequency(char* cmd) {
  update_frequency = atoi(cmd);

  Serial.print("Setting CAN update frequency to ");
  Serial.print(update_frequency);
  Serial.println("ms");
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
  motor.PID_current_q.I = 0; 
  motor.PID_current_q.D = 0; 

  motor.PID_current_d.P = 0.5;
  motor.PID_current_d.I = 0;
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
  command.add('U', on_update_frequency, "update frequency");

  id = HAL_GetUIDw0();
  
  // Configure ACANFD_STM32 for FD mode with bit rate switching
  ACANFD_STM32_Settings settings(1000000, DataBitRateFactor::x5);
  
  ACANFD_STM32_StandardFilters standardFilters;
  standardFilters.addSingle(id, ACANFD_STM32_FilterAction::FIFO0);
  settings.mNonMatchingStandardFrameReception = ACANFD_STM32_FilterAction::REJECT;
  
  const uint32_t errorCode = fdcan1.beginFD(settings, standardFilters);
  if (errorCode == 0) {
    Serial.println("CANFD configuration OK");
  } else {
    Serial.print("CANFD configuration error: 0x");
    Serial.println(errorCode, HEX);
  }

  Serial.print("My CAN ID: 0x");
  Serial.println(id, HEX);

  Serial.println("SimpleFOC ready!");

  _delay(1000);
}

long old_time = 0;

void loop() {
  motor.loopFOC();
  motor.move();

  command.run();
  // motor.monitor();

  if (fdcan1.receiveFD0(CAN_RX_msg)) {
    Serial.println((char*)CAN_RX_msg.data);
    command.run((char*)CAN_RX_msg.data);
  }

  if (millis() - old_time > update_frequency) {
    float angle = magnetic_sensor.getAngle();
    float velocity = magnetic_sensor.getVelocity();

    union {
      float f;
      unsigned char b[4];
    } angle_to_bytes;
    angle_to_bytes.f = motor.sensor_direction == Direction::CW ? angle : -angle;

    union {
      float f;
      unsigned char b[4];
    } velocity_to_bytes;
    velocity_to_bytes.f = velocity;

    // Message type: 0x00 = angle/velocity data
    CAN_TX_msg.id = id; // Standard ID for angle/velocity (no type bits)
    CAN_TX_msg.len = 12;
    CAN_TX_msg.ext = true;
    CAN_TX_msg.type = CANFDMessage::CANFD_WITH_BIT_RATE_SWITCH;
    CAN_TX_msg.data[0] = 0x00; // Message type = angle/velocity
    CAN_TX_msg.data[1] = angle_to_bytes.b[0];
    CAN_TX_msg.data[2] = angle_to_bytes.b[1];
    CAN_TX_msg.data[3] = angle_to_bytes.b[2];
    CAN_TX_msg.data[4] = angle_to_bytes.b[3];
    CAN_TX_msg.data[5] = velocity_to_bytes.b[0];
    CAN_TX_msg.data[6] = velocity_to_bytes.b[1];
    CAN_TX_msg.data[7] = velocity_to_bytes.b[2];
    CAN_TX_msg.data[8] = velocity_to_bytes.b[3];
    CAN_TX_msg.data[9] = 0x00;
    CAN_TX_msg.data[10] = 0x00;
    CAN_TX_msg.data[11] = 0x00;

    fdcan1.tryToSendReturnStatusFD(CAN_TX_msg);

    old_time = millis();

    // PhaseCurrent_s phase_currents = current_sense.getPhaseCurrents();

    // Serial.print("Phase currents: ");
    // Serial.print(phase_currents.a);
    // Serial.print(" ");
    // Serial.print(phase_currents.b);
    // Serial.print(" ");
    // Serial.println(phase_currents.c);
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
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

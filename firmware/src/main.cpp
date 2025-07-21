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
static uint8_t init_errors = 0;

// Consolidated CAN send function
void send_can_message(uint8_t message_type, const uint8_t* data, uint8_t data_len = 8) {
  if (id == 0) return; // Only send if CAN is initialized
  
  CAN_TX_msg.id = id;
  CAN_TX_msg.len = data_len;
  CAN_TX_msg.ext = true;
  CAN_TX_msg.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH;
  
  CAN_TX_msg.data[0] = message_type;
  
  // Copy data to remaining bytes (CAN FD can handle up to 64 bytes)
  for (int i = 0; i < data_len - 1 && i < 63; i++) {
    CAN_TX_msg.data[1 + i] = data[i];
  }
  
  // Pad remaining bytes with zeros
  for (int i = data_len - 1; i < 63; i++) {
    CAN_TX_msg.data[1 + i] = 0x00;
  }
  
  fdcan1.tryToSendReturnStatusFD(CAN_TX_msg);
}

// Custom CAN Debug Interface - acts like Serial but sends via CAN
class CANDebugInterface : public Print {
private:
  char message_buffer[64];
  int buffer_pos = 0;
  
public:
  virtual size_t write(uint8_t c) override {
    if (c == '\n' || c == '\r') {
      // Send complete message when we hit newline
      if (buffer_pos > 0) {
        message_buffer[buffer_pos] = '\0';
        send_debug_via_can(message_buffer);
        buffer_pos = 0;
      }
    } else {
      // Add character to buffer
      if (buffer_pos < 63) {
        message_buffer[buffer_pos++] = c;
      }
    }
    return 1;
  }
  
  virtual size_t write(const uint8_t *buffer, size_t size) override {
    for (size_t i = 0; i < size; i++) {
      write(buffer[i]);
    }
    return size;
  }
  
private:
  void send_debug_via_can(const char* msg) {
    if (id == 0) return; // Only send if CAN is initialized
    
    // Use full CAN FD capacity (64 bytes)
    uint8_t debug_data[63] = {0}; // 63 bytes for data + 1 byte for message type
    
    // Copy message to debug data
    int msg_len = strlen(msg);
    int copy_len = (msg_len > 62) ? 62 : msg_len; // Leave room for null terminator
    
    for (int i = 0; i < copy_len; i++) {
      debug_data[i] = msg[i];
    }
    
    send_can_message(0x04, debug_data, 64);
  }
};

// Global CAN debug interface instance
CANDebugInterface CANDebug;

// CAN Debug function
void send_can_debug(const __FlashStringHelper* msg, ...) {
  // Create debug message
  char debug_buffer[64];
  strcpy_P(debug_buffer, (const char*)msg);
  
  // Pack debug message into bytes (truncate if longer than 7 chars)
  uint8_t debug_data[7] = {0};
  for (int i = 0; i < 7 && debug_buffer[i] != '\0'; i++) {
    debug_data[i] = debug_buffer[i];
  }
  
  send_can_message(0x04, debug_data, 8);
}

// CAN Debug Stream wrapper - makes CANDebugInterface compatible with Stream
class CANDebugStream : public Stream {
private:
  CANDebugInterface* can_debug;
  
public:
  CANDebugStream(CANDebugInterface* debug) : can_debug(debug) {}
  
  // Stream interface methods
  int available() override { return 0; } // No input available
  int read() override { return -1; } // No input
  int peek() override { return -1; } // No input
  
  // Print interface methods (inherited from Print)
  size_t write(uint8_t c) override {
    if (can_debug) return can_debug->write(c);
    return 0;
  }
  
  size_t write(const uint8_t *buffer, size_t size) override {
    if (can_debug) return can_debug->write(buffer, size);
    return 0;
  }
};

// Commander interface
CANDebugStream can_stream(&CANDebug);
Commander command = Commander(can_stream);

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
  
  // Pack status information into 8 bytes:
  // Byte 0: Message type (0x01 for status)
  // Byte 1: Status flags (lock, vcc_uvlo, vds_p, reset, thsd)
  // Byte 2: Driver ready flag
  // Byte 3: Driver fault flag
  // Bytes 4-7: Reserved for future use
  
  // Pack status flags into byte 1
  uint8_t status_flags = 0;
  if (status.lock) status_flags |= 0x01;
  if (status.vcc_uvlo) status_flags |= 0x02;
  if (status.vds_p) status_flags |= 0x04;
  if (status.reset) status_flags |= 0x08;
  if (status.thsd) status_flags |= 0x10;
  
  uint8_t status_data[7] = {
    status_flags,
    driver.isReady() ? 0x01 : 0x00,
    driver.isFault() ? 0x01 : 0x00,
    0x00, 0x00, 0x00, 0x00  // Reserved bytes
  };
  
  send_can_message(0x01, status_data, 8);
  
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

void on_init_status(char* cmd) {
  // Send initialization status through CAN
  uint8_t init_data[7] = {
    init_errors, // Error flags
    motor.motor_status,
    0x00, 0x00, 0x00, 0x00, 0x00  // Reserved bytes
  };
  
  send_can_message(0x03, init_data, 8);
  
  Serial.println("Initialization status sent via CAN");
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

  id = HAL_GetUIDw0();
  
  // Configure ACANFD_STM32 for FD mode with bit rate switching
  ACANFD_STM32_Settings settings(1000000, DataBitRateFactor::x5);
  
  ACANFD_STM32_StandardFilters standardFilters;
  standardFilters.addSingle(id, ACANFD_STM32_FilterAction::FIFO0);
  settings.mNonMatchingStandardFrameReception = ACANFD_STM32_FilterAction::REJECT;

  ACANFD_STM32_ExtendedFilters extendedFilters;
  extendedFilters.addSingle(id, ACANFD_STM32_FilterAction::FIFO0);
  settings.mNonMatchingExtendedFrameReception = ACANFD_STM32_FilterAction::REJECT;
  
  const uint32_t errorCode = fdcan1.beginFD(settings, standardFilters, extendedFilters);
  if (errorCode == 0) {
    Serial.println("CANFD configuration OK");
  } else {
    Serial.print("CANFD configuration error: 0x");
    Serial.println(errorCode, HEX);
  }

  Serial.print("My CAN ID: 0x");
  Serial.println(id, HEX);

  SimpleFOCDebug::enable(&CANDebug);

  Serial.println("Initializing CORDIC");
  if (!SimpleFOC_CORDIC_Config()) {
    Serial.println("CORDIC init failed");
    init_errors |= 0x01;
  }

  driver.voltage_power_supply = 24.0f;
  driver.voltage_limit = 24.0f;

  // Check driver initialization - returns 1 for success, 0 for failure
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
  
  // Additional driver status check
  if (driver.isFault()) {
    init_errors |= 0x02;
  }

  // motor init - returns 1 for success, 0 for failure
  if (motor.init() == 0) {
    Serial.println("Motor init failed!");
    init_errors |= 0x08;
  } else {
    Serial.println("Motor init success!");
  }

  // Initialise the op amps for the current sensing
  opamp_init();
  if (current_sense.init()) {
    Serial.println("Current sense init success!");
  } else {
    Serial.println("Current sense init failed!");
    init_errors |= 0x04;
  }
  motor.linkCurrentSense(&current_sense);

  // Initialise the magnetic sensor for position sensing
  magnetic_sensor.init(&spi_class);
  motor.linkSensor(&magnetic_sensor);

  motor.monitor_downsample = 0;
  motor.monitor_variables = 0;
  motor.useMonitoring(CANDebug);

  // Setup motor limits
  motor.current_limit = 0.4;
  motor.voltage_sensor_align = 1.0;
  motor.velocity_limit = 50.0;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;

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

  // Align encoder and start FOC - returns 1 for success, 0 for failure
  delay(100);
  if (motor.initFOC() == 0) {
    Serial.println("Motor FOC init failed!");
    init_errors |= 0x10;
  } else {
    Serial.println("Motor FOC init success!");
  }

  // Please don't start, wait for commands
  motor.disable();

  pinMode(PC2, OUTPUT);
  command.add('M', on_motor, "motor");
  command.add('L', on_led, "led control");
  command.add('T', on_status, "driver status");
  command.add('R', on_reset, "driver reset");
  command.add('S', on_stop, "stop");
  command.add('U', on_update_frequency, "update frequency");
  command.add('I', on_init_status, "init status");
  // Send final initialization status
  if (init_errors == 0) {
    Serial.println("All components initialized successfully!");
  } else {
    Serial.print("Initialization completed with errors: 0x");
    Serial.println(init_errors, HEX);
  }

  Serial.println("SimpleFOC ready!");

  _delay(1000);
}

long old_time = 0;

void loop() {
  motor.loopFOC();
  motor.move();
  motor.monitor();

  command.run();

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
    uint8_t motion_data[11] = {
      angle_to_bytes.b[0], angle_to_bytes.b[1], angle_to_bytes.b[2], angle_to_bytes.b[3],
      velocity_to_bytes.b[0], velocity_to_bytes.b[1], velocity_to_bytes.b[2], velocity_to_bytes.b[3],
      0x00, 0x00, 0x00
    };

    send_can_message(0x00, motion_data, 12);

    old_time = millis();
  }
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

#include <Arduino.h>

#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/stspin32g4/STSPIN32G4.h"
#include "stm32g4xx_hal_opamp.h"
#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"
#include "utilities/stm32math/STM32G4CORDICTrigFunctions.h"
#include "ACANFD_STM32.h"
#include <cstring>

// Magnetic sensor instance - I2C
MagneticSensorSPI magnetic_sensor = MagneticSensorSPI(AS5048_SPI, PD2);
SPIClass spi_class(PB5, PB4, PB3);

// Low side current sense sensor 
// (gain values comes from B-G431B-ESC1, multiplied by 10 because that is what worked)
LowsideCurrentSense current_sense = LowsideCurrentSense(0.01f, -64.0f / 7.0f * 10.0f, PB1, _NC, PA2);

// BLDC motor & driver instance
STSPIN32G4 driver = STSPIN32G4();
BLDCMotor motor = BLDCMotor(7);  // Reverted to 7PP per user confirmation

// CAN communication
static CANFDMessage CAN_TX_msg;
static CANFDMessage CAN_RX_msg;
static uint32_t id;
static long update_frequency = 100;
static uint8_t init_errors = 0;

// Consolidated CAN send function with internal CAN FD DLC rounding
void send_can_message(uint8_t message_type, const uint8_t* data, uint8_t data_len = 8) {
  if (id == 0) return; // Only send if CAN is initialized

  // Ensure at least room for message_type
  if (data_len == 0) data_len = 1;
  // Round to valid CAN FD payload size
  auto round_dlc = [](uint8_t n)->uint8_t {
    if (n <= 8)  return n;      // 0..8 contiguous
    if (n <= 12) return 12;
    if (n <= 16) return 16;
    if (n <= 20) return 20;
    if (n <= 24) return 24;
    if (n <= 32) return 32;
    if (n <= 48) return 48;
    return 64;
  };
  uint8_t dlc = round_dlc(data_len);

  CAN_TX_msg.id = id;
  CAN_TX_msg.len = dlc;
  CAN_TX_msg.ext = true;
  CAN_TX_msg.type = CANFDMessage::CANFD_NO_BIT_RATE_SWITCH;
  CAN_TX_msg.data[0] = message_type;

  int copy_bytes = (data_len > 1) ? (data_len - 1) : 0;
  if (copy_bytes > 63) copy_bytes = 63;
  for (int i = 0; i < copy_bytes; i++) {
    CAN_TX_msg.data[1 + i] = data[i];
  }
  // Pad remaining bytes up to dlc-1
  for (int i = copy_bytes; i < (dlc - 1); i++) {
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
  void send_string(const char* msg) {
    if (id == 0 || msg == nullptr) return;
    size_t total = strlen(msg);
    size_t offset = 0;
    while (offset < total) {
      size_t chunk = total - offset;
      if (chunk > 63) chunk = 63; // max per frame (excluding message type)
      uint8_t frame[63] = {0};
      if (chunk) memcpy(frame, msg + offset, chunk);
      send_can_message(0x04, frame, static_cast<uint8_t>(chunk + 1));
      offset += chunk;
    }
  }

  virtual size_t write(uint8_t c) override {
    if (c == '\n' || c == '\r') {
      if (buffer_pos > 0) {
        message_buffer[buffer_pos] = '\0';
        send_string(message_buffer);
        buffer_pos = 0;
      }
    } else {
      if (buffer_pos >= 63) {
        // buffer full - send chunk and start new line continuation
        message_buffer[63-1] = '\0';
        send_string(message_buffer);
        buffer_pos = 0;
      }
      message_buffer[buffer_pos++] = c;
    }
    return 1;
  }
  
  virtual size_t write(const uint8_t *buffer, size_t size) override {
    for (size_t i = 0; i < size; i++) {
      write(buffer[i]);
    }
    return size;
  }

  void flush_buffer() {
    if (buffer_pos > 0) {
      message_buffer[buffer_pos] = '\0';
      send_string(message_buffer);
      buffer_pos = 0;
    }
  }
};

// Global CAN debug interface instance
CANDebugInterface CANDebug;

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

// PID storage in flash (last 2KB page of 128KB: 0x0801F800)
#define PID_FLASH_MAGIC 0x50494453u  // "PIDS"
#define PID_FLASH_PAGE 63
#define PID_FLASH_ADDR (0x08000000u + (PID_FLASH_PAGE * 2048u))

#pragma pack(push, 1)
struct PidFlashData {
  uint32_t magic;
  float cq_p, cq_i, cq_ramp;
  float cd_p, cd_i, cd_ramp;
  float v_p, v_i, v_d, v_ramp;
  float angle_p, angle_limit;
  float velocity_limit;
};
#pragma pack(pop)

// Commander interface
CANDebugStream can_stream(&CANDebug);
Commander command = Commander(can_stream);

void on_stop(char* cmd) {
  motor.disable();
}

float user_target = 0;

void on_motor(char* cmd) {
  command.motor(&motor, cmd);
}

void print_pid_gains() {
  CANDebug.print("PID cq: P="); CANDebug.print(motor.PID_current_q.P, 6);
  CANDebug.print(" I="); CANDebug.print(motor.PID_current_q.I, 6);
  CANDebug.print(" ramp="); CANDebug.println(motor.PID_current_q.output_ramp, 3);

  CANDebug.print("PID cd: P="); CANDebug.print(motor.PID_current_d.P, 6);
  CANDebug.print(" I="); CANDebug.print(motor.PID_current_d.I, 6);
  CANDebug.print(" ramp="); CANDebug.println(motor.PID_current_d.output_ramp, 3);

  CANDebug.print("PID v : P="); CANDebug.print(motor.PID_velocity.P, 6);
  CANDebug.print(" I="); CANDebug.print(motor.PID_velocity.I, 6);
  CANDebug.print(" D="); CANDebug.print(motor.PID_velocity.D, 6);
  CANDebug.print(" ramp="); CANDebug.println(motor.PID_velocity.output_ramp, 3);

  CANDebug.print("P angle: P="); CANDebug.print(motor.P_angle.P, 6);
  CANDebug.print(" limit="); CANDebug.println(motor.P_angle.limit, 6);
}

void on_pid(char* cmd) {
  // Tokenize in-place to avoid sscanf float parsing issues on embedded builds.
  char* mode = strtok(cmd, " \t");
  if (mode == nullptr || mode[0] == '?') {
    print_pid_gains();
    CANDebug.println("Usage: P cq P I [ramp] | P cd P I [ramp] | P v P I D [ramp] | P a P [limit] | P vl limit");
    return;
  }

  auto parse_float = [](char* tok, float& out) -> bool {
    if (tok == nullptr) return false;
    char* end = nullptr;
    out = strtof(tok, &end);
    return end != tok && *end == '\0';
  };

  float p = 0.0f, i = 0.0f, d = 0.0f, ramp = 0.0f, limit = 0.0f;

  if (strcmp(mode, "cq") == 0) {
    char* p_tok = strtok(nullptr, " \t");
    char* i_tok = strtok(nullptr, " \t");
    char* r_tok = strtok(nullptr, " \t");
    if (!parse_float(p_tok, p) || !parse_float(i_tok, i)) {
      CANDebug.println("Invalid cq args");
      return;
    }
    motor.PID_current_q.P = p;
    motor.PID_current_q.I = i;
    if (parse_float(r_tok, ramp)) motor.PID_current_q.output_ramp = ramp;
  } else if (strcmp(mode, "cd") == 0) {
    char* p_tok = strtok(nullptr, " \t");
    char* i_tok = strtok(nullptr, " \t");
    char* r_tok = strtok(nullptr, " \t");
    if (!parse_float(p_tok, p) || !parse_float(i_tok, i)) {
      CANDebug.println("Invalid cd args");
      return;
    }
    motor.PID_current_d.P = p;
    motor.PID_current_d.I = i;
    if (parse_float(r_tok, ramp)) motor.PID_current_d.output_ramp = ramp;
  } else if (strcmp(mode, "v") == 0) {
    char* p_tok = strtok(nullptr, " \t");
    char* i_tok = strtok(nullptr, " \t");
    char* d_tok = strtok(nullptr, " \t");
    char* r_tok = strtok(nullptr, " \t");
    if (!parse_float(p_tok, p) || !parse_float(i_tok, i) || !parse_float(d_tok, d)) {
      CANDebug.println("Invalid v args");
      return;
    }
    motor.PID_velocity.P = p;
    motor.PID_velocity.I = i;
    motor.PID_velocity.D = d;
    if (parse_float(r_tok, ramp)) motor.PID_velocity.output_ramp = ramp;
  } else if (strcmp(mode, "a") == 0) {
    char* p_tok = strtok(nullptr, " \t");
    char* l_tok = strtok(nullptr, " \t");
    if (!parse_float(p_tok, p)) {
      CANDebug.println("Invalid a args");
      return;
    }
    motor.P_angle.P = p;
    if (parse_float(l_tok, limit)) motor.P_angle.limit = limit;
  } else if (strcmp(mode, "vl") == 0) {
    char* l_tok = strtok(nullptr, " \t");
    if (!parse_float(l_tok, limit)) {
      CANDebug.println("Invalid vl args");
      return;
    }
    motor.velocity_limit = limit;
    CANDebug.print("Set motor.velocity_limit = ");
    CANDebug.println(motor.velocity_limit, 6);
  } else {
    CANDebug.println("Unknown PID mode. Use cq, cd, v, a, or ?");
    return;
  }

  print_pid_gains();
}

static bool save_pid_to_flash(void) {
  PidFlashData data = {};
  data.magic = PID_FLASH_MAGIC;
  data.cq_p = motor.PID_current_q.P;
  data.cq_i = motor.PID_current_q.I;
  data.cq_ramp = motor.PID_current_q.output_ramp;
  data.cd_p = motor.PID_current_d.P;
  data.cd_i = motor.PID_current_d.I;
  data.cd_ramp = motor.PID_current_d.output_ramp;
  data.v_p = motor.PID_velocity.P;
  data.v_i = motor.PID_velocity.I;
  data.v_d = motor.PID_velocity.D;
  data.v_ramp = motor.PID_velocity.output_ramp;
  data.angle_p = motor.P_angle.P;
  data.angle_limit = motor.P_angle.limit;
  data.velocity_limit = motor.velocity_limit;

  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = PID_FLASH_PAGE;
  erase.NbPages = 1;
  uint32_t pageError = 0;
  if (HAL_FLASHEx_Erase(&erase, &pageError) != HAL_OK) {
    HAL_FLASH_Lock();
    return false;
  }
  const uint8_t* src = (const uint8_t*)&data;
  uint32_t addr = PID_FLASH_ADDR;
  size_t len = sizeof(PidFlashData);
  for (size_t i = 0; i < len; i += 8) {
    uint64_t dword = 0;
    for (int j = 0; j < 8 && (i + j) < len; j++) {
      ((uint8_t*)&dword)[j] = src[i + j];
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dword) != HAL_OK) {
      HAL_FLASH_Lock();
      return false;
    }
  }
  HAL_FLASH_Lock();
  return true;
}

static bool load_pid_from_flash(void) {
  const PidFlashData* data = (const PidFlashData*)PID_FLASH_ADDR;
  if (data->magic != PID_FLASH_MAGIC) return false;
  motor.PID_current_q.P = data->cq_p;
  motor.PID_current_q.I = data->cq_i;
  motor.PID_current_q.output_ramp = data->cq_ramp;
  motor.PID_current_d.P = data->cd_p;
  motor.PID_current_d.I = data->cd_i;
  motor.PID_current_d.output_ramp = data->cd_ramp;
  motor.PID_velocity.P = data->v_p;
  motor.PID_velocity.I = data->v_i;
  motor.PID_velocity.D = data->v_d;
  motor.PID_velocity.output_ramp = data->v_ramp;
  motor.P_angle.P = data->angle_p;
  motor.P_angle.limit = data->angle_limit;
  motor.velocity_limit = data->velocity_limit;
  return true;
}

void on_save_pid(char* cmd) {
  (void)cmd;
  if (save_pid_to_flash()) {
    CANDebug.println("PID saved to flash.");
  } else {
    CANDebug.println("PID save failed.");
  }
}

void on_led(char* cmd) {
  digitalWrite(PC2, cmd[0] == '0' ? LOW : HIGH);
}

void on_status(char* cmd) {
  STSPIN32G4Status status = driver.status();
  STSPIN32G4NFault nfault = driver.getNFaultRegister();
  STSPIN32G4Ready ready = driver.getReadyRegister();
  
  uint8_t status_data[6] = {
    status.reg,
    (uint8_t)(driver.isReady() ? 0x01 : 0x00),
    (uint8_t)(driver.isFault() ? 0x01 : 0x00),
    nfault.reg,
    ready.reg,
    0x00
  };
  
  send_can_message(0x01, status_data, 8);
  
  // Serial diagnostics
  Serial.print("Status: 0x"); Serial.print(status.reg, HEX);
  Serial.print(" | NFault: 0x"); Serial.print(nfault.reg, HEX);
  Serial.print(" | Ready: 0x"); Serial.println(ready.reg, HEX);
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
  // hopamp1.Init.InternalOutput = ENABLE;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_32_OR_MINUS_31;

  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp3.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  // hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  // hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  // hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_32_OR_MINUS_31;

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
  #ifndef CAN_DEBUG
  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200);
  #endif

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
  // magnetic_sensor.min_elapsed_time = 0.00075;
  motor.linkSensor(&magnetic_sensor);

  motor.monitor_downsample = 0;
  motor.monitor_variables = 0;
  motor.useMonitoring(CANDebug);

  // Setup motor limits - keep safe but allow enough drive to overcome stiction
  motor.current_limit = 3.0;
  motor.voltage_limit = 8.0;      
  motor.velocity_limit = 50.0;
  motor.voltage_sensor_align = 1.5;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;

  // Inner Current Loop PID - Diagnostic Stability Baseline
  motor.PID_current_q.P = 0.02;      // Lower P for safer initial stability
  motor.PID_current_q.I = 10.0;
  motor.PID_current_q.output_ramp = 1000.0;  
  motor.PID_current_d.P = 0.02;
  motor.PID_current_d.I = 10.0;
  motor.PID_current_d.output_ramp = 1000.0;
  
  motor.LPF_current_q.Tf = 0.001;    // Sharpened to 1ms to stabilize commutation
  motor.LPF_current_d.Tf = 0.001;     
  
  // Lower velocity filtering to reduce lag/choppiness at low speed
  motor.LPF_velocity.Tf = 0.01;      

  // Outer Loops - tuned values from on-device testing
  motor.PID_velocity.P = 0.35;
  motor.PID_velocity.I = 0.01;
  motor.PID_velocity.D = 0.007;
  motor.PID_velocity.output_ramp = 250.0;
  
  // Angle loop tuning
  motor.P_angle.P = 6.5;
  motor.P_angle.limit = 40.0;

  // Load saved PID values from flash if present
  if (load_pid_from_flash()) {
    Serial.println("Loaded PID values from flash.");
  }

  // Align encoder and start FOC - Automatic Calibration
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
  command.add('P', on_pid, "pid tune/print");
  command.add('W', on_save_pid, "save PID to flash");
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
  command.run();

  // Safety: Sync target to current position while disabled 
  // to prevent a jump-to-zero when enabling.
  if (!motor.enabled) {
    motor.target = motor.shaft_angle;
  }

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

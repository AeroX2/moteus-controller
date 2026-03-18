#include "commands/commands.h"

#include "app/app_context.h"
#include "debug/can_debug.h"
#include "storage/pid_flash.h"

#include <cstring>

static CANDebugStream can_stream(&CANDebug);
Commander command = Commander(can_stream);

static void print_pid_gains() {
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

static void on_stop(char* cmd) {
  (void)cmd;
  motor.disable();
}

static void on_motor(char* cmd) {
  command.motor(&motor, cmd);
}

static void on_pid(char* cmd) {
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

static void on_save_pid(char* cmd) {
  (void)cmd;
  if (save_pid_to_flash()) CANDebug.println("PID saved to flash.");
  else CANDebug.println("PID save failed.");
}

static void on_led(char* cmd) {
  digitalWrite(PC2, cmd[0] == '0' ? LOW : HIGH);
}

static void on_status(char* cmd) {
  (void)cmd;
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
}

static void on_reset(char* cmd) {
  (void)cmd;
  driver.disable();
  driver.clearFaults();
  driver.enable();
}

static void on_update_frequency(char* cmd) {
  update_frequency = atoi(cmd);
}

static void on_controller_mode(char* cmd) {
  if (cmd == nullptr) {
    CANDebug.println("Usage: C a|v  (a=angle, v=velocity)");
    return;
  }

  while (*cmd == ' ' || *cmd == '\t') { cmd++; }
  char mode = *cmd;
  if (mode != 'a' && mode != 'v') {
    CANDebug.println("Usage: C a|v  (a=angle, v=velocity)");
    return;
  }

  if (mode == 'v') {
    motor.controller = MotionControlType::velocity;
    motor.target = 0.0f;
    CANDebug.println("Controller mode: velocity");
  } else {
    motor.controller = MotionControlType::angle;
    motor.target = motor.shaft_angle;
    CANDebug.println("Controller mode: angle");
  }
}

static void on_init_status(char* cmd) {
  (void)cmd;
  uint8_t init_data[7] = {
    init_errors,
    motor.motor_status,
    0x00, 0x00, 0x00, 0x00, 0x00
  };
  send_can_message(0x03, init_data, 8);
}

void register_commands() {
  command.add('M', on_motor, "motor");
  command.add('L', on_led, "led control");
  command.add('T', on_status, "driver status");
  command.add('R', on_reset, "driver reset");
  command.add('S', on_stop, "stop");
  command.add('U', on_update_frequency, "update frequency");
  command.add('C', on_controller_mode, "controller mode (a=angle, v=velocity)");
  command.add('I', on_init_status, "init status");
  command.add('P', on_pid, "pid tune/print");
  command.add('W', on_save_pid, "save PID to flash");
}

void run_command_loop() {
  command.run();
  if (fdcan1.receiveFD0(CAN_RX_msg)) {
    Serial.println((char*)CAN_RX_msg.data);
    command.run((char*)CAN_RX_msg.data);
  }
}


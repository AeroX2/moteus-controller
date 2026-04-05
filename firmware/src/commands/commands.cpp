#include "commands/commands.h"

#include "debug/can_debug.h"
#include "drivers/motor_objects.h"
#include "platform/fdcan.h"
#include "storage/pid_flash.h"
#include "telemetry/messages.h"

#include <cstring>

using namespace telemetry;

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
  BLDCMotor* m = &motor;

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
    m->PID_current_q.P = p;
    m->PID_current_q.I = i;
    if (parse_float(r_tok, ramp)) m->PID_current_q.output_ramp = ramp;
  } else if (strcmp(mode, "cd") == 0) {
    char* p_tok = strtok(nullptr, " \t");
    char* i_tok = strtok(nullptr, " \t");
    char* r_tok = strtok(nullptr, " \t");
    if (!parse_float(p_tok, p) || !parse_float(i_tok, i)) {
      CANDebug.println("Invalid cd args");
      return;
    }
    m->PID_current_d.P = p;
    m->PID_current_d.I = i;
    if (parse_float(r_tok, ramp)) m->PID_current_d.output_ramp = ramp;
  } else if (strcmp(mode, "v") == 0) {
    char* p_tok = strtok(nullptr, " \t");
    char* i_tok = strtok(nullptr, " \t");
    char* d_tok = strtok(nullptr, " \t");
    char* r_tok = strtok(nullptr, " \t");
    if (!parse_float(p_tok, p) || !parse_float(i_tok, i) || !parse_float(d_tok, d)) {
      CANDebug.println("Invalid v args");
      return;
    }
    m->PID_velocity.P = p;
    m->PID_velocity.I = i;
    m->PID_velocity.D = d;
    if (parse_float(r_tok, ramp)) m->PID_velocity.output_ramp = ramp;
  } else if (strcmp(mode, "a") == 0) {
    char* p_tok = strtok(nullptr, " \t");
    char* l_tok = strtok(nullptr, " \t");
    if (!parse_float(p_tok, p)) {
      CANDebug.println("Invalid a args");
      return;
    }
    m->P_angle.P = p;
    if (parse_float(l_tok, limit)) m->P_angle.limit = limit;
  } else if (strcmp(mode, "vl") == 0) {
    char* l_tok = strtok(nullptr, " \t");
    if (!parse_float(l_tok, limit)) {
      CANDebug.println("Invalid vl args");
      return;
    }
    m->velocity_limit = limit;
    CANDebug.print("Set motor.velocity_limit = ");
    CANDebug.println(m->velocity_limit, 6);
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
  STSPIN32G4* d = &driver;
  STSPIN32G4Status status = d->status();
  STSPIN32G4NFault nfault = d->getNFaultRegister();
  STSPIN32G4Ready ready = d->getReadyRegister();

  telemetry::DriverStatusPayload payload = {};
  payload.status_reg = status.reg;
  payload.ready = d->isReady() ? 1 : 0;
  payload.fault = d->isFault() ? 1 : 0;
  payload.nfault_reg = nfault.reg;
  payload.ready_reg = ready.reg;

  uint8_t buf[telemetry::DRIVER_STATUS_PAYLOAD_SIZE];
  telemetry::driver_status_encode(&payload, buf);
  send_can_message(telemetry::CanMsgType::DriverStatus, buf,
                   telemetry::DRIVER_STATUS_PAYLOAD_SIZE);
}

static void on_reset(char* cmd) {
  NVIC_SystemReset();
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

  BLDCMotor* m = &motor;
  if (mode == 'v') {
    m->controller = MotionControlType::velocity;
    m->target = 0.0f;
    CANDebug.println("Controller mode: velocity");
  } else {
    m->controller = MotionControlType::angle;
    m->target = m->shaft_angle;
    CANDebug.println("Controller mode: angle");
  }
}

static void on_init_status(char* cmd) {
  (void)cmd;
  telemetry::InitStatusPayload payload = {};
  payload.init_errors = init_errors;
  payload.motor_status = motor.motor_status;

  uint8_t buf[telemetry::INIT_STATUS_PAYLOAD_SIZE];
  telemetry::init_status_encode(&payload, buf);
  send_can_message(telemetry::CanMsgType::InitStatus, buf,
                   telemetry::INIT_STATUS_PAYLOAD_SIZE);
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
  CANFDMessage rx_msg;
  if (fdcan_recv(&rx_msg)) {
    Serial.println((char*)rx_msg.data);
    command.run((char*)rx_msg.data);
  }
}


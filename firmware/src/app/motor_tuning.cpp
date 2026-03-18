#include "app/motor_tuning.h"

#include "app/app_context.h"

void motor_apply_tuning_defaults() {
  // Setup motor limits - keep safe but allow enough drive to overcome stiction
  motor.current_limit = 3.0;
  motor.voltage_limit = 8.0;
  motor.velocity_limit = 100.0;
  motor.voltage_sensor_align = 1.5;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.controller = MotionControlType::angle;

  // Inner Current Loop PID - Diagnostic Stability Baseline
  motor.PID_current_q.P = 0.02;
  motor.PID_current_q.I = 10.0;
  motor.PID_current_q.output_ramp = 1000.0;
  motor.PID_current_d.P = 0.02;
  motor.PID_current_d.I = 10.0;
  motor.PID_current_d.output_ramp = 1000.0;

  motor.LPF_current_q.Tf = 0.001;
  motor.LPF_current_d.Tf = 0.001;

  // Lower velocity filtering to reduce lag/choppiness at low speed
  motor.LPF_velocity.Tf = 0.02;

  // Outer Loops - tuned values from on-device testing
  motor.PID_velocity.P = 0.15;
  motor.PID_velocity.I = 0.003;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 400.0;

  // Angle loop tuning
  motor.P_angle.P = 10.0;
  motor.P_angle.limit = 70.0;
}


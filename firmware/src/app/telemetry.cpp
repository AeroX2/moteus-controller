#include "app/telemetry.h"

#include <Arduino.h>

#include "app/app_context.h"
#include "debug/can_debug.h"

void telemetry_tick() {
  static long old_time = 0;

  if (millis() - old_time <= update_frequency) return;

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

  uint8_t motion_data[11] = {
    angle_to_bytes.b[0], angle_to_bytes.b[1], angle_to_bytes.b[2], angle_to_bytes.b[3],
    velocity_to_bytes.b[0], velocity_to_bytes.b[1], velocity_to_bytes.b[2], velocity_to_bytes.b[3],
    0x00, 0x00, 0x00
  };

  send_can_message(0x00, motion_data, 12);
  old_time = millis();
}


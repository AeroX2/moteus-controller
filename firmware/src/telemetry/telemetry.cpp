#include "telemetry/telemetry.h"

#include <Arduino.h>

#include "debug/can_debug.h"
#include "drivers/motor_objects.h"
#include "telemetry/messages.h"

using namespace telemetry;

void telemetry_tick() {
  static long old_time = 0;

  if (millis() - old_time <= update_frequency) return;

  float angle = magnetic_sensor.getAngle();
  float velocity = magnetic_sensor.getVelocity();
  if (motor.sensor_direction == Direction::CCW) angle = -angle;

  telemetry::MotionDataPayload payload = {};
  payload.angle = angle;
  payload.velocity = velocity;

  uint8_t buf[telemetry::MOTION_DATA_PAYLOAD_SIZE];
  telemetry::motion_data_encode(&payload, buf);
  send_can_message(telemetry::CanMsgType::MotionData, buf,
                   telemetry::MOTION_DATA_PAYLOAD_SIZE);
  old_time = millis();
}

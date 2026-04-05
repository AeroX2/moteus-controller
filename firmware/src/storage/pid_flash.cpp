#include "storage/pid_flash.h"

#include "drivers/motor_objects.h"

#include "stm32g4xx_hal_flash.h"
#include "stm32g4xx_hal_flash_ex.h"

// PID storage in flash (last 2KB page of 128KB: 0x0801F800)
namespace {
  constexpr uint32_t PID_FLASH_MAGIC = 0x50494453u;  // "PIDS"
  constexpr uint32_t PID_FLASH_PAGE = 63;
  constexpr uint32_t PID_FLASH_ADDR = 0x08000000u + (PID_FLASH_PAGE * 2048u);
}  // namespace

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

namespace {

bool erase_pid_page_unlocked() {
  FLASH_EraseInitTypeDef erase = {};
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks = FLASH_BANK_1;
  erase.Page = PID_FLASH_PAGE;
  erase.NbPages = 1;
  uint32_t pageError = 0;
  return HAL_FLASHEx_Erase(&erase, &pageError) == HAL_OK;
}

}  // namespace

bool erase_pid_from_flash() {
  HAL_FLASH_Unlock();
  if (!erase_pid_page_unlocked()) {
    HAL_FLASH_Lock();
    return false;
  }
  HAL_FLASH_Lock();
  return true;
}

bool save_pid_to_flash() {
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
  if (!erase_pid_page_unlocked()) {
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

bool load_pid_from_flash() {
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


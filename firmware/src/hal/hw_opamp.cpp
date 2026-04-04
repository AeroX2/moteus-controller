#include "hal/hw_opamp.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_opamp.h"

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp3;

static void configure_standalone_opamp(OPAMP_HandleTypeDef* hopamp, OPAMP_TypeDef* inst) {
  hopamp->Instance = inst;
  hopamp->Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp->Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp->Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp->Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp->Init.InternalOutput = DISABLE;
  hopamp->Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp->Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
}

void opamp_init(void) {
  configure_standalone_opamp(&hopamp1, OPAMP1);
  configure_standalone_opamp(&hopamp3, OPAMP3);

  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK || HAL_OPAMP_Init(&hopamp3) != HAL_OK) {
    while (1) {}
  }

  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK || HAL_OPAMP_Start(&hopamp3) != HAL_OK) {
    while (1) {}
  }
}

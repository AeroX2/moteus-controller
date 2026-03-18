#include "hal/hw_opamp.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_opamp.h"

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp3;

void opamp_init(void) {
  hopamp1.Instance = OPAMP1;
  hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp1.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp1.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_NO;
  hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_32_OR_MINUS_31;

  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
  hopamp3.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp3.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;

  if (HAL_OPAMP_Init(&hopamp1) != HAL_OK || HAL_OPAMP_Init(&hopamp3) != HAL_OK) {
    while (1) {}
  }

  if (HAL_OPAMP_Start(&hopamp1) != HAL_OK || HAL_OPAMP_Start(&hopamp3) != HAL_OK) {
    while (1) {}
  }
}


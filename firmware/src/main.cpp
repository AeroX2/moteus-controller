#include <Arduino.h>

#include "app/can_init.h"
#include "app/app_context.h"
#include "app/motor_init.h"
#include "app/telemetry.h"
#include "debug/can_debug.h"
#include "debug/debug_init.h"
#include "commands/commands.h"

// Defines `fdcan1` instance + IRQ handlers (must be included in ONE TU only).
#include "ACANFD_STM32.h"

void setup() {
  #ifndef CAN_DEBUG
  Serial.setRx(PA10);
  Serial.setTx(PA9);
  Serial.begin(115200);
  #endif

  can_init();

  debug_init();
  motor_init();

  pinMode(PC2, OUTPUT);
  register_commands();
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

void loop() {
  motor.loopFOC();
  motor.move();
  run_command_loop();
  telemetry_tick();
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

#include <Arduino.h>
#ifdef FOC_USE_HARDWARE_TIMER
#include <HardwareTimer.h>
#endif

#include "app/can_init.h"
#include "app/motor_init.h"
#include "commands/commands.h"
#include "debug/can_debug.h"
#include "debug/debug_init.h"
#include "drivers/motor_objects.h"
#include "telemetry/telemetry.h"

#ifndef FOC_LOOP_HZ
#define FOC_LOOP_HZ 10000
#endif

#ifdef FOC_USE_HARDWARE_TIMER
static HardwareTimer s_foc_timer(TIM6);

void foc_timer_isr() {
  motor.loopFOC();
  motor.move();
}
#else
static uint32_t foc_next_us = 0;
#endif

// Per-cycle only: interval = wall time since previous FOC *start* (not cumulative backlog).
static constexpr uint32_t FOC_TIMING_WARN_INTERVAL_OVER_PERIOD_US = 50;  // warn if interval > period + this
static constexpr uint32_t FOC_TIMING_WARN_EXEC_FRAC = 9;                 // warn if exec*10 > period*9
static constexpr uint32_t FOC_TIMING_WARN_MIN_INTERVAL_MS = 250;

#ifndef FOC_USE_HARDWARE_TIMER
static void foc_run_cooperative() {
  const uint32_t period_us = 1000000UL / FOC_LOOP_HZ;
  const uint32_t now = micros();
  if ((int32_t)(now - foc_next_us) < (int32_t)period_us) {
    return;
  }
  foc_next_us += period_us;
  const uint32_t foc_t0 = micros();
  static uint32_t last_foc_start_us = 0;
  uint32_t interval_us = 0;
  if (last_foc_start_us != 0) {
    interval_us = foc_t0 - last_foc_start_us;
  }
  last_foc_start_us = foc_t0;
  motor.loopFOC();
  motor.move();
  const uint32_t exec_us = micros() - foc_t0;
  const bool slow_interval =
      interval_us > 0 && interval_us > period_us + FOC_TIMING_WARN_INTERVAL_OVER_PERIOD_US;
  const bool slow_exec = exec_us * 10U > period_us * FOC_TIMING_WARN_EXEC_FRAC;
  if (slow_interval || slow_exec) {
    static uint32_t last_foc_timing_warn_ms = 0;
    const uint32_t ms = millis();
    if (ms - last_foc_timing_warn_ms >= FOC_TIMING_WARN_MIN_INTERVAL_MS) {
      last_foc_timing_warn_ms = ms;
      Serial.print("FOC slow: interval=");
      Serial.print(interval_us);
      Serial.print(" us exec=");
      Serial.print(exec_us);
      Serial.print(" us period=");
      Serial.print(period_us);
      if (slow_interval) {
        Serial.print(" (interval>");
        Serial.print(period_us + FOC_TIMING_WARN_INTERVAL_OVER_PERIOD_US);
        Serial.print(" us)");
      }
      if (slow_exec) {
        Serial.print(" exec>");
        Serial.print(period_us * FOC_TIMING_WARN_EXEC_FRAC / 10U);
        Serial.print(" us");
      }
      Serial.println();
    }
  }
}
#endif  // !FOC_USE_HARDWARE_TIMER

void setup() {
  #ifndef CAN_DEBUG_ENABLED
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

#ifdef FOC_USE_HARDWARE_TIMER
  s_foc_timer.setOverflow(FOC_LOOP_HZ, HERTZ_FORMAT);
  s_foc_timer.attachInterrupt(foc_timer_isr);
  s_foc_timer.resume();
#else
  foc_next_us = micros();
#endif
}

void loop() {
  run_command_loop();
  telemetry_tick();
#ifndef FOC_USE_HARDWARE_TIMER
  foc_run_cooperative();
#endif
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
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
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

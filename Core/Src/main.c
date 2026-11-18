/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd/lcd.h"
#include "ir_sensor.h"
#include "ultrasonic.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static IR_Sensors ir;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* Startup log: each call adds a line and redraws the TFT immediately.
 * tft_update clears the buffer after drawing, so all lines are reprinted each time. */
static void boot_log(const char *msg) {
  static const char *lines[10];
  static unsigned count;
  if (count < 10) lines[count++] = msg;
  for (unsigned i = 0; i < count; i++) tft_prints(0, i, "%s", lines[i]);
  tft_update(0);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  /* PWM timers are configured but NEVER started in this sensor-only demo. */
  tft_init(PIN_ON_TOP, BLACK, WHITE, CYAN, DARK_GREY);
  boot_log("SENSOR STARTUP");
  boot_log("TFT OK");

  IR_Init(&ir, IR_DEFAULT_TRIGGER);
  boot_log((ir.ok[0] && ir.ok[1] && ir.ok[2] && ir.ok[3]) ? "IR OK" : "IR ADC FAIL");

  boot_log(Ultrasonic_Start() == HAL_OK ? "US OK" : "US FAIL");

  boot_log("TOF INIT...");
  AR_ToFBoardInit();
  boot_log(AR_ToFBoardStatus());

  HAL_Delay(1500);   /* keep the startup log on screen briefly */
  static const char *const ir_names[IR_COUNT] = {"LL", "LM", "RM", "RR"};
  int32_t us_mm = -1;
  uint32_t last_us = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Read sensors */
    IR_Update(&ir);
    AR_ToFBoardPoll();
    if (HAL_GetTick() - last_us >= 60) {   /* ultrasonic needs ~60 ms between pings */
      last_us = HAL_GetTick();
      us_mm = Ultrasonic_ReadMm();
    }

    /* Fill the screen buffer (tft_update clears it after each refresh) */
    tft_prints(0, 0, "IR  RAW TRIG B");
    for (unsigned i = 0; i < IR_COUNT; i++) {
      char state = !ir.ok[i] ? '?' : ir.black[i] ? '#' : '.';
      tft_prints(0, 1 + i, "%s %4d %4d %c", ir_names[i],
                 (int)ir.raw[i], (int)ir.trigger[i], state);
    }
    tft_prints(0, 5, "US %d mm", (int)us_mm);
    tft_prints(0, 6, "T1 %d mm", (int)AR_ToFBoardDistanceAt(0));
    tft_prints(0, 7, "T2 %d mm", (int)AR_ToFBoardDistanceAt(1));
    tft_prints(0, 8, "%s", AR_ToFBoardStatus());
    tft_prints(0, 9, "%s", us_mm < 0 ? "US NO ECHO" : "US OK");

    /* Push to the TFT every 50 ms */
    tft_update(50);
  }
  

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* Remove PWM on a HAL fatal error; do not depend on initialized motor handles. */
  if (__HAL_RCC_TIM8_IS_CLK_ENABLED()) TIM8->CCR2 = 0;
  if (__HAL_RCC_TIM12_IS_CLK_ENABLED()) TIM12->CCR1 = 0;
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

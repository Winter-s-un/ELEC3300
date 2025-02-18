/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "gpio.h"
#include "fsmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
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
extern volatile int isrFlag;
int status = 1;

const uint8_t char_data[] = 
{
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x78, 0x04, 0x02, 0x78, 0x7c, 0x1e, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x02, 0x01, 0x10, 0x42, 0x7d, 0x10, 0x42, 0x45, 0x90, 0x42, 0x45, 0x90, 0x4a, 0x45, 0x90, 0x4a, 0x45, 0xfc, 0x7a, 0x7d, 0x90, 0x4a, 0x01, 0x90, 0x4a, 0x35, 0x90, 0x42, 0x35, 0x10, 0x42, 0x45, 0x10, 0x42, 0x7f, 0x10, 0x42, 0x7f, 0x10, 0x02, 0x05, 0x00, 0x1e, 0x05, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_FSMC_Init();
  /* USER CODE BEGIN 2 */
  LCD_INIT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */


    if(isrFlag)
    {
     status++;
     LCD_Clear (0, 0, 240, 320, BACKGROUND);
     isrFlag = 0;
    }

    if(status % 2 == 0)
    {
      LCD_DrawLine(10, 30, 40, 60, RED);
      LCD_DrawLine(10, 90, 40, 120, RED);
      LCD_DrawLine(10, 200, 40, 150, RED);

      LCD_DrawLine(65, 55, 155, 55, RED);
      LCD_DrawLine(110, 30, 110, 70, RED);
      LCD_DrawLine(75, 70, 145, 70, RED);

      LCD_DrawLine(50, 80, 170, 80, RED);
      LCD_DrawLine(170, 80, 160, 90, RED);

      LCD_DrawLine(75, 90, 145, 90, RED);
      LCD_DrawLine(110, 90, 110, 115, RED);
      LCD_DrawLine(75, 115, 145, 115, RED);

      LCD_DrawLine(50, 125, 170, 125, RED);

      LCD_DrawLine(50, 145, 100, 145, RED);
      LCD_DrawLine(50, 195, 100, 195, RED);
      LCD_DrawLine(50, 145, 50, 195, RED);
      LCD_DrawLine(100, 145, 100, 195, RED);

      LCD_DrawLine(120, 145, 170, 145, RED);
      LCD_DrawLine(150, 135, 150, 195, RED);
      LCD_DrawLine(150, 195, 145, 180, RED);
      LCD_DrawLine(135, 170, 130, 160, RED);


  
    }
    else
    {
        LCD_DrawString(20, 40, "ZHANG Wentao");

        LCD_DrawLine(10, 60, 150, 60, RED);

        LCD_DrawEllipse(120, 160, 25, 75, BLACK);
    }
    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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

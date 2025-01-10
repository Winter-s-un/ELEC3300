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
#include "adc.h"
#include "i2c.h"
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
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x78, 0x04, 0x02, 0x78, 0x7c, 0x1e, 0x7f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x02, 0x01, 
0x10, 0x42, 0x7d, 0x10, 0x42, 0x45, 
0x90, 0x42, 0x45, 0x90, 0x4a, 0x45, 
0x90, 0x4a, 0x45, 0xfc, 0x7a, 0x7d, 
0x90, 0x4a, 0x01, 0x90, 0x4a, 0x35,
0x90, 0x42, 0x35, 0x10, 0x42, 0x45,
0x10, 0x42, 0x7f, 0x10, 0x42, 0x7f, 
0x10, 0x02, 0x05, 0x00, 0x1e, 0x05, 
0x00, 0x00, 0x01, 0x00, 0x00, 0x00
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
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  LCD_INIT();
  HAL_ADCEx_Calibration_Start(&hadc1);
  uint8_t HMC5883L_Addr = 0x1E<<1;
  uint8_t CRA = 0x70;
  uint8_t CRB = 0xA0;
  uint8_t X_MSB = 0x03;
  uint8_t X_LSB = 0x04;
  uint8_t Z_MSB = 0x05;
  uint8_t Z_LSB = 0x06;
  uint8_t Y_MSB = 0x07;
  uint8_t Y_LSB = 0x08;

  int angle;
  HAL_I2C_Mem_Write(&hi2c2,HMC5883L_Addr<<1,0x00,1,&CRA,1,100);
  HAL_I2C_Mem_Write(&hi2c2,HMC5883L_Addr<<1,0x01,1,&CRB,1,100);

  uint8_t data_direction[6];
  int16_t whole_value[3];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_I2C_Mem_Read(&hi2c2,HMC5883L_Addr,X_MSB,1,&data_direction[0],1,100);
	  HAL_I2C_Mem_Read(&hi2c2,HMC5883L_Addr,X_LSB,1,&data_direction[1],1,100);
	  HAL_I2C_Mem_Read(&hi2c2,HMC5883L_Addr,Y_MSB,1,&data_direction[2],1,100);
	  HAL_I2C_Mem_Read(&hi2c2,HMC5883L_Addr,Y_LSB,1,&data_direction[3],1,100);
	  HAL_I2C_Mem_Read(&hi2c2,HMC5883L_Addr,Z_MSB,1,&data_direction[4],1,100);
	  HAL_I2C_Mem_Read(&hi2c2,HMC5883L_Addr,Z_LSB,1,&data_direction[5],1,100);

	  whole_value[0] = (int16_t)((data_direction[0] << 8)|data_direction[1]);
	  whole_value[1] = (int16_t)((data_direction[2] << 8)|data_direction[3]);
	  whole_value[2] = (int16_t)((data_direction[4] << 8)|data_direction[5]);

	  angle = atan2(whole_value[1],whole_value[0])*(180/3.14)+180;

	  char char_angle[10];
	  sprintf(char_angle,"%d",angle);
	  LCD_DrawString(20, 40, "angle");
	  LCD_DrawString(20, 60, char_angle);
	  HAL_Delay(100);

	  /*HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  unsigned int value = HAL_ADC_GetValue(&hadc1);
	  char data_hex[10];
	  char data[10];
	  sprintf(data,"%d",value);
	  sprintf(data_hex,"%x",value);
	  LCD_DrawString(20, 20, "DEC");
	  LCD_DrawString(60, 20, "HEX");
	  LCD_DrawString(20, 40, data);
	  LCD_DrawString(60, 40, data_hex);*/
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

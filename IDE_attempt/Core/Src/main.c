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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//i2c commands and addresses defined in hex
#define I2C_ADDRESS  0xE0
#define SLEEP 0xB098
#define WAKE_UP 0x3517
#define MEASURE 0x5C24
#define DEVICE_ID 0

#define I2C_MEASURE_TIME 15 //times all in ms
#define I2C_WAKE_UP_TIME 1

#define READ_TIME 1000
#define DISPLAY_TIME 500

#define TX_START  73
#define TX_END    5
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t temp_val;
uint16_t humd_val;
uint16_t light_val; //do we need this for bird people?
uint16_t battery_val;
uint8_t sensor;
uint8_t data_buff[7];
uint8_t i2c_buff[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void display (uint8_t num, uint8_t digit);
void i2c_measure(void);
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
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(HAL_GPIO_ReadPin(PB0_GPIO_Port,PB0_Pin)==0) {
		HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
	}
	if(HAL_GPIO_ReadPin(PB1_GPIO_Port,PB1_Pin)==0) {
			HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
	}
	i2c_measure();
	HAL_Delay(1000);
	/*
	display(0,0);
	HAL_Delay(100);
	display(1,1);
	HAL_Delay(100);
	display(2,2);
	HAL_Delay(100);
	*/
    /* USER CODE END WHILE */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void i2c_measure() {
	uint16_t temp = WAKE_UP;
	HAL_I2C_Master_Transmit(&hi2c1,I2C_ADDRESS,&temp,2,1000);
	HAL_Delay(I2C_WAKE_UP_TIME);
	temp = MEASURE;
	HAL_I2C_Master_Transmit(&hi2c1,I2C_ADDRESS,&temp,2,1000);
	HAL_Delay(I2C_MEASURE_TIME);
	HAL_I2C_Master_Receive(&hi2c1,I2C_ADDRESS,i2c_buff,6,1000);
	temp = SLEEP;
	HAL_I2C_Master_Transmit(&hi2c1,I2C_ADDRESS,&temp,2,1000);
}

void display (uint8_t num, uint8_t digit) {
  HAL_GPIO_WritePin(GRID1_DRV_GPIO_Port,GRID1_DRV_Pin, RESET);
  HAL_GPIO_WritePin(GRID2_DRV_GPIO_Port,GRID2_DRV_Pin, RESET);
  HAL_GPIO_WritePin(GRID3_DRV_GPIO_Port,GRID3_DRV_Pin, RESET);
  switch(num) {
  case 0:
    HAL_GPIO_WritePin(A_DRV_GPIO_Port,A_DRV_Pin, SET);;
    HAL_GPIO_WritePin(B_DRV_GPIO_Port,B_DRV_Pin, SET);;
    HAL_GPIO_WritePin(C_DRV_GPIO_Port,C_DRV_Pin, SET);;
    HAL_GPIO_WritePin(D_DRV_GPIO_Port,D_DRV_Pin, SET);;
    HAL_GPIO_WritePin(E_DRV_GPIO_Port,E_DRV_Pin, SET);;
    HAL_GPIO_WritePin(F_DRV_GPIO_Port,F_DRV_Pin, SET);;
    HAL_GPIO_WritePin(G_DRV_GPIO_Port,G_DRV_Pin, RESET);;
  break;
  case 1:
    HAL_GPIO_WritePin(A_DRV_GPIO_Port,A_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(B_DRV_GPIO_Port,B_DRV_Pin, SET);;
    HAL_GPIO_WritePin(C_DRV_GPIO_Port,C_DRV_Pin, SET);;
    HAL_GPIO_WritePin(D_DRV_GPIO_Port,D_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(E_DRV_GPIO_Port,E_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(F_DRV_GPIO_Port,F_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(G_DRV_GPIO_Port,G_DRV_Pin, RESET);;
  break;
  case 2:
    HAL_GPIO_WritePin(A_DRV_GPIO_Port,A_DRV_Pin, SET);;
    HAL_GPIO_WritePin(B_DRV_GPIO_Port,B_DRV_Pin, SET);;
    HAL_GPIO_WritePin(C_DRV_GPIO_Port,C_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(D_DRV_GPIO_Port,D_DRV_Pin, SET);;
    HAL_GPIO_WritePin(E_DRV_GPIO_Port,E_DRV_Pin, SET);;
    HAL_GPIO_WritePin(F_DRV_GPIO_Port,F_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(G_DRV_GPIO_Port,G_DRV_Pin, SET);;
  break;
  case 3:
    HAL_GPIO_WritePin(A_DRV_GPIO_Port,A_DRV_Pin, SET);;
    HAL_GPIO_WritePin(B_DRV_GPIO_Port,B_DRV_Pin, SET);;
    HAL_GPIO_WritePin(C_DRV_GPIO_Port,C_DRV_Pin, SET);;
    HAL_GPIO_WritePin(D_DRV_GPIO_Port,D_DRV_Pin, SET);;
    HAL_GPIO_WritePin(E_DRV_GPIO_Port,E_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(F_DRV_GPIO_Port,F_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(G_DRV_GPIO_Port,G_DRV_Pin, SET);;
  break;
  case 4:
    HAL_GPIO_WritePin(A_DRV_GPIO_Port,A_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(B_DRV_GPIO_Port,B_DRV_Pin, SET);;
    HAL_GPIO_WritePin(C_DRV_GPIO_Port,C_DRV_Pin, SET);;
    HAL_GPIO_WritePin(D_DRV_GPIO_Port,D_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(E_DRV_GPIO_Port,E_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(F_DRV_GPIO_Port,F_DRV_Pin, SET);;
    HAL_GPIO_WritePin(G_DRV_GPIO_Port,G_DRV_Pin, SET);;
  break;
  case 5:
    HAL_GPIO_WritePin(A_DRV_GPIO_Port,A_DRV_Pin, SET);;
    HAL_GPIO_WritePin(B_DRV_GPIO_Port,B_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(C_DRV_GPIO_Port,C_DRV_Pin, SET);;
    HAL_GPIO_WritePin(D_DRV_GPIO_Port,D_DRV_Pin, SET);;
    HAL_GPIO_WritePin(E_DRV_GPIO_Port,E_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(F_DRV_GPIO_Port,F_DRV_Pin, SET);;
    HAL_GPIO_WritePin(G_DRV_GPIO_Port,G_DRV_Pin, SET);;
  break;
  case 6:
    HAL_GPIO_WritePin(A_DRV_GPIO_Port,A_DRV_Pin, SET);;
    HAL_GPIO_WritePin(B_DRV_GPIO_Port,B_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(C_DRV_GPIO_Port,C_DRV_Pin, SET);;
    HAL_GPIO_WritePin(D_DRV_GPIO_Port,D_DRV_Pin, SET);;
    HAL_GPIO_WritePin(E_DRV_GPIO_Port,E_DRV_Pin, SET);;
    HAL_GPIO_WritePin(F_DRV_GPIO_Port,F_DRV_Pin, SET);;
    HAL_GPIO_WritePin(G_DRV_GPIO_Port,G_DRV_Pin, SET);;
  break;
  case 7:
    HAL_GPIO_WritePin(A_DRV_GPIO_Port,A_DRV_Pin, SET);;
    HAL_GPIO_WritePin(B_DRV_GPIO_Port,B_DRV_Pin, SET);;
    HAL_GPIO_WritePin(C_DRV_GPIO_Port,C_DRV_Pin, SET);;
    HAL_GPIO_WritePin(D_DRV_GPIO_Port,D_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(E_DRV_GPIO_Port,E_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(F_DRV_GPIO_Port,F_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(G_DRV_GPIO_Port,G_DRV_Pin, RESET);;
  break;
  case 8:
    HAL_GPIO_WritePin(A_DRV_GPIO_Port,A_DRV_Pin, SET);;
    HAL_GPIO_WritePin(B_DRV_GPIO_Port,B_DRV_Pin, SET);;
    HAL_GPIO_WritePin(C_DRV_GPIO_Port,C_DRV_Pin, SET);;
    HAL_GPIO_WritePin(D_DRV_GPIO_Port,D_DRV_Pin, SET);;
    HAL_GPIO_WritePin(E_DRV_GPIO_Port,E_DRV_Pin, SET);;
    HAL_GPIO_WritePin(F_DRV_GPIO_Port,F_DRV_Pin, SET);;
    HAL_GPIO_WritePin(G_DRV_GPIO_Port,G_DRV_Pin, SET);;
  break;
  case 9:
    HAL_GPIO_WritePin(A_DRV_GPIO_Port,A_DRV_Pin, SET);;
    HAL_GPIO_WritePin(B_DRV_GPIO_Port,B_DRV_Pin, SET);;
    HAL_GPIO_WritePin(C_DRV_GPIO_Port,C_DRV_Pin, SET);;
    HAL_GPIO_WritePin(D_DRV_GPIO_Port,D_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(E_DRV_GPIO_Port,E_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(F_DRV_GPIO_Port,F_DRV_Pin, SET);;
    HAL_GPIO_WritePin(G_DRV_GPIO_Port,G_DRV_Pin, SET);;
  break;
  default:
   HAL_GPIO_WritePin(A_DRV_GPIO_Port,A_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(B_DRV_GPIO_Port,B_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(C_DRV_GPIO_Port,C_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(D_DRV_GPIO_Port,D_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(E_DRV_GPIO_Port,E_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(F_DRV_GPIO_Port,F_DRV_Pin, RESET);;
    HAL_GPIO_WritePin(G_DRV_GPIO_Port,G_DRV_Pin, RESET);;
  break;}
   if (digit == 1) {
	   HAL_GPIO_WritePin(GRID1_DRV_GPIO_Port,GRID1_DRV_Pin, RESET);
	   HAL_GPIO_WritePin(GRID2_DRV_GPIO_Port,GRID2_DRV_Pin, SET);
	   HAL_GPIO_WritePin(GRID3_DRV_GPIO_Port,GRID3_DRV_Pin, RESET);
    //digitalWrite(DP, HIGH);
  } else if (digit == 2) {
	  HAL_GPIO_WritePin(GRID1_DRV_GPIO_Port,GRID1_DRV_Pin, RESET);
	  HAL_GPIO_WritePin(GRID2_DRV_GPIO_Port,GRID2_DRV_Pin, RESET);
	  HAL_GPIO_WritePin(GRID3_DRV_GPIO_Port,GRID3_DRV_Pin, SET);
    //digitalWrite(DP, LOW);
  } else {
	  HAL_GPIO_WritePin(GRID1_DRV_GPIO_Port,GRID1_DRV_Pin, SET);
	  HAL_GPIO_WritePin(GRID2_DRV_GPIO_Port,GRID2_DRV_Pin, RESET);
	  HAL_GPIO_WritePin(GRID3_DRV_GPIO_Port,GRID3_DRV_Pin, RESET);
	//digitalWrite(DP, LOW);
  }
}
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

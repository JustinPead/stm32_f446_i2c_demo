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
#include "fatfs.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd_stm32f0.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DATA_LENGTH 7
#define RX_START 0x54
#define RX_END 0x70
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t led_val = 0;
uint16_t delay_ms = 500;

uint8_t last_device_id = 0;
uint8_t last_sensor = 0;
uint16_t last_heartbeat = 0;
uint16_t last_val = 0;

char incoming_byte;
uint8_t output_buff[5];
volatile char recvd_data[DATA_LENGTH];
char line_one[25];
char line_two[25];
uint16_t cnt = 0;
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
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  init_LCD_pins();
  init_LCD();
  HAL_Delay(10);

  update_lcd();
  HAL_UART_Receive_IT(&huart5,&incoming_byte,1); //receive data from data buffer interrupt mode

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_Delay(1);
    cnt++;
    if(cnt%delay_ms == 0) {
      led_val++;
      last_heartbeat++;
      cnt = 0;
      update_lcd();
    }
    update_leds();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 50;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void clear_rx_data() {
 for(uint8_t i = 0; i<DATA_LENGTH;i++) {
    recvd_data[i] = 0;
  }
}

void update_lcd() {
  sprintf(line_one, "D_ID:%d pTx:%d     ",last_device_id,last_heartbeat);
  sprintf(line_two, "S_ID:%d Val:%d     ", last_sensor, last_val);
  line_one[16] = 0;
  line_two[16] = 0;
  lcd_command(CURSOR_HOME);
  lcd_putstring(line_one);
  lcd_command(LINE_TWO);
  lcd_putstring(line_two);
}

void update_leds() {
  GPIOB->ODR = led_val;
}

//UART 5 receive complete callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  for(int8_t i = DATA_LENGTH-1; i>=1 ;i--) {
    recvd_data[i] = recvd_data[i-1];
  }
  recvd_data[0] = incoming_byte;
	HAL_GPIO_TogglePin(LED7_GPIO_Port,LED7_Pin);
  if(recvd_data[DATA_LENGTH-1]==RX_START && recvd_data[0] == RX_END) {
    if(recvd_data[1] = (recvd_data[5]+recvd_data[4]+recvd_data[3]+recvd_data[2])) {
      last_device_id = recvd_data[5];
      last_sensor = recvd_data[4];
      last_val = ((recvd_data[3]*256) + recvd_data[2]);
      last_heartbeat = 0;

      
      output_buff[1] = (uint8_t)last_val;
      output_buff[2] = last_val>>8;
      output_buff[3] = last_sensor;
      output_buff[4] = last_device_id;
      output_buff[0] = output_buff[4]+output_buff[3]+output_buff[2]+output_buff[1];
      HAL_UART_Transmit(&huart1,output_buff,5,100);
      clear_rx_data();
      led_val = 0;
    }
  }
	HAL_UART_Receive_IT(&huart5,&incoming_byte,1); //start next data receive interrupt
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

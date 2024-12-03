/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_RS_Pin GPIO_PIN_14
#define LCD_RS_GPIO_Port GPIOC
#define LCD_E_Pin GPIO_PIN_15
#define LCD_E_GPIO_Port GPIOC
#define SPI2_NSS_Pin GPIO_PIN_3
#define SPI2_NSS_GPIO_Port GPIOC
#define PUSH0_Pin GPIO_PIN_0
#define PUSH0_GPIO_Port GPIOA
#define PUSH1_Pin GPIO_PIN_1
#define PUSH1_GPIO_Port GPIOA
#define PUSH2_Pin GPIO_PIN_2
#define PUSH2_GPIO_Port GPIOA
#define PUSH3_Pin GPIO_PIN_3
#define PUSH3_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_2
#define LED2_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_12
#define LCD_D6_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_15
#define LCD_D7_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_3
#define LED3_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_4
#define LED4_GPIO_Port GPIOB
#define LED5_Pin GPIO_PIN_5
#define LED5_GPIO_Port GPIOB
#define LED6_Pin GPIO_PIN_6
#define LED6_GPIO_Port GPIOB
#define LED7_Pin GPIO_PIN_7
#define LED7_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_8
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_9
#define LCD_D5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

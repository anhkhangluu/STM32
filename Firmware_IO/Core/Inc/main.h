/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define W5500_CS_Pin GPIO_PIN_4
#define W5500_CS_GPIO_Port GPIOA
#define W5500_SCK_Pin GPIO_PIN_5
#define W5500_SCK_GPIO_Port GPIOA
#define W5500_MISO_Pin GPIO_PIN_6
#define W5500_MISO_GPIO_Port GPIOA
#define W5500_MOSI_Pin GPIO_PIN_7
#define W5500_MOSI_GPIO_Port GPIOA
#define OUT0_Pin GPIO_PIN_0
#define OUT0_GPIO_Port GPIOB
#define OUT1_Pin GPIO_PIN_1
#define OUT1_GPIO_Port GPIOB
#define OUT2_Pin GPIO_PIN_2
#define OUT2_GPIO_Port GPIOB
#define OUT3_Pin GPIO_PIN_7
#define OUT3_GPIO_Port GPIOE
#define SENSOR0_Pin GPIO_PIN_9
#define SENSOR0_GPIO_Port GPIOE
#define SENSOR1_Pin GPIO_PIN_11
#define SENSOR1_GPIO_Port GPIOE
#define IN1_Pin GPIO_PIN_10
#define IN1_GPIO_Port GPIOD
#define IN0_Pin GPIO_PIN_11
#define IN0_GPIO_Port GPIOD
#define BT_PREV_Pin GPIO_PIN_12
#define BT_PREV_GPIO_Port GPIOD
#define BT_NEXT_Pin GPIO_PIN_13
#define BT_NEXT_GPIO_Port GPIOD
#define BT_MENU_Pin GPIO_PIN_14
#define BT_MENU_GPIO_Port GPIOD
#define BT_RESET_Pin GPIO_PIN_15
#define BT_RESET_GPIO_Port GPIOD
#define BT_SET_Pin GPIO_PIN_6
#define BT_SET_GPIO_Port GPIOC
#define BT_RESERVED_Pin GPIO_PIN_7
#define BT_RESERVED_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_8
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_8
#define LED3_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_9
#define LED4_GPIO_Port GPIOA
#define SD_CS_Pin GPIO_PIN_0
#define SD_CS_GPIO_Port GPIOD
#define SD_SCK_Pin GPIO_PIN_1
#define SD_SCK_GPIO_Port GPIOD
#define SD_MISO_Pin GPIO_PIN_3
#define SD_MISO_GPIO_Port GPIOD
#define SD_MOSI_Pin GPIO_PIN_4
#define SD_MOSI_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
extern SPI_HandleTypeDef 	hspi1;
#define HSPI_W5500			&hspi1

extern SPI_HandleTypeDef 	hspi2;
#define HSPI_SDCARD		 	&hspi2

extern RTC_HandleTypeDef 	hrtc;
extern TIM_HandleTypeDef 	htim6;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

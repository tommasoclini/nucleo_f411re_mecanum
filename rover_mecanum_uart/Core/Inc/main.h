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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define ROBOT_ENB_A_Pin GPIO_PIN_5
#define ROBOT_ENB_A_GPIO_Port GPIOA
#define ROBOT_IN4_A_Pin GPIO_PIN_6
#define ROBOT_IN4_A_GPIO_Port GPIOA
#define ROBOT_IN3_A_Pin GPIO_PIN_7
#define ROBOT_IN3_A_GPIO_Port GPIOA
#define ROBOT_IN4_B_Pin GPIO_PIN_10
#define ROBOT_IN4_B_GPIO_Port GPIOB
#define ROBOT_IN1_A_Pin GPIO_PIN_7
#define ROBOT_IN1_A_GPIO_Port GPIOC
#define ROBOT_ENB_B_Pin GPIO_PIN_8
#define ROBOT_ENB_B_GPIO_Port GPIOA
#define ROBOT_ENA_A_Pin GPIO_PIN_9
#define ROBOT_ENA_A_GPIO_Port GPIOA
#define ROBOT_ENA_B_Pin GPIO_PIN_10
#define ROBOT_ENA_B_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ROBOT_IN1_B_Pin GPIO_PIN_3
#define ROBOT_IN1_B_GPIO_Port GPIOB
#define ROBOT_IN3_B_Pin GPIO_PIN_4
#define ROBOT_IN3_B_GPIO_Port GPIOB
#define ROBOT_IN2_B_Pin GPIO_PIN_5
#define ROBOT_IN2_B_GPIO_Port GPIOB
#define ROBOT_IN2_A_Pin GPIO_PIN_6
#define ROBOT_IN2_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

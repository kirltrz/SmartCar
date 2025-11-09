/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
void setLED(bool);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define SW_Pin GPIO_PIN_1
#define SW_GPIO_Port GPIOA
#define AD_CS2_Pin GPIO_PIN_3
#define AD_CS2_GPIO_Port GPIOA
#define AD_CS1_Pin GPIO_PIN_4
#define AD_CS1_GPIO_Port GPIOA
#define IMU_CS_Pin GPIO_PIN_12
#define IMU_CS_GPIO_Port GPIOB
#define L_MOTOR_IN1_Pin GPIO_PIN_8
#define L_MOTOR_IN1_GPIO_Port GPIOA
#define L_MOTOR_IN2_Pin GPIO_PIN_9
#define L_MOTOR_IN2_GPIO_Port GPIOA
#define R_MOTOR_IN1_Pin GPIO_PIN_10
#define R_MOTOR_IN1_GPIO_Port GPIOA
#define R_MOTOR_IN2_Pin GPIO_PIN_11
#define R_MOTOR_IN2_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
extern bool offGround;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

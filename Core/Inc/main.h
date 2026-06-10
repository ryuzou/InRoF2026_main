/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

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
#define DCMotorSwing_PULSE_Pin GPIO_PIN_0
#define DCMotorSwing_PULSE_GPIO_Port GPIOC
#define DCMotorSwing_DIR_Pin GPIO_PIN_1
#define DCMotorSwing_DIR_GPIO_Port GPIOC
#define DCMotorRack_PULSE_Pin GPIO_PIN_2
#define DCMotorRack_PULSE_GPIO_Port GPIOC
#define DCMotorRack_DIR_Pin GPIO_PIN_3
#define DCMotorRack_DIR_GPIO_Port GPIOC
#define VREF1_Pin GPIO_PIN_4
#define VREF1_GPIO_Port GPIOA
#define VREF2_Pin GPIO_PIN_5
#define VREF2_GPIO_Port GPIOA
#define LimitSW_Swing_Pin GPIO_PIN_1
#define LimitSW_Swing_GPIO_Port GPIOB
#define LimitSW_Swing_EXTI_IRQn EXTI1_IRQn
#define LimitSW_Rack_Close_Pin GPIO_PIN_2
#define LimitSW_Rack_Close_GPIO_Port GPIOB
#define LimitSW_Rack_Close_EXTI_IRQn EXTI2_IRQn
#define START_SW_Pin GPIO_PIN_10
#define START_SW_GPIO_Port GPIOB
#define START_SW_EXTI_IRQn EXTI15_10_IRQn
#define USER_SW_Pin GPIO_PIN_11
#define USER_SW_GPIO_Port GPIOB
#define USER_SW_EXTI_IRQn EXTI15_10_IRQn
#define STEP1_Pin GPIO_PIN_6
#define STEP1_GPIO_Port GPIOC
#define DIR1_Pin GPIO_PIN_7
#define DIR1_GPIO_Port GPIOC
#define STEP2_Pin GPIO_PIN_8
#define STEP2_GPIO_Port GPIOC
#define DIR2_Pin GPIO_PIN_9
#define DIR2_GPIO_Port GPIOC
#define ENABLE_Pin GPIO_PIN_10
#define ENABLE_GPIO_Port GPIOC
#define RESET_Pin GPIO_PIN_11
#define RESET_GPIO_Port GPIOC
#define FAULT_Pin GPIO_PIN_12
#define FAULT_GPIO_Port GPIOC
#define FAULT_EXTI_IRQn EXTI15_10_IRQn
#define LimitSW_Rack_Open_Pin GPIO_PIN_4
#define LimitSW_Rack_Open_GPIO_Port GPIOB
#define LimitSW_Rack_Open_EXTI_IRQn EXTI4_IRQn
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_9
#define BUZZER_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

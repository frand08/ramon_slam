/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#define PWM_FREQ 40000
#define PWM_RESOLUTION 1000
#define ENCODER_COUNTER 10000
#define LD0_Pin GPIO_PIN_7
#define LD0_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_4
#define LD1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOC
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_9
#define PWM1_GPIO_Port GPIOE
#define PWM3_Pin GPIO_PIN_13
#define PWM3_GPIO_Port GPIOE
#define PWM4_Pin GPIO_PIN_14
#define PWM4_GPIO_Port GPIOE
#define MPU_AD0_Pin GPIO_PIN_15
#define MPU_AD0_GPIO_Port GPIOD
#define ENCODER1_Pin GPIO_PIN_6
#define ENCODER1_GPIO_Port GPIOC
#define ENCODER2_Pin GPIO_PIN_7
#define ENCODER2_GPIO_Port GPIOC
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define MPU_INT_Pin GPIO_PIN_0
#define MPU_INT_GPIO_Port GPIOD
#define MPU_FSYNC_Pin GPIO_PIN_1
#define MPU_FSYNC_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

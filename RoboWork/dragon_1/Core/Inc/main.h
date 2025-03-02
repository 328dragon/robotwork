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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define motor_l_pwm_Pin GPIO_PIN_0
#define motor_l_pwm_GPIO_Port GPIOC
#define motor_2_pwm_Pin GPIO_PIN_1
#define motor_2_pwm_GPIO_Port GPIOC
#define motor_1_ph_Pin GPIO_PIN_2
#define motor_1_ph_GPIO_Port GPIOF
#define motor_2_ph_Pin GPIO_PIN_2
#define motor_2_ph_GPIO_Port GPIOA
#define cs_1_Pin GPIO_PIN_5
#define cs_1_GPIO_Port GPIOC
#define SH_A_Pin GPIO_PIN_0
#define SH_A_GPIO_Port GPIOB
#define ccd_1_Pin GPIO_PIN_1
#define ccd_1_GPIO_Port GPIOB
#define ICG_A_Pin GPIO_PIN_2
#define ICG_A_GPIO_Port GPIOB
#define ccd_2_Pin GPIO_PIN_9
#define ccd_2_GPIO_Port GPIOE
#define cs_4_Pin GPIO_PIN_11
#define cs_4_GPIO_Port GPIOB
#define servos_1_Pin GPIO_PIN_6
#define servos_1_GPIO_Port GPIOC
#define servos_2_Pin GPIO_PIN_7
#define servos_2_GPIO_Port GPIOC
#define servos_3_Pin GPIO_PIN_8
#define servos_3_GPIO_Port GPIOC
#define servos_4_Pin GPIO_PIN_9
#define servos_4_GPIO_Port GPIOC
#define ENA_Pin GPIO_PIN_10
#define ENA_GPIO_Port GPIOC
#define DIR_Pin GPIO_PIN_11
#define DIR_GPIO_Port GPIOC
#define PUL_Pin GPIO_PIN_12
#define PUL_GPIO_Port GPIOC
#define cs_3_Pin GPIO_PIN_0
#define cs_3_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
  void delay_us(uint32_t us);
	#include "motor.h"
#include "pid.h"
#include "stdio.h"
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

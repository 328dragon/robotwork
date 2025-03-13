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
#define ENA_Pin GPIO_PIN_4
#define ENA_GPIO_Port GPIOE
#define DIR_Pin GPIO_PIN_5
#define DIR_GPIO_Port GPIOE
#define PUL_Pin GPIO_PIN_6
#define PUL_GPIO_Port GPIOE
#define motor_1_ph_Pin GPIO_PIN_2
#define motor_1_ph_GPIO_Port GPIOF
#define motor_2_ph_Pin GPIO_PIN_2
#define motor_2_ph_GPIO_Port GPIOA
#define cs_1_Pin GPIO_PIN_5
#define cs_1_GPIO_Port GPIOC
#define SI_A_Pin GPIO_PIN_0
#define SI_A_GPIO_Port GPIOB
#define ccd_a_Pin GPIO_PIN_1
#define ccd_a_GPIO_Port GPIOB
#define CLK_A_Pin GPIO_PIN_2
#define CLK_A_GPIO_Port GPIOB
#define ccd_b_Pin GPIO_PIN_9
#define ccd_b_GPIO_Port GPIOE
#define CLK_B_Pin GPIO_PIN_10
#define CLK_B_GPIO_Port GPIOE
#define SI_B_Pin GPIO_PIN_11
#define SI_B_GPIO_Port GPIOE
#define cs_4_Pin GPIO_PIN_11
#define cs_4_GPIO_Port GPIOB
#define cs_3_Pin GPIO_PIN_0
#define cs_3_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */
	  void delay_us(uint32_t us);
	#include "motor.h"
#include "pid.h"
#include "bsp_IR_i2c.h"
#include "stdio.h"
#include "TSL1401.h"
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

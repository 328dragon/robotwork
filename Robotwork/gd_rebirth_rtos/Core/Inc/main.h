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
#define dir1_Pin GPIO_PIN_4
#define dir1_GPIO_Port GPIOE
#define dir2_Pin GPIO_PIN_5
#define dir2_GPIO_Port GPIOE
#define dir3_Pin GPIO_PIN_6
#define dir3_GPIO_Port GPIOE
#define dir4_Pin GPIO_PIN_13
#define dir4_GPIO_Port GPIOC
#define TFT_RST_Pin GPIO_PIN_14
#define TFT_RST_GPIO_Port GPIOC
#define TFT_CS_Pin GPIO_PIN_15
#define TFT_CS_GPIO_Port GPIOC
#define TFT_RS_Pin GPIO_PIN_9
#define TFT_RS_GPIO_Port GPIOF
#define spi1_cs_Pin GPIO_PIN_0
#define spi1_cs_GPIO_Port GPIOB
#define ccd_d1_Pin GPIO_PIN_1
#define ccd_d1_GPIO_Port GPIOB
#define ccd_d2_Pin GPIO_PIN_8
#define ccd_d2_GPIO_Port GPIOE
#define spi4_cs_Pin GPIO_PIN_15
#define spi4_cs_GPIO_Port GPIOE
#define icg_1_Pin GPIO_PIN_14
#define icg_1_GPIO_Port GPIOD
#define sh_1_Pin GPIO_PIN_15
#define sh_1_GPIO_Port GPIOD
#define spi3_cs_Pin GPIO_PIN_15
#define spi3_cs_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_2
#define LED_G_GPIO_Port GPIOD
#define ICM_INT2_Pin GPIO_PIN_7
#define ICM_INT2_GPIO_Port GPIOD
#define ICM_INT1_Pin GPIO_PIN_3
#define ICM_INT1_GPIO_Port GPIOB
#define ccd_m1_Pin GPIO_PIN_6
#define ccd_m1_GPIO_Port GPIOB
#define LDE_B_Pin GPIO_PIN_7
#define LDE_B_GPIO_Port GPIOB
#define ccd_m2_Pin GPIO_PIN_8
#define ccd_m2_GPIO_Port GPIOB
#define icg_2_Pin GPIO_PIN_9
#define icg_2_GPIO_Port GPIOB
#define sh_2_Pin GPIO_PIN_0
#define sh_2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

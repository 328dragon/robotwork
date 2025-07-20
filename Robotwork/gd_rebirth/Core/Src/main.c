/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tcs230.h"
#include "bsp_usart.h"
#include "tcd1103.h"
#include "dc_motor.h"
#include "gray.h"
#include "mainwork.h"
int goods_color = 90;
static uint32_t fac_us = 0; // us延时倍乘数
USARTInstance uart2 = {0};
void usart2_callback(void)
{
}
USART_Init_Config_s usart2_config = {
    .recv_buff_size = 64,
    .usart_handle = &huart2,
    .module_callback = usart2_callback,
};
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void delay_us(uint32_t nus)
{
  uint32_t ticks;
  uint32_t told, tnow, tcnt = 0;
  uint32_t reload = SysTick->LOAD; // LOAD的值
  ticks = nus * fac_us;            // 需要的节拍数
  told = SysTick->VAL;             // 刚进入时的计数器值
  while (1)
  {
    tnow = SysTick->VAL;
    if (tnow != told)
    {
      if (tnow < told)
        tcnt += told - tnow; // 这里注意一下SYSTICK是一个递减的计数器就可以了.
      else
        tcnt += reload - tnow + told;
      told = tnow;
      if (tcnt >= ticks)
        break; // 时间超过/等于要延迟的时间,则退出.
    }
  };
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim6.Instance)
  {
    if (HAL_GetTick() - icg_flag >= 10)
    {
      HAL_GPIO_WritePin(ICG_1_GPIO_Port, ICG_1_Pin, 1);
      HAL_GPIO_WritePin(SH_1_GPIO_Port, SH_1_Pin, 0);
      delay_us(2);
      HAL_GPIO_WritePin(SH_1_GPIO_Port, SH_1_Pin, 1);
      delay_us(4);
      HAL_GPIO_WritePin(ICG_1_GPIO_Port, ICG_1_Pin, 0);
      HAL_ADC_Start_DMA(&hadc3, (uint32_t *)ccd_rawdata, 1546);
      icg_flag = HAL_GetTick();
    }
    else
    {
      HAL_GPIO_WritePin(SH_1_GPIO_Port, SH_1_Pin, 0);
      delay_us(2);
      HAL_GPIO_WritePin(SH_1_GPIO_Port, SH_1_Pin, 1);
      delay_us(4);
    }
  }
}
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
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_LPUART1_UART_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_USB_PCD_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_ADC3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM20_Init();
  MX_ADC4_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */

  // 指示灯
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, 0);
  // 电机配置
	//重要：encoder对应引脚记得上拉
  IncEncoderInit(&encoder_0, 0, 1);
  IncEncoderInit(&encoder_1, 1, 1);
  PID_struct_init(&pid_0, POSITION_PID, 800, 200, 50, 1, 0);
  PID_struct_init(&pid_1, POSITION_PID, 800, 200, 50, 1,0);
  DCMotorInit(&motor_0, 0, 1, &encoder_0, &pid_0);
  DCMotorInit(&motor_1, 1, 0, &encoder_1, &pid_1);
  //舵机配置
  
	
  // debug串口
  USARTRegister(&uart2, &usart2_config);
  memset(uart2.recv_buff, 0, uart2.recv_buff_size);
//  // 颜色传感器添加完成
//  HAL_UART_Receive_IT(&huart1, &RxData, 1);
//  printf("AT+LIGHT+ON\r\n");
//  printf("AT+LIGHT+ON\r\n");
//  printf("AT+LIGHT+ON\r\n");
//  printf("AT+LIGHT+ON\r\n");
//  printf("AT+LIGHT+ON\r\n");
//  printf("AT+LIGHT+ON\r\n");

//  for (int i = 0; i < 10; i++)
//  {
//    goods_color = Color_Recognize();
//    HAL_Delay(200);
//  }
//普通灰度
	while(Ping())
	{
	HAL_Delay(5);	
	}
	
  // tcd1103配置
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start(&htim7);
  __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 60);
  // 定时器16开启，1ms的更新中断
  HAL_TIM_Base_Start_IT(&htim16);
	// 定时器17开启，5ms的更新中断
  HAL_TIM_Base_Start_IT(&htim17);
	
  // 定时器5开启，1us的更新中断，原本是电机的encoder读入，现在用作ccd的严格时序运行
  HAL_TIM_Base_Start_IT(&htim5);
main_work();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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

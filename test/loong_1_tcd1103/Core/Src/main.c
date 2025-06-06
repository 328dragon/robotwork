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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
	#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define max(a, b) ((a) >= (b) ? (a) : (b))
#define min(a, b) ((a) <= (b) ? (a) : (b))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	static uint32_t fac_us = 0; // us延时倍乘数

	 
	 uint8_t  ir_x[8] ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void delay_us(uint32_t nus)
{
	uint32_t ticks;
	uint32_t told, tnow, tcnt = 0;
	uint32_t reload = SysTick->LOAD; // LOAD的值
	ticks = nus * fac_us;		//需要的节拍数
	told = SysTick->VAL;		//刚进入时的计数器值
	while (1)
	{
		tnow = SysTick->VAL;
		if (tnow != told)
		{
			if (tnow < told)
				tcnt += told - tnow; //这里注意一下SYSTICK是一个递减的计数器就可以了.
			else
				tcnt += reload - tnow + told;
			told = tnow;
			if (tcnt >= ticks)
				break; //时间超过/等于要延迟的时间,则退出.
		}
	};
}

 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == htim6.Instance)
    {    
			if (HAL_GetTick() - icg_flag >= 10)
        {
            HAL_GPIO_WritePin(ICG_GPIO_Port, ICG_Pin, 1);
            HAL_GPIO_WritePin(SH_GPIO_Port, SH_Pin, 0);
            delay_us(2);
            HAL_GPIO_WritePin(SH_GPIO_Port, SH_Pin, 1);
            delay_us(4);
            HAL_GPIO_WritePin(ICG_GPIO_Port, ICG_Pin, 0);
            HAL_ADC_Start_DMA(&hadc3, (uint32_t *)ccd_rawdata, 1546);
            icg_flag = HAL_GetTick();
        }
        else
        {
            HAL_GPIO_WritePin(SH_GPIO_Port, SH_Pin, 0);
            delay_us(2);
            HAL_GPIO_WritePin(SH_GPIO_Port, SH_Pin, 1);
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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  MX_TIM8_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_ADC3_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2S3_Init();
  MX_SPI4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	//以下是定时器占用说明
	//tim1是给两个普通直流电机的，tim3是编码器1，tim5是编码器2，tim2只用于普通定时（利用中断）
	//tim6是给ccd的SH用，tim7是用来adc触发的  tim8是用于输出pwm给ccd的M tim4是用来给舵机用的  配置：340-1，10000-1
//  中断开启
HAL_TIM_Base_Start_IT(&htim2);
HAL_TIM_Base_Start_IT(&htim6);
HAL_TIM_Base_Start(&htim7);
//普通电机开启
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
//舵机
 HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
 HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
 HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);

//灰度配置
//  set_adjust_mode(1);
//  HAL_Delay(500);
//
//  set_adjust_mode(0);
//  HAL_Delay(500);

 //tcs230配置 
  HAL_UART_Receive_IT(&huart3, &RxData, 1);

  printf("AT+LIGHT+ON\r\n");
  printf("AT+LIGHT+ON\r\n");
  printf("AT+LIGHT+ON\r\n");
  printf("AT+LIGHT+ON\r\n");
  printf("AT+LIGHT+ON\r\n");
  printf("AT+LIGHT+ON\r\n");
	//普通电机配置
	speed_left = 0;
  speed_right = 0;
  speed_all = 0;
	PID_struct_init(&pid_left, POSITION_PID, 950, 700, 50, 0.5, 0);
  PID_struct_init(&pid_right, POSITION_PID, 950, 700, 50, 0.5, 0);
	
//ccd
int debug=0;
__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,60);

 //舵机 ，定时器4：通道3给钩子舵机，通道1，2给圆筒舵机，
 __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,1050);//580最上面到1050最下面
 int pw_servos=0;
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,1000);

 __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,1000);

//调试
char t[12];
sprintf(t,"l_w=%d,r_w=%d",l_w,r_w);
pid_left.set=15;
pid_right.set=15;
char enc[128];

step_move(3200,1);
  // set_motor_pwm(200,200);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		memcpy(ccd_data,ccd_rawdata,128*2);
//		HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//pw_servos=500;
//__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,pw_servos);
//__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,pw_servos);		
//pw_servos=900;
//HAL_Delay(1000);
//__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,pw_servos);
//__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_2,pw_servos);
//HAL_Delay(1000);
		
//  sprintf(enc,"%ld\r\n",Encode_L);
//  HAL_UART_Transmit(&huart1,(uint8_t*)enc,strlen(enc),HAL_MAX_DELAY);
		
		deal_IRdata(&ir_x1,&ir_x2,&ir_x3,&ir_x4,&ir_x5,&ir_x6,&ir_x7,&ir_x8);
	
		sprintf(huidu,"%d,%d,%d,%d,%d,%d,%d,%d\r\t",ir_x1,ir_x2,ir_x3,ir_x4,ir_x5,ir_x6,ir_x7,ir_x8);
  HAL_UART_Transmit(&huart1,(uint8_t*)huidu,strlen(huidu),HAL_MAX_DELAY);
	HAL_Delay(300);	
//    int32_t sum = 0;
//    _max = 0, _min = 65535;
//    for (int i = 0; i < 128; i++)
//    {
//        int32_t data = 0;
//        for (int j = 0; j < 8; j++)
//            data += (ccd_rawdata[272 + i * 8 + j] - 380);
//        ccd_data[i] = (data / 8);
//        sum += ccd_data[i];
//        _max = max(_max, ccd_data[i]);
//        _min = min(_min, ccd_data[i]);
//    }
//    avg = sum / 128;
		
//   FindLines(&l, &r, ccd_data, 500, &l_w, &r_w); 
////		sprintf(t,"l_w=%d,r_w=%d",l_w,r_w);
////		
//HAL_UART_Transmit(&huart1,t,strlen(t),50);
//HAL_Delay(100);		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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

#ifdef  USE_FULL_ASSERT
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

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32g4xx_it.c
 * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
#include "FreeRTOS.h"
#include "task.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tcs230.h"
#include "bsp_usart.h"
#include "tcd1103.h"
#include "dc_motor.h"
#include "gray.h"
#include "mainwork.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_adc4;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;
/* USER CODE BEGIN EV */
#define stepmotor_on() HAL_GPIO_WritePin(step_mot_en_GPIO_Port, step_mot_en_Pin, 1);
#define stepmotor_off() HAL_GPIO_WritePin(step_mot_en_GPIO_Port, step_mot_en_Pin, 0);
#define stepmotor_pos() HAL_GPIO_WritePin(step_mot_dir_GPIO_Port, step_mot_dir_Pin, 1);
#define stepmotor_neg() HAL_GPIO_WritePin(step_mot_dir_GPIO_Port, step_mot_dir_Pin, 0);
#define step_pulse_high() HAL_GPIO_WritePin(step_mot_pulse_GPIO_Port, step_mot_pulse_Pin, 1);
#define step_pulse_low() HAL_GPIO_WritePin(step_mot_pulse_GPIO_Port, step_mot_pulse_Pin, 0);

extern USARTInstance uart2;

extern int dc_step_distance; // 步进距离
extern int step_complete_flag; // 步进模式直流电机完成标志位
extern int dc_step_flag;
// 电机使用量
extern int motor_0_user_speed;
extern int motor_1_user_speed;
int16_t encoder_data[2] = {0};
int motor_0_speed;
int motor_1_speed;
int motor_0_active;
int motor_1_active;
int tem_encoder_a = 0;
int tem_encoder_b = 0;
// 电机状态机使用量
int encoder_l_temp = 0;
int encoder_r_temp = 0;
int dc_motor_state = 0;
//灰度传感器使用量
//前面
float gray_bia=0;
float gray_bia_p=1.8;
//背面
float gray_bia_back=0;
float gray_bia_p_back=1.2;
//逻辑总使用量
float gray_bia_genaral=0;
float gray_bia_p_general=1.2;
// 步进电机使用变量
int step_motor_active = 0;
int step_motor_speed = 0;
int step_motor_direction = 0; // 0:正转,1:反转
int step_motor_state = 0;
int step_motor_delay = 0;
int step_motor_temp_count = 0;
int delay_no_conflict(int *delay_temp_count, int delay_time)
{

  (*delay_temp_count)++;
  if (*delay_temp_count >= delay_time)
  {
    *delay_temp_count = 0;
    return 1;
  }
  else
    return 0;
}

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
  {
#endif /* INCLUDE_xTaskGetSchedulerState */
  xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
  }
#endif /* INCLUDE_xTaskGetSchedulerState */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */

  /* USER CODE END DMA1_Channel2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc4);
  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */

  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM17 global interrupt.
  */
void TIM1_TRG_COM_TIM17_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 0 */
  // 5ms一次，对于编码器很合适
  int32_t sum = 0;
  _max = 0, _min = 65535;
  for (int i = 0; i < 128; i++)
  {
    int32_t data = 0;
    for (int j = 0; j < 8; j++)
      data += (ccd_rawdata[272 + i * 8 + j] - 380);
    ccd_data[i] = (data / 8);
    sum += ccd_data[i];
    _max = max(_max, ccd_data[i]);
    _min = min(_min, ccd_data[i]);
  }
  avg = sum / 128;
  FindLines(&l, &r, ccd_data, 500, &l_w, &r_w);
 
gray_bia=get_black_line_position(front);
gray_bia_back=get_black_line_position(back);	

switch (motor_mode)
{
case MOTOR_MODE_NORMAL:
{

	gray_bia_genaral=gray_bia;
	gray_bia_p_general=gray_bia_p	;
  break;
}
case MOTOR_MDOE_NORMAL_BACK:
{
  	gray_bia_genaral=gray_bia_back;
	gray_bia_p_general=gray_bia_p_back;
  break;
}
case MOTOR_MODE_TURN:
{
gray_bia_p_general=0;//让车子速度完全等于命令值
  break;
}
default:
  break;
}

 motor_1_speed =motor_1_user_speed+gray_bia_p_general*gray_bia_genaral;
 motor_0_speed =motor_0_user_speed-gray_bia_p_general*gray_bia_genaral;

if(dc_step_flag==1)
{
    switch (dc_motor_state)
    {
    case 0:
    {
      encoder_l_temp = encoder_0.pulse_sum;
      encoder_r_temp = encoder_1.pulse_sum;
      dc_motor_state++;
      break;
    }
    case 1:
    {
      int encoder_1_distance = encoder_0.pulse_sum - encoder_l_temp;
      int encoder_2_distance = encoder_1.pulse_sum - encoder_r_temp;
      if (dc_step_distance >= 0)
      {
        if ((encoder_1_distance >= dc_step_distance) || (encoder_2_distance >= dc_step_distance))
        {
          motor_0_speed = 0;
          motor_1_speed = 0;         
          step_complete_flag=1;
        }
      }
      else if (dc_step_distance < 0)
      {
        if ((encoder_1_distance <= dc_step_distance) && (encoder_2_distance <= dc_step_distance))
        {
          motor_0_speed = 0;
          motor_1_speed = 0;
          step_complete_flag=1;
        }
      }
      break;
    }

    default:
      break;
    }
}


  //  char string_ccd[30] = {0};
  //  string_ccd[0] = l_w;
  //  string_ccd[1] = avg;
  //  string_ccd[2] = r_w;
  //  sprintf(string_ccd, "%d,%d,%d\r\n", string_ccd[0], string_ccd[1], string_ccd[2]);
  //  USARTSend(&uart2, (uint8_t *)string_ccd, 30, USART_TRANSFER_DMA);
if(motor_mode==MOTOR_STOP)
{
motor_0_speed=0;
motor_1_speed=0;
}
  // 电机控制,motor_0_speed和motor_1_speed是电机实际输入速度
  DCMotorSetSpeedCloseLoop(&motor_0, motor_0_speed, motor_0_active);
  DCMotorSetSpeedCloseLoop(&motor_1, motor_1_speed, motor_1_active);
  // 打印信息
  encoder_data[0] = encoder_0.pulse;
  encoder_data[1] = encoder_1.pulse;
  char string_encoder[20] = {0};
  sprintf(string_encoder, "%d,%d\n", encoder_data[0], encoder_data[1]);
  USARTSend(&uart2, (uint8_t *)string_encoder, 20, USART_TRANSFER_DMA);

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  HAL_TIM_IRQHandler(&htim17);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM17_IRQn 1 */

  /* USER CODE END TIM1_TRG_COM_TIM17_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
  if (step_motor_active == 1)
    stepmotor_on() else if (step_motor_active == 0)
        stepmotor_off() if (step_motor_direction == 0) // 正向
        stepmotor_pos() else stepmotor_neg()

            switch (step_motor_state)
    {
    case 0:
    {
      step_pulse_high()
          step_motor_state += delay_no_conflict(&step_motor_temp_count, step_motor_speed);
      break;
    }
    case 1:
    {
      step_pulse_low()
          step_motor_state = 0;
      break;
    }
    default:
      break;
    }

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles TIM6 global interrupt, DAC1 and DAC3 channel underrun error interrupts.
  */
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/**
  * @brief This function handles FDCAN2 interrupt 1.
  */
void FDCAN2_IT1_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN2_IT1_IRQn 0 */

  /* USER CODE END FDCAN2_IT1_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan2);
  /* USER CODE BEGIN FDCAN2_IT1_IRQn 1 */

  /* USER CODE END FDCAN2_IT1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

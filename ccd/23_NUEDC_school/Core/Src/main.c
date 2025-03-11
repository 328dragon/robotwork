/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "delay.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "pid.h"
#include "kalmanfilter.h"
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
uint16_t ccd_rawdata[1546]; // 储存CCD接收的原始数�???????????
uint16_t ccd_data[128];     // 储存平均后的CCD数据

float L_speed_setpoint = 12, R_speed_setpoint = 12;

uint32_t icg_flag, stop_line_flag;
uint8_t l, r;
uint8_t l_w, r_w;
uint16_t avg, _max, _min;

uint8_t inner_flag = 1;

float delta;

uint16_t light_setpoint = 200;
float Kp = 0.09;

KalmanFilter_t ccd_filter;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DriveMotors(int16_t l, int16_t r)
{
    if (l > 1000)
        l = 1000;
    if (r > 1000)
        r = 1000;
    HAL_GPIO_WritePin(PH1_GPIO_Port, PH1_Pin, (GPIO_PinState)(l >= 0));
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, l > 0 ? l : (0 - l));
    HAL_GPIO_WritePin(PH2_GPIO_Port, PH2_Pin, (GPIO_PinState)(r >= 0));
    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, r > 0 ? r : (0 - r));
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
            HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ccd_rawdata, 1546);
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
    else if (htim->Instance == htim10.Instance)
    {
        // volatile float light = -pid_calc(&pid_light, avg, 270);
        // htim6.Instance->PSC = 500 + (int16_t)light;
        uint8_t line = inner_flag ? l : r;
        int16_t psc;
        psc = htim6.Instance->PSC;
        psc += (avg - 240) * Kp;
        if (psc < 30)
            psc = 30;
        htim6.Instance->PSC = psc;
				
				
				
        int16_t ccr_l = (int16_t)__HAL_TIM_GetCounter(&htim1), ccr_r = (int16_t)__HAL_TIM_GetCounter(&htim3);
        __HAL_TIM_SetCounter(&htim1, 0);
        __HAL_TIM_SetCounter(&htim3, 0);
        char t[128];
        sprintf(t, "X1=%d, X2=%d, X3=%d\r\n", psc, l_w, r_w);
        HAL_UART_Transmit(&huart1, t, strlen(t), 1000);

        if (l != 128)
            delta = 0.2 * pid_calc(&pid_pos, line, 64);
        volatile float speed_r = -pid_calc(&pid_r, -ccr_r, (R_speed_setpoint + delta));
        volatile float speed_l = pid_calc(&pid_l, ccr_l, (L_speed_setpoint - delta));
        DriveMotors(10 * speed_l, 10 * speed_r);
    }
}

// 128点找左右
void FindLines(uint8_t *l, uint8_t *r, uint16_t *data, uint16_t threshold, uint8_t *l_w, uint8_t *r_w)
{
    *r = *l = 128;
    for (int i = 0; i < 128 - 6; i++)
    {
        if (data[i + 6] - data[i] > threshold)
        {
            for (int j = i + 1; j < 128 - 6; j++)
                if (data[j] - data[j + 6] > threshold)
                {
                    *l = (i + j + 6) / 2, *l_w = j + 6 - i;
                    break;
                }
            if (*l == 128)
                *l = (i + 127) / 2, *l_w = 127 - i;
            break;
        }
        if(data[i] - data[i + 6] > threshold)
        {
            *l = i  / 2, *l_w = i;
            break;
        }
    }
    for (int i = 127; i >= 0 + 6; i--)
    {
        if (data[i - 6] - data[i] > threshold)
        {
            for (int j = i - 1; j >= 6; j--)
                if (data[j] - data[j - 6] > threshold)
                {
                    *r = (i + j - 6) / 2, *r_w = i - j + 6;
                    break;
                }
            if (*r == 128)
                *r = i / 2, *r_w = i;
            break;
        }
        if(data[i] - data[i - 6] > threshold)
        {
            *r = (i + 127) / 2, *l_w = 127 - i;
            break;
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
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_ADC1_Init();
    MX_TIM9_Init();
    MX_USART1_UART_Init();
    MX_TIM8_Init();
    MX_TIM6_Init();
    MX_TIM10_Init();
    /* USER CODE BEGIN 2 */
    PID_struct_init(&pid_r, DELTA_PID, 95, 95, 2.f, 1.f, 1.f);
    PID_struct_init(&pid_l, DELTA_PID, 95, 95, 2.f, 1.f, 1.f);
    PID_struct_init(&pid_pos, POSITION_PID, 95, 95, 0.4f, 0.f, 0.33f);
    PID_struct_init(&pid_light, POSITION_PID, 450, 320, 0.5f, 0.004f, 0.5f);

    KalmanFilter_init(&ccd_filter, 3, 80);

    HAL_TIM_Base_Start(&htim8);
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim10);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

    OLED_Init();
    delay_init(168);

    HAL_Delay(500);

    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
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
//        OLED_ShowChart(ccd_data, 1, 1700, (_max * 7 + _min) / 8);
        FindLines(&l, &r, ccd_data, 500, &l_w, &r_w);
//        HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, r_w > 50);
				
        if (r_w > 50 && HAL_GetTick() - stop_line_flag >= 500)
        {
            inner_flag = !inner_flag;
            stop_line_flag = HAL_GetTick();
        }
//        OLED_ShowChar(l, 0, 'L', 8, 1);
//        OLED_ShowChar(r, 8, 'R', 8, 1);
//        OLED_Refresh();

        // for(int i = 0; i < 1500; i++)
        // {
        //     char t[128];
        //     sprintf(t, "X1=%d\r\n", ccd_rawdata[i + 32]);
        //     HAL_UART_Transmit(&huart1, t, strlen(t), 1000);
        // }
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
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

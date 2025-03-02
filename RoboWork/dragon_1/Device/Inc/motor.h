#ifndef Motor_H
#define Motor_H


#include "stdbool.h"
#include "main.h"
#include "stm32g4xx_hal.h" 
#define abs(a) (a>0)?(a):(-a)
#define sp_motor_speed  20
#define sp_motor_on    HAL_GPIO_WritePin(ENA_GPIO_Port,ENA_Pin,1)
#define sp_motor_off     HAL_GPIO_WritePin(ENA_GPIO_Port,ENA_Pin,0)

	#define dir_pst   HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,1)
#define dir_ngt     HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,0)

#define pulse_on     HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin,1)
#define pulse_off     HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin,0)

void step_move(uint32_t step,int  direction);
void step_drift(int direction);
void step_arrive(int position);
extern int now_position ;

#endif

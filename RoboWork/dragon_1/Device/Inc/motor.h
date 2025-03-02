#ifndef Motor_H
#define Motor_H


#include "stdbool.h"
#include "main.h"
#include "stm32g4xx_hal.h"  
#include "tim.h"
#define abs(a) (a>0)?(a):(-a)
#define sp_motor_speed  20
#define sp_motor_on    HAL_GPIO_WritePin(ENA_GPIO_Port,ENA_Pin,1)
#define sp_motor_off     HAL_GPIO_WritePin(ENA_GPIO_Port,ENA_Pin,0)

	#define dir_pst   HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,1)
#define dir_ngt     HAL_GPIO_WritePin(DIR_GPIO_Port,DIR_Pin,0)

#define pulse_on     HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin,1)
#define pulse_off     HAL_GPIO_WritePin(PUL_GPIO_Port,PUL_Pin,0)



#define motor1_front HAL_GPIO_WritePin(motor_1_ph_GPIO_Port,motor_1_ph_Pin,1);//左正转

#define motor1_back HAL_GPIO_WritePin(motor_1_ph_GPIO_Port,motor_1_ph_Pin,0);//左反转

#define motor2_front HAL_GPIO_WritePin(motor_2_ph_GPIO_Port,motor_2_ph_Pin,0);//右正转

#define motor2_back HAL_GPIO_WritePin(motor_2_ph_GPIO_Port,motor_2_ph_Pin,1);//右反转

void step_move(uint32_t step,int  direction);
void step_drift(int direction);
void step_arrive(int position);
extern int now_position ;


#define PWM_limit 950


extern float pid_L;
extern float pid_R;

extern int speed_left ;
extern int speed_right ;
extern int speed_all;


extern int  encoder_1_flag;
extern int encoder_2_flag;


extern int16_t Encode_L;
extern int16_t Encode_R;
extern int turn_flag;


extern int  PWM_duoji ;

extern int encoder_1_distance;//用于距离
extern int encoder_2_distance;
extern int dest_distance;
void set_motor_pwm(int motor_left,int motor_right);
void Read_Encoder();
void go_step(int speed,int destination);
 void DuoJi_Angle(int angle , int time);

#endif

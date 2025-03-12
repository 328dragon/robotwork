#include "motor.h"
int now_position=1;
int speed_left = 0;
int speed_right = 0;
int speed_all = 0;//总速度，绝对值


//普通电机
int  encoder_1_flag=0;//当开始用固定距离时把他设为1
int encoder_2_flag=0;
int encoder_1_pid;//用于pid
int encoder_2_pid;
int encoder_1_distance;//用于距离
int encoder_2_distance;
int dest_distance;
int turn_flag;

float pid_L;
float pid_R;

int16_t Encode_L;
int16_t Encode_R;

//步进电机底层转动多少
void step_move(uint32_t step,int  direction) {
    sp_motor_on;
    for (uint32_t i = 0; i < step; i++)
    {
        if(direction)
        {
            dir_pst;
            pulse_on;
            delay_us(sp_motor_speed);
            pulse_off;
            delay_us(sp_motor_speed);
        } else if(direction==-1)
        {
            dir_ngt;
            pulse_on;
            delay_us(sp_motor_speed);
            pulse_off;
            delay_us(sp_motor_speed);
        }
    }
    sp_motor_off;
}
//步进电机相邻之间位置移动

//需要调cycle和mapping值
void step_drift(int direction)
{   //转多少圈（cycle）
    uint16_t cycle=1;
    //一圈对应多长
    uint16_t mapping=1;
    uint32_t fixed_distance=(32)*(20)*cycle*mapping;
    if(direction)
    {
        step_move(fixed_distance,1);
        now_position+=1;
    } else if(direction==-1)
    {
        step_move(fixed_distance,0);
        now_position-=1;
    }
}

//步进电机具体转到哪个位置
void step_arrive(int des_position)
{
    //position分为3个位置，上中下::3,2,1
    int drift;
    uint16_t direction;
    if(des_position>=now_position)
    {
        direction=1;
    }
    else
        direction=-1;
    drift=abs(des_position-now_position);
    for(int i=0; i++; i<=drift)
        step_drift(direction);
}







/// @brief 用于设置pwm,motor_left是左边的,motor_right是右边的
/// @param motor_left
/// @param motor_right
void set_motor_pwm(int motor_left,int motor_right)
{

    //限幅
    if (motor_left > PWM_limit)  motor_left = PWM_limit;
    if (motor_left < -PWM_limit)  motor_left = -PWM_limit;
    if (motor_right > PWM_limit)  motor_right = PWM_limit;
    if (motor_right < -PWM_limit)  motor_right = -PWM_limit;
//左电机 HTIM5 TIM_CHANNEL1
    if(motor_left>=0)
    {
        motor1_front;

        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,motor_left);


    }
    else
    {
        motor1_back;

        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, -motor_left);

    }
//右电机  HTIM5 TIM_CHANNEL2
    if(motor_right>=0)
    {
        motor2_front;

        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,motor_right);


    } else {
        motor2_back;
        __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, -motor_right);

    }


}



/// @brief 用于读取编码器值
/// @param TIMX
/// @return 编码器值
void Read_Encoder()
{
    Encode_L = __HAL_TIM_GetCounter(&htim3);
    Encode_R = -__HAL_TIM_GetCounter(&htim5);
    __HAL_TIM_SetCounter(&htim3, 0);
    __HAL_TIM_SetCounter(&htim5, 0);


}

void go_step(int speed,int destination)
{

    speed_all = speed;
    dest_distance=destination;
    encoder_1_flag=1;
    while(1)
    {
        if(encoder_1_flag == 0)

            break;
    }

    turn_flag=1;

}
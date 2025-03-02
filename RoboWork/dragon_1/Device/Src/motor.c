#include "motor.h"
 int now_position=1;
 
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
{ //转多少圈（cycle）
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
	{direction=1;}
	else
		direction=-1;
	drift=abs(des_position-now_position);
	for(int i=0;i++;i<=drift)
	step_drift(direction);
}


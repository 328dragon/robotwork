#include "Catch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mainwork.h"
#include <stdlib.h>
extern int read_cololr_flag;
extern int goods_color;

int catch_finish_flag = 0;
//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);//大转盘，950刚好一个对齐屁股，
// __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,1250);//1250卡住，2200松开，左小蓝机
//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,1800);//1800卡住，900松开，右小蓝机	
//__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4 ,1820);//小转盘，1820为里，600为外
//__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,2000);//抬升，500最高，800中间，1500最低
//		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1 ,1300);//夹爪，1800紧，1500松
//		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2 ,1900);//夹爪1100锁紧，1500松
// 放置的时候
//    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1970);  // 绿色
//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,2300); // 白
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,2630); // 红
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1300); // 蓝
//		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,960); // 黑色

// 500 蓝
// 1200 绿
// 1500  白
// 1850 红
// 2180  黑
	

	
void Sort(enum color_enum color)
{
	switch (color)
	{
		case green_color: __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1150);break;
		case white_color: __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1500);break;
		case blue_color: __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,500);break;
		case red_color: __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1850);break;
		case black_color: __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,2180);break;
	}

	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,500);// 抬升 保证不干涉
	switch (color)
	{
		case green_color: vTaskDelay(800);break;
		case white_color: vTaskDelay(800);break;
		case blue_color: vTaskDelay(800);break;
		case red_color: vTaskDelay(800);break;
		case black_color: vTaskDelay(800);break;
	}
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4 ,1850);//小转盘 转到里面
		vTaskDelay(800);
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1 ,1360);//夹爪，1800紧，1400松
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2 ,1640);//夹爪1100锁紧，1600松
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4 ,600);//小转盘 转到外面
	vTaskDelay(200);
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,600);// 下降
}

enum color_enum Catch()
{
	__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1850);// 先让红色在中间
			__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,500);// 抬升 保证不干涉
		vTaskDelay(500);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4 ,600);// 保证夹爪朝外
	vTaskDelay(1000);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,1300);// 下降
	vTaskDelay(850);
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2 ,1100);// 夹紧
	__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1 ,1800);// 夹紧
	vTaskDelay(850);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,500);// 抬升
	catch_finish_flag = 1;							 // 抓取标志位置1
	// 去识别颜色
vTaskDelay(500);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4 ,1520);//小转盘 转到里面
	vTaskDelay(500);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,800);// 下降

//	color_return = Get_Color();
	read_cololr_flag = 1;
	vTaskDelay(1500);
	while(1)
	{	
		if(goods_color != -1)
		{
			Sort(goods_color);
			goods_color = -1;
			return 0;
		}
	vTaskDelay(200);
	}
}

enum color_enum Catch_dragout(int delay_time)
{
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1850);// 先让红色在中间
            __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,500);// 抬升 保证不干涉
        vTaskDelay(500);
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4 ,600);// 保证夹爪朝外
    vTaskDelay(1000);
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,1300);// 下降
    vTaskDelay(200);
	catch_finish_flag = 1;							  // 抓取标志位置1
    vTaskDelay(delay_time);
    vTaskDelay(650);
    __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2 ,1100);// 夹紧
    __HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1 ,1800);// 夹紧
    vTaskDelay(850);
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,500);// 抬升
    // 去识别颜色
vTaskDelay(500);
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4 ,1520);//小转盘 转到里面
    vTaskDelay(500);
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,800);// 下降

//    color_return = Get_Color();
    read_cololr_flag = 1;
    vTaskDelay(1500);
    while(1)
    {    
        if(goods_color != -1)
    {
    Sort(goods_color);
        goods_color = -1;
    return 0;
    }
    vTaskDelay(200);
    }
}

void Drop(enum color_enum color)
{

	switch(color)
	{
		case green_color: __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1970); break; // 绿色
		case white_color: __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,2300); break;// 白
		case red_color:   __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,2630); break; // 红
    case blue_color:  __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,1300); break; // 蓝
		case black_color: __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,960);  break;// 黑色
	}
	vTaskDelay(850);
	int steps = (2200 - 1250) > (1950 - 900) ? (2200 - 1250) : (1950 - 900);
	steps = abs(steps);  // 确保步数是正数

	// 同时缓慢移动两个舵机
	for(int i = 0; i <= steps; i++) {
			// 计算左舵机当前值（从1200到2200）
			int left_pos = 1200 + (2200 - 1200) * i / steps;
			
			// 计算右舵机当前值（从1950到900）
			int right_pos = 1950 - (1950 - 900) * i / steps;
			
			// 设置PWM占空比
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, left_pos);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, right_pos);
			
			HAL_Delay(1);  // 调整这个延时可以改变整体速度（单位：毫秒）
	}

	// 确保最终到达目标值
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2200);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 900);	
}
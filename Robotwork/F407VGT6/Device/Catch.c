#include "Catch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mainwork.h"
extern int read_cololr_flag;
extern int goods_color;
//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);//大转盘，950刚好一个对齐屁股，
//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,1800);//1800卡住，2500松开，左小蓝机
//__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,1800);//1800卡住，900松开，右小蓝机	
//__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4 ,1820);//小转盘，1820为里，600为外
//__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,2000);//抬升，500最高，800中间，1500最低
//		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1 ,1300);//夹爪，1800紧，1500松
//		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2 ,1900);//夹爪1100锁紧，1500松


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
	vTaskDelay(1500);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4 ,1850);//小转盘 转到里面
		vTaskDelay(1500);
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_1 ,1360);//夹爪，1800紧，1400松
		__HAL_TIM_SET_COMPARE(&htim9,TIM_CHANNEL_2 ,1640);//夹爪1100锁紧，1600松
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4 ,600);//小转盘 转到外面
	vTaskDelay(200);
//	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,600);// 下降
}

enum color_enum Catch()
{
	
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
	// 去识别颜色
vTaskDelay(500);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4 ,1520);//小转盘 转到里面
	vTaskDelay(500);
	__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3 ,800);// 下降

//	color_return = Get_Color();
	read_cololr_flag=1;
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
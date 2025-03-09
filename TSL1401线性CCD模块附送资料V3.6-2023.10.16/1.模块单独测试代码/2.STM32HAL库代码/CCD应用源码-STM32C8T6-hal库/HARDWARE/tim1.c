#include "tim1.h"
int pixel_array[128]={0};      //整型数组，储存从左到右128个像素的灰度值

/**************************************************************************
函数功能：延时
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++);      
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		u8 i=0,tslp=0;
		
		//SI高电平激活CCD采集、输出任务
		TSL_CLK=1; //延长上一CLK时钟周期状态
		TSL_SI=0; 				
		TSL_SI=1;  //SI高电平
		TSL_CLK=0;
		Dly_us();		
		TSL_CLK=1; //注意SI的下降沿必须在CLK高电平时发生
		TSL_SI=0;  //SI低电平  
		Dly_us();	 
		
		//激活后的128个CLK时钟周期由单片机ADC采集CCD的AO引脚输出的灰度值（电压值）
		//同时CCD硬件自动采集灰度值，为下一次AO引脚输出灰度值做准备
		for(i=0;i<128;i++)
		{ 
			TSL_CLK=0; 
			Dly_us();	  //调节曝光时间  
			pixel_array[tslp]=(Get_Adc(ADC_CHANNEL_1))>>4; //下降沿后读取GPIOA1上的ADC值(灰度值),并逐个保存到pixel_array数组
//			                                               //ADC取值范围大小为4096，灰度值取值范围大小为256
//			                                               //所以将ADC读取值移右4位，即除以16
			++tslp;
			TSL_CLK=1;
			Dly_us();	 //调节曝光时间
		} 
	}
}





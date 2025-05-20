#include "TSL1401.h"
int ADV[128]={0};      //整型数组，储存从左到右128个像素的灰度值
uint8_t Flag_Target;
 uint8_t CCD_Zhongzhi,CCD_Yuzhi;
 
 //延时
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++);      
}

		uint16_t Get_Adc(uint8_t ch)   
{
	uint16_t result;
	ADC_ChannelConfTypeDef sConfig;//通道初始化
	sConfig.Channel=ch;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5 ;		//采用周期239.5周期
	sConfig.Rank = 1;
	HAL_ADC_ConfigChannel(&hadc3,&sConfig);		
	
	HAL_ADC_Start(&hadc3);								//启动转换
	HAL_ADC_PollForConversion(&hadc3,30);				//等待转化结束
	if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc3), HAL_ADC_STATE_REG_EOC))
	{
		result=HAL_ADC_GetValue(&hadc3);	//返回最近一次ADC1规则组的转换结果
	}
	return result;
}
 void RD_TSL(void) 
{
		u8 i=0,tslp=0;
		
		//SI高电平激活CCD采集、输出任务
HAL_GPIO_WritePin(CLK_A_GPIO_Port,CLK_A_Pin,1);	//延长上一CLK时钟周期状态
	HAL_GPIO_WritePin(SI_A_GPIO_Port,SI_A_Pin,0);				
	HAL_GPIO_WritePin(SI_A_GPIO_Port,SI_A_Pin,1);	 //SI高电平
HAL_GPIO_WritePin(CLK_A_GPIO_Port,CLK_A_Pin,0);
		Dly_us();		
	HAL_GPIO_WritePin(CLK_A_GPIO_Port,CLK_A_Pin,1); //注意SI的下降沿必须在CLK高电平时发生
	HAL_GPIO_WritePin(SI_A_GPIO_Port,SI_A_Pin,0);  //SI低电平  
		Dly_us();	 
		
		//激活后的128个CLK时钟周期由单片机ADC采集CCD的AO引脚输出的灰度值（电压值）
		//同时CCD硬件自动采集灰度值，为下一次AO引脚输出灰度值做准备
		for(i=0;i<128;i++)
		{ 
		HAL_GPIO_WritePin(CLK_A_GPIO_Port,CLK_A_Pin,0);
			Dly_us();	  //调节曝光时间  
			ADV[tslp]=(Get_Adc(ADC_CHANNEL_2))>>4; //下降沿后读取GPIOA1上的ADC值(灰度值),并逐个保存到pixel_array数组
//			                                               //ADC取值范围大小为4096，灰度值取值范围大小为256
//			                                               //所以将ADC读取值移右4位，即除以16
			++tslp;
			HAL_GPIO_WritePin(CLK_A_GPIO_Port,CLK_A_Pin,1);
			Dly_us();	 //调节曝光时间
		}  
}

/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static uint16_t i,j,Left,Right;
	 static uint16_t value1_max,value1_min;
	
	   value1_max=ADV[0];  //动态阈值算法，读取最大和最小值
     for(i=5;i<123;i++)   //两边各去掉5个点
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //最小值
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
		 
		 
	 for(i = 5;i<118; i++)   //寻找左边跳变沿
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//寻找右边跳变沿
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//计算中线位置
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //计算中线的偏差，如果太大
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差
}

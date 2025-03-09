#include "tim1.h"
int ADV[128]={0};      //整型数组，储存从左到右128个像素的灰度值
u8 Flag_Target;
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
	if(htim == &htim3)
	{
		Flag_Target=!Flag_Target;
		if(Flag_Target)	RD_TSL();                           		 //===读取线性CCD数据 
		Find_CCD_Zhongzhi();			          		 //===提取中线 
	}
}

/**************************************************************************
函数功能：CCD数据采集
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
 void RD_TSL(void) 
{
 u8 i=0,tslp=0;
   TSL_CLK=1;
  TSL_SI=0; 
  Dly_us();
	
      
   
  TSL_CLK=0;
	TSL_SI=1;
  Dly_us();
 Dly_us();
  Dly_us();
	
	 TSL_CLK=1;
  TSL_SI=1;
  Dly_us();
	
  TSL_CLK=1;
  TSL_SI=0;
  Dly_us();  
  for(i=0;i<128;i++)
  { 
    TSL_CLK=0; 
    Dly_us();  //调节曝光时间
		Dly_us();  //调节曝光时间
    ADV[tslp]=(Get_Adc(ADC_CHANNEL_4))>>4;
    ++tslp;
    TSL_CLK=1;
     Dly_us();
  }  
}

/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right;
	 static u16 value1_max,value1_min;
	
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


#include "TSL1401.h"
uint16_t ADV[128]= {0};     //整型数组，储存从左到右128个像素的灰度值
uint8_t Flag_Target;
uint8_t CCD_Zhongzhi,CCD_Yuzhi,Last_CCD_Zhongzhi;
uint8_t SciBuf[200];  //存储上传到上位机的信息;

int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

//延时
void Dly_us(void)
{
    int ii;
    for(ii=0; ii<10; ii++);
}

uint16_t Get_Adc(uint8_t ch)
{
    uint16_t result;
    ADC_ChannelConfTypeDef sConfig;//通道初始化
    sConfig.Channel=ch;
    sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5 ;		//采用周期239.5周期
    sConfig.Rank = 1;
    HAL_ADC_ConfigChannel(&hadc3, &sConfig);

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
    clk_B_on;
	Dly_us();
	si_b_off;
	Dly_us();

	clk_B_on;
	Dly_us();
	si_b_off;
	Dly_us();

	clk_B_on;
	Dly_us();
	si_b_off;
	Dly_us();
    //激活后的128个CLK时钟周期由单片机ADC采集CCD的AO引脚输出的灰度值（电压值）
    //同时CCD硬件自动采集灰度值，为下一次AO引脚输出灰度值做准备
    for(i=0; i<128; i++)
    {
		clk_B_off;
        Dly_us();	  //调节曝光时间
        Dly_us();	  //调节曝光时间
        ADV[tslp]=(Get_Adc(2))>>4; //下降沿后读取GPIOA1上的ADC值(灰度值),并逐个保存到pixel_array数组
			                                               //ADC取值范围大小为4096，灰度值取值范围大小为256
			                                               //所以将ADC读取值移右4位，即除以16
        ++tslp;
		clk_B_on;
        Dly_us();	 //调节曝光时间
    }
}

//利用串口1发送数据
void usart1_send(uint8_t *data,uint8_t len)
{
	char send_data[128];
	HAL_UART_Transmit(&huart1,data,len,0xffff);
}



/******************************************************************************
***
* FUNCTION NAME: void sendToPc(void) *
* CREATE DATE : 20170707 *
* CREATED BY : XJU *
* FUNCTION : 将待发送的信息通过串口发送至上位机*
* MODIFY DATE : NONE *
* INPUT : void *
* OUTPUT : NONE *
* RETURN : NONE *
*******************************************************************************
**/ 	 	 
void sendToPc(void)
{ 
	int i;
	slove_data();
	usart1_send("*",1);
	usart1_send("LD",2);
	for(i=2;i<134;i++)
	{ 
		usart1_send(binToHex_high(SciBuf[i]),1); //以字符形式发送高4位对应的16进制
		usart1_send(binToHex_low(SciBuf[i]),1);  //以字符形式发送低4位对应的16进制:

	}
    usart1_send("00",2);   //通信协议要求
	usart1_send("#",1);    //通信协议要求
}

	 void slove_data(void)
{
   int i;
   RD_TSL();
SciBuf[0] = 0; 
 SciBuf[1] = 132;
SciBuf[2] = 0; 
SciBuf[3] = 0;
 SciBuf[4] = 0;
SciBuf[5] = 0; 
   for(i=0;i<128;i++)
	   SciBuf[6+i] = ADV[i];
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
	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //计算中线的偏差，如果太大
	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差
}




	 /******************************************************************************
***
* FUNCTION NAME: binToHex_low(u8 num) *
* CREATE DATE : 20170707 *
* CREATED BY : XJU *
* FUNCTION : 将二进制低8位转换16进制*
* MODIFY DATE : NONE *
* INPUT : u8 *
* OUTPUT : NONE *
* RETURN : char *
*******************************************************************************
**/ 	 	 
 char binToHex_low(u8 num)
 {u8 low_four;
	 low_four=num&0x0f;
	 if(low_four==0)
		 return('0');
	 else if(low_four==1)
		  return('1');
	 else if(low_four==2)
		  return('2');
	 else if(low_four==3)
		  return('3');
	 else if(low_four==4)
		  return('4');
	 else if(low_four==5)
		  return('5');
	 else if(low_four==6)
		  return('6');
	 else if(low_four==7)
		  return('7');
	 else if(low_four==8)
		  return('8');
	 else if(low_four==9)
		  return('9');
	 else if(low_four==10)
		  return('A');
	 else if(low_four==11)
		  return('B');
	 else if(low_four==12)
		  return('C');
	 else if(low_four==13)
		  return('D');
	 else if(low_four==14)
		  return('E');
	 else if(low_four==15)
		  return('F');
 }
 
/******************************************************************************
***
* FUNCTION NAME: binToHex_low(u8 num) *
* CREATE DATE : 20170707 *
* CREATED BY : XJU *
* FUNCTION : 将二进制高8位转换16进制*
* MODIFY DATE : NONE *
* INPUT : u8 *
* OUTPUT : NONE *
* RETURN : char *
*******************************************************************************
**/ 						 
 char binToHex_high(u8 num)
 {
		u8 high_four;
		high_four=(num>>4)&0x0f;
		if(high_four==0)
			return('0');
				else if(high_four==1)
					return('1');
					else if(high_four==2)
							return('2');
							else if(high_four==3)
								return('3');
								else if(high_four==4)
								return('4');
									else if(high_four==5)
									return('5');
										else if(high_four==6)
											return('6');
											else if(high_four==7)
											return('7');
												else if(high_four==8)
												return('8');
													else if(high_four==9)
														return('9');
														else if(high_four==10)
															return('A');
															else if(high_four==11)
																return('B');
																else if(high_four==12)
																	return('C');
																	else if(high_four==13)
																		return('D');
																		else if(high_four==14)
																			return('E');
																			else if(high_four==15)
																				return('F');
 }









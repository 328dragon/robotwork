#include "tim1.h"
int ADV[128]={0};      //�������飬���������128�����صĻҶ�ֵ
u8 Flag_Target;
/**************************************************************************
�������ܣ���ʱ
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
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
		if(Flag_Target)	RD_TSL();                           		 //===��ȡ����CCD���� 
		Find_CCD_Zhongzhi();			          		 //===��ȡ���� 
	}
}

/**************************************************************************
�������ܣ�CCD���ݲɼ�
��ڲ�������
����  ֵ����
��    �ߣ�ƽ��С��֮��
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
    Dly_us();  //�����ع�ʱ��
		Dly_us();  //�����ع�ʱ��
    ADV[tslp]=(Get_Adc(ADC_CHANNEL_4))>>4;
    ++tslp;
    TSL_CLK=1;
     Dly_us();
  }  
}

/**************************************************************************
�������ܣ�����CCDȡ��ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u16 i,j,Left,Right;
	 static u16 value1_max,value1_min;
	
	   value1_max=ADV[0];  //��̬��ֵ�㷨����ȡ������Сֵ
     for(i=5;i<123;i++)   //���߸�ȥ��5����
     {
        if(value1_max<=ADV[i])
        value1_max=ADV[i];
     }
	   value1_min=ADV[0];  //��Сֵ
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ADV[i])
        value1_min=ADV[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //���������������ȡ����ֵ
		 
		 
	 for(i = 5;i<118; i++)   //Ѱ�����������
	{
		if(ADV[i]>CCD_Yuzhi&&ADV[i+1]>CCD_Yuzhi&&ADV[i+2]>CCD_Yuzhi&&ADV[i+3]<CCD_Yuzhi&&ADV[i+4]<CCD_Yuzhi&&ADV[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//Ѱ���ұ�������
  {
		if(ADV[j]<CCD_Yuzhi&&ADV[j+1]<CCD_Yuzhi&&ADV[j+2]<CCD_Yuzhi&&ADV[j+3]>CCD_Yuzhi&&ADV[j+4]>CCD_Yuzhi&&ADV[j+5]>CCD_Yuzhi)
		{	
		  Right=j;
		  break;	
		}
  }
	CCD_Zhongzhi=(Right+Left)/2;//��������λ��
//	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>90)   //�������ߵ�ƫ����̫��
//	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //��ȡ��һ�ε�ֵ
//	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //������һ�ε�ƫ��
}


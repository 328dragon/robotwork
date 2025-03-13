#include "TSL1401.h"
uint16_t ccd_data[128];     // 储存平均后的CCD数据
uint16_t  CCD_Yuzhi;
uint16_t  CCD_Zhongzhi;
 //延时
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++);      
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
	
	   value1_max=ccd_data[0];  //动态阈值算法，读取最大和最小值
     for(i=5;i<123;i++)   //两边各去掉5个点
     {
        if(value1_max<=ccd_data[i])
        value1_max=ccd_data[i];
     }
	   value1_min=ccd_data[0];  //最小值
     for(i=5;i<123;i++) 
     {
        if(value1_min>=ccd_data[i])
        value1_min=ccd_data[i];
     }
   CCD_Yuzhi=(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
		 
		 
	 for(i = 5;i<118; i++)   //寻找左边跳变沿
	{
		if(ccd_data[i]>CCD_Yuzhi&&ccd_data[i+1]>CCD_Yuzhi&&ccd_data[i+2]>CCD_Yuzhi&&ccd_data[i+3]<CCD_Yuzhi&&ccd_data[i+4]<CCD_Yuzhi&&ccd_data[i+5]<CCD_Yuzhi)
		{	
			Left=i;
			break;	
		}
	}
	 for(j = 118;j>5; j--)//寻找右边跳变沿
  {
		if(ccd_data[j]<CCD_Yuzhi&&ccd_data[j+1]<CCD_Yuzhi&&ccd_data[j+2]<CCD_Yuzhi&&ccd_data[j+3]>CCD_Yuzhi&&ccd_data[j+4]>CCD_Yuzhi&&ccd_data[j+5]>CCD_Yuzhi)
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

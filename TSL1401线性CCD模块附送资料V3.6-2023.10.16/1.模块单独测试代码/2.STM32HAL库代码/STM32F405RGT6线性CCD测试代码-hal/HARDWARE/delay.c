/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-05

All rights reserved
***********************************************/

#include "delay.h"

static u8  fac_us=0;							//us延时倍乘数			   
static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数

void delay_init(void)
{
	SysTick->CTRL &= ~(1<<2) ;   //配置SysTick使用外部时钟源，是AHB总线时钟的1/8  有 168MHz/8 = 21MHz
	fac_us= 21;                   //SysTick计算一个数需要 1/21MHz 秒 ， 计算9个数则需要 9* 1/21MHz = 1us  ，所以延时函数delay_us传入的数值是“需要多少个1us”,delay_ms同理
	fac_ms=(u16)fac_us*1000;     //1ms = 1000us
}

/**************************************************************************
Function: Delay function（us）
Input   : nus：The number of us to delay
Output  : none
函数功能：延时函数（us）
入口参数：nus：要延时的us数	
返回  值：无
**************************************************************************/			    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; 								//时间加载	  		 
	SysTick->VAL=0x00;        								//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数	  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));			//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;      					 				//清空计数器	 
}
/**************************************************************************
Function: Delay function（ms）
Input   : mus：The number of ms to delay
Output  : none
函数功能：延时函数（us）
入口参数：mus：要延时的ms数	
返回  值：无
**************************************************************************/
//注意nms的范围
//SysTick->LOAD为24位寄存器,所以,最大延时为:
//nms<=0xffffff*8*1000/SYSCLK
//SYSCLK单位为Hz,nms单位为ms
//对72M条件下,nms<=1864
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;		   
	SysTick->LOAD=(u32)nms*fac_ms;						//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;												//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ;	//开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));			//等待时间到达   
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器
	SysTick->VAL =0X00;       								//清空计数器	  	    
} 

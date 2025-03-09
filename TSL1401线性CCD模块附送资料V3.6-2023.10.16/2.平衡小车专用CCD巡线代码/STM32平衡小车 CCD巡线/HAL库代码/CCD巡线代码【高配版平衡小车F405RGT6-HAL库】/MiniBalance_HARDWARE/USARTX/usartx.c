#include "usartx.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
u8 Usart2_Receive;
u8 Usart2_Receive_buf[1];//串口2接收中断数据存放的缓冲区
/**************************************************************************
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) //接收回调函数
{	
	if(UartHandle->Instance == USART2) //接收到数据
	{	  
	 static	int uart_receive=0;//蓝牙接收相关变量
	 static u8 Flag_PID,i,j,Receive[50],Flag_CSB;
	 static float Data;
	  Usart2_Receive=Usart2_Receive_buf[0]; 
	
		if(uart_receive==0x59)  Flag_sudu=2;  //低速挡（默认值）
		if(uart_receive==0x58)  Flag_sudu=1;  //高速档 
	  if(uart_receive==0x62)Flag_Hover=!Flag_Hover;  
		if(uart_receive==0x61)Flag_CSB=!Flag_CSB;
		 if(Bi_zhang!=0)
		 {		
			 Bi_zhang=Flag_CSB+1;   //循环选择避障模式和跟随模式
		 }
		
	  if(uart_receive>10)  //默认使用app
    {			
			Remoter_Ch1=0,Remoter_Ch2=0; //APP接收到信号之后 屏蔽航模遥控
			if(uart_receive==0x5A)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
			else if(uart_receive==0x41)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////前
			else if(uart_receive==0x45)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////后
			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
														Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //左
			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    //右
														Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
			else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
  	}
		if(uart_receive<10)     //备用app为：MiniBalanceV1.0  因为MiniBalanceV1.0的遥控指令为A~H 其HEX都小于10
		{		
      Remoter_Ch1=0,Remoter_Ch2=0; //APP接收到信号之后 屏蔽航模遥控			
			if(uart_receive==0x00)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
			else if(uart_receive==0x01)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////前
			else if(uart_receive==0x05)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////后
			else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)	
														Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //左
			else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	    //右
														Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
			else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////刹车
  	}	

		if(Usart2_Receive==0x7B) Flag_PID=1;  //指令起始位
		if(Usart2_Receive==0x7D) Flag_PID=2;  //指令停止位
		 if(Flag_PID==1)                      //记录参数
		 {
			Receive[i]=Usart2_Receive;
			i++;
		 }
		 if(Flag_PID==2)   //执行
		 {
			     Show_Data_Mb=i;
			     if(Receive[3]==0x50) 	       PID_Send=1;     //获取设备参数
					 else  if(Receive[3]==0x57) 	 Flash_Send=1;   //掉电保存参数
					 else  if(Receive[1]!=0x23)                    //更新PID参数
					 {								
						for(j=i;j>=4;j--)
						{
						  Data+=(Receive[j-1]-48)*pow(10,i-j);
						}
						switch(Receive[1])
						 {
							 case 0x30:  Balance_Kp=Data/100;break;
							 case 0x31:  Balance_Kd=Data/100;break;
							 case 0x32:  Velocity_Kp=Data/100;break;
							 case 0x33:  Velocity_Ki=Data/100;break;
							 case 0x34:  break;
							 case 0x35:  break;
							 case 0x36:  break;
							 case 0x37:  break;
							 case 0x38:  break;
						 }
					 }				 
					 Flag_PID=0;   //相关标志位清零
					 i=0;
					 j=0;
					 Data=0;
					 memset(Receive, 0, sizeof(u8)*50);
		 } 
		HAL_UART_Receive_IT(&huart2,Usart2_Receive_buf,sizeof(Usart2_Receive_buf));//串口2回调函数执行完毕之后，需要再次开启接收中断等待下一次接收中断的发生
	}  											 
} 



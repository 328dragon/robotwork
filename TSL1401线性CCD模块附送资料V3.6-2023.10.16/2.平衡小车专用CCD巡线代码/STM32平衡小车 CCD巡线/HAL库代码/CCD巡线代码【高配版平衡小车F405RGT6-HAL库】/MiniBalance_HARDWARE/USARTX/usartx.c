#include "usartx.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
u8 Usart2_Receive;
u8 Usart2_Receive_buf[1];//����2�����ж����ݴ�ŵĻ�����
/**************************************************************************
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) //���ջص�����
{	
	if(UartHandle->Instance == USART2) //���յ�����
	{	  
	 static	int uart_receive=0;//����������ر���
	 static u8 Flag_PID,i,j,Receive[50],Flag_CSB;
	 static float Data;
	  Usart2_Receive=Usart2_Receive_buf[0]; 
	
		if(uart_receive==0x59)  Flag_sudu=2;  //���ٵ���Ĭ��ֵ��
		if(uart_receive==0x58)  Flag_sudu=1;  //���ٵ� 
	  if(uart_receive==0x62)Flag_Hover=!Flag_Hover;  
		if(uart_receive==0x61)Flag_CSB=!Flag_CSB;
		 if(Bi_zhang!=0)
		 {		
			 Bi_zhang=Flag_CSB+1;   //ѭ��ѡ�����ģʽ�͸���ģʽ
		 }
		
	  if(uart_receive>10)  //Ĭ��ʹ��app
    {			
			Remoter_Ch1=0,Remoter_Ch2=0; //APP���յ��ź�֮�� ���κ�ģң��
			if(uart_receive==0x5A)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
			else if(uart_receive==0x41)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ǰ
			else if(uart_receive==0x45)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////��
			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
														Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //��
			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    //��
														Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
			else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
  	}
		if(uart_receive<10)     //����appΪ��MiniBalanceV1.0  ��ΪMiniBalanceV1.0��ң��ָ��ΪA~H ��HEX��С��10
		{		
      Remoter_Ch1=0,Remoter_Ch2=0; //APP���յ��ź�֮�� ���κ�ģң��			
			if(uart_receive==0x00)	Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
			else if(uart_receive==0x01)	Flag_Qian=1,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ǰ
			else if(uart_receive==0x05)	Flag_Qian=0,Flag_Hou=1,Flag_Left=0,Flag_Right=0;//////////////��
			else if(uart_receive==0x02||uart_receive==0x03||uart_receive==0x04)	
														Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=1;  //��
			else if(uart_receive==0x06||uart_receive==0x07||uart_receive==0x08)	    //��
														Flag_Qian=0,Flag_Hou=0,Flag_Left=1,Flag_Right=0;
			else Flag_Qian=0,Flag_Hou=0,Flag_Left=0,Flag_Right=0;//////////////ɲ��
  	}	

		if(Usart2_Receive==0x7B) Flag_PID=1;  //ָ����ʼλ
		if(Usart2_Receive==0x7D) Flag_PID=2;  //ָ��ֹͣλ
		 if(Flag_PID==1)                      //��¼����
		 {
			Receive[i]=Usart2_Receive;
			i++;
		 }
		 if(Flag_PID==2)   //ִ��
		 {
			     Show_Data_Mb=i;
			     if(Receive[3]==0x50) 	       PID_Send=1;     //��ȡ�豸����
					 else  if(Receive[3]==0x57) 	 Flash_Send=1;   //���籣�����
					 else  if(Receive[1]!=0x23)                    //����PID����
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
					 Flag_PID=0;   //��ر�־λ����
					 i=0;
					 j=0;
					 Data=0;
					 memset(Receive, 0, sizeof(u8)*50);
		 } 
		HAL_UART_Receive_IT(&huart2,Usart2_Receive_buf,sizeof(Usart2_Receive_buf));//����2�ص�����ִ�����֮����Ҫ�ٴο��������жϵȴ���һ�ν����жϵķ���
	}  											 
} 



#include "show.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
float Vol;
/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{
			OLED_Show_CCD();
//		//=============第一行显示小车模式=======================//	
//		     if(Way_Angle==1)	OLED_ShowString(0,0,"DMP");
//		else if(Way_Angle==2)	OLED_ShowString(0,0,"Kalman");
//		else if(Way_Angle==3)	OLED_ShowString(0,0,"Hubu");
//                   
//	       if(Bi_zhang==1)	OLED_ShowString(60,0,"Bizhang");
//		     else             OLED_ShowString(60,0,"Putong");
	  //=============第二行显示温度和距离===============//	
		OLED_ShowNumber(0,10,Temperature/10,2,12);
		OLED_ShowNumber(23,10,Temperature%10,1,12);
		OLED_ShowString(13,10,".");
		OLED_ShowString(35,10,"`C");
		OLED_ShowNumber(70,10,(u16)Distance,5,12);
			                    OLED_ShowString(105,10,"mm");
		//=============第三行显示编码器1=======================//	
		                      OLED_ShowString(00,20,"Z");
		if( CCD_Zhongzhi<0)		OLED_ShowString(10,20,"-"),
		                      OLED_ShowNumber(25,20,-CCD_Zhongzhi,3,12);
		else                 	OLED_ShowString(10,20,"+"),
		                      OLED_ShowNumber(25,20, CCD_Zhongzhi,3,12);
				                  OLED_ShowString(70,20,"Y");
		if( CCD_Yuzhi<0)		OLED_ShowString(80,20,"-"),
		                      OLED_ShowNumber(95,20,-CCD_Yuzhi,3,12);
		else                 	OLED_ShowString(80,20,"+"),
		                      OLED_ShowNumber(95,20, CCD_Yuzhi,3,12);
  	//=============第四行显示编码器2=======================//	
		                      OLED_ShowString(00,30,"L");		
		if(Encoder_Left<0)		OLED_ShowString(10,30,"-"),
		                      OLED_ShowNumber(25,30,-Encoder_Left,3,12);
		else               		OLED_ShowString(10,30,"+"),
		                      OLED_ShowNumber(25,30,Encoder_Left,3,12);	
				                  OLED_ShowString(70,30,"R");		
		if(Encoder_Right<0)		OLED_ShowString(80,30,"-"),
		                      OLED_ShowNumber(95,30,-Encoder_Right,3,12);
		else               		OLED_ShowString(80,30,"+"),
		                      OLED_ShowNumber(95,30,Encoder_Right,3,12);	
		//=============第五行显示电压=======================//
		                      OLED_ShowString(00,40,"Volta");
		                      OLED_ShowString(58,40,".");
		                      OLED_ShowString(80,40,"V");
		                      OLED_ShowNumber(45,40,Voltage/100,2,12);
		                      OLED_ShowNumber(68,40,Voltage%100,2,12);
		 if(Voltage%100<10) 	OLED_ShowNumber(62,40,0,2,12);
		//=============第六行显示角度=======================//
		                      OLED_ShowString(0,50,"Angle");
		if(Angle_Balance<0)		OLED_ShowNumber(45,50,Angle_Balance+360,3,12);
		else					        OLED_ShowNumber(45,50,Angle_Balance,3,12);
		//=============刷新=======================//
		OLED_Refresh_Gram();	
	}
/**************************************************************************
函数功能：向APP发送数据
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void APP_Show(void)
{    
	  static u8 flag;
	  int app_2,app_3,app_4;
		app_4=(Voltage-1110)*2/3;		if(app_4<0)app_4=0;if(app_4>100)app_4=100;   //对电压数据进行处理
		app_3=Encoder_Right*1.1; if(app_3<0)app_3=-app_3;			                   //对编码器数据就行数据处理便于图形化
		app_2=Encoder_Left*1.1;  if(app_2<0)app_2=-app_2;
	  flag=!flag;
	if(PID_Send==1)//发送PID参数
	{
		printf("{C%d:%d:%d:%d:%d:%d:%d:%d:%d}$",(int)(Balance_Kp*100),(int)(Balance_Kd*100),(int)(Velocity_Kp*100),(int)(Velocity_Ki*100),0,0,0,0,0);//打印到APP上面	
		PID_Send=0;	
	}	
   else	if(flag==0)// 
   printf("{A%d:%d:%d:%d}$",(u8)app_2,(u8)app_3,app_4,(int)Angle_Balance); //打印到APP上面
	 else
	 printf("{B%d:%d:%d:%d}$",(int)Angle_Balance,Distance,Encoder_Left,Encoder_Right);//打印到APP上面 显示波形
}
/**************************************************************************
函数功能：虚拟示波器往上位机发送数据 关闭显示屏
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void DataScope(void)
{   
    Vol=(float)Voltage/100;
		DataScope_Get_Channel_Data( Angle_Balance, 1 );       //显示角度 单位：度（°）
		DataScope_Get_Channel_Data( Distance/10, 2 );         //显示超声波测量的距离 单位：CM 
		DataScope_Get_Channel_Data( Vol, 3 );                 //显示电池电压 单位：V
//		DataScope_Get_Channel_Data( 0 , 4 );   
//		DataScope_Get_Channel_Data(0, 5 ); //用您要显示的数据替换0就行了
//		DataScope_Get_Channel_Data(0 , 6 );//用您要显示的数据替换0就行了
//		DataScope_Get_Channel_Data(0, 7 );
//		DataScope_Get_Channel_Data( 0, 8 ); 
//		DataScope_Get_Channel_Data(0, 9 );  
//		DataScope_Get_Channel_Data( 0 , 10);
		Send_Count = DataScope_Data_Generate(3);
		for( i = 0 ; i < Send_Count; i++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
		}
}
void OLED_DrawPoint_Shu(u8 x,u8 y,u8 t)
{ 
	 u8 i=0;
  OLED_DrawPoint(x,y,t);
	OLED_DrawPoint(x,y,t);
	  for(i = 0;i<8; i++)
  {
      OLED_DrawPoint(x,y+i,t);
  }
}

void OLED_Show_CCD(void)
{ 
	 u8 i,t;
	 for(i = 0;i<128; i++)
  {
		if(ADV[i]<CCD_Yuzhi) t=1; else t=0;
		OLED_DrawPoint_Shu(i,0,t);
  }
}

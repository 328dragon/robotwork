#include "control.h"	
#include "filter.h"	
#include "MPU6050.h"
#include "inv_mpu.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
int Balance_Pwm,Velocity_Pwm,Turn_Pwm=0;
u8 Flag_Target;
u32 Flash_R_Count;
int Voltage_Temp,Voltage_Count,Voltage_All;
/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步				 
**************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim3)
	{ 
		   Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==1)	 delay_50=0,delay_flag=0;                     //给主函数提供50ms的精准延时
			 } 
		  if(Flag_Target==1)                                                  
			{
			 RD_TSL();                                                       //===读取线性CCD数据
		     Voltage=Get_battery_volt();                                     //===获取电池电压/	                                          
			}    		
		 if(Flag_Target==0) 
		 {
			Encoder_Left=Read_Encoder(2);                                        //===读取编码器的值，因为两个电机的旋转了180度的，所以对其中一个取反，保证输出极性一致
			Encoder_Right=Read_Encoder(4);                                        //===读取编码器的值
	  	Get_Angle(Way_Angle);                                                 //===更新姿态	
			 
  		Led_Flash(100);                                                       //===LED闪烁;常规模式 1s改变一次指示灯的状态	
        Find_CCD_Zhongzhi();			                                            //===提取中线
			 
		Key();                                                                //===扫描按键状态 单击双击可以改变小车运行状态
 		Balance_Pwm =balance(Angle_Balance,Gyro_Balance);                     //===平衡PID控制	
  	    Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                    //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
 	    Turn_Pwm    =turn(CCD_Zhongzhi,Gyro_Turn);  
			 //===转向环PID控制     
 		Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                              //===计算左轮电机最终PWM
 	  	Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                            //===计算右轮电机最终PWM
   		Xianfu_Pwm();                                                       //===PWM限幅
      if(Turn_Off(Angle_Balance,Voltage)==0)                              //===如果不存在异常
 			Set_Pwm(Moto1,Moto2);                                               //===赋值给PWM寄存器    
	}       	
} 
}
/**************************************************************************
函数功能：直立PD控制
入口参数：角度、角速度
返回  值：直立控制PWM
作    者：平衡小车之家
**************************************************************************/
int balance(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-Zhongzhi;                       //===求出平衡的角度中值 和机械相关
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}

/**************************************************************************
函数功能：速度PI控制 修改前进后退速度，请修Target_Velocity，比如，改成60就比较慢了
入口参数：左轮编码器、右轮编码器
返回  值：速度控制PWM
作    者：平衡小车之家
**************************************************************************/
int velocity(int encoder_left,int encoder_right)
{  
      static float Velocity,Encoder_Least,Encoder,Movement;
	  static float Encoder_Integral;
	  float kp=80,ki=0.4;
    	Movement=-120;   //速度设定
   //=============速度PI控制器=======================//	
		Encoder_Least =(encoder_left+encoder_right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.8f;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2f;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  	Encoder_Integral=10000;             //===积分限幅
		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===积分限幅	
		Velocity=Encoder*kp+Encoder_Integral*ki;                          //===速度控制	
		if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1)   Encoder_Integral=0;      //===电机关闭后清除积分
	  return Velocity;
}

/**************************************************************************
函数功能：转向控制  巡线
入口参数：CCD提取的中线 Z轴陀螺仪
返回  值：转向控制PWM
作    者：平衡小车之家
**************************************************************************/
int turn(u8 CCD,float gyro)//转向控制
{
	  float Turn;     
    float Bias,kp=30,Kd=0.22;	  
	  Bias=CCD-64;
	  Turn=-Bias*kp-gyro*Kd;
	  return Turn;
}
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：左轮PWM、右轮PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int moto1,int moto2)
{
    	if(moto1>0)			AIN2=0,			AIN1=1;
			else 	          AIN2=1,			AIN1=0;
			PWMA=myabs(moto1)*1.17;
		  if(moto2>0)	BIN1=0,			BIN2=1;
			else        BIN1=1,			BIN2=0;
			PWMB=myabs(moto2)*1.17;	
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：无
返回  值：无
**************************************************************************/
void Xianfu_Pwm(void)
{	
	  int Amplitude=8200;    //===PWM满幅是8400 限制在8200
//		if(Flag_Qian==1)  Moto1+=DIFFERENCE;  //DIFFERENCE是一个衡量平衡小车电机和机械安装差异的一个变量。直接作用于输出，让小车具有更好的一致性。
//	  if(Flag_Hou==1)   Moto2-=DIFFERENCE;
    if(Moto1<-Amplitude) Moto1=-Amplitude;	
		if(Moto1>Amplitude)  Moto1=Amplitude;	
	  if(Moto2<-Amplitude) Moto2=-Amplitude;	
		if(Moto2>Amplitude)  Moto2=Amplitude;		
	
}
/**************************************************************************
函数功能：按键修改小车运行状态 
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp,tmp2;
	tmp=click_N_Double(50); 
	if(tmp==1)Flag_Stop=!Flag_Stop;//单击控制小车的启停
	if(tmp==2)Flag_Show=!Flag_Show;//双击控制小车的显示状态
	tmp2=Long_Press();                   
  if(tmp2==1) Bi_zhang=!Bi_zhang;		//长按控制小车是否进入超声波模式 
}

/**************************************************************************
函数功能：异常关闭电机
入口参数：倾角和电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	    u8 temp;
			if(angle<(-40+Zhongzhi)||angle>(40+Zhongzhi)||1==Flag_Stop||voltage<1110)//电池电压低于11.1V关闭电机
			{	                                                 //===倾角大于40度关闭电机
      temp=1;                                            //===Flag_Stop置1关闭电机
			AIN1=0;                                            
			AIN2=0;
			BIN1=0;
			BIN2=0;
      }
			else
      temp=0;
      return temp;			
}
	
/**************************************************************************
函数功能：获取角度 三种算法经过我们的调校，都非常理想 
入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
void Get_Angle(u8 way)
{ 
	    float Accel_Y,Accel_Angle,Accel_Z,Gyro_X,Gyro_Z;
	   	Temperature=Read_Temperature();      //===读取MPU6050内置温度传感器数据，近似表示主板温度。
	    if(way==1)                           //===DMP的读取在数据采集中断读取，严格遵循时序要求
			{	
					Read_DMP();                      //===读取加速度、角速度、倾角
					Angle_Balance=-Roll;             //===更新平衡倾角
					Gyro_Balance=-gyro[0];            //===更新平衡角速度
					Gyro_Turn=gyro[2];               //===更新转向角速度
				  Acceleration_Z=accel[2];         //===更新Z轴加速度计
			}			
      else
      {
			Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取Y轴陀螺仪
			Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		  Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
	  	Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		  if(Gyro_X>32768)  Gyro_X-=65536;                       //数据类型转换  也可通过short强制类型转换
			if(Gyro_Z>32768)  Gyro_Z-=65536;                       //数据类型转换
	  	if(Accel_Y>32768) Accel_Y-=65536;                      //数据类型转换
		  if(Accel_Z>32768) Accel_Z-=65536;                      //数据类型转换
			Gyro_Balance=Gyro_X;                                  //更新平衡角速度
	   	Accel_Angle=atan2(Accel_Y,Accel_Z)*180/PI;                 //计算倾角	
			Gyro_X=Gyro_X/16.4f;                                    //陀螺仪量程转换	
      if(Way_Angle==2)		  	Kalman_Filter(Accel_Angle,Gyro_X);//卡尔曼滤波	
			else if(Way_Angle==3)   Yijielvbo(Accel_Angle,Gyro_X);    //互补滤波
	    Angle_Balance=angle;                                   //更新平衡倾角
			Gyro_Turn=Gyro_Z;                                      //更新转向角速度
			Acceleration_Z=Accel_Z;                                //===更新Z轴加速度计	
		}
}
/**************************************************************************
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：检测小车是否被拿起
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	if(flag==0)                                                                   //第一步
	 {
	      if(myabs(encoder_left)+myabs(encoder_right)<30)                         //条件1，小车接近静止
				count0++;
        else 
        count0=0;		
        if(count0>10)				
		    flag=1,count0=0; 
	 } 
	 if(flag==1)                                                                  //进入第二步
	 {
		    if(++count1>200)       count1=0,flag=0;                                 //超时不再等待2000ms
	      if(Acceleration>26000&&(Angle>(-20+Zhongzhi))&&(Angle<(20+Zhongzhi)))   //条件2，小车是在0度附近被拿起
		    flag=2; 
	 } 
	 if(flag==2)                                                                  //第三步
	 {
		  if(++count2>100)       count2=0,flag=0;                                   //超时不再等待1000ms
	    if(myabs(encoder_left+encoder_right)>160)                                 //条件3，小车的轮胎因为正反馈达到最大的转速   
      {
				flag=0;                                                                                     
				return 1;                                                               //检测到小车被拿起
			}
	 }
	return 0;
}
/**************************************************************************
函数功能：检测小车是否被放下
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                           //防止误检      
   return 0;	                 
	 if(flag==0)                                               
	 {
	      if(Angle>(-10+Zhongzhi)&&Angle<(10+Zhongzhi)&&encoder_left==0&&encoder_right==0)         //条件1，小车是在0度附近的
		    flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                                          //超时不再等待 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left<-3&&encoder_right<-3&&encoder_left>-60&&encoder_right>-60)                //条件2，小车的轮胎在未上电的时候被人为转动  
      {
				flag=0;
				flag=0;
				return 1;                                             //检测到小车被放下
			}
	 }
	return 0;
}


/**************************************************************************
函数功能：采集遥控器的信号
入口参数：无
返回  值：无
**************************************************************************/
void  Get_MC6(void)
{ 
	if(Flag_Left==0&&Flag_Right==0)
	{	
    	 if((Remoter_Ch1>1650&&Remoter_Ch1<2100)||(Remoter_Ch1>21650&&Remoter_Ch1<22100))	Flag_Qian=1,Flag_Hou=0,Flag_sudu=1;//////////////前
	else if((Remoter_Ch1<1350&&Remoter_Ch1>900) ||(Remoter_Ch1<21350&&Remoter_Ch1>20900))	Flag_Qian=0,Flag_Hou=1,Flag_sudu=1;//////////////后
  else if ((Remoter_Ch1>1350&&Remoter_Ch1<1650) ||(Remoter_Ch1>21350&&Remoter_Ch1<21650))	Flag_Qian=0,Flag_Hou=0;//////////////停
	}
	if(Flag_Qian==0&&Flag_Hou==0)
	{	
	    	 if((Remoter_Ch2>1650&&Remoter_Ch2<2100)||(Remoter_Ch2>21650&&Remoter_Ch2<22100))Flag_Left=1,Flag_Right=0,Flag_sudu=1;//////////////左
  	else if((Remoter_Ch2<1350&&Remoter_Ch2>900) ||(Remoter_Ch2<21350&&Remoter_Ch2>20900))Flag_Left=0,Flag_Right=1,Flag_sudu=1;//////////////右
	  else if ((Remoter_Ch2>1350&&Remoter_Ch2<1650) ||(Remoter_Ch2>21350&&Remoter_Ch2<21650))Flag_Left=0,Flag_Right=0;//////////////停
	}
}	

/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
void  Find_CCD_Zhongzhi(void)
{ 
	 static u8 i,j,Left,Right,Last_CCD_Zhongzhi;
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
	if(myabs(CCD_Zhongzhi-Last_CCD_Zhongzhi)>70)   //计算中线的偏差，如果太大
	CCD_Zhongzhi=Last_CCD_Zhongzhi;    //则取上一次的值
	Last_CCD_Zhongzhi=CCD_Zhongzhi;  //保存上一次的偏差
}	

/**************************************************************************
函数功能：延时
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<30;ii++);      
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


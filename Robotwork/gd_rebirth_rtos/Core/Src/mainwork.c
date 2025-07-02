
// ​​Planner模块​​：负责轨迹规划，提供开环和闭环两种控制模式
// ​​Controller模块​​：负责控制算法执行和电机控制
// ​​Kinematic模块​​：负责运动学正逆解计算和里程计更新
// ​​FreeRTOS任务​​：提供实时调度框架

#include "mainwork.h"
#include "FreeRTOS.h"
#include "task.h"
#include "tcs230.h"
#include "gray.h"
#include "usart.h"


// #include "lcd.h"
// #include "lcd_init.h"
// #include "pic.h"
// 传感器信息
int goods_color = 0;
unsigned char Digtal_gray;
unsigned char Anolog_gray[8] = {0};
unsigned char Normal[8] = {0};
 #define max(a, b) ((a) >= (b) ? (a) : (b))
#define min(a, b) ((a) <= (b) ? (a) : (b))
// 状态机
int read_cololr_flag = 0; // 颜色传感器读取标志位
TaskHandle_t Chassic_control_handle; // 底盘控制
TaskHandle_t main_cpp_handle;        // 主函数
TaskHandle_t Planner_update_handle;  // 轨迹规划
TaskHandle_t gray_read_handle;       // 灰度传感器
TaskHandle_t tcs230_read_handle;     // tcs230颜色传感器读取
TaskHandle_t IMU_read_handle;        // IMU读取
TaskHandle_t LCD_Show_handle;        // 显示
TaskHandle_t wheel_state_read_handle; // tcd1103读取
void OnChassicControl(void *pvParameters);
void OnPlannerUpdate(void *pvParameters);
void Onmaincpp(void *pvParameters);
void IMU_Read_task(void *pvParameters);
void LCD_Show_task(void *pvParameters);
void tcs230_read_task(void *pvParameters);
void gray_read_task(void *pvParameters);
void wheel_state_read_task(void *pvParameters);

void main_work(void)
{ // 颜色传感器添加完成
    HAL_UART_Receive_IT(&huart1, &RxData, 1);

    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");

    BaseType_t ok2 = xTaskCreate(OnChassicControl, "Chassic_control", 800, NULL, 3, &Chassic_control_handle);
    BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 600, NULL, 4, &main_cpp_handle);
    BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 400, NULL, 4, &Planner_update_handle);
    BaseType_t ok5 = xTaskCreate(IMU_Read_task, "IMU_Read_task", 200, NULL, 4, &IMU_read_handle);
    BaseType_t ok6 = xTaskCreate(LCD_Show_task, "LCD_Show_task", 200, NULL, 1, &LCD_Show_handle);
    BaseType_t ok7 = xTaskCreate(tcs230_read_task, "tcs230_read_task", 200, NULL, 2, &tcs230_read_handle);
    BaseType_t ok8 = xTaskCreate(gray_read_task, "gray_read_task", 200, NULL, 2, &gray_read_handle);
	BaseType_t ok9 = xTaskCreate(wheel_state_read_task, "wheel_state_read_task", 200, NULL, 2, &wheel_state_read_handle);
    if (ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS || ok5 != pdPASS||ok6 !=pdPASS||ok7 != pdPASS||ok8 != pdPASS)
    {
        // 任务创建失败，进入死循环
        while (1)
        {
            // uart_printf("create task failed\n");
        }
    }

}

void wheel_state_read_task(void *pvParameters)
{

    while(1)
    {
        

        vTaskDelay(300);
    }


}

void tcs230_read_task(void *pvParameters)
{

    while (1)
    {
        if (read_cololr_flag == 1)
        {
            goods_color = Color_Recognize();
            read_cololr_flag = 0; // 清除标志位
                                  // uart_printf("tcs230 read color\n");
        }
        // 读取颜色传感器数据
        vTaskDelay(200); // 延时200ms
    }
}
void gray_read_task(void *pvParameters)
{
    while (Ping())
    {
        vTaskDelay(5);
    }
    while (1)
    {
        // 读取灰度传感器数据
        Digtal_gray = IIC_Get_Digtal();

        // 获取传感器模拟量结果
        if (IIC_Get_Anolog(Anolog_gray, 8))
        {
        }

        // 获取传感器归一化结果
        IIC_Anolog_Normalize(0xff); // 所有通道归一化都打开
        vTaskDelay(10);             // 设置完，需要等上一会。stm8的运算速度没stm32快，等一下，让传感器把数据刷新一下。
        if (IIC_Get_Anolog(Normal, 8))
        {
        }
        IIC_Anolog_Normalize(0x00); // 为了下一次循环是非归一化，所以清零

        vTaskDelay(100); // 延时100ms
    }
}

void LCD_Show_task(void *pvParameters)
{
    // 屏幕
    //    LCD_Init();
    //    LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
    while (1)
    {
        //        // 显示
        //        // 陀螺仪
        //        LCD_ShowFloatNum1(0, 20, gyro[0], 4, RED, WHITE, 16);
        //        LCD_ShowString(48, 20, ",", RED, WHITE, 16, 0);
        //        LCD_ShowFloatNum1(58, 20, gyro[1], 4, RED, WHITE, 16);
        //        LCD_ShowString(106, 40, ",", RED, WHITE, 16, 0);
        //        LCD_ShowFloatNum1(116, 20, gyro[2], 4, RED, WHITE, 16);
        //        // 加速度
        //        LCD_ShowFloatNum1(0, 40, accel[0], 4, RED, WHITE, 16);
        //        LCD_ShowString(48, 40, ",", RED, WHITE, 16, 0);
        //        LCD_ShowFloatNum1(58, 40, accel[1], 4, RED, WHITE, 16);
        //        LCD_ShowString(106, 40, ",", RED, WHITE, 16, 0);
        //        LCD_ShowFloatNum1(116, 40, accel[2], 4, RED, WHITE, 16);
        //        // 显示temp
        //        LCD_ShowFloatNum1(10, 60, temp, 4, RED, WHITE, 16);
        //        LCD_ShowString(52, 60, ",", RED, WHITE, 16, 0);
        //        LCD_ShowString(62, 60, "gyro", RED, WHITE, 16, 0);
        //        LCD_ShowString(100, 60, ",", RED, WHITE, 16, 0);
        //        LCD_ShowString(106, 60, "accel", RED, WHITE, 16, 0);
        vTaskDelay(100);
    }
}

void IMU_Read_task(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(1000);

    }
}

void Onmaincpp(void *pvParameters)
{


    while (1)
    {

        vTaskDelay(200);
    }
}

void OnPlannerUpdate(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(50);
    }
}

void OnChassicControl(void *pvParameters)
{

    while (1)
    {

        vTaskDelay(10);
    }
}

// 操作步骤：
// a.根据延时时间和定时器所选时钟频率，计算出定时器要计数的时间数值；
// b.获取当前数值寄存器的数值；
// c.以当前数值为基准开始计数；
// d.当所计数值等于（大于）需要延时的时间数值时退出。
// 注：计数时间值的计算，我们以延时10us，时钟频率为72MHZ的STM32F103C8T6来计算，
//         计数值 = 延时时间/1S × 时钟频率 = 0.000 01/1 *72 000 000 =  720
void delay_us(uint32_t nus)
{ 
       uint32_t ticks;
       uint32_t told,tnow,reload,tcnt=0;
       if((0x0001&(SysTick->CTRL)) ==0)    //定时器未工作
              vPortSetupTimerInterrupt();  //初始化定时器
 
       reload = SysTick->LOAD;                     //获取重装载寄存器值
       ticks = nus * (SystemCoreClock / 1000000);  //计数时间值
       
       vTaskSuspendAll();//阻止OS调度，防止打断us延时
       told=SysTick->VAL;  //获取当前数值寄存器值（开始时数值）
       while(1)
       {
              tnow=SysTick->VAL; //获取当前数值寄存器值
              if(tnow!=told)  //当前值不等于开始值说明已在计数
              {         
                     if(tnow<told)  //当前值小于开始数值，说明未计到0
                          tcnt+=told-tnow; //计数值=开始值-当前值
 
                     else     //当前值大于开始数值，说明已计到0并重新计数
                            tcnt+=reload-tnow+told;   //计数值=重装载值-当前值+开始值  （
                                                      //已从开始值计到0） 
 
                     told=tnow;   //更新开始值
                     if(tcnt>=ticks)break;  //时间超过/等于要延迟的时间,则退出.
              } 
       }  
       xTaskResumeAll();	//恢复OS调度		   
} 
//SystemCoreClock为系统时钟(system_stmf4xx.c中)，通常选择该时钟作为
//systick定时器时钟，根据具体情况更改
        

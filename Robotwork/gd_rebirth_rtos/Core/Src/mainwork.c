// ​​Planner模块​​：负责轨迹规划，提供开环和闭环两种控制模式
// ​​Controller模块​​：负责控制算法执行和电机控制
// ​​Kinematic模块​​：负责运动学正逆解计算和里程计更新
// ​​FreeRTOS任务​​：提供实时调度框架

#include "mainwork.h"
#include "FreeRTOS.h"
#include "task.h"

#include "usart.h"
//#include "lcd.h"
//#include "lcd_init.h"
//#include "pic.h"
float DEBUG = 0.0f;
float DEBUG2 = 0.0f;
float DEBUG3 = 0.0f;
 uint8_t orangepi_data[8];






TaskHandle_t LCD_Show_handle;        // 显示
TaskHandle_t Chassic_control_handle; // 底盘控制
TaskHandle_t main_cpp_handle;        // 主函数
TaskHandle_t Planner_update_handle;  // 轨迹规划
TaskHandle_t IMU_read_handle;        // IMU读取
void OnChassicControl(void *pvParameters);
void OnPlannerUpdate(void *pvParameters);
void Onmaincpp(void *pvParameters);
void IMU_Read_task(void *pvParameters);     void LCD_Show_task(void *pvParameters);

void main_work(void)
{



    BaseType_t ok2 = xTaskCreate(OnChassicControl, "Chassic_control", 1000, NULL, 3, &Chassic_control_handle);
    BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 600, NULL, 4, &main_cpp_handle);
    BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 600, NULL, 4, &Planner_update_handle);
    BaseType_t ok5 = xTaskCreate(IMU_Read_task, "IMU_Read_task", 400, NULL, 4, &IMU_read_handle);
    BaseType_t ok6 = xTaskCreate(LCD_Show_task, "LCD_Show_task", 600, NULL, 1, &LCD_Show_handle);
    if (ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS || ok5 != pdPASS)
    {
        // 任务创建失败，进入死循环
        while (1)
        {
            // uart_printf("create task failed\n");
        }
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

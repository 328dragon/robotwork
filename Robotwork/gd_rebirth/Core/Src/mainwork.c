
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
#include "tim.h"
#include "dc_motor.h"
// 主函数逻辑
int main_state = 0;
// 电机模式
extern int motor_0_speed;
extern int motor_1_speed;
extern int motor_0_active;
extern int motor_1_active;
enum MotorMode
{
    MOTOR_MODE_NORMAL = 0, // 开环模式
    MOTOR_MODE_STEP = 1    // 步进模式
};
int dc_step_distance = 0;           // 步进距离
int motor_mode = MOTOR_MODE_NORMAL; // 0:开环,1:闭环
int step_complete_flag = 0;         // 步进模式直流电机完成标志位
// 舵机控制量
int debug_hook_pwm = 0;
int debug_up_pwm = 0;
int debug_down_pwm = 0;
// 传感器信息
extern int goods_color;
unsigned char Digtal_gray;
unsigned char Anolog_gray[8] = {0};
unsigned char Normal[8] = {0};
#define max(a, b) ((a) >= (b) ? (a) : (b))
#define min(a, b) ((a) <= (b) ? (a) : (b))
// 状态机

int read_cololr_flag = 0; // 颜色传感器读取标志位
int read_color_state = 0; // 颜色传感器读取状态
int temp_color = -1;
// freertos句柄
TaskHandle_t main_cpp_handle;         // 主函数
TaskHandle_t gray_read_handle;        // 灰度传感器
TaskHandle_t tcs230_read_handle;      // tcs230颜色传感器读取
TaskHandle_t IMU_read_handle;         // IMU读取
TaskHandle_t wheel_state_read_handle; // tcd1103读取

void Onmaincpp(void *pvParameters);
void IMU_Read_task(void *pvParameters);
void tcs230_read_task(void *pvParameters);
void gray_read_task(void *pvParameters);
void wheel_state_read_task(void *pvParameters);
void vtask_delay_second(int seconds)
{
    // FreeRTOS延时函数
    vTaskDelay(seconds * 1000);
}

void main_work(void)
{ // 颜色传感器添加完成
    HAL_UART_Receive_IT(&huart1, &RxData, 1);

    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");

    BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 100, NULL, 4, &main_cpp_handle);
    BaseType_t ok5 = xTaskCreate(IMU_Read_task, "IMU_Read_task", 100, NULL, 4, &IMU_read_handle);
    BaseType_t ok7 = xTaskCreate(tcs230_read_task, "tcs230_read_task", 100, NULL, 2, &tcs230_read_handle);
    BaseType_t ok8 = xTaskCreate(gray_read_task, "gray_read_task", 100, NULL, 2, &gray_read_handle);
    BaseType_t ok9 = xTaskCreate(wheel_state_read_task, "wheel_state_read_task", 100, NULL, 2, &wheel_state_read_handle);
    if (ok3 != pdPASS | ok5 != pdPASS || ok7 != pdPASS || ok8 != pdPASS)
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

    while (1)
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

            switch (read_color_state)
            {
            case 0:
            {
                R = 0;
                G = 0;
                B = 0;
                temp_color = -1;
                printf("AT+COLOR\r\n");
                vTaskDelay(500);
                printf("AT+COLOR\r\n");
                read_color_state++;
                break;
            }
            case 1:
            {
                // 读取颜色传感器数据
                if (Get_RxFlag() && R != 0 && G != 0 && B != 0)
                {
                    temp_color = Get_Color();
                    if (temp_color != -1)
                    {
                        goods_color = temp_color;
                        read_color_state++;
                    }

                    else
                    {
                        // uart_printf("tcs230 read color failed\n");
                    }
                }
                printf("AT+COLOR\r\n");
                break;
            }
            case 2:
            {
                temp_color = -1;
                read_color_state = 0; // 重置状态机
                read_cololr_flag = 0;
                break;
            }
            default:
                break;
            }
        }
        // 读取颜色传感器数据
        vTaskDelay(100); // 延时200ms
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

void IMU_Read_task(void *pvParameters)
{
    while (1)
    {
        vTaskDelay(1000);
    }
}

// 电机步进模式配置函数，减少重复代码
static void setMotorStepMode(int8_t speed0, int8_t speed1, int32_t distance) {
    motor_0_active = 1;
    motor_1_active = 1;
    motor_0_speed = speed0;
    motor_1_speed = speed1;
    dc_step_distance = distance;
    motor_mode = MOTOR_MODE_STEP;
}

void Onmaincpp(void *pvParameters)
{

    while (1)
    {
        switch (main_state)
        {
        case 0:
        {

            vTaskDelay(1000);
            main_state++;
            break;
        }
        case 1:
        {
         setMotorStepMode(6,6, 4000); // 设置电机步进模式，速度6，距离4000
            main_state++;
            break;
        }
        case 2:
        {
            if (step_complete_flag == 1)
            {
                motor_mode = MOTOR_MODE_NORMAL; // 设置为正常模式
                main_state++;
                vtask_delay_second(4);
            }
            break;
        }
        case 3:
        {
           setMotorStepMode(-6, -6, -4000); // 设置电机步进模式，速度-6，距离-4000
            main_state++;
            break;
        }
        case 4:
        {
            if (step_complete_flag == 1)
            {
                motor_mode = MOTOR_MODE_NORMAL; // 设置为正常模式
                main_state++;
                vtask_delay_second(2);
            }
						break;
        }
        case 5:
        {
            // 读取颜色传感器
            read_cololr_flag = 1;
          setMotorStepMode(6, 6, 4000); // 设置电机步进模式，速度6，距离4000
            main_state++;
            break;
        }
        default:
            break;
        }
        vTaskDelay(100);
    }
}

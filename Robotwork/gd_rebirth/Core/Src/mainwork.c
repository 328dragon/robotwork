
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
// 位置
task_POSITION now_position = ORIGIN;    // 当前位置
task_POSITION last_position = ORIGIN;   // 目标位置
task_POSITION target_position = ORIGIN; // 目标位置
// 电机模式
extern int dc_motor_state;
int motor_0_user_speed = 0;
int motor_1_user_speed = 0;
extern int motor_0_speed;
extern int motor_1_speed;
extern int motor_0_active;
extern int motor_1_active;
// 步进模式直流电机
int dc_step_flag = 0;
int dc_step_distance = 0;                 // 步进距离
MotorMode motor_mode = MOTOR_MODE_NORMAL; // 0:开环,1:闭环
int step_complete_flag = 0;               // 步进模式直流电机完成标志位
int stop_motor_flag = 0;                  // 停止电机标志位
// 舵机控制量
int debug_hook_pwm = 0;
int debug_up_pwm = 0;
int debug_down_pwm = 0;
// 传感器信息

extern int goods_color;
// 总灰度逻辑
gray_state real_time_gray_state = orgin_gray; // 主灰度状态
int all_black_count = 0;                      // 全黑计数
int primary_gray_count = 0;                   // 初级灰度计数
int senior_gray_count = 0;                    // 高级灰度计数
// 前面灰度
uint8_t digital_gray_data[8];
int sensor_weights[8] = {-5, -3, -2, -1, 1, 2, 3, 5}; // 传感器权重
unsigned char Digtal_gray;
unsigned char Anolog_gray[8] = {0};
unsigned char Normal[8] = {0};
// 后面灰度
uint8_t digital_gray_data_back[8];
int sensor_weights_back[8] = {-5, -3, -2, -1, 1, 2, 3, 5}; // 传感器权重
unsigned char Digtal_gray_back;
unsigned char Anolog_gray_back[8] = {0};
unsigned char Normal_back[8] = {0};
#define max(a, b) ((a) >= (b) ? (a) : (b))
#define min(a, b) ((a) <= (b) ? (a) : (b))
// 状态机
int read_cololr_flag = 0; // 颜色传感器读取标志位
int read_color_state = 0; // 颜色传感器读取状态
int temp_color = -1;
// freertos句柄
TaskHandle_t main_cpp_handle;              // 主函数
TaskHandle_t gray_read_handle;             // 灰度传感器
TaskHandle_t tcs230_read_handle;           // tcs230颜色传感器读取
TaskHandle_t IMU_read_handle;              // IMU读取
TaskHandle_t position_state_manage_handle; // tcd1103读取

void Onmaincpp(void *pvParameters);
void IMU_Read_task(void *pvParameters);
void tcs230_read_task(void *pvParameters);
void gray_read_task(void *pvParameters);
void position_state_manage_task(void *pvParameters);
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

    BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 300, NULL, 4, &main_cpp_handle);
    BaseType_t ok5 = xTaskCreate(IMU_Read_task, "IMU_Read_task", 100, NULL, 4, &IMU_read_handle);
    BaseType_t ok7 = xTaskCreate(tcs230_read_task, "tcs230_read_task", 100, NULL, 2, &tcs230_read_handle);
    BaseType_t ok8 = xTaskCreate(gray_read_task, "gray_read_task", 100, NULL, 2, &gray_read_handle);
    BaseType_t ok9 = xTaskCreate(position_state_manage_task, "position_state_manage_task", 100, NULL, 2, &position_state_manage_handle);
    if (ok3 != pdPASS | ok5 != pdPASS || ok7 != pdPASS || ok8 != pdPASS)
    {
        // 任务创建失败，进入死循环
        while (1)
        {
            // uart_printf("create task failed\n");
        }
    }
}

void position_state_manage_task(void *pvParameters)
{

    while (1)
    {

        vTaskDelay(2);
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
        Digtal_gray = IIC_Get_Digtal(front);
        Digtal_gray_back = IIC_Get_Digtal(back);
        for (int i = 0; i < 8; i++)
        {
            digital_gray_data[i] = 1 - ((Digtal_gray >> i) & 0x01);           // 读取数字灰度传感器数据
            digital_gray_data_back[i] = 1 - ((Digtal_gray_back >> i) & 0x01); // 读取背面数字灰度传感器数据
        }

        // 获取传感器模拟量结果
        if (IIC_Get_Anolog(Anolog_gray, 8, front) && IIC_Get_Anolog(Anolog_gray_back, 8, back))
        {
        }

        if (digital_gray_data[0] == 1 && digital_gray_data[1] == 1 && digital_gray_data[2] == 1 && digital_gray_data[3] == 1 && digital_gray_data[4] == 1 && digital_gray_data[5] == 1 && digital_gray_data[6] == 1 && digital_gray_data[7] == 1)
        {
            real_time_gray_state = all_black;
        }
        else
        {
            real_time_gray_state = orgin_gray;
        }
        // 获取传感器归一化结果
        IIC_Anolog_Normalize(0xff, front); // 所有通道归一化都打开
        IIC_Anolog_Normalize(0xff, back);
        vTaskDelay(10); // 设置完，需要等上一会。stm8的运算速度没stm32快，等一下，让传感器把数据刷新一下。
        if (IIC_Get_Anolog(Normal, 8, front) && IIC_Get_Anolog(Normal_back, 8, back))
        {
        }
        IIC_Anolog_Normalize(0xff, front); // 为了下一次循环是非归一化，所以清零
        IIC_Anolog_Normalize(0xff, back);

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

static void setMotorNormal(int8_t speed0, int8_t speed1, int mode_temp)
{
    dc_motor_state = 0;
    dc_step_flag = 0;
    motor_mode = mode_temp;
    if (real_time_gray_state == all_black)
    {
        stop_motor_flag = 1;
        motor_mode = MOTOR_STOP;
    }
    if (stop_motor_flag == 0)
    {
        motor_0_user_speed = speed0;
        motor_1_user_speed = speed1;
    }
}

// 电机步进模式配置函数，减少重复代码
static void setMotorStepMode(int8_t speed0, int8_t speed1, int32_t distance, int mode_temp)
{
    motor_mode = mode_temp;
    step_complete_flag = 0;
    motor_0_user_speed = speed0;
    motor_1_user_speed = speed1;
    dc_step_distance = distance;
    dc_step_flag = 1; // 设置步进模式标志位
}
// 计算黑线位置（返回值范围：0-7，对应传感器位置）
float get_black_line_position(int ordinal)
{
    float position_sum = 0.0f;
    float position_sum_back = 0.0f;

    // 遍历所有传感器，累加黑线位置
    for (uint8_t i = 0; i < 8; i++)
    {
        position_sum += digital_gray_data[i] * sensor_weights[i];
    }

    for (uint8_t i = 0; i < 8; i++)
    {
        position_sum_back += digital_gray_data_back[i] * sensor_weights_back[i];
    }

    if (ordinal == front)
    {
        return position_sum / 8.0f;
    }
    else if (ordinal == back)
    {
        // 返回黑线平均位置
        return position_sum_back / 8.0f;
    }
}
static void setmotor_stop_tang(int8_t speed0, int8_t speed1, int mode_temp)
{
            // 设置电机正常模式，速度8,直到碰到黑线
            setMotorNormal(speed0, speed1, mode_temp);
            if (stop_motor_flag == 1)
            {
                stop_motor_flag=0;
                main_state++;
            }

}
void Onmaincpp(void *pvParameters)
{
    motor_0_active = 1;
    motor_1_active = 1;
    while (1)
    {
        switch (main_state)
        {
        case 0:
        {

            vTaskDelay(100);
            main_state++;
            break;
        }
       
        case 1:
        {
            // 设置电机正常模式，速度8,直到碰到黑线
            setMotorNormal(10, 10, 0);
            if (stop_motor_flag == 1)
            {
                stop_motor_flag=0;
                main_state++;
            }
            break;
        }
        case 2:
        {
            setMotorStepMode(8,8, 450, 0); // 设置电机步进模式，速度6，距离400
            main_state++;
            break;
        }
        case 3:
        {
            if (step_complete_flag == 1)
            {
                // 设置电机正常模式，速度8,直到碰到黑线
                setMotorNormal(10, 10, 0);
                if (stop_motor_flag == 1)
                {
                    stop_motor_flag=0;
                    step_complete_flag=0;
                    main_state++;
                }
            }
            break;
        }
        case 4:
        {
              setMotorStepMode(8, 8, 450, 0); // 设置电机步进模式，速度6，距离400
            main_state++;
            break;
        }
        case 5:
        {
               
            if (step_complete_flag == 1)
            {
                // 设置电机正常模式，速度8,直到碰到黑线
                setMotorNormal(8, 8, 0);
                if (stop_motor_flag == 1)
                {
                    stop_motor_flag=0;
                    step_complete_flag=0;
                    main_state++;
                }
            }
            main_state++;
            break;
        }
        //**************** */ 这里是defualt：break线********************************************************************************
        default:
            break;
        }
        vTaskDelay(100);
    }
}

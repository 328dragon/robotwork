// ​​Planner模块​​：负责轨迹规划，提供开环和闭环两种控制模式
// ​​Controller模块​​：负责控制算法执行和电机控制
// ​​Kinematic模块​​：负责运动学正逆解计算和里程计更新
// ​​FreeRTOS任务​​：提供实时调度框架

#include "mainwork.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ZDTstepmotor.h"
#include "Kinematic.h"
#include "controller.h"
#include "planner.h"
#include "usart.h"
#include "BMI088driver.h"
#include "lcd.h"
#include "lcd_init.h"
#include "pic.h"
#include "string.h"
#include "bsp_usart.h"
#include "tim.h"
#include "tcs230.h"
#include "gray.h"
#include "Catch.h"
#include "hwt905.h"
#include "ch040.h"
#define BUZZER_ON HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
#define BUZZER_OFF HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
int all_back_flag_finish = 0;
//int all_back_stop_flag = 0;
int all_back_count = 0;
int all_back_last_count;
int all_back_time = 0;
int can_increase_all_back = 0;

int edge_black_flag_finish = 1;
int edge_black_count = 0;
int edge_black_last_count;
int edge_black_time = 0;
int can_increase_edge_black = 0;
//IMU结构体
IMU_t *hwt905_imu={0};
float hwt905_yaw_true=0;
float ch040_yaw_last = 0;
float ch040_yaw_update=0;
// 灰度转弯值
int catch_flag = 0;
__IO int turn_stop_flag = 0; // 转弯停止标志位
int turn_dir_all = -1;
__IO int turn_state = 0;
// 主函数状态机
int main_state = 0;
int motor_mode = 0;
int question_one_catch_count = 0;	//前三个物料抓取计数
int question_two_catch_count = 0;	//后五个物料抓取计数(F处和G处可分为两波)
int drop_count = 0;					//放置物料累计计数
// 颜色传感器 状态机
__IO int goods_color = -1;
int read_cololr_flag = 0; // 颜色传感器读取标志位
int read_color_state = 0; // 颜色传感器读取状态
int temp_color = -1;
// 灰度
gray_state real_time_gray_state = orgin_gray; // 主灰度状态
// 前面灰度
float gray_front_p = 0.004f; // 前面灰度传感器的神秘小参数
float gray_data_front_middle = 0;
float gray_data_front_middle_temp = 0;
//int gray_count = 0;      // 前面灰度计数
//int gray_count_last = 0; // 上次前面灰度计数
uint8_t digital_gray_data[8];							//存储八路灰度数字量
int sensor_weights[8] = {-4, -3, -2, -1, 1, 2, 3, 4}; 	// 灰度传感器权重
unsigned char Digtal_gray;
unsigned char Anolog_gray[8] = {0};
unsigned char Normal[8] = {0};
//
int debug_pwm = 0;
int close_flag = 0;
int safe_flag = 0;
USARTInstance uart6 = {0};
void usart6_callback(void)
{
//   update(hwt905_imu) ;
	if (uart6.recv_buff[0] == 0x5A && uart6.recv_buff[1] == 0xA5)
    {
        ch040_get_data(uart6.recv_buff);
    }
	//防止yaw角的超范围突变影响
	if(fabsf(ch040_yaw-ch040_yaw_last) >= 0.2)
	{
		if(fabs(ch040_yaw+(360.0*0.0011)-ch040_yaw_last) >= 0.2)
			ch040_yaw -= (360.0*0.0011);
		else ch040_yaw += (360.0*0.0011);
	}
	ch040_yaw_last = ch040_yaw;
	//限幅防止疯转
	if(ch040_yaw >= (360*0.0011))ch040_yaw = (360*0.0011);
	else if(ch040_yaw <= -(360*0.0011))ch040_yaw = -(360*0.0011);
}
USART_Init_Config_s uart6_cfg = {
    .recv_buff_size = 100,
    .usart_handle = &huart6,
    .module_callback = usart6_callback,
};
//
float DEBUG = 0.0f;
float DEBUG2 = 0.0f;
float DEBUG3 = 0.0f;
int position_flag = 0;
int begin_flag = 1;
int safe_guard = 0;
cmd_vel_t debug_target_vel = {0, 0, 0};
odom_t debug_target_odom = {0, 0, 0};
odom_t debug_target_erro = {0.05, 0.05, 0.01};

// 实例化
static Controller_t ChassisControl_instance;
static Kinematic_t kinematic_instance;
static Planner_t planner_instance;
static StepMotorZDT_t zdt_stepmotor_instances[4]; // 静态实例

Controller_t *ChassisControl_ptr; // 控制器实例
Kinematic_t *kinematic_ptr;       // 麦轮实例
Planner_t *planner_ptr;           // 规划
StepMotorZDT_t *zdt_stepmotor_ptr[4] = {
    &zdt_stepmotor_instances[0],
    &zdt_stepmotor_instances[1],
    &zdt_stepmotor_instances[2],
    &zdt_stepmotor_instances[3]};

TaskHandle_t LCD_Show_handle;        // 显示
TaskHandle_t Chassic_control_handle; // 底盘控制
TaskHandle_t main_cpp_handle;        // 主函数
TaskHandle_t Planner_update_handle;  // 轨迹规划
TaskHandle_t IMU_read_handle;        // IMU读取
TaskHandle_t tcs230_read_handle;     // tcs230颜色传感器读取
TaskHandle_t gray_read_handle;       // 灰度传感器
void OnChassicControl(void *pvParameters);
void OnPlannerUpdate(void *pvParameters);
void Onmaincpp(void *pvParameters);
void IMU_Read_task(void *pvParameters);
void LCD_Show_task(void *pvParameters);
void tcs230_read_task(void *pvParameters);
void gray_read_task(void *pvParameters);

void main_work(void)
{
    HAL_UART_Receive_IT(&huart4, &RxData, 1);
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");
    printf("AT+LIGHT+ON\r\n");

    while (BMI088_init())
    {
        ;
    }
    USARTRegister(&uart6, &uart6_cfg);
    memset(uart6.recv_buff, 0, uart6.recv_buff_size);
    // 注意电机编号如下所示

    Step_ZDT_Init(zdt_stepmotor_ptr[0], 1, &huart3, 0, 0.06f, false); // 左上
    Step_ZDT_Init(zdt_stepmotor_ptr[1], 2, &huart3, 1, 0.06f, false); // 右上
    Step_ZDT_Init(zdt_stepmotor_ptr[2], 4, &huart3, 0, 0.06f, false); // 左下
    Step_ZDT_Init(zdt_stepmotor_ptr[3], 3, &huart3, 1, 0.06f, true);  // 右下

    ChassisControl_ptr = &ChassisControl_instance;
    kinematic_ptr = &kinematic_instance;
    planner_ptr = &planner_instance;
    Kinematic_init(kinematic_ptr, 0.6, 2, X_shape);
    Controller_Init(ChassisControl_ptr, zdt_stepmotor_ptr, kinematic_ptr);
    Planner_init(planner_ptr, ChassisControl_ptr);

    BaseType_t ok2 = xTaskCreate(OnChassicControl, "Chassic_control", 800, NULL, 3, &Chassic_control_handle);
    BaseType_t ok3 = xTaskCreate(Onmaincpp, "main_cpp", 600, NULL, 4, &main_cpp_handle);
    BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 300, NULL, 4, &Planner_update_handle);
    BaseType_t ok5 = xTaskCreate(IMU_Read_task, "IMU_Read_task", 200, NULL, 4, &IMU_read_handle);
    BaseType_t ok6 = xTaskCreate(LCD_Show_task, "LCD_Show_task", 300, NULL, 1, &LCD_Show_handle);
    BaseType_t ok7 = xTaskCreate(tcs230_read_task, "tcs230_read_task", 100, NULL, 2, &tcs230_read_handle);
    BaseType_t ok8 = xTaskCreate(gray_read_task, "gray_read_task", 100, NULL, 2, &gray_read_handle);
    if (ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS || ok5 != pdPASS || ok7 != pdPASS)
    {
        // 任务创建失败，进入死循环
        while (1)
        {
            // uart_printf("create task failed\n");
        }
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
	uint8_t black_count = 0;	//八路灰度黑色个数计数
    while (Ping())
    {
        vTaskDelay(5);
    }
    while (1)
    {
        //gray_count++;
		//若全黑寻找完成标志位为1则不寻找全黑状态
        if (all_back_flag_finish == 1)
        {
			//完成时清空标志
            all_back_time = 0;
            all_back_count=0;
			all_back_last_count = 0;
        }
		//若左右边缘为黑寻找完成标志位为1则不寻找边缘黑状态
		if(edge_black_flag_finish == 1)
		{
			//完成时清空标志
            edge_black_time = 0;
            edge_black_count=0;
			edge_black_last_count = 0;
		}
		
		//边缘黑死区计数
        edge_black_count++;
        if ((edge_black_count - edge_black_last_count) > 10)
        {
			//只有计数达到一定数量才允许边缘黑状态自加，防止连续自加
            can_increase_edge_black = 1;
        }
		//全黑死区计数
        all_back_count++;
        if ((all_back_count - all_back_last_count) > 12)
        {
			//只有计数达到一定数量才允许全黑状态自加，防止连续自加
            can_increase_all_back = 1;
        }
		
        // 读取灰度传感器数据
        Digtal_gray = IIC_Get_Digtal();
        for (int i = 0; i < 8; i++)
        {
            digital_gray_data[i] = 1 - ((Digtal_gray >> i) & 0x01); // 读取数 �灰度传感器数据
        }

        // 获取传感器模拟量结果
        if (IIC_Get_Anolog(Anolog_gray, 8))
        {
        }

		//若为全黑
        if (digital_gray_data[1] == 1 && digital_gray_data[3] == 1 && digital_gray_data[4] == 1 && digital_gray_data[2] == 1 && digital_gray_data[5] == 1&&digital_gray_data[6] == 1 ) // 中间6个
        {
            real_time_gray_state = all_black;				//灰度实时状态置为全黑
            if (can_increase_all_back == 1)					//若允许自加
            {
				BUZZER_ON
                all_back_time++;
                all_back_last_count = all_back_count;
                can_increase_all_back = 0;					//允许自加标志位清零
            }
        }
		//若为边缘黑
		else if(digital_gray_data[0] == 1 || digital_gray_data[7] == 1)
		{
			real_time_gray_state = edge_black;				//灰度实时状态置为全黑
            if (can_increase_edge_black == 1)				//若允许自加
            {
				BUZZER_ON
                edge_black_time++;
                edge_black_last_count = edge_black_count;
                can_increase_edge_black = 0;				//允许自加标志位清零
            }
		}
        else
        {
            BUZZER_OFF
            real_time_gray_state = orgin_gray;
        }

        // 获取传感器归一化结果
        IIC_Anolog_Normalize(0xff); // 所有通道归一化都打开
        vTaskDelay(10);             // 设置完，需要等上一会。stm8的运算速度没stm32快，等一下，让传感器把数据刷新一下。
        if (IIC_Get_Anolog(Normal, 8))
        {
        }
        IIC_Anolog_Normalize(0xff); // 为了下一次循环是非归一化，所以清零

		black_count = 0;		//计数先清零
        for (int i = 0; i < 8; i++)
        {
			if(digital_gray_data[i] == 1)
				black_count++;
            gray_data_front_middle_temp += gray_front_p * digital_gray_data[i] * sensor_weights[i]; // 计算前面灰度传感器的中间值
        }
        gray_data_front_middle = gray_data_front_middle_temp/(float)black_count;					//均值化处理
        gray_data_front_middle_temp = 0;
        vTaskDelay(10); // 延时100ms
    }
}
void LCD_Show_task(void *pvParameters)
{
    // 屏幕
    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, WHITE);
    while (1)
    {
        // 显示
        // 陀螺仪
		LCD_ShowIntNum(0,0,drop_count,2,RED,WHITE,16);
		LCD_ShowIntNum(0,20,main_state,2,RED,WHITE,16);
		LCD_ShowIntNum(24,20,edge_black_time,2,RED,WHITE,16);
		LCD_ShowFloatNum1(64, 20,fabs(ch040_yaw)*10.0, 5, RED, WHITE, 16);
		LCD_ShowIntNum(16,40,digital_gray_data[0],1,RED,WHITE,16);
		LCD_ShowIntNum(16*2,40,digital_gray_data[1],1,RED,WHITE,16);
		LCD_ShowIntNum(16*3,40,digital_gray_data[2],1,RED,WHITE,16);
		LCD_ShowIntNum(16*4,40,digital_gray_data[3],1,RED,WHITE,16);
		LCD_ShowIntNum(16*5,40,digital_gray_data[4],1,RED,WHITE,16);
		LCD_ShowIntNum(16*6,40,digital_gray_data[5],1,RED,WHITE,16);
		LCD_ShowIntNum(16*7,40,digital_gray_data[6],1,RED,WHITE,16);
		LCD_ShowIntNum(16*8,40,digital_gray_data[7],1,RED,WHITE,16);
		LCD_ShowIntNum(0,60,all_back_time,2,RED,WHITE,16);
		LCD_ShowFloatNum1(64,60,fabs(ch040_yaw_update)*10.0,5,RED,WHITE,16);
		
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
        BMI088_read(gyro, accel, &temp);

        if (catch_flag == 1)
        {
            Catch();
            catch_flag = 0;
        }
		else if(catch_flag == 2)
		{
			Catch_dragout(850);
            catch_flag = 0;
		}

        if (turn_dir_all == -1)
        {
            turn_stop_flag = 0;
            turn_state = 0;
        }
        if ((turn_dir_all == 0) && (real_time_gray_state != all_black))
        {
            switch (turn_state)
            {

            case 0:
            {
                if (digital_gray_data[0] == 1)
                {

                    turn_state++;
                }
                break;
            }
            case 1:
            {
                if (digital_gray_data[1] == 1)

                {

                    turn_state++;
                }

                break;
            }
            case 2:
            {
                if (digital_gray_data[2] == 1)

                {

                    turn_state++;
                }

                break;
            }
            case 3:
            {
                if (digital_gray_data[3] == 1)

                {

                    turn_stop_flag = 1;
                    turn_state = 0;
                }

                break;
            }
                //            case 4:
                //            {
                //                if (digital_gray_data[4] == 1)
                //                {
                //                    turn_stop_flag = 1;
                //                    turn_state = 0;
                //                }

                //                break;
                //            }
            default:
                break;
            }
        }
        else if (turn_dir_all == 1)
        {
        }
        vTaskDelay(10);
    }
}

static void move_to_next_line()
{
    motor_mode = 0;
    debug_target_vel = (cmd_vel_t){0.2, 0, 0};
    if (real_time_gray_state == all_black)
    {
        debug_target_vel = (cmd_vel_t){0, 0, 0};
        main_state++;
    }
}

static void turn_to_next_line()
{
    motor_mode = 0;
    turn_dir_all = 0;
    debug_target_vel = (cmd_vel_t){0, 0, 0.05};
    if (turn_stop_flag == 1)
    {
        debug_target_vel = (cmd_vel_t){0, 0, 0};
        turn_dir_all = -1; // 重置转弯状态
        main_state++;
    }
}
static void move_step_distance(float odom_x, float odom_y, float odom_yaw,bool clearodom)
{
    motor_mode = 1;
    debug_target_odom = (odom_t){odom_x, odom_y, odom_yaw};
    debug_target_erro = (odom_t){0.005, 0.005, 0.003};
    position_flag++;
    Planner_LoactaionCloseControl(planner_ptr, &debug_target_odom, 0.5, &debug_target_erro, clearodom);
    main_state++;
}

void Onmaincpp(void *pvParameters)
{
    int safe_count = 0; // 保护锁
    while (1)
    {
        //纯速度式验证没问题
        //Controller_set_vel_target(ChassisControl_ptr, debug_target_vel, false);
        safe_count++;
        if (safe_count >= 3)
        {
            safe_guard = 1; // 保护锁打开
			
			//0~1:车体行至中心点
			//2~9::抓取A、C、E处物料
			//10~19:抓取F处物料
			//20~ :放置物料至靶子处
            switch (main_state)
            {
            case 0:
            {
				//首先使用灰度巡线闭环行走，直到识别累计3次"十字"后停止
                motor_mode = 0;
                debug_target_vel = (cmd_vel_t){0.3, 0,-gray_data_front_middle};
                if (all_back_time == 3)
                {
                    debug_target_vel = (cmd_vel_t){0, 0, 0};
                    all_back_flag_finish = 1;
                    main_state++;
                }
                break;
            }
			case 1:
			{
				//清除里程计，向前走一小段修正姿态
				move_step_distance(0.07,0,0,1);
				//更新yaw角，认为此时yaw角是车的零点
				ch040_yaw_update = ch040_yaw;
				
				//main_state = 21;
				break;
			}
			
			
			case 2:
			{
				//清除里程计，转向 
			    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					if(question_one_catch_count == 0)		//向左转90度，朝向A点方向
						move_step_distance(0,0,0.103,1);
					else if(question_one_catch_count == 1)	//不转动，朝向C点方向
						move_step_distance(0,0,0,1);
					else if(question_one_catch_count == 2)	//向右转90度，朝向E点方向
						move_step_distance(0,0,-0.097,1);
				}
				break;
			}
			case 3:
			{
				//不清除里程计，不使用灰度巡线行至物料存放处
				if (SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					if(question_one_catch_count == 0)
						move_step_distance(0.205,0,0.103,0);	//A点
					else if(question_one_catch_count == 1)
						move_step_distance(0.19,0,0,0);			//C点
					else if(question_one_catch_count == 2)
						move_step_distance(0.205,0,-0.097,0);	//E点
				}
				break;
			}
			case 4:
			{
				//不清除里程计，使用灰度纠正车身Y方向
				if (SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					if(question_one_catch_count == 0)
						move_step_distance(0.205,-2.8*gray_data_front_middle,0.103,0);	//A点
					else if(question_one_catch_count == 1)
						move_step_distance(0.19,-2.8*gray_data_front_middle,0,0);		//C点
					else if(question_one_catch_count == 2)
						move_step_distance(0.205,-2.8*gray_data_front_middle,-0.097,0);	//E点
				}
				break;
			}
			case 5:
			{	  
				//动作执行完成后 夹取物料
				if (SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					catch_flag=1;
					main_state++;
				}
				break;
			}
			case 6:
			{
				//若抓取到物料，不清除里程计，返回到中心点(多给一点-0.05防止回不去)
				if(catch_finish_flag == 1)
				{
					if(question_one_catch_count == 0)
						move_step_distance(-0.05,0,0.103,0);	//A点
					else if(question_one_catch_count == 1)
						move_step_distance(-0.05,0,0,0);		//C点
					else if(question_one_catch_count == 2)
						move_step_distance(-0.05,0,-0.097,0);	//E点
					
					catch_finish_flag = 0;
				}
				break;
			}
			case 7:
			{
				//识别到黑线，清除里程计，回正停车
				if(real_time_gray_state == all_black)
				{
					move_step_distance(0,0,0,1);
				}
				break;
			}
			case 8:
			{
				//向前走，用于修正车体，使之在中心
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					move_step_distance(0.3,0,0,1);
				}
				break;
			}
			case 9:
			{
				//直到识别不到黑线，停车，即认为是中心点
				if(real_time_gray_state != all_black)
				{
					move_step_distance(0,0,0,1);
					main_state = 2;
					question_one_catch_count ++;
					//若A、C、E三个物料收集完成则到下一个状态
					if(question_one_catch_count == 3)
						main_state = 10;
				}
				break;
			}
			
			case 10:
			{
				//清除里程计，转向 
			    if (SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					if(question_two_catch_count <= 4)
						move_step_distance(0,0,0.152,1);				//转到F点方向
					else move_step_distance(0,0,-0.150,1);				//转到G点方向
				}
				break;
			}
			case 11:
			{
				//若完成，则进入到下一个状态
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					main_state++;
				}
				break;
			}
			case 12:
			{
				//使用灰度巡线行至累计识别到2次"十字"后停下
                motor_mode = 0;
                debug_target_vel = (cmd_vel_t){0.3, 0,-1.1*gray_data_front_middle};
				all_back_flag_finish = 0;
				if(question_two_catch_count == 0 || question_two_catch_count == 5)
				{
					if(all_back_time == 2)
					{
						motor_mode = 0;
						debug_target_vel = (cmd_vel_t){0,0,0};
						all_back_flag_finish = 1;
						main_state++;
					}
				}
				else
				{
					if(real_time_gray_state == all_black)
					{
						motor_mode = 0;
						debug_target_vel = (cmd_vel_t){0,0,0};
						all_back_flag_finish = 1;
						main_state++;
					}
				}
				break;
			}
			case 13:
			{
				//清除里程计，后退一段距离
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					if(question_two_catch_count <= 4)
						move_step_distance(-0.05,0,0.152,1);		//在F点处
					else move_step_distance(-0.05,0,-0.150,1);		//在G点处
				}
				break;
			}	
			case 14:
			{
				//清除里程计，修正车体Y方向
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					if(question_two_catch_count <= 4)
						move_step_distance(0,-2.9*gray_data_front_middle,0.152,1);		//在F点处
					else move_step_distance(0,-2.9*gray_data_front_middle,-0.150,1);	//在G点处
				}
				break;
			}
			case 15:
			{
				//微调至5个物料处
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					switch(question_two_catch_count)
					{
						//F处
						case 0:			//F处B点
							move_step_distance(0.03,0.025,0.152,1);
							break;
						case 1:			//F处A点
							move_step_distance(0.03,-0.025,0.152,1);
							break;
						case 2:			//F处C点
							move_step_distance(0.06,0.05,0.152,1);
							break;
						case 3:			//F处E点
							move_step_distance(0.06,-0.03,0.152,1);
							break;
						case 4:			//D
							move_step_distance(0.1,0,0.152,1);
							break;
						
						//G处
						case 5:			//G处A点
							move_step_distance(0.03,0.03,-0.150,1);
							break;
						case 6:			//G处B点
							move_step_distance(0.03,-0.03,-0.150,1);
							break;
						case 7:			//G处E点
							move_step_distance(0.06,0.05,-0.150,1);
							break;
						case 8:			//G处C点
							move_step_distance(0.06,-0.05,-0.150,1);
							break;
						case 9:			//D
							move_step_distance(0.1,0,-0.150,1);
							break;
						default:break;
					}
				}
				break;
			}
			case 16:
			{
				//抓取
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					catch_flag = 2;		//该抓取标志位为爪子伸进去向后拖拽一段距离，防止干扰紧邻的物料
					main_state++;
				}
				break;
			}
			case 17:
			{
				//若抓到则，清除里程计，后退
				if(catch_finish_flag == 1)
				{
					switch(question_two_catch_count)
					{
						//F处
						case 0:			//B
							move_step_distance(-0.1,-0.02,0.152,1);
							break;
						case 1:			//A
							move_step_distance(-0.1,0.02,0.152,1);
							break;
						case 4:			//D
							move_step_distance(-0.1,0,0.152,1);
							break;
						//G处
						case 5:			//B
						case 6:			//A
						case 9:			//D
							move_step_distance(-0.1,0,-0.150,1);
							break;
						//CD多退一点
						case 2:			//C
							move_step_distance(-0.1,-0.04,0.152,1);
							break;
						case 3:			//E
							move_step_distance(-0.1,0.04,0.152,1);
							break;
						case 7:			//C
							move_step_distance(-0.1,-0.04,-0.150,1);
							break;
						case 8:			//E
							move_step_distance(-0.1,0.04,-0.150,1);
							break;
						default:break;
					}
					catch_finish_flag = 0;
				}
				break;
			}
			case 18:
			{
				//根据已抓取物料个数决定是再次抓取还是返回中心点
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					question_two_catch_count++;
					
					//不抓D点的物料了
					if(question_two_catch_count == 4)
						question_two_catch_count = 5;
					
					if(question_two_catch_count == 5 || question_two_catch_count == 10)
						main_state = 19;
					else main_state = 12;
				}
				break;
			}
			case 19:
			{
				//返回中心点
                motor_mode = 0;
				//由于是倒车，故这里的yaw角矫正应相反
                debug_target_vel = (cmd_vel_t){-0.3, 0,gray_data_front_middle};
				all_back_flag_finish = 0;
				//识别到两次全黑停车
				if(all_back_time == 2)
				{
					motor_mode = 0;
					debug_target_vel = (cmd_vel_t){0,0,0};
					all_back_flag_finish = 1;
					main_state++;
					//不抓G处的5个物料了
					//if(question_two_catch_count == 5)
					//	main_state = 10;
				}
				break;
			}
			
			case 20:
			{
				//转弯至靶子处
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					switch(drop_count)
					{
						case 0:					//转弯至背朝绿色靶子处
							move_step_distance(0,0,-0.103,1);
							break;
						case 1:					//转弯至背朝白色靶子处
							move_step_distance(0,0,-0.152,1);
							break;
						case 2:					//转弯至背朝红色靶子处
							move_step_distance(0,0,-0.198,1);
							break;
						case 3:					//转弯至背朝黑色靶子处
							move_step_distance(0,0,-0.25,1);
							break;
						case 4:					//转弯至背朝蓝色靶子处
							move_step_distance(0,0,-0.302,1);
							break;
						default:break;
					}
				}
				break;
			}
			case 21:
			{
				//若完成，则进入到下一个状态
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					//main_state++;
					main_state = 22;
				}
				break;
			}
			case 22:
			{
				//if(catch_finish_flag == 1 || drop_count == 1 || drop_count == 3)
				{
					//使用灰度巡线行至累计识别到2次"边缘黑"或者一次“边缘黑”一次“全黑”后停下(由于有黑色靶心)
					motor_mode = 0;
					debug_target_vel = (cmd_vel_t){-0.3, 0,gray_data_front_middle};
					if(real_time_gray_state == orgin_gray)		//当识别不到全黑才允许识别
					{
						edge_black_flag_finish = 0;
						//all_back_flag_finish = 0;
					}
					if(edge_black_time == 1)
					{
						motor_mode = 0;
						debug_target_vel = (cmd_vel_t){0,0,0};
						edge_black_flag_finish = 1;
						//all_back_flag_finish = 1;
						main_state++;
						catch_finish_flag = 0;
					}
				}
				break;
			}
			case 23:
			{
				//倒退至靶子处
				switch(drop_count)
				{
					case 0:					//绿色靶子处
						move_step_distance(-0.24,0,-0.097,1);
						break;
					case 1:					//白色靶子处
						move_step_distance(-0.24,0,-0.152,1);
						break;
					case 2:					//红色靶子处
						move_step_distance(-0.24,0,-0.198,1);
						break;
					case 3:					//黑色靶子处
						move_step_distance(-0.24,0,-0.25,1);
						break;
					case 4:					//蓝色靶子处
						move_step_distance(-0.24,0,-0.302,1);
						break;
					default:break;
				}
				break;
			}
			case 24:
			{
				//调整位置
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					switch(drop_count)
					{
						case 0:					//绿色靶子处
							move_step_distance(0,-2.8*gray_data_front_middle,-0.097,1);
							break;
						case 1:					//白色靶子处
							move_step_distance(0,-2.8*gray_data_front_middle,-0.152,1);
							break;
						case 2:					//红色靶子处
							move_step_distance(0,-2.8*gray_data_front_middle,-0.198,1);
							break;
						case 3:					//黑色靶子处
							move_step_distance(0,-2.8*gray_data_front_middle,-0.25,1);
							break;
						case 4:					//蓝色靶子处
							move_step_distance(0,-2.8*gray_data_front_middle,-0.302,1);
							break;
						default:break;
					}
				}
				break;
			}
			case 25:
			{
				//直接开始随地拉一坨大的
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					switch(drop_count)
					{
						case 0:
							Drop(green_color);
							break;
						case 1:
							Drop(white_color);
							break;
						case 2:
							Drop(red_color);
							break;
						case 3:
							Drop(black_color);
							break;
						case 4:
							Drop(blue_color);
							break;
						default:break;
					}
					main_state++;
				}
				break;
			}
			case 26:
			{
				//使用灰度巡线行至中心点
                motor_mode = 0;
                //debug_target_vel = (cmd_vel_t){0.15, 0,-1.2*gray_data_front_middle};
				debug_target_vel = (cmd_vel_t){0.3,0,-gray_data_front_middle};
				edge_black_flag_finish = 0;
				if(edge_black_time == 1)		//先经过一次“边缘黑”
				{
					edge_black_flag_finish = 1;
					all_back_flag_finish = 0;
					main_state++;
				}
				break;
			}
			case 27:
			{
                motor_mode = 0;
				debug_target_vel = (cmd_vel_t){0.3,-2.8*gray_data_front_middle,0};
				if(all_back_time == 1)		//再经过一次全黑，到达中心点
				{
					all_back_flag_finish = 1;
					main_state++;
					
					motor_mode = 0;
					debug_target_vel = (cmd_vel_t){0,0,0};
				}
				break;
			}
			case 28:
			{
				//清除里程计，向前走一小段修正姿态
				switch(drop_count)
				{
					case 0:					//绿色靶子处
						move_step_distance(0.06,0,-0.097,1);
						break;
					case 1:					//白色靶子处
						move_step_distance(0.06,0,-0.152,1);
						break;
					case 2:					//红色靶子处
						move_step_distance(0.06,0,-0.198,1);
						break;
					case 3:					//黑色靶子处
						move_step_distance(0.06,0,-0.25,1);
						break;
					case 4:					//蓝色靶子处
						move_step_distance(0.06,0,-0.302,1);
						break;
					default:break;
				}
				break;
			}
			case 29:
			{
				if(SimpleStatus_t_isResolved(&planner_ptr->promise))
				{
					//本来用于纠正方向的，结果效果不太好
					switch(drop_count)
					{
						case 0:					//绿色靶子处
							//ch040_yaw_update = ch040_yaw + 0.097;
							break;
						case 1:					//白色靶子处
							//ch040_yaw_update = ch040_yaw + 0.152;
							break;
						case 2:					//红色靶子处
							//ch040_yaw_update = ch040_yaw + 0.205;
							break;
						case 3:					//黑色靶子处
							break;
						case 4:					//蓝色靶子处
							break;
						default:break;
					}
					main_state++;
					drop_count++;			//放置物料累计计数
					//判断是继续放置物料还是进入下一状态
					if(drop_count != 5)main_state = 20;
				}
				break;
			}
            default:break;
            }
        }

		//运动模式决策
        switch (motor_mode)
        {
        case 0:
        {
            Controller_set_vel_target(ChassisControl_ptr, debug_target_vel, false);
            break;
        }
        case 1:
        {
            break;
        }
        default:
            break;
        }
        vTaskDelay(20);
    }
}

// 轨迹规划更新任务
void OnPlannerUpdate(void *pvParameters)
{
    uint16_t last_tick = xTaskGetTickCount();
    while (1)
    {
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();
        Planner_update(planner_ptr, dt); // 轨迹规划
        vTaskDelay(50);
    }
}
// 底盘更新任务,包括执行层
void OnChassicControl(void *pvParameters)
{
    uint16_t last_tick = xTaskGetTickCount();

    while (1)
    {
        uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
        last_tick = xTaskGetTickCount();
        if (safe_guard)
        {
            Controller_KinematicAndControlUpdateWithYaw(ChassisControl_ptr, dt,ch040_yaw-ch040_yaw_update);
            // // 步进不需要速度环，此处仅为了读取电机速度
            ChassisControl_ptr->Controller_MotorUpdate(ChassisControl_ptr, dt);
        }
        else
        {
            float zero_speed[4] = {0, 0, 0, 0};
            ChassisControl_ptr->setmotor_speed(ChassisControl_ptr, zero_speed);
        }
        vTaskDelay(10);
    }
}

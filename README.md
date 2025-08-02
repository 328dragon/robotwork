mainwork.c里是逻辑上的主函数
在tcs230_read_task任务中完成了颜色传感器的读取，方法：当需要读取时，将read_cololr_flag置为1，这个是瞬时的可能需要等待一会
gray_read_task读取了灰度情况：digital_gray_data数组是灰度值读取情况，读到黑线为1，白线为0，并且任务里加了一个蜂鸣器（当中间四个检测到黑线时，响起）
LCD_Show_task显示屏显示陀螺仪数据，有点小bug暂时不管
IMU_Read_task读取板子上自带imu，但是零飘很严重
Onmaincpp是主函数运行，点到点的运行方式例子如下：
从（0，0，0）到（0.5，0，0）再到（{-0.5, -0.5, 0}）
void Onmaincpp(void *pvParameters)
{   
int safe_count=0;
    while (1)
    {
        // 速度位置式有问题
        //             Controller_set_pos_vel_target(ChassisControl_ptr, deubg_target_odom, debug_target_vel, false);
        // 纯速度式验证没问题
//      Controller_set_vel_target(ChassisControl_ptr, debug_target_vel, false);
			safe_count++;
			if(safe_count>=3)
			{
			       if (begin_flag == 1)
       {
				safe_guard=1;
					switch (position_flag)
           {
           case 0:
           {              
               if (SimpleStatus_t_isResolved(&planner_ptr->promise))
               {

                   debug_target_odom = (odom_t){0.5, 0, 0};
                //    debug_target_vel = (cmd_vel_t){0.1, 0.1, 0.1};
                   debug_target_erro = (odom_t){0.01, 0.01, 0.01};
                   position_flag++;
									Planner_LoactaionCloseControl(planner_ptr, &debug_target_odom, 0.5, &debug_target_erro, 0);   
               }
               break;
           }
           case 1:
           {
             
               if (SimpleStatus_t_isResolved(&planner_ptr->promise))
               {
                   debug_target_odom = (odom_t){-0.5, -0.5, 0};
                //    debug_target_vel = (cmd_vel_t){0.1, 0.1, 0.1};
                   position_flag++;
									Planner_LoactaionCloseControl(planner_ptr, &debug_target_odom, 0.3, &debug_target_erro, 0);   
               }
               break;
           }
					 case 2:
					 {
						   if (SimpleStatus_t_isResolved(&planner_ptr->promise))
               {
							 						 begin_flag=0;
					 position_flag=0;
							 }

					 break;
					 }
           default:
               break;
           }
       }
			}

         
        vTaskDelay(200);
    }
}
OnPlannerUpdate负责规划路径，当用速度模式时，用不到
OnChassicControl是底盘控制函数，逻辑验证过正确，但是可能轮子旋转方向有问题导致反向（侧移动）,唯一建议改动的就是里面pid的参数
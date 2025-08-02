#ifndef __MAINWORK_H
#define __MAINWORK_H

#include "main.h"
#include "cmsis_os.h"

typedef enum MotorMode
{
   
    MOTOR_MODE_NORMAL = 0, // 正常向前循迹
    MOTOR_MDOE_NORMAL_BACK = 1,
    MOTOR_MODE_TURN=2,
	 MOTOR_STOP=10000,
}MotorMode;
typedef enum POSITION
{
ORIGIN=0,
MID_POSITION ,
CENTER_POSITION ,
place_A,
place_B,
place_C,
place_D,
place_E,
place_F,
place_G,
place_A_PLUS,
place_B_PLUS,
place_C_PLUS,
place_D_PLUS,
place_E_PLUS,
place_F_PLUS,
place_G_PLUS
}task_POSITION;
typedef enum gray_state
{
    orgin_gray=0,
     all_black,//全黑
   primary_gray,
    senior_gray,
    undetermined_gray
   
} gray_state;
extern MotorMode motor_mode;
float get_black_line_position(int ordinal);
void main_work(void);
#endif

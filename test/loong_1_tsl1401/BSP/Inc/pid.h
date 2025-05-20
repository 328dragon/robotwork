 #ifndef __PID_H
#define __PID_H

#include "main.h"
#include <stdint.h>
//类型转换
typedef unsigned char uint8;
typedef unsigned short uint16;

typedef char int8;
typedef short int16;

extern float motor_set_1;
extern float motor_set_2;


typedef volatile uint8 vuint8;
typedef volatile uint16 vuint16;
//举出pid状态量
enum
{
  LLAST = 0,
  LAST,
  NOW,
  POSITION_PID,
  DELTA_PID,
};
//pid结构体,p、i、d和这几个
typedef struct pid_t
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3];

  float pout;
  float iout;
  float dout;
  float out;

  float input_max_err;   // input max err;
  float output_deadband; // output deadband;

  uint32_t pid_mode;//pid模式
  uint32_t max_out;//限幅作用的输出限制
  uint32_t integral_limit;//积分限制

  void (*f_param_init)(struct pid_t *pid,
                       uint32_t pid_mode,
                       uint32_t max_output,
                       uint32_t inte_limit,
                       float p,
                       float i,
                       float d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);

} pid_t;


//灰度pid结构体
typedef struct 
{
  float p;
  float i;
  float d;

  float set;
  float get;
  float err[3]; //error

  float pout; 
  float iout; 
  float dout; 
  float out;

  float input_max_err;    //input max err;
  float output_deadband;  //output deadband; 

  float p_far;
  float p_near;
  float grade_range;
  
  uint32_t pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;

  void (*f_param_init)(struct pid_t *pid, 
                       uint32_t      pid_mode,
                       uint32_t      max_output,
                       uint32_t      inte_limit,
                       float         p,
                       float         i,
                       float         d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);
 
} grade_pid_t;

//封装pid初始化
void PID_struct_init(
    pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd);
		
void abs_limit(float *a,float  max_abs);

 float pid_calc(pid_t *pid, float get, float set);//速度闭环,pid为对象，get为编码器读到的值，set是目标值
float position_pid_calc(pid_t *pid, float fdb, float ref);//位置式速度闭环

extern pid_t pid_left;
extern pid_t pid_right;

#endif

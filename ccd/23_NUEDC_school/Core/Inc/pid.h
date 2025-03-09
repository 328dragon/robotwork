/*
 * @Author: Ptisak
 * @Date: 2022-06-16 12:27:24
 * @LastEditors: Ptisak
 * @LastEditTime: 2022-07-25 15:47:30
 * @Version: Do not edit
 */
#ifndef __pid_H__
#define __pid_H__

#include "main.h"
/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

typedef unsigned char uint8;       // �޷���  8 bits
typedef unsigned short int uint16; // �޷��� 16 bits
// typedef unsigned long int uint32;  // �޷��� 32 bits
// typedef unsigned long long uint64; // �޷��� 64 bits

typedef char int8;       // �з���  8 bits
typedef short int int16; // �з��� 16 bits
// typedef long int int32;  // �з��� 32 bits
// typedef long long int64; // �з��� 64 bits

typedef volatile uint8 vuint8;   // �ױ������� �޷���  8 bits
typedef volatile uint16 vuint16; // �ױ������� �޷��� 16 bits
// typedef volatile uint32 vuint32; // �ױ������� �޷��� 32 bits
// typedef volatile uint64 vuint64; // �ױ������� �޷��� 64 bits

typedef volatile int8 vint8;   // �ױ������� �з���  8 bits
typedef volatile int16 vint16; // �ױ������� �з��� 16 bits
// typedef volatile int32 vint32; // �ױ������� �з��� 32 bits
// typedef volatile int64 vint64; // �ױ������� �з��� 64 bits
// #define uint32_t uint16
enum
{
  LLAST = 0,
  LAST,
  NOW,
  POSITION_PID,
  DELTA_PID,
};
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

  uint32_t pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;

  void (*f_param_init)(struct pid_t *pid,
                       uint32_t pid_mode,
                       uint32_t max_output,
                       uint32_t inte_limit,
                       float p,
                       float i,
                       float d);
  void (*f_pid_reset)(struct pid_t *pid, float p, float i, float d);

} pid_t;

#if 0
#define PID_PARAM_DEFAULT \
  {                       \
    0,                    \
        0,                \
        0,                \
        0,                \
        0,                \
        {0, 0, 0},        \
        0,                \
        0,                \
        0,                \
        0,                \
        0,                \
        0,                \
  }\

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
#endif

void PID_struct_init(
    pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd);

float pid_calc(pid_t *pid, float get, float set);
float position_pid_calc(pid_t *pid, float fdb, float ref);
void ControlLoop(void);
void ControtLoopTaskInit(void);

extern pid_t pid_l;
extern pid_t pid_r;
extern pid_t pid_pos;
extern pid_t pid_light;
// extern pid_t pid_turn;
#endif

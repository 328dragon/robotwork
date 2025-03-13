#include "pid.h"
#include "main.h"
#include "stm32g4xx_it.h"
#include "math.h"
//pid的参数初始化


pid_t pid_left;
pid_t pid_right;

void abs_limit(float *a,float  max_abs)
{
    if(*a>max_abs) *a=max_abs;
    else if(*a<-max_abs) *a=-max_abs;
}

//PID参数初始化
static  void f_param_init( pid_t *pid,
                           uint32_t mode,
                           uint32_t maxout,
                           uint32_t intergral_limit,
                           float kp,
                           float ki,
                           float kd)
{
    pid->pid_mode=mode;
    pid->max_out=maxout;
    pid->integral_limit=intergral_limit;
    pid->p=kp;
    pid->i=ki;
    pid->d=kd;

}

//pid状态初始化

static void f_pid_reset(pid_t *pid, float kp, float ki, float kd) {
    pid->p=kp;
    pid->i=ki;
    pid->d=kd;

    pid->pout=0;
    pid->dout=0;
    pid->iout=0;
    pid->out=0;
}

//pid总初始化

void PID_struct_init(
    pid_t *pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd) {

    pid->f_param_init=f_param_init;
    pid->f_pid_reset=f_pid_reset;

    pid->f_param_init(pid,mode,maxout,intergral_limit,kp,ki,kd);
    pid->f_pid_reset(pid,kp,ki,kd);
}

/// @brief 计算pid,两种模式：position和delta
/// @param pid
/// @param get
/// @param set
/// @return
float pid_calc(pid_t *pid, float get, float set) {
    pid->set=set;
    pid->get=get;
    pid->err[NOW]=set-get;
    if (pid->pid_mode == POSITION_PID) { //POSITION pid
        pid->pout=pid->p *pid->err[NOW];
        pid->iout+=pid->i*pid->err[NOW];
        if((Encode_L+Encode_R )<=2)
        {
            pid->dout=0;
        }
        pid->dout=pid->d*(pid->err[NOW]-pid->err[LAST]);

        abs_limit(&(pid->iout),pid->integral_limit);//积分限幅
        pid->out=pid->pout+pid->iout+pid->dout;
        abs_limit(&(pid->out),pid->max_out);//输出限幅
    } else if(pid->pid_mode==DELTA_PID)//delta pid
    {
        pid->pout=pid->p*(pid->err[NOW]-pid->err[LAST]);
        pid->iout=pid->i*pid->err[NOW];
        pid->dout= pid->d*(pid->err[NOW]-2*pid->err[LAST]+pid->err[LLAST]);

        pid->out=pid->pout+pid->iout+pid->dout;
        abs_limit(&(pid->out),pid->max_out);//输出限幅
    }

    pid->err[NOW]=pid->err[LAST];
    pid->err[LAST]=pid->err[LLAST];
    if(!pid->set) { //目标为0
        return 0;
    }

    if((pid->output_deadband!=0)&&((pid->out)<pid->output_deadband)) {
        return 0;
    }
    else {
        return pid->out;

    }
}


float position_pid_calc(pid_t *pid, float get, float set) { //只有位置pid
    pid->set=set;
    pid->get=get;
    pid->err[NOW]=set-get;
    pid->pout=pid->p *pid->err[NOW];
    pid->iout+=pid->i*pid->err[NOW];
    pid->dout=pid->d*(pid->err[NOW]-pid->err[LAST]);
    abs_limit(&(pid->iout),pid->integral_limit);//积分限幅
    pid->out=pid->pout+pid->iout+pid->dout;
    abs_limit(&(pid->out),pid->max_out);//输出限幅
    pid->err[NOW]=pid->err[LAST];
    pid->err[LAST]=pid->err[LLAST];
    return pid->out;
}




#ifndef __PID_H_
#define __PID_H_

#include "main.h"

typedef struct
{
  float tau;
  float kp;
  float ki;
  float kd;
  float err;
  float last_err;
  float i_err;
  float d_err;
  float integral_limit;//积分限幅
  float pid_out;       //最后整体的输出
  float pid_dead;      //死区
  float out_limit;     //输出限幅
  float p_out;
  float i_out;
  float d_out;	
}PID;

void pid_init(PID *pid, float tau, float kp, float ki, float kd, float out_limit, float integral_limit, float pid_dead);
float calc_pid(PID *pid,float actual,float target);
float calc_Mit(PID *pid, float actual, float target);
void pid_reset(PID *pid);

#endif

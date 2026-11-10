#include "pid.h"
#include "math.h"
#include <string.h>
#include "cmsis_os2.h"

static void calc_limit(float *value, float limit)
{
    if (value != NULL && limit > 0.0f)
    {
        if (*value > limit)
        {
            *value = limit;
        }
        else if (*value < -limit)
        {
            *value = -limit;
        }
    }
}

void pid_init(PID *pid, float tau, float kp, float ki, float kd, float out_limit, float integral_limit, float pid_dead)
{
    // 先将整个结构体清零
    memset(pid, 0, sizeof(PID));

    pid->tau = tau;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->out_limit = out_limit;
    pid->integral_limit = integral_limit;
    pid->pid_dead = pid_dead;
}

void pid_reset(PID *pid) // 清空pid参数
{
    pid->err = 0;
    pid->last_err = 0;
    pid->i_err = 0;
    pid->d_err = 0;
    pid->p_out = 0;
    pid->i_out = 0;
    pid->d_out = 0;
    pid->pid_out = 0;
}

float calc_pid(PID *pid, float actual, float target)
{
    pid->err = target - actual;
    pid->d_err = pid->err - pid->last_err;

    if (fabsf(pid->err) <= fabsf(pid->pid_dead))
    {
        pid->pid_out = 0;
        pid->last_err = pid->err;
        return pid->pid_out; // 直接返回，不累加积分
    }
    else
    {
        pid->i_err += pid->err;
        calc_limit(&pid->i_err, pid->integral_limit);
        pid->p_out = pid->kp * pid->err;
        pid->i_out = pid->ki * pid->i_err;
        pid->d_out = pid->kd * pid->d_err;

        pid->pid_out = pid->tau + pid->p_out + pid->i_out + pid->d_out;
    }
    calc_limit(&pid->pid_out, pid->out_limit);
    pid->last_err = pid->err;

    return pid->pid_out;
}

float calc_Mit(PID *pid, float actual, float target)
{
    pid->err = target - actual;
    pid->d_err = pid->err - pid->last_err;

    pid->p_out = pid->kp * pid->err;
    pid->d_out = pid->kd * pid->d_err;

    pid->pid_out = pid->tau + pid->p_out + pid->d_out;

    calc_limit(&pid->pid_out, pid->out_limit);
    pid->last_err = pid->err;

    return pid->pid_out;
}
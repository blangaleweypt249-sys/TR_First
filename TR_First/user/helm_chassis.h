#ifndef __HELM_CHASSIS_H__
#define __HELM_CHASSIS_H__

#include <stdbool.h>
#include <stdint.h>
#include "pid.h"
#include "DJI.h"
#include "vesc.h"
#include "WpLib.h"

#define half_dy 0.238        // 几何中心到轮子中心的y轴距离
#define half_dx 0.252        // 几何中心到轮子中心的x轴距离

typedef struct
{
	float vx;
	float vy;
	float vw;
	float actual_vx;
	float actual_vy;
	float actual_vw;
}Speed;

typedef struct
{
    float helm_angle[4];
    float helm_speed[4];
    float actual_helm_angle[4]; // 归一化后的舵向反馈角，单位为弧度
    float delta[4];
    float set_angle[4];
}Helm_chassis;

typedef enum
{
    helm_idle = 0,
    helm_calibration,
    helm_calibration_success
}helm_state;

typedef struct
{
	float actual_angle;
	float target_angle;
	float angle_out;
	float last_angle;
}Angle;

typedef struct
{
  bool all_helm_success_flag;
  bool helm_calibration_flag[4];
}calibration;

extern volatile uint8_t ready_target_initialized;
extern VESC vesc_motor[4];
extern DJI_Motor helm_motor[4];
extern PID pid_CarAnale;
extern Angle car_angle;

void Helm_Calibration(void);
void helm_calculate(Speed *body_speed,Helm_chassis *helm_chassises);
void helm_chassis_ready(void);
float speed_planning(float target,float *last_target,float accel_step,float decel_step);
float CosKspeed(float set_angle, float actual_angle);
void car_angle_maintain(Speed *world_speed, Angle *angle);
void World_to_body(Speed *world_speed, Speed *body_speed, float imu_angle);
float wrap_pi(float angle);
float wrap_angle(float angle);
void Min_Angle(float *delta, float *speed);

#endif

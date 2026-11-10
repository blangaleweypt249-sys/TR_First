#ifndef __PATH_H__
#define __PATH_H__

#include "main.h"
#include "stdbool.h"
#include "math.h"
#include "helm_chassis.h"
#include "imu_100A.h"

typedef struct
{
    //轮子本体坐标
    float b_x;
    float b_y;
    //轮子世界坐标
    float w_x;
    float w_y;
    //轮子速度
    float v_x;
    float v_y;
}Wheel;

extern Wheel wheel[4];

void Cacl_wheel(float yaw, float lidar_x, float lidar_y);
bool QuarterLidarTurn(float turn_speed, float lidar_x, float lidar_y);
/* LorF > 0 turns left; LorF < 0 turns right. R is the chassis-center radius. */
bool QuarterTurn(float turn_speed, float R, float LorF);
#endif

#include "Leg.h"

// 后腿电机
DJI_Motor leg_Left;
DJI_Motor leg_Right;
// 后腿电机2006
DJI_Motor leg_Left_2006;
DJI_Motor leg_Right_2006;

void Leg_Init(void)
{
    // 后腿电机初始化
    DJI_Init(&leg_Left, &hfdcan2, DJI_M3508, 1, 1);
    DJI_Init(&leg_Right, &hfdcan2, DJI_M3508, 1, 2);
    DJI_Init(&leg_Left_2006, &hfdcan2, DJI_M2006, 2, 1);
    DJI_Init(&leg_Right_2006, &hfdcan2, DJI_M2006, 2, 2);
}

void Leg_Clear(void)
{
    DJI_SignZero(&leg_Left);
    DJI_SignZero(&leg_Right);
    DJI_SignZero(&leg_Left_2006);
    DJI_SignZero(&leg_Right_2006);
}
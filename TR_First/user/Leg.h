#ifndef __LEG_H__
#define __LEG_H__

#include "main.h"
#include "DJI.h"


void Leg_Init(void);
void Leg_Clear(void);

// 后腿电机
extern DJI_Motor leg_Left;
extern DJI_Motor leg_Right;
// 后腿电机2006
extern DJI_Motor leg_Left_2006;
extern DJI_Motor leg_Right_2006;

#endif
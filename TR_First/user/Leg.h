#ifndef __LEG_H__
#define __LEG_H__

#include "main.h"
#include "DJI.h"
#include "cmsis_os.h"

//前腿PB7

void Leg_Init(void);
void Leg_Clear(void);
void Climb_Leg(float distance);
void Leg_Front(uint8_t out);    //前腿气缸 1 伸出
void Leg_Mid(uint8_t out);      //中腿气缸 1 伸出


extern uint8_t front_flag;
// 后腿电机
extern DJI_Motor leg_Left;
extern DJI_Motor leg_Right;
// 后腿电机2006
extern DJI_Motor leg_Left_2006;
extern DJI_Motor leg_Right_2006;

#endif
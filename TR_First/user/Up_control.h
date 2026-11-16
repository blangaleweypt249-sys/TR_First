#ifndef __UP_CONTROL_H__
#define __UP_CONTROL_H__

#include "main.h"
#include "cmsis_os2.h"
#include "DJI.h"
#include "RS00.h"
#include "RS05.h"
#include "MG5010E.h"

//机械臂，一台臂 4 个关节 + 末端气路
typedef struct
{
    MG5010E joint1;         //大臂第一个电机
    RS00 joint2;            //大臂第二个电机
    RS05 joint3;            //小臂
    RS05 joint4;            //腕
    DJI_Motor slide;        //底座滑台，只有右臂有
    uint8_t has_slide;

    GPIO_TypeDef *pump_port;    //气泵
    uint16_t pump_pin;
    GPIO_TypeDef *vac_port;     //真空阀
    uint16_t vac_pin;
    GPIO_TypeDef *grip_port;    //夹爪(电磁阀)
    uint16_t grip_pin;
} Arm;

extern Arm left_arm;
extern Arm right_arm;
extern volatile uint8_t arm_boot_cnt;    //RS 电机上电 announce 帧计数，6 帧到齐说明都启动好了

void Up_Init(void);
void arm_attraction(void);
void Arm_PollTimer(void);               //TIM7 回调里调，读大臂电机位置
void Arm_Suck(Arm *arm);                //吸取：气泵开、真空阀关
void Arm_Release(Arm *arm);             //放下：气泵关、真空阀开
void Arm_Off(Arm *arm);                 //完成：全关
void Arm_Grip(Arm *arm, uint8_t on);    //夹爪

extern uint8_t act_flag;

#endif

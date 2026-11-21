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
    uint8_t id;             //0 左臂 1 右臂，对应吸块表里的偏移

    MG5010E joint1;         //大臂第一个电机
    RS00 joint2;            //大臂第二个电机
    RS05 joint3;            //小臂
    RS05 joint4;            //腕
    DJI_Motor slide;        //底座滑台，只有右臂有
    uint8_t has_slide;

    float target[4];        //关节1~4 目标：第 1 个度，后 3 个弧度
    uint8_t suction;        //0 关 1 吸 2 放

    GPIO_TypeDef *pump_port;    //气泵
    uint16_t pump_pin;
    GPIO_TypeDef *vac_port;     //真空阀
    uint16_t vac_pin;
} Arm;

//机械臂动作状态
typedef enum
{
    eUpIdle = 0,
    eUpStoreSky,        //存天空块
    eUpSuckGround1,     //吸取地面1层块
    eUpVertical,        //竖直排列
}Up_State;

//分段动作：先动前一段，走到 90% 再放下一段
typedef struct
{
    uint8_t stage;      //分段阶段
    float joint1_start; //关节1起步位置
    float joint2_start; //关节2起步位置
    float joint3_start; //关节3起步位置
    float joint4_start; //关节4起步位置
}Arm_Stage;

extern Arm left_arm;
extern Arm right_arm;
extern Up_State up_state;
extern volatile uint8_t arm_boot_cnt;    //RS 电机上电 announce 帧计数，6 帧到齐说明都启动好了

void Up_Init(void);
void Arm_Home(Arm *arm);                    //回零：4 个关节目标都置 0
void Arm_SuckBlock(Arm *arm);               //去吸块位
void Arm_MoveTo(Arm *arm, const float *pos); //改目标，单位：度/弧度混合
void Arm_Suction(Arm *arm, uint8_t cmd);    //0 全关 1 吸取 2 放下
void Arm_Suck(Arm *arm);                    //吸取：气泵开、真空阀关
void Arm_Release(Arm *arm);                 //放下：气泵关、真空阀开
void Arm_Off(Arm *arm);                     //全关
void Arm_PollTimer(void);                   //TIM7 回调里调，读大臂电机位置

#endif

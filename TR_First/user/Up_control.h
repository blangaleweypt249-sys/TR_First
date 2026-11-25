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
    eUpStoreSky2,       //存两个天空块
    eUpSuckGround1,     //吸取地面1层块
    eUpVertical,        //竖直排列
}Up_State;

//分段动作：等关节 1、2 走稳了再放末端，末端也稳了算完
typedef struct
{
    uint8_t stage;      //0 空闲 1 等1、2关节 2 等末端 3 已稳
    uint8_t still;      //连续稳定计数
    float last1;        //上一拍 5010 位置
    float last2;        //上一拍 RS00/RS05 位置
    uint32_t tick;      //开始等稳定的时刻
}Arm_Stage;

extern Arm left_arm;
extern Arm right_arm;
extern Up_State up_state;
extern volatile uint8_t Motor_cnt;       //RS 电机上电帧计数，6 帧到齐说明都启动好了
extern uint8_t arm_set_home;             //1 时只标零，不发运动指令
extern volatile uint8_t M5010_pos_flag;  //200ms 读一次大臂位置
extern float slide_pos;                  //右臂底座滑台目标
extern osSemaphoreId_t Reboot_motorHandle;          

//机械臂姿态表：前 4 个左臂、后 4 个右臂，单位 度 / 弧度
extern const float suck_block_pos[8];
extern const float suck_ball_pos[4];
extern const float ball_done_pos[4];
extern const float sky_block_pos[8];
extern const float store_sky_pos[4];
extern const float vertical_pos[8];
extern const float vertical_put_pos[8];
extern const float layer2_pos[8];
extern const float left_layer2_pos[4];

void Up_Init(void);
void Arm_StageTick(void);                   //三个分段状态机，任务每拍调一次
void Arm_Send(Arm *arm);                    //按目标下发 4 个关节：5010 + RS00 + 2 个 RS05
void Arm_Suction_Off(void);                 //放下 100ms 后关气路
void Arm_Home(Arm *arm);                    //回零：4 个关节目标都置 0
void Arm_SuckBlock(Arm *arm);               //去吸块位
void Arm_MoveTo(Arm *arm, const float *pos); //改目标，单位：度/弧度混合
void Arm_Suction(Arm *arm, uint8_t cmd);    //0 全关 1 吸取 2 放下
void Arm_Suck(Arm *arm);                    //吸取：气泵开、真空阀关
void Arm_Release(Arm *arm);                 //放下：气泵关、真空阀开
void Arm_Off(Arm *arm);                     //全关

//按键要用到的动作入口，内部状态都留在本模块
void Arm_SuckGround1(void);                 //吸地面1层块，两臂
void Arm_HomeAll(void);                     //回零，按当前状态决定先等大臂还是直接回
void Arm_EnterStoreSky(void);               //进存天空块
void Arm_ExitStoreSky(void);                //退存天空块
void Arm_EnterVertical(void);               //竖直排列预备
void Arm_PreparePutLeft(void);              //竖直放置：左臂
void Arm_PreparePutRight(void);             //竖直放置：右臂，先只走 1、2 关节
void Arm_Suction_Toggle(void);              //左右气路一起切
void Arm_PollTimer(void);                   //TIM7 回调里调，置读位置标志

#endif

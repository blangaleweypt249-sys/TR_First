#ifndef __DJI_H__
#define __DJI_H__

#include <stdint.h>
#include <stdbool.h>
#include "fdcan.h"

// 电机型号
typedef enum
{
    DJI_M2006 = 0,
    DJI_M3508
} DJI_Type;

// 命令，数值必须和从机一致：0停止 1速度 2位置 3回零 4标零
typedef enum
{
    DJI_STOP = 0,   //停止，电流给0
    DJI_SPEED,      //速度模式
    DJI_POSITION,   //位置模式
    DJI_to_zero,    //回零，转到偏置角
    DJI_sign_zero   //标零，当前位置记为零点
} DJI_Mode;

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    DJI_Type type;              // M2006 / M3508，决定帧基址
    uint32_t can_id;            // 命令帧基址 0x300 / 0x400
    uint8_t board_id;           // 小板 id
    uint8_t motor_id;           // 槽位，舵轮板恒为1

    DJI_Mode mode;
    float target_speed;
    float target_position;

    float Feedback_angle;       // 舵向反馈角 单位：度
    float Feedback_speed;       // 反馈转速 单位：rpm
    uint8_t status;             // 从机状态位

    // 舵轮标定用
    bool calibration_valid;
    float angle_cos;
    float speed_cosk;

    int num[3];                 // 在线检测
} DJI_Motor;

void DJI_Init(DJI_Motor *motor, FDCAN_HandleTypeDef *hfdcan, DJI_Type type, uint8_t board_id, uint8_t motor_id);
void DJI_Speed(DJI_Motor *motor, float speed);
void DJI_Position(DJI_Motor *motor, float position);
void DJI_Stop(DJI_Motor *motor);
void DJI_ToZero(DJI_Motor *motor);
void DJI_SignZero(DJI_Motor *motor);
void DJI_Analysisdata(DJI_Motor *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);

#endif

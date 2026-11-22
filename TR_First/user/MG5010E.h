#ifndef __MG5010E_H__
#define __MG5010E_H__

#include "main.h"
#include "fdcan.h"

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t can_id;         //0x140 | can_id
    float ratio;            //减速比

    float pos;              //输出端角度 单位：度
    float target_pos;
    float speed_limit;      //回零/点到点转速上限 单位：rpm

    uint32_t last_tick;     //最近一次收到回执，毫秒
} MG5010E;

void MG5010E_Init(MG5010E *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t can_id, float ratio);
void MG5010E_Speed(MG5010E *motor, float speed, float current);
void MG5010E_Position(MG5010E *motor, float pos);
void MG5010E_SpeedPos(MG5010E *motor, float pos, float speed_limit);
void MG5010E_ReadPosition(MG5010E *motor);
void MG5010E_SetOrigin(MG5010E *motor);
void MG5010E_Stop(MG5010E *motor);
void MG5010E_Reboot(MG5010E *motor);
void MG5010E_GetPosition(MG5010E *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data);

#endif

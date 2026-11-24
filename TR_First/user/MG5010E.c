#include "MG5010E.h"
#include "bsp_can.h"
#include <string.h>

#define RPM_TO_DPS 6.0f

//命令，数值按厂家协议
enum
{
    eSpeedCommand = 0xA2,
    ePositionCommand = 0xA3,
    eSpeedPosCommand = 0xA4,
    eSetOriginCommand = 0x19,
    eReadStateCommand = 0x9C,
    eReadPositionCommand = 0x92,
    eClearErrorCommand = 0x9B,
    eEnableCommand = 0x88
};

void MG5010E_Init(MG5010E *motor, FDCAN_HandleTypeDef *hfdcan, uint8_t can_id, float ratio)
{
    motor->hfdcan = hfdcan;
    motor->can_id = can_id;
    motor->ratio = ratio;
    motor->pos = 0.0f;
    motor->target_pos = 0.0f;
    motor->speed_limit = 0.0f;
    motor->last_tick = HAL_GetTick();
}

//速度模式 speed rpm  current A
void MG5010E_Speed(MG5010E *motor, float speed, float current)
{
    uint8_t tx_data[8] = {0};
    int32_t speed_raw = (int32_t)(speed * motor->ratio * RPM_TO_DPS * 100.0f);
    int16_t current_raw;

    if (current > 33.0f)
    {
        current = 33.0f;
    }
    else if (current < -33.0f)
    {
        current = -33.0f;
    }
    current_raw = (int16_t)(current / 33.0f * 2048.0f);

    tx_data[0] = eSpeedCommand;
    tx_data[2] = (uint8_t)((uint16_t)current_raw & 0xFFU);
    tx_data[3] = (uint8_t)((uint16_t)current_raw >> 8);
    tx_data[4] = (uint8_t)((uint32_t)speed_raw & 0xFFU);
    tx_data[5] = (uint8_t)(((uint32_t)speed_raw >> 8) & 0xFFU);
    tx_data[6] = (uint8_t)(((uint32_t)speed_raw >> 16) & 0xFFU);
    tx_data[7] = (uint8_t)(((uint32_t)speed_raw >> 24) & 0xFFU);

    CAN_Transmit(motor->hfdcan, 0, 0x140U | motor->can_id, tx_data, 8);
}

//多圈位置模式 pos 度
void MG5010E_Position(MG5010E *motor, float pos)
{
    uint8_t tx_data[8] = {0};
    int32_t pos_raw = (int32_t)(pos * motor->ratio * 100.0f);

    motor->target_pos = pos;

    tx_data[0] = ePositionCommand;
    tx_data[4] = (uint8_t)((uint32_t)pos_raw & 0xFFU);
    tx_data[5] = (uint8_t)(((uint32_t)pos_raw >> 8) & 0xFFU);
    tx_data[6] = (uint8_t)(((uint32_t)pos_raw >> 16) & 0xFFU);
    tx_data[7] = (uint8_t)(((uint32_t)pos_raw >> 24) & 0xFFU);

    CAN_Transmit(motor->hfdcan, 0, 0x140U | motor->can_id, tx_data, 8);
}

//多圈位置模式 + 限速 pos 度 speed_limit rpm
void MG5010E_SpeedPos(MG5010E *motor, float pos, float speed_limit)
{
    uint8_t tx_data[8] = {0};
    int32_t pos_raw = (int32_t)(pos * motor->ratio * 100.0f);
    int32_t speed_raw = (int32_t)(speed_limit * motor->ratio * RPM_TO_DPS);

    motor->target_pos = pos;
    motor->speed_limit = speed_limit;

    tx_data[0] = eSpeedPosCommand;
    tx_data[2] = (uint8_t)((uint32_t)speed_raw & 0xFFU);
    tx_data[3] = (uint8_t)(((uint32_t)speed_raw >> 8) & 0xFFU);
    tx_data[4] = (uint8_t)((uint32_t)pos_raw & 0xFFU);
    tx_data[5] = (uint8_t)(((uint32_t)pos_raw >> 8) & 0xFFU);
    tx_data[6] = (uint8_t)(((uint32_t)pos_raw >> 16) & 0xFFU);
    tx_data[7] = (uint8_t)(((uint32_t)pos_raw >> 24) & 0xFFU);

    CAN_Transmit(motor->hfdcan, 0, 0x140U | motor->can_id, tx_data, 8);
}

//读位置，一收一发，回复在 MG5010E_GetPosition 里解
void MG5010E_ReadPosition(MG5010E *motor)
{
    uint8_t tx_data[8] = {0};

    tx_data[0] = eReadPositionCommand;
    CAN_Transmit(motor->hfdcan, 0, 0x140U | motor->can_id, tx_data, 8);
}

//设当前角度为原点，发完要断电重上电才生效
void MG5010E_SetOrigin(MG5010E *motor)
{
    uint8_t tx_data[8] = {0};

    tx_data[0] = eSetOriginCommand;
    CAN_Transmit(motor->hfdcan, 0, 0x140U | motor->can_id, tx_data, 8);
}

void MG5010E_Stop(MG5010E *motor)
{
    MG5010E_Speed(motor, 0.0f, 0.0f);
}

//清电机错误标志，手册里的 0x9B
void MG5010E_ClearError(MG5010E *motor)
{
    uint8_t tx_data[8] = {0};

    tx_data[0] = eClearErrorCommand;
    CAN_Transmit(motor->hfdcan, 0, 0x140U | motor->can_id, tx_data, 8);
}

// 将电机切换到运行状态。清错命令不会改变 0x80/0x88 的运行状态。
void MG5010E_Enable(MG5010E *motor)
{
    uint8_t tx_data[8] = {0};

    tx_data[0] = eEnableCommand;
    CAN_Transmit(motor->hfdcan, 0, 0x140U | motor->can_id, tx_data, 8);
}

void MG5010E_GetPosition(MG5010E *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
    uint8_t u8[8];
    int64_t raw;

    if (motor == NULL || rx_header == NULL || data == NULL)
    {
        return;
    }
    if (rx_header->IdType != FDCAN_STANDARD_ID)
    {
        return;
    }
    if (rx_header->Identifier != (0x140U | motor->can_id))
    {
        return;
    }

    motor->last_tick = HAL_GetTick();//收到回执就刷新心跳，0xA4 的也算

    if (data[0] != eReadPositionCommand)
    {
        return;
    }

    for (uint8_t i = 0; i < 7; i++)
    {
        u8[i] = data[i + 1];
    }
    //第 8 字节只有最高位是符号
    u8[7] = (data[7] & 0x80U) ? 0xFFU : 0x00U;

    memcpy(&raw, u8, sizeof(raw));
    motor->pos = (float)raw / motor->ratio / 100.0f;
}

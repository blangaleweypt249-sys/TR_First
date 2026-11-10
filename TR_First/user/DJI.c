#include "DJI.h"
#include "bsp_can.h"
#include "WpLib.h"
#include <string.h>

// 静态函数
static void DJI_Send(DJI_Motor *motor, DJI_Mode mode, float value);

// 命令帧基址，2006 用 0x300，3508 用 0x400
static uint32_t DJI_Command_Id(DJI_Type type)
{
    if (type == DJI_M3508)
    {
        return 0x400U;
    }
    return 0x300U;
}

// 反馈帧基址比命令帧高 0x90
static uint32_t DJI_Feedback_Id(DJI_Type type)
{
    return DJI_Command_Id(type) + 0x90U;
}

void DJI_Init(DJI_Motor *motor, FDCAN_HandleTypeDef *hfdcan, DJI_Type type, uint8_t board_id, uint8_t motor_id)
{
    memset(motor, 0, sizeof(*motor));
    motor->hfdcan = hfdcan;
    motor->type = type;
    motor->can_id = DJI_Command_Id(type);
    motor->board_id = board_id;
    motor->motor_id = motor_id;
    motor->mode = DJI_STOP;
}

void DJI_Speed(DJI_Motor *motor, float speed)
{
    DJI_Send(motor, DJI_SPEED, speed);
}

void DJI_Position(DJI_Motor *motor, float position)
{
    DJI_Send(motor, DJI_POSITION, position);
}

void DJI_Stop(DJI_Motor *motor)
{
    DJI_Send(motor, DJI_STOP, 0.0f);
}

void DJI_ToZero(DJI_Motor *motor)
{
    DJI_Send(motor, DJI_to_zero, 0.0f);
}

void DJI_SignZero(DJI_Motor *motor)
{
    DJI_Send(motor, DJI_sign_zero, 0.0f);
}

// 命令帧：ID = 基址 | 命令<<4 | 板号
// data[0]槽位  data[1..4]float  空三个字节
static void DJI_Send(DJI_Motor *motor, DJI_Mode mode, float value)
{
    float_u8 packet_data;
    uint8_t data[8] = {0};

    motor->mode = mode;
    if (mode == DJI_SPEED)
    {
        motor->target_speed = value;
    }
    else if (mode == DJI_POSITION)
    {
        motor->target_position = value;
    }

    packet_data.f = value;
    data[0] = motor->motor_id;
    for (uint8_t i = 0; i < 4; i++)
    {
        data[1 + i] = packet_data.u8[i];
    }

    CAN_Transmit(motor->hfdcan, 0, motor->can_id | ((uint32_t)mode << 4) | motor->board_id, data, 8);
}

void DJI_Analysisdata(DJI_Motor *motor, FDCAN_RxHeaderTypeDef *rx_header, uint8_t *data)
{
    if (motor == NULL || rx_header == NULL || data == NULL)
    {
        return;
    }
    if (rx_header->IdType != FDCAN_STANDARD_ID)
    {
        return;
    }
    if (rx_header->Identifier != (DJI_Feedback_Id(motor->type) + motor->board_id))
    {
        return;
    }
    if (rx_header->DataLength != FDCAN_DLC_BYTES_8)
    {
        return;
    }
    // 同一块板带多个电机时，靠槽位区分
    if (data[0] != motor->motor_id)
    {
        return;
    }

    memcpy(&motor->Feedback_angle, &data[1], sizeof(motor->Feedback_angle));
    motor->Feedback_speed = (float)(int16_t)(((uint16_t)data[5] << 8) | data[6]);
    motor->status = data[7];

    if (motor->num[0] >= 1000)
    {
        motor->num[0] = 0;
    }
    else
    {
        motor->num[0]++;
    }
}

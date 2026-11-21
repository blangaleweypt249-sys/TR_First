#include "DJI.h"
#include "bsp_can.h"
#include <math.h>
#include <string.h>

static int16_t DJI_Limit_Current(DJI_Motor *motor, float current);
static float DJI_Limit_Speed(DJI_Motor *motor, float speed);

// 4 个槽的电流，发 0x200 时一帧全带走
int16_t dji_current[4];

void DJI_Init(DJI_Motor *motor, CAN_HandleTypeDef *hcan, uint8_t board_id, uint8_t motor_id)
{
    memset(motor, 0, sizeof(*motor));

    motor->hcan = hcan;
    motor->board_id = board_id;
    motor->motor_id = motor_id;
    motor->mode = DJI_STOP;

#if BOARD_TYPE == BOARD_HELM
    motor->ratio = HELM_RATIO;
    motor->zero_offset = HELM_OFFSET_DEG;
    motor->speed_max = 500.0f;
    motor->current_max = 10000.0f;
    pid_init(&motor->speed_pid, 0.0f, 15.0f, 0.1f, 8.0f, 10000.0f, 500.0f, 0.5f);
    pid_init(&motor->position_pid, 0.0f, 35.0f, 1.0f, 270.0f, 380.0f, 0.30f, 0.5f);
#else
    motor->ratio = DJI_RATIO;
#if DJI_MODEL == DJI_MODEL_3508
    motor->speed_max = 480.0f;
    motor->current_max = 16384.0f;
    pid_init(&motor->speed_pid, 0.0f, 12.0f, 0.0f, 8.0f, 16384.0f, 800.0f, 0.2f);
    pid_init(&motor->position_pid, 0.0f, 6.0f, 0.0f, 50.0f, 160.0f, 20.0f, 1.0f);
#else
    motor->speed_max = 500.0f;
    motor->current_max = 10000.0f;
    pid_init(&motor->speed_pid, 0.0f, 15.0f, 0.5f, 10.0f, 10000.0f, 500.0f, 0.0f);
    pid_init(&motor->position_pid, 0.0f, 7.0f, 0.00f, 40.0f, 180.0f, 0.30f, 0.5f);
#endif
#endif
}

// 读电机反馈，只记数据，控制放到 1kHz 定时器里
void DJI_Receive(DJI_Motor *motor, CAN_RxHeaderTypeDef *header, uint8_t data[8])
{
    int32_t delta;

    if (motor == NULL || header == NULL || data == NULL)
    {
        return;
    }
    if (header->IDE != CAN_ID_STD || header->StdId != (0x200U + motor->motor_id))
    {
        return;
    }

    motor->encoder = ((uint16_t)data[0] << 8) | data[1];
    motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->torque = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->online = 1U;
    motor->status |= DJI_ONLINE;

    if (motor->first_feedback == 0U)
    {
        motor->last_encoder = motor->encoder;
        motor->first_feedback = 1U;
    }
    else
    {
        delta = (int32_t)motor->encoder - (int32_t)motor->last_encoder;
        // 跨圈处理，编码器一圈 8192
        if (delta > 4096)
        {
            delta -= 8192;
        }
        else if (delta < -4096)
        {
            delta += 8192;
        }
        motor->total_encoder += delta;
        motor->last_encoder = motor->encoder;
    }

    motor->angle = (float)motor->total_encoder * 360.0f / (8192.0f * motor->ratio);
}

// 主控下来的命令，只存目标
void DJI_SetCommand(DJI_Motor *motor, DJI_Mode mode, float target)
{
    if (motor == NULL || mode > DJI_CURRENT)
    {
        return;
    }

    motor->last_command_tick = HAL_GetTick();
    motor->status &= (uint8_t)(~DJI_TIMEOUT);

    if (motor->mode != mode)
    {
        pid_reset(&motor->speed_pid);
        pid_reset(&motor->position_pid);
    }
    motor->mode = mode;

    if (mode != DJI_TO_ZERO)
    {
        motor->to_zero_active = 0U;
    }

    switch (mode)
    {
    case DJI_SPEED:
        motor->target_speed = target;
        break;

    case DJI_POSITION_MODE:
        motor->target_position = target;
        break;

    case DJI_CURRENT:
        motor->target_current = (int16_t)target;
        break;

    case DJI_TO_ZERO:
        if (motor->to_zero_active == 0U)
        {
            motor->target_position = motor->zero_offset;
            motor->to_zero_active = 1U;
            motor->stead_flag = 0U;
            motor->status &= (uint8_t)(~DJI_ZEROED);
        }
        break;

    case DJI_SIGN_ZERO:
        DJI_SignZero(motor);
        motor->mode = DJI_SPEED;
        motor->target_speed = 0.0f;
        break;

    default:
        break;
    }
}

// 一个控制周期算一次，1kHz 由 TIM1 调用
void DJI_Control(DJI_Motor *motor)
{
    float speed_target;
    float current;

    if (motor == NULL)
    {
        return;
    }

    //超时时间，1s 没收到命令就停
    if ((HAL_GetTick() - motor->last_command_tick) > 1000U)
    {
        motor->status |= DJI_TIMEOUT;
        DJI_Stop(motor);
        return;
    }

    if (motor->online == 0U || motor->mode == DJI_STOP)
    {
        DJI_SetCurrent(motor, 0);
        return;
    }

    switch (motor->mode)
    {
    case DJI_CURRENT:
        DJI_SetCurrent(motor, motor->target_current);
        break;

    case DJI_SPEED:
        speed_target = DJI_Limit_Speed(motor, motor->target_speed);
        current = calc_pid(&motor->speed_pid, (float)motor->speed_rpm, speed_target * motor->ratio);
        DJI_SetCurrent(motor, (int16_t)current);
        break;

    case DJI_POSITION_MODE:
        speed_target = calc_pid(&motor->position_pid, motor->angle, motor->target_position);
        speed_target = DJI_Limit_Speed(motor, speed_target);
        current = calc_pid(&motor->speed_pid, (float)motor->speed_rpm, speed_target * motor->ratio);
        DJI_SetCurrent(motor, (int16_t)current);
        break;

    case DJI_TO_ZERO:
        if (motor->to_zero_active == 0U)
        {
            motor->target_position = 0.0f;
        }
        else
        {
            // 差 4 度内开始数，200 次（1kHz，约 200ms）数满才标零
            if (fabsf(motor->target_position - motor->angle) < 4.0f)
            {
                if (motor->stead_flag < 200U)
                {
                    motor->stead_flag++;
                }
            }
            else
            {
                motor->stead_flag = 0U;
            }

            // 一轮只标一次，等主控发新动作
            if (motor->stead_flag >= 200U && (motor->status & DJI_ZEROED) == 0U)
            {
                DJI_SignZero(motor);
                break;
            }
        }
        speed_target = calc_pid(&motor->position_pid, motor->angle, motor->target_position);
        speed_target = DJI_Limit_Speed(motor, speed_target);
        current = calc_pid(&motor->speed_pid, (float)motor->speed_rpm, speed_target * motor->ratio);
        DJI_SetCurrent(motor, (int16_t)current);
        break;

    case DJI_SIGN_ZERO:
        DJI_SignZero(motor);
        motor->mode = DJI_SPEED;
        motor->target_speed = 0.0f;
        break;

    default:
        DJI_SetCurrent(motor, 0);
        break;
    }
}

// 4 个槽合成一帧 0x200
void DJI_Transmit(CAN_HandleTypeDef *hcan)
{
    uint8_t data[8];

    if (hcan == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < 4U; i++)
    {
        data[i * 2U] = (uint8_t)((uint16_t)dji_current[i] >> 8);
        data[i * 2U + 1U] = (uint8_t)((uint16_t)dji_current[i] & 0xFFU);
    }

    CAN_Transmit(hcan, 0, 0x200U, data, 8);
}

// 反馈给主控：槽位 + 角度 + 转速 + 状态
void DJI_Back_To_Master(DJI_Motor *motor, CAN_HandleTypeDef *hcan)
{
    uint8_t data[8] = {0};
    float angle;
    int16_t speed;

    if (motor == NULL || hcan == NULL)
    {
        return;
    }

    angle = motor->angle;
    speed = motor->speed_rpm;

    data[0] = motor->motor_id;
    memcpy(&data[1], &angle, sizeof(angle));
    data[5] = (uint8_t)((uint16_t)speed >> 8);
    data[6] = (uint8_t)((uint16_t)speed & 0xFFU);
    data[7] = motor->status;

    CAN_Transmit(hcan, 0, DJI_FEEDBACK_BASE + motor->board_id, data, 8);
}

void DJI_SetCurrent(DJI_Motor *motor, int16_t current)
{
    if (motor == NULL || motor->motor_id < 1U || motor->motor_id > 4U)
    {
        return;
    }

    if (motor->online == 0U)
    {
        current = 0;
    }

    dji_current[motor->motor_id - 1U] = DJI_Limit_Current(motor, (float)current);
    motor->output_current = dji_current[motor->motor_id - 1U];
}

void DJI_SignZero(DJI_Motor *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->total_encoder = 0;
    motor->last_encoder = motor->encoder;
    motor->angle = 0.0f;
    motor->target_position = 0.0f;
    // 保持 to_zero_active：主控会连续发回零，到 0 点后不要再启动一次
    motor->status |= DJI_ZEROED;

    pid_reset(&motor->speed_pid);
    pid_reset(&motor->position_pid);
    DJI_SetCurrent(motor, 0);
}

void DJI_Stop(DJI_Motor *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->mode = DJI_STOP;
    motor->to_zero_active = 0U;
    DJI_SetCurrent(motor, 0);

    pid_reset(&motor->speed_pid);
    pid_reset(&motor->position_pid);
}

static int16_t DJI_Limit_Current(DJI_Motor *motor, float current)
{
    if (current > motor->current_max)
    {
        current = motor->current_max;
    }
    else if (current < -motor->current_max)
    {
        current = -motor->current_max;
    }
    return (int16_t)current;
}

static float DJI_Limit_Speed(DJI_Motor *motor, float speed)
{
    if (speed > motor->speed_max)
    {
        speed = motor->speed_max;
    }
    else if (speed < -motor->speed_max)
    {
        speed = -motor->speed_max;
    }
    return speed;
}

#include "RS05.h"

#define P_MIN -12.57f
#define P_MAX 12.57f

#define V_MIN -50.0f
#define V_MAX 50.0f

#define KP_MIN 0.0f
#define KP_MAX 500.0f

#define KD_MIN 0.0f
#define KD_MAX 5.0f

#define T_MIN -5.5f
#define T_MAX 5.5f

uint16_t float_to_uint16(float value, float min, float max)
{
    if (value < min)
        value = min;
    if (value > max)
        value = max;
    return (uint16_t)((value - min) / (max - min) * 65535.0f);
}

uint32_t float_to_uint32(float value, float min, float max)
{
    Un_RS05 converter = {0};

    if (value < min)
        value = min;
    if (value > max)
        value = max;

    converter.flo[0] = value;
    return converter.u32[0];
}

float uint32_to_float(uint32_t value, float min, float max)
{
    Un_RS05 converter = {0};

    converter.u32[0] = value;
    if (converter.flo[0] < min)
        converter.flo[0] = min;
    if (converter.flo[0] > max)
        converter.flo[0] = max;

    return converter.flo[0];
}

float uint16_to_float(uint16_t value, float min, float max)
{
    return (float)value / 65535.0f * (max - min) + min;
}

static inline float clampf(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

int RS05_Init(RS05 *Motor, bool zero_flag, FDCAN_HandleTypeDef *hcan, uint16_t motor_id, run_mode mode)
{
    Motor->hcan = *hcan;
    Motor->motor_id = motor_id;
    Motor->mit.init_flag = false;
    Motor->mit.last_cmd_position = 0.0f;
    Interpolation_Init(&Motor->mit.position_interpolation);
    Motor->mit.position_interpolation.last_update_ms = HAL_GetTick();
    if (zero_flag == 1)
    {
        RS05_SetZero(Motor);
        osDelay(10);
    }
    for (uint8_t retry = 0; retry < 10U; retry++)//防止初始化失败
    {
        RS05_SetMode(Motor, mode);
        osDelay(10);

        RS05_Enable(Motor);
        osDelay(10);

        if (Motor->mit.init_flag)
        {
            return true;
        }
    }

    return false;
}

void RS05_Enable(RS05 *rs05)
{
    uint8_t motor_enalbe[8] = {0};
    rs05->mode = 0x3;
    rs05->mst_id = 0x00;
    rs05->id = ((uint32_t)rs05->mode << 24) + ((uint32_t)rs05->mst_id << 8) + ((uint32_t)rs05->motor_id);
    CAN_Transmit(&rs05->hcan, 1, rs05->id, motor_enalbe, 8);
}

void RS05_Stop(RS05 *rs05)
{
    uint8_t motor_disable[8] = {0};
    rs05->mode = 0x4;
    rs05->mst_id = 0x00;
    rs05->id = ((uint32_t)rs05->mode << 24) + ((uint32_t)rs05->mst_id << 8) + ((uint32_t)rs05->motor_id);
    CAN_Transmit(&rs05->hcan, 1, rs05->id, motor_disable, 8);
}

void RS05_SetZero(RS05 *rs05)
{
    uint8_t motor_set_zero[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    rs05->mode = 0x6;
    rs05->mst_id = 0x00;
    rs05->id = ((uint32_t)rs05->mode << 24) + ((uint32_t)rs05->mst_id << 8) + ((uint32_t)rs05->motor_id);
    CAN_Transmit(&rs05->hcan, 1, rs05->id, motor_set_zero, 8);
}

void RS05_WriteParameter(RS05 *rs05, uint16_t index, uint32_t data)
{
    uint8_t motor_write_parameter[8];
    rs05->mode = 0x12;
    rs05->mst_id = 0x00;
    motor_write_parameter[0] = index & 0xFF;
    motor_write_parameter[1] = (index >> 8) & 0xFF;
    motor_write_parameter[2] = 0;
    motor_write_parameter[3] = 0;
    motor_write_parameter[4] = data & 0xFF;
    motor_write_parameter[5] = (data >> 8) & 0xFF;
    motor_write_parameter[6] = (data >> 16) & 0xFF;
    motor_write_parameter[7] = (data >> 24) & 0xFF;
    rs05->id = ((uint32_t)rs05->mode << 24) + ((uint32_t)rs05->mst_id << 8) + ((uint32_t)rs05->motor_id);
    CAN_Transmit(&rs05->hcan, 1, rs05->id, motor_write_parameter, 8);
}

void RS05_SaveParameter(RS05*rs05)
{
    uint8_t motor_save_parameter[8]={1,2,3,4,5,6,7,8};
    rs05->mode = 0x16;
    rs05->mst_id = 0x00;
    rs05->id = ((uint32_t)rs05->mode << 24) + ((uint32_t)rs05->mst_id << 8) + ((uint32_t)rs05->motor_id);
    CAN_Transmit(&rs05->hcan, 1, rs05->id, motor_save_parameter, 8);
}

void RS05_SetMode(RS05 *rs05, run_mode mode)
{
    RS05_WriteParameter(rs05, 0x7005, (uint32_t)mode);
}

void RS05_SetActiveReporting(RS05 *rs05)
{
    uint8_t motor_active_reporting[7] = {1, 2, 3, 4, 5, 6, 1};
    rs05->mode = 0x18;
    rs05->mst_id = 0x00;
    rs05->id = ((uint32_t)rs05->mode << 24) + ((uint32_t)rs05->mst_id << 8) + ((uint32_t)rs05->motor_id);
    CAN_Transmit(&rs05->hcan, 1, rs05->id, motor_active_reporting, 7);
}

void RS05_Mit(RS05 *rs05, float torque, float position, float speed,
              float kp, float kd, uint8_t interpolation_enable,
              uint32_t run_time_ms)
{
    uint8_t mit_data[8];

    //尚未收到位置反馈时，禁止从默认的 0 位置开始插值。
    if ((interpolation_enable != 0U) && !rs05->mit.init_flag)
        return;

    position = clampf(position, P_MIN, P_MAX);
    rs05->mit.target_torque = clampf(torque, T_MIN, T_MAX);
    rs05->mit.target_pos = Interpolation_PositionCommand(
        &rs05->mit.position_interpolation,
        rs05->mit.last_cmd_position,
        position,
        interpolation_enable,
        run_time_ms);
    rs05->mit.target_speed = speed = clampf(speed, V_MIN, V_MAX);
    rs05->mit.kp = kp = clampf(kp, KP_MIN, KP_MAX);
    rs05->mit.kd = kd = clampf(kd, KD_MIN, KD_MAX);
    rs05->mit.pos_out = float_to_uint16(rs05->mit.target_pos, P_MIN, P_MAX);
    rs05->mit.speed_out = float_to_uint16(rs05->mit.target_speed, V_MIN, V_MAX);
    rs05->mit.torque_out = float_to_uint16(rs05->mit.target_torque, T_MIN, T_MAX);
    rs05->mit.kp_out = float_to_uint16(rs05->mit.kp, KP_MIN, KP_MAX);
    rs05->mit.kd_out = float_to_uint16(rs05->mit.kd, KD_MIN, KD_MAX);
    rs05->mode = 0x1;
    rs05->mst_id = 0x00;
    rs05->id = ((uint32_t)rs05->mode << 24) + ((uint32_t)rs05->mit.torque_out << 8) + ((uint32_t)rs05->motor_id);
    mit_data[0] = (rs05->mit.pos_out >> 8) & 0xFF;
    mit_data[1] = rs05->mit.pos_out & 0xFF;
    mit_data[2] = (rs05->mit.speed_out >> 8) & 0xFF;
    mit_data[3] = rs05->mit.speed_out & 0xFF;
    mit_data[4] = (rs05->mit.kp_out >> 8) & 0xFF;
    mit_data[5] = rs05->mit.kp_out & 0xFF;
    mit_data[6] = (rs05->mit.kd_out >> 8) & 0xFF;
    mit_data[7] = rs05->mit.kd_out & 0xFF;
    CAN_Transmit(&rs05->hcan, 1, rs05->id, mit_data, 8);
    if (rs05->mit.init_flag)
    {
        rs05->mit.last_cmd_position = rs05->mit.target_pos;
        rs05->mit.last_cmd_speed = rs05->mit.target_speed;
        rs05->mit.last_cmd_torque = rs05->mit.target_torque;
    }
}

void RS05_Current(RS05 *rs05, float current)
{
    uint32_t current_u = float_to_uint32(current, -11, 11);
    RS05_WriteParameter(rs05, 0x7006, current_u);
}

void RS05_Speed(RS05 *rs05, float limit_cur, float acc_rad, float speed)
{
    uint32_t limit_cur_u = float_to_uint32(limit_cur, 0, 11);
    uint32_t acc_rad_u = float_to_uint32(acc_rad, 0, 100);
    uint32_t speed_u = float_to_uint32(speed, -50, 50);
    RS05_WriteParameter(rs05, 0x7018, limit_cur_u);
    RS05_WriteParameter(rs05, 0x7022, acc_rad_u);
    RS05_WriteParameter(rs05, 0x700A, speed_u);
}

void RS05_CSP(RS05 *rs05, float limit_spd, float loc_ref)
{
    uint32_t limit_spd_u = float_to_uint32(limit_spd, 0, 50);
    uint32_t loc_ref_u = float_to_uint32(loc_ref, -12.57, 12.57);
    RS05_WriteParameter(rs05, 0x7017, limit_spd_u);
    RS05_WriteParameter(rs05, 0x7016, (uint32_t)loc_ref_u);
}

void RS05_PP(RS05 *rs05, float vel_max, float acc_set, float loc_ref)
{
    uint32_t vel_max_u = float_to_uint32(vel_max, 0, 50);
    uint32_t acc_set_u = float_to_uint32(acc_set, 0, 100);
    uint32_t loc_ref_u = float_to_uint32(loc_ref, -12.57, 12.57);
    RS05_WriteParameter(rs05, 0x7024, (uint32_t)vel_max_u);
    RS05_WriteParameter(rs05, 0x7025, (uint32_t)acc_set_u);
    RS05_WriteParameter(rs05, 0x7016, (uint32_t)loc_ref_u);
}

void RS05_ReadData(RS05 *rs05, uint8_t back_data[8], FDCAN_RxHeaderTypeDef *rx_header)
{
    if ((rx_header->IdType == FDCAN_EXTENDED_ID) && (rx_header->DataLength == FDCAN_DLC_BYTES_8))
    {
        uint32_t comm_type = rx_header->Identifier >> 24;
        if (((comm_type == 0x2U) || (comm_type == 0x18U)) &&
            ((rx_header->Identifier >> 8 & 0xFFU) == rs05->motor_id))
        {
            uint8_t data[8];
            for (int i = 0; i < 8; i++)
            {
                data[i] = back_data[i];
            }
            rs05->feedback.pos_in = (data[0] << 8) | data[1];
            rs05->feedback.speed_in = (data[2] << 8) | data[3];
            rs05->feedback.torque_in = (data[4] << 8) | data[5];
            rs05->feedback.temp_in = (data[6] << 8) | data[7];
            rs05->feedback.pos_fed = uint16_to_float(rs05->feedback.pos_in, P_MIN, P_MAX);
            rs05->feedback.speed_fed = uint16_to_float(rs05->feedback.speed_in, V_MIN, V_MAX);
            rs05->feedback.torque_fed = uint16_to_float(rs05->feedback.torque_in, T_MIN, T_MAX);
            rs05->feedback.temp_fed = (float)rs05->feedback.temp_in / 10.0f;
            if (rs05->mit.init_flag == false)
            {
                rs05->mit.init_flag = true;
                rs05->mit.last_cmd_position = rs05->feedback.pos_fed;
                rs05->mit.last_cmd_speed = rs05->feedback.speed_fed;
                rs05->mit.last_cmd_torque = rs05->feedback.torque_fed;
                Interpolation_Init(&rs05->mit.position_interpolation);
                rs05->mit.position_interpolation.actual_pos =
                    rs05->feedback.pos_fed;
                rs05->mit.position_interpolation.last_update_ms = HAL_GetTick();
            }

            if (rs05->num[0] >= 1000)
                rs05->num[0] = 0;
            else
                rs05->num[0]++;
        }
    }
}

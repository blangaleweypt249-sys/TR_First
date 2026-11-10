#include "RS06.h"

#define P_MIN -12.57f
#define P_MAX 12.57f

#define V_MIN -50.0f
#define V_MAX 50.0f

#define KP_MIN 0.0f
#define KP_MAX 5000.0f

#define KD_MIN 0.0f
#define KD_MAX 100.0f

#define T_MIN -36.0f
#define T_MAX 36.0f

static inline float clampf(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

int RS06_Init(RS06 *Motor, bool zero_flag, FDCAN_HandleTypeDef *hcan, uint16_t motor_id, run_mode mode)
{
    Motor->hcan = *hcan;
    Motor->motor_id = motor_id;
    Motor->mit.init_flag = false;
    Motor->mit.last_cmd_position = 0.0f;
    Interpolation_Init(&Motor->mit.position_interpolation);
    Motor->mit.position_interpolation.last_update_ms = HAL_GetTick();
    if (zero_flag == 1)
    {
        RS06_SetZero(Motor);
        osDelay(10);
    }
    for (uint8_t retry = 0; retry < 10U; retry++)//防止初始化失败
    {
        RS06_SetMode(Motor, mode);
        osDelay(10);

        RS06_Enable(Motor);
        osDelay(10);

        if (Motor->mit.init_flag)
        {
            return true;
        }
    }

    return false;
}

void RS06_Enable(RS06 *rs06)
{
    uint8_t motor_enalbe[8] = {0};
    rs06->mode = 0x3;
    rs06->mst_id = 0x00;
    rs06->id = ((uint32_t)rs06->mode << 24) + ((uint32_t)rs06->mst_id << 8) + ((uint32_t)rs06->motor_id);
    CAN_Transmit(&rs06->hcan, 1, rs06->id, motor_enalbe, 8);
}

void RS06_Stop(RS06 *rs06)
{
    uint8_t motor_disable[8] = {0};
    rs06->mode = 0x4;
    rs06->mst_id = 0x00;
    rs06->id = ((uint32_t)rs06->mode << 24) + ((uint32_t)rs06->mst_id << 8) + ((uint32_t)rs06->motor_id);
    CAN_Transmit(&rs06->hcan, 1, rs06->id, motor_disable, 8);
}

void RS06_SetZero(RS06 *rs06)
{
    uint8_t motor_set_zero[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    rs06->mode = 0x6;
    rs06->mst_id = 0x00;
    rs06->id = ((uint32_t)rs06->mode << 24) + ((uint32_t)rs06->mst_id << 8) + ((uint32_t)rs06->motor_id);
    CAN_Transmit(&rs06->hcan, 1, rs06->id, motor_set_zero, 8);
}

void RS06_WriteParameter(RS06 *rs06, uint16_t index, uint32_t data)
{
    uint8_t motor_write_parameter[8];
    rs06->mode = 0x12;
    rs06->mst_id = 0x00;
    motor_write_parameter[0] = index & 0xFF;
    motor_write_parameter[1] = (index >> 8) & 0xFF;
    motor_write_parameter[2] = 0;
    motor_write_parameter[3] = 0;
    motor_write_parameter[4] = data & 0xFF;
    motor_write_parameter[5] = (data >> 8) & 0xFF;
    motor_write_parameter[6] = (data >> 16) & 0xFF;
    motor_write_parameter[7] = (data >> 24) & 0xFF;
    rs06->id = ((uint32_t)rs06->mode << 24) + ((uint32_t)rs06->mst_id << 8) + ((uint32_t)rs06->motor_id);
    CAN_Transmit(&rs06->hcan, 1, rs06->id, motor_write_parameter, 8);
}

void RS06_SaveParameter(RS06*rs06)
{
    uint8_t motor_save_parameter[8]={1,2,3,4,5,6,7,8};
    rs06->mode = 0x16;
    rs06->mst_id = 0x00;
    rs06->id = ((uint32_t)rs06->mode << 24) + ((uint32_t)rs06->mst_id << 8) + ((uint32_t)rs06->motor_id);
    CAN_Transmit(&rs06->hcan, 1, rs06->id, motor_save_parameter, 8);
}

void RS06_SetMode(RS06 *rs06, run_mode mode)
{
    RS06_WriteParameter(rs06, 0x7005, (uint32_t)mode);
}

void RS06_SetActiveReporting(RS06 *rs06)
{
    uint8_t motor_active_reporting[7] = {1, 2, 3, 4, 5, 6, 1};
    rs06->mode = 0x18;
    rs06->mst_id = 0x00;
    rs06->id = ((uint32_t)rs06->mode << 24) + ((uint32_t)rs06->mst_id << 8) + ((uint32_t)rs06->motor_id);
    CAN_Transmit(&rs06->hcan, 1, rs06->id, motor_active_reporting, 7);
}

void RS06_Mit(RS06 *rs06, float torque, float position, float speed,
              float kp, float kd, uint8_t interpolation_enable,
              uint32_t run_time_ms)
{
    uint8_t mit_data[8];

    //尚未收到位置反馈时，禁止从默认的 0 位置开始插值
    if ((interpolation_enable != 0U) && !rs06->mit.init_flag)
        return;

    position = clampf(position, P_MIN, P_MAX);
    rs06->mit.target_torque = clampf(torque, T_MIN, T_MAX);
    rs06->mit.target_pos = Interpolation_PositionCommand(
        &rs06->mit.position_interpolation,
        rs06->mit.last_cmd_position,
        position,
        interpolation_enable,
        run_time_ms);
    rs06->mit.target_speed = speed = clampf(speed, V_MIN, V_MAX);
    rs06->mit.kp = kp = clampf(kp, KP_MIN, KP_MAX);
    rs06->mit.kd = kd = clampf(kd, KD_MIN, KD_MAX);
    rs06->mit.pos_out = float_to_uint16(rs06->mit.target_pos, P_MIN, P_MAX);
    rs06->mit.speed_out = float_to_uint16(rs06->mit.target_speed, V_MIN, V_MAX);
    rs06->mit.torque_out = float_to_uint16(rs06->mit.target_torque, T_MIN, T_MAX);
    rs06->mit.kp_out = float_to_uint16(rs06->mit.kp, KP_MIN, KP_MAX);
    rs06->mit.kd_out = float_to_uint16(rs06->mit.kd, KD_MIN, KD_MAX);
    rs06->mode = 0x1;
    rs06->mst_id = 0x00;
    rs06->id = ((uint32_t)rs06->mode << 24) + ((uint32_t)rs06->mit.torque_out << 8) + ((uint32_t)rs06->motor_id);
    mit_data[0] = (rs06->mit.pos_out >> 8) & 0xFF;
    mit_data[1] = rs06->mit.pos_out & 0xFF;
    mit_data[2] = (rs06->mit.speed_out >> 8) & 0xFF;
    mit_data[3] = rs06->mit.speed_out & 0xFF;
    mit_data[4] = (rs06->mit.kp_out >> 8) & 0xFF;
    mit_data[5] = rs06->mit.kp_out & 0xFF;
    mit_data[6] = (rs06->mit.kd_out >> 8) & 0xFF;
    mit_data[7] = rs06->mit.kd_out & 0xFF;
    CAN_Transmit(&rs06->hcan, 1, rs06->id, mit_data, 8);
    if (rs06->mit.init_flag)
    {
        rs06->mit.last_cmd_position = rs06->mit.target_pos;
        rs06->mit.last_cmd_speed = rs06->mit.target_speed;
        rs06->mit.last_cmd_torque = rs06->mit.target_torque;
    }
}

void RS06_Current(RS06 *rs06, float current)
{
    uint32_t current_u = float_to_uint32(current, -57, 57);
    RS06_WriteParameter(rs06, 0x7006, current_u);
}

void RS06_Speed(RS06 *rs06, float limit_cur, float acc_rad, float speed)
{
    uint32_t limit_cur_u = float_to_uint32(limit_cur, 0, 57);
    uint32_t acc_rad_u = float_to_uint32(acc_rad, 0, 100);
    uint32_t speed_u = float_to_uint32(speed, -50, 50);
    RS06_WriteParameter(rs06, 0x7018, limit_cur_u);
    RS06_WriteParameter(rs06, 0x7022, acc_rad_u);
    RS06_WriteParameter(rs06, 0x700A, speed_u);
}

void RS06_CSP(RS06 *rs06, float limit_spd, float loc_ref)
{
    uint32_t limit_spd_u = float_to_uint32(limit_spd, 0, 50);
    uint32_t loc_ref_u = float_to_uint32(loc_ref, -12.57, 12.57);
    RS06_WriteParameter(rs06, 0x7017, limit_spd_u);
    RS06_WriteParameter(rs06, 0x7016, (uint32_t)loc_ref_u);
}

void RS06_PP(RS06 *rs06, float vel_max, float acc_set, float loc_ref)
{
    uint32_t vel_max_u = float_to_uint32(vel_max, 0, 50);
    uint32_t acc_set_u = float_to_uint32(acc_set, 0, 100);
    uint32_t loc_ref_u = float_to_uint32(loc_ref, -12.57, 12.57);
    RS06_WriteParameter(rs06, 0x7024, (uint32_t)vel_max_u);
    RS06_WriteParameter(rs06, 0x7025, (uint32_t)acc_set_u);
    RS06_WriteParameter(rs06, 0x7016, (uint32_t)loc_ref_u);
}

void RS06_ReadData(RS06 *rs06, uint8_t back_data[8], FDCAN_RxHeaderTypeDef *rx_header)
{
    if ((rx_header->IdType == FDCAN_EXTENDED_ID) && (rx_header->DataLength == FDCAN_DLC_BYTES_8))
    {
        uint32_t comm_type = rx_header->Identifier >> 24;
        if (((comm_type == 0x2U) || (comm_type == 0x18U)) &&
            ((rx_header->Identifier >> 8 & 0xFFU) == rs06->motor_id))
        {
            uint8_t data[8];
            for (int i = 0; i < 8; i++)
            {
                data[i] = back_data[i];
            }
            rs06->feedback.pos_in = (data[0] << 8) | data[1];
            rs06->feedback.speed_in = (data[2] << 8) | data[3];
            rs06->feedback.torque_in = (data[4] << 8) | data[5];
            rs06->feedback.temp_in = (data[6] << 8) | data[7];
            rs06->feedback.pos_fed = uint16_to_float(rs06->feedback.pos_in, P_MIN, P_MAX);
            rs06->feedback.speed_fed = uint16_to_float(rs06->feedback.speed_in, V_MIN, V_MAX);
            rs06->feedback.torque_fed = uint16_to_float(rs06->feedback.torque_in, T_MIN, T_MAX);
            rs06->feedback.temp_fed = (float)rs06->feedback.temp_in / 10.0f;
            if (rs06->mit.init_flag == false)
            {
                rs06->mit.init_flag = true;
                rs06->mit.last_cmd_position = rs06->feedback.pos_fed;
                rs06->mit.last_cmd_speed = rs06->feedback.speed_fed;
                rs06->mit.last_cmd_torque = rs06->feedback.torque_fed;
                Interpolation_Init(&rs06->mit.position_interpolation);
                rs06->mit.position_interpolation.actual_pos =
                    rs06->feedback.pos_fed;
                rs06->mit.position_interpolation.last_update_ms = HAL_GetTick();
            }

            if (rs06->num[0] >= 1000)
                rs06->num[0] = 0;
            else
                rs06->num[0]++;
        }
    }
}



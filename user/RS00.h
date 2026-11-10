#ifndef RS00_H
#define RS00_H

#include "vesc.h"
#include "mymain.h"
#include "RS05.h"
#include "Interpolation.h"

typedef struct
{
    // float
    float target_torque;
    float target_pos;
    float target_speed;

    float kp;
    float kd;

    // 转换为uint16发送
    uint16_t pos_out;
    uint16_t speed_out;
    uint16_t torque_out;
    uint16_t kp_out;
    uint16_t kd_out;

    float last_cmd_position;
    float last_cmd_speed;
    float last_cmd_torque;
    Interpolation_t position_interpolation;
    bool init_flag;
} RS00_MIT;

typedef struct
{
    uint16_t pos_in;
    uint16_t speed_in;
    uint16_t torque_in;
    uint16_t temp_in;

    float pos_fed;
    float speed_fed;
    float torque_fed;
    float temp_fed;
} RS00_feedback;

typedef struct
{
    uint8_t mode;
    uint16_t motor_id;
    uint8_t mst_id;
    uint32_t id;
    RS00_MIT mit;
    RS00_feedback feedback;
    FDCAN_HandleTypeDef hcan;
    int num[3];
} RS00;

int RS00_Init(RS00 *Motor, bool zero_flag, FDCAN_HandleTypeDef *hcan, uint16_t motor_id, run_mode mode);
void RS00_Enable(RS00 *rs00);
void RS00_Stop(RS00 *rs00);
void RS00_SetZero(RS00 *rs00);
void RS00_WriteParameter(RS00 *rs00, uint16_t index, uint32_t data);
void RS00_SaveParameter(RS00*rs00);
void RS00_SetMode(RS00 *rs00, run_mode mode);
void RS00_SetActiveReporting(RS00 *rs00);
/* run_time_ms 单位为毫秒；插值开启时需在控制循环中周期调用。 */
void RS00_Mit(RS00 *rs00, float torque, float position, float speed,
              float kp, float kd, uint8_t interpolation_enable,
              uint32_t run_time_ms);
void RS00_Current(RS00 *rs00, float current);
void RS00_Speed(RS00 *rs00, float limit_cur, float acc_rad, float speed);
void RS00_CSP(RS00 *rs00, float limit_spd, float loc_ref);
void RS00_PP(RS00 *rs00, float vel_max, float acc_set, float loc_ref);
void RS00_ReadData(RS00 *rs00, uint8_t back_data[8], FDCAN_RxHeaderTypeDef *rx_header);

#endif



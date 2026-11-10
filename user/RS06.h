#ifndef RS06_H
#define RS06_H

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
} RS06_MIT;

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
} RS06_feedback;

typedef struct
{
    uint8_t mode;
    uint16_t motor_id;
    uint8_t mst_id;
    uint32_t id;
    RS06_MIT mit;
    RS06_feedback feedback;
    FDCAN_HandleTypeDef hcan;
    int num[3];
} RS06;

int RS06_Init(RS06 *Motor, bool zero_flag, FDCAN_HandleTypeDef *hcan, uint16_t motor_id, run_mode mode);
void RS06_Enable(RS06 *rs06);
void RS06_Stop(RS06 *rs06);
void RS06_SetZero(RS06 *rs06);
void RS06_WriteParameter(RS06 *rs06, uint16_t index, uint32_t data);
void RS06_SaveParameter(RS06*rs06);
void RS06_SetMode(RS06 *rs06, run_mode mode);
void RS06_SetActiveReporting(RS06 *rs06);
/* run_time_ms 单位为毫秒；插值开启时需在控制循环中周期调用。 */
void RS06_Mit(RS06 *rs06, float torque, float position, float speed,
              float kp, float kd, uint8_t interpolation_enable,
              uint32_t run_time_ms);
void RS06_Current(RS06 *rs06, float current);
void RS06_Speed(RS06 *rs06, float limit_cur, float acc_rad, float speed);
void RS06_CSP(RS06 *rs06, float limit_spd, float loc_ref);
void RS06_PP(RS06 *rs06, float vel_max, float acc_set, float loc_ref);
void RS06_ReadData(RS06 *rs06, uint8_t back_data[8], FDCAN_RxHeaderTypeDef *rx_header);

#endif



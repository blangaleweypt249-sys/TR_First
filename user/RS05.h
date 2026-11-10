#ifndef RS05_H
#define RS05_H

#include "vesc.h"
#include "mymain.h"
#include "Interpolation.h"

typedef enum
{
    mit,
    pp, // 位置(无法标零)
    speed,
    current,
    csp = 5, // 位置
} run_mode;

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
} RS05_MIT;

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
} RS05_feedback;

typedef struct
{
    uint8_t mode;
    uint16_t motor_id;
    uint8_t mst_id;
    uint32_t id;
    RS05_MIT mit;
    RS05_feedback feedback;
    FDCAN_HandleTypeDef hcan;
    int num[3];
} RS05;

typedef union
{
    uint32_t u32[1];
    float flo[1];
    uint16_t u16[2];
    uint8_t u8[4];
} Un_RS05;

uint16_t float_to_uint16(float value, float min, float max);
uint32_t float_to_uint32(float value, float min, float max);
float uint32_to_float(uint32_t value, float min, float max);
float uint16_to_float(uint16_t value, float min, float max);
int RS05_Init(RS05 *Motor, bool zero_flag, FDCAN_HandleTypeDef *hcan, uint16_t motor_id, run_mode mode);
void RS05_Enable(RS05 *rs05);
void RS05_Stop(RS05 *rs05);
void RS05_SetZero(RS05 *rs05);
void RS05_WriteParameter(RS05 *rs05, uint16_t index, uint32_t data);
void RS05_SaveParameter(RS05*rs05);
void RS05_SetMode(RS05 *rs05, run_mode mode);
void RS05_SetActiveReporting(RS05 *rs05);
/* run_time_ms 单位为毫秒；插值开启时需在控制循环中周期调用。 */
void RS05_Mit(RS05 *rs05, float torque, float position, float speed,
              float kp, float kd, uint8_t interpolation_enable,
              uint32_t run_time_ms);
void RS05_Current(RS05 *rs05, float current);
void RS05_Speed(RS05 *rs05, float limit_cur, float acc_rad, float speed);
void RS05_CSP(RS05 *rs05, float limit_spd, float loc_ref);
void RS05_PP(RS05 *rs05, float vel_max, float acc_set, float loc_ref);
void RS05_ReadData(RS05 *rs05, uint8_t back_data[8], FDCAN_RxHeaderTypeDef *rx_header);

#endif

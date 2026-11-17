#ifndef GYRO_100A_H
#define GYRO_100A_H

#include "WpLib.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    // 原始角速度
    int32_t orginal_speed;
    // 原始温度
    int16_t orginal_temp;

    float basic_speed;

    float gyro_speed;
    float gyro_temp;

    int num[3];
} ImuData_t;

extern ImuData_t imu_100A;
extern volatile float yaw;

bool Imu_ReadData(const uint8_t data[8]);
void Imu_Calc(void);
void Imu_ClearYaw(void);
void SendRad(void);

#endif

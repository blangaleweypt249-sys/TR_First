#ifndef __IMU_100A_H__
#define __IMU_100A_H__
#include <stdint.h>

typedef union 
{
   float flo;
   uint8_t u8[4];
}flo_to_u8;

extern float Imu_angle;
extern int imu_num[3];
void Imu_Analysis(uint8_t *data);

#endif

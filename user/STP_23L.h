#ifndef __STP_23L_H__
#define __STP_23L_H__

#include "main.h"
#include "WpLib.h"

typedef struct
{
    int16_t distance;   // 测量距离
    uint16_t noise;     // 环境噪声
    uint32_t peak;      // 接收强度信息
    uint8_t confidence; // 置信度
    uint32_t intg;      // 积分次数
    int16_t reftof;     // 温度表征值
} STP_Data;

void STP_23L_ReadData(uint8_t *data, float *stp_distance);

#endif
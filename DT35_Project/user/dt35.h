#ifndef __DT35_H
#define __DT35_H

#include "main.h"
#include "i2c.h"

//两个激光的I2C地址：左 0b100000，前 0b100001
#define DT35_ADDR_L 0x20U
#define DT35_ADDR_F 0x21U

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;           //7位地址
    uint8_t online;         //1 上电握手成功
    uint16_t distance_mm;   //最近一次读到的距离
}DT35;

void DT35_Init(DT35 *dt35, I2C_HandleTypeDef *hi2c, uint8_t addr);
uint8_t DT35_Read(DT35 *dt35);   //读一次，成功返回1，距离在 dt35->distance_mm

#endif

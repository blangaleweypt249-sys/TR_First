#ifndef __DT35_H
#define __DT35_H

#include "main.h"
#include "i2c.h"

//两颗 INA226 的 7 位地址：左 0b1000100 = 0x44，前 0b1000101 = 0x45
#define DT35_ADDR_L 0x44U
#define DT35_ADDR_F 0x45U

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t addr;           //7位地址
    uint8_t online;         //1 上电握手成功
    uint16_t distance;   //最近一次读到的距离
}DT35;

void DT35_Init(DT35 *dt35, I2C_HandleTypeDef *hi2c, uint8_t addr);
uint8_t DT35_Read(DT35 *dt35);   //读一次，成功返回1，距离在 dt35->distance

#endif

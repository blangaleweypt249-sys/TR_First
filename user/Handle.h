#ifndef __HANDLE_H__
#define __HANDLE_H__

#include "stdint.h"

typedef union
{
    uint16_t u16[4];
    uint8_t u8[8];
}U8_to_U16;

typedef struct 
{
  U8_to_U16 rocker;
  U8_to_U16 shoulder;
  uint8_t up_key[6];
  uint8_t left_key[8];
  uint8_t right_key[8];
  int num[3];
}HandleData;

void Handle_Analysis(uint8_t *data);
int16_t Rock_trans(uint16_t rock);

extern HandleData handle_data;
extern int16_t handle_value[4]; // 测试遥控数据

#endif
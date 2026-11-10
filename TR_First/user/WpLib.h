#ifndef __WPLIB_H__
#define __WPLIB_H__

#include "stdint.h"
#define PI 3.14159265358979323846f

typedef union
{
    float f;
    uint8_t u8[4];
} float_u8;

typedef union
{
    uint32_t u32;
    uint8_t u8[4];
} u32_u8;

typedef union
{
    uint16_t u16;
    uint8_t u8[2];
} u16_u8;

typedef union
{
    int32_t i32;
    uint8_t u8[4];
} i32_u8;

typedef union
{
    int16_t i16;
    uint8_t u8[2];
} i16_u8;

typedef union
{
    uint32_t u32;
    int32_t i32;
} u32_i32;

typedef union
{
    uint16_t u16;
    int16_t i16;
} u16_i16;

uint8_t My_CrcSum(uint8_t *data, uint8_t len);
void RadToAngle(float rad, float *angle);
void AngleToRad(float angle, float *rad);
#endif

#ifndef __WPLIB_H__
#define __WPLIB_H__

#include "stdbool.h"
#include "stdint.h"
#include "math.h"
#include "string.h"
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

typedef struct {
    uint8_t *buffer;
    uint16_t size;
    volatile uint16_t readIndex;
    volatile uint16_t writeIndex;
} RingBuffer_t;

uint8_t My_CrcSum(uint8_t *data, uint8_t len);
float wrap_angle(float angle);
void RadToAngle(float rad, float *angle);
void AngleToRad(float angle, float *rad);

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buf, uint16_t size);
void AddReadIndex(RingBuffer_t *rb, uint16_t length);
uint8_t ReadBuffer(RingBuffer_t *rb, uint16_t i);
uint16_t GetBufferLength(RingBuffer_t *rb);
uint16_t GetRemain(RingBuffer_t *rb);
uint8_t WriteBuffer(RingBuffer_t *rb, uint8_t *data, uint16_t length);
void RingBuffer_Clear(RingBuffer_t *rb);
uint8_t RingBuffer_Pop(RingBuffer_t *rb, uint8_t *data);
uint8_t FindTrueData(RingBuffer_t *rb, uint8_t *data);
#endif

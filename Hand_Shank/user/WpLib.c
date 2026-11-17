#include "WpLib.h"

// 指令的最小长度
#define MIN_LENGTH 8U

#define PI 3.14159265358979323846f

uint8_t My_CrcSum(uint8_t *data, uint8_t len) // 注意此处从0开始计算
{
    uint16_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc += data[i];
    }
    crc &= 0xFF;
    return (uint8_t)crc;
}

float wrap_angle(float angle)
{
    if (angle > 180.0f || angle < -180.0f)
    {
        angle = fmodf(angle, 360.0f);
        if (angle > 180.0f)
        {
            angle -= 360.0f;
        }
        else if (angle < -180.0f)
        {
            angle += 360.0f;
        }
    }
    return angle;
}

void RadToAngle(float rad, float *angle)
{
    *angle = rad * 180.0f / PI;
}

void AngleToRad(float angle, float *rad)
{
    *rad = angle * PI / 180.0f;
}

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buf, uint16_t size)
{
    rb->buffer = buf;
    rb->size = size;
    rb->readIndex = 0U;
    rb->writeIndex = 0U;
}

void AddReadIndex(RingBuffer_t *rb, uint16_t length)
{
    uint16_t newIndex = (uint16_t)(rb->readIndex + length);
    rb->readIndex = newIndex % rb->size;
}

uint8_t ReadBuffer(RingBuffer_t *rb, uint16_t i)
{
    uint16_t index = i % rb->size;
    return rb->buffer[index];
}

uint16_t GetBufferLength(RingBuffer_t *rb)
{
    uint16_t readIndex = rb->readIndex;
    uint16_t writeIndex = rb->writeIndex;

    if (writeIndex == readIndex)
    {
        return 0U;
    }

    if ((writeIndex + 1U == readIndex) || (writeIndex == rb->size - 1U && readIndex == 0U))
    {
        return rb->size;
    }

    if (readIndex <= writeIndex)                                     
    {
        return writeIndex - readIndex;
    }
    else
    {
        return rb->size - readIndex + writeIndex;
    }
}

uint16_t GetRemain(RingBuffer_t *rb)
{
    return rb->size - GetBufferLength(rb);
}

uint8_t WriteBuffer(RingBuffer_t *rb, uint8_t *data, uint16_t length)
{
    // 剩余空间不够写
    // GetRemain 包含预留位，因此相等时也不能写入
    if (GetRemain(rb) <= length)
    {
        return 0U; // 丢弃此帧
    }

    if (rb->writeIndex + length < rb->size)
    {
        memcpy(rb->buffer + rb->writeIndex, data, length);
        rb->writeIndex += length;
    }
    else
    {
        uint16_t firstLength = rb->size - rb->writeIndex;
        memcpy(rb->buffer + rb->writeIndex, data, firstLength);
        memcpy(rb->buffer, data + firstLength, length - firstLength);
        rb->writeIndex = length - firstLength;
    }
    return (uint8_t)length; 
}

void RingBuffer_Clear(RingBuffer_t *rb)
{
    rb->readIndex = 0U;
    rb->writeIndex = 0U;
}

// 取一个字节，队列空返回0
uint8_t RingBuffer_Pop(RingBuffer_t *rb, uint8_t *data)
{
    if (rb->readIndex == rb->writeIndex)
    {
        return 0U;
    }
    *data = rb->buffer[rb->readIndex];
    AddReadIndex(rb, 1U);
    return 1U;
}

uint8_t FindTrueData(RingBuffer_t *rb, uint8_t *data)
{
    uint8_t crc;

    while (1)
    {
        if (GetBufferLength(rb) < MIN_LENGTH)
        {
            return 0U;
        }
        if(ReadBuffer(rb, rb->readIndex) != 0x80U)
        {
            AddReadIndex(rb, 1U);
            continue;
        }
        crc = 0U;
        for(uint8_t i=1U; i<5U; i++)
        {
            crc ^= ReadBuffer(rb, rb->readIndex + i);
        }
        if(crc != ReadBuffer(rb, rb->readIndex + 5U))
        {
            AddReadIndex(rb, 1U);
            continue;
        }

        for(uint8_t i=0U; i<8U; i++)
        {
            data[i] = ReadBuffer(rb, rb->readIndex + i);
        }
        AddReadIndex(rb, 8U);
        return 1U;
    }
}

#include "my_main.h"
#include "i2c.h"
#include "usart.h"

DT35 dt35_l;   //左
DT35 dt35_f;   //前
DT35_U distance;

void My_Main_Init(void)
{
    DT35_Init(&dt35_l, &hi2c1, DT35_ADDR_L);
    DT35_Init(&dt35_f, &hi2c1, DT35_ADDR_F);
}

//20ms 一轮：读两个激光，从 USART1 发 0x35 + 前/左/后/右 + 校验和 + 0x53
void My_Main_Task(void)
{
    static uint32_t tick;
    static uint8_t frame[19];
    uint8_t sum = 0;

    if (HAL_GetTick() - tick < 20U)
    {
        return;
    }
    tick = HAL_GetTick();

    DT35_Read(&dt35_l);
    DT35_Read(&dt35_f);

    distance.flo[0] = (float)dt35_f.distance; //前
    distance.flo[1] = (float)dt35_l.distance; //左
    distance.flo[2] = 0.0f;                   //后，还没装
    distance.flo[3] = 0.0f;                   //右，还没装

    frame[0] = 0x35;
    for (uint8_t i = 0; i < 16; i++)
    {
        frame[1 + i] = distance.u8[i];
        sum += distance.u8[i];
    }
    frame[17] = sum;
    frame[18] = 0x53;

    HAL_UART_Transmit_DMA(&huart1, frame, sizeof(frame));
}

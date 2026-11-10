#include "Handle.h"

HandleData handle_data;
int16_t handle_value[4]; // 测试遥控数据

void Handle_Analysis(uint8_t *data)
{
    uint16_t crc = 0;
    if (data[0] == 0xAA && data[18] == 0xBB)
    {
        for (uint8_t i = 1; i <= 16; i++)
        {
            crc += data[i];
        }
        if ((crc & 0xFF) == data[17])
        {
            // 获取摇杆值
            for (uint8_t i = 0; i < 8; i++)
            {
                handle_data.rocker.u8[i] = data[i + 1]; // 8个u8转4个u16
            }
            handle_value[0] = -Rock_trans(handle_data.rocker.u16[0]);
            handle_value[1] = -Rock_trans(handle_data.rocker.u16[1]);
            // handle_value[2] = -Rock_trans(handle_data.rocker.u16[2]);   //暂时用不到右摇杆的y轴
            handle_value[3] = Rock_trans(handle_data.rocker.u16[3]);

            for (uint8_t i = 0; i < 4; i++)
            {
                handle_data.shoulder.u8[i] = data[i + 9];
            }

            // 上方的6个开关
            for (uint8_t i = 0; i < 6; i++)
            {
                handle_data.up_key[i] = data[13] >> i & 0x01;
            }

            // 左边的8个按键
            for (uint8_t i = 0; i < 8; i++)
            {
                handle_data.left_key[i] = data[14] >> i & 0x01;
            }

            // 右边的8个按键
            for (uint8_t i = 0; i < 8; i++)
            {
                handle_data.right_key[i] = data[15] >> i & 0x01;
            }

            if (handle_data.num[0] >= 1000)
                handle_data.num[0] = 0;
            else
                handle_data.num[0]++;
        }
    }
}

int16_t Rock_trans(uint16_t rock) // 把遥控器数值映射到-1000到1000
{
    int16_t value = 0;

    if (rock >= 2195)
    {
        value = (rock - 2195) * 1000.0f / (4095 - 2195);

        if (value > 1000)
            value = 1000;
    }
    else if (rock < 1900)
    {
        value = -(1900 - rock) * 1000.0f / 1900;

        if (value < -1000)
            value = -1000;
    }

    return value;
}
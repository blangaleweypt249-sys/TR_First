#include "Handle.h"
#include "usart.h"
#include <string.h>

static uint8_t handle_stream[64];
static uint16_t handle_stream_len;
uint8_t receive_handle[17];
uint8_t handle_dma[68];             //UART9 收包缓冲，一次能装 4 帧，不然两帧挤一起就溢出

HandleData handle_data;
volatile uint8_t wifi_ok;
bool handle_lock;
static uint8_t handle_switch_init;
static uint32_t handle_last_tick;   //上一帧时间
static uint8_t handle_sync_num;     //上电/重连后还要对齐几帧

HandleKey handle_key[33U];

//链路超过 200ms 没有有效帧就算断了，这是遥控在线判断的唯一依据
static uint8_t Handle_LinkOk(void)
{
    return (HAL_GetTick() - handle_last_tick) < 200U;
}

//松开那一拍触发一次
uint8_t Handle_KeyRelease(HandleKeyId id)
{
    if (!Handle_LinkOk())//链路断了只对齐状态，不报边沿，免得回来补一个假动作
    {
        handle_key[id].state = handle_key[id].key;
        return 0U;
    }

    if (handle_key[id].key == 1U)
    {
        handle_key[id].state = 1U;
        return 0U;
    }
    if (handle_key[id].state == 1U)
    {
        handle_key[id].state = 0U;
        return 1U;
    }
    return 0U;
}

//状态一变触发一次
uint8_t Handle_KeyChange(HandleKeyId id)
{
    if (!Handle_LinkOk())//同上
    {
        handle_key[id].state = handle_key[id].key;
        return 0U;
    }

    if (handle_key[id].key != handle_key[id].state)
    {
        handle_key[id].state = handle_key[id].key;
        return 1U;
    }
    return 0U;
}

void Key_Detect(void)
{
    if (handle_switch_init == 0U)//第一帧只对齐按键状态，不触发
    {
        if (handle_data.num[0] == 0U)
            return;
        for (uint8_t i = 0U; i < 33U; i++)
            handle_key[i].state = handle_key[i].key;
        handle_switch_init = 1U;
        return;
    }

    if (Handle_KeyChange(eKeyUp1))//锁车开关
        handle_lock = !handle_lock;
}

//流式缓存
void Handle_RxFeed(const uint8_t *data, uint16_t size)
{
    while (size--)
    {
        if (*data == 'O')
            wifi_ok = 1;

        if (handle_stream_len >= sizeof(handle_stream))
        {
            handle_stream_len = 0;
        }

        handle_stream[handle_stream_len++] = *data++;

        while (handle_stream_len >= HANDLE_FRAME_SIZE)
        {
            if (handle_stream[0] == 0xAA &&
                handle_stream[16] == 0xBB)
            {
                (void)osMessageQueuePut(handleQueueHandle,handle_stream, 0U, 0U);
                handle_stream_len -= HANDLE_FRAME_SIZE;
                memmove(handle_stream,&handle_stream[HANDLE_FRAME_SIZE],handle_stream_len);
            }
            else
            {
                memmove(handle_stream,&handle_stream[1],--handle_stream_len);
            }
        }
    }
}

int16_t handle_value[4]; // 测试遥控数据

static void Wifi_Init(void)
{
    //默认使用2.4ghz 6信道
    const char ap_cmd[] = "AT+WLSTARTAP=ssid,UPRE_WIFI,ch,6,sec,wpa2,pw,12345678\r\n";
    const char server_cmd[] = "AT+SKTSERVER=0,1,,6060,1\r\n";
    for (uint8_t retry = 0; retry < 3; retry++)
    {
        wifi_ok = 0;
        HAL_UART_Transmit_DMA(&huart9, (uint8_t *)ap_cmd, sizeof(ap_cmd) - 1U);
        for (uint8_t wait = 0; wait < 100; wait++)
        {
            if (wifi_ok)
                break;
            osDelay(10);
        }
        if (wifi_ok)
            break;
    }

    for (uint8_t retry = 0; retry < 3; retry++)
    {
        wifi_ok = 0;
        HAL_UART_Transmit_DMA(&huart9, (uint8_t *)server_cmd, sizeof(server_cmd) - 1U);
        for (uint8_t wait = 0; wait < 100; wait++)
        {
            if (wifi_ok)
                break;
            osDelay(10);
        }
        if (wifi_ok)
            break;
    }
}

void HandleTask(void *argument)
{
    Wifi_Init();

    for (;;)
    {
        if (osMessageQueueGet(handleQueueHandle, receive_handle, NULL, 0U) == osOK)
        {
            Handle_Analysis(receive_handle);
        }

        osDelay(1);
    }
}

void Handle_Analysis(uint8_t *data)
{
    uint16_t crc = 0;

    if (data[0] == 0xAA && data[16] == 0xBB)
    {
        for (uint8_t i = 1; i <= 14; i++)
        {
            crc += data[i];
        }

        if ((crc & 0xFF) == data[15])
        {
            // 遥控器已经发的是 -1000 到 1000
            memcpy(&handle_data.x, &data[1], sizeof(float));
            memcpy(&handle_data.y, &data[5], sizeof(float));
            memcpy(handle_data.shoulder, &data[9], 2);

            //摇杆值必须落在 ±1000 内，顺便挡掉 NaN；乱码凑出来的 float 基本都过不了这一关
            if (!(handle_data.x >= -1000.0f && handle_data.x <= 1000.0f)) return;
            if (!(handle_data.y >= -1000.0f && handle_data.y <= 1000.0f)) return;

            handle_value[0] = (int16_t)handle_data.x;
            handle_value[1] = (int16_t)handle_data.y;
            handle_value[2] = handle_data.shoulder[0];
            handle_value[3] = handle_data.shoulder[1];

            memset(handle_data.up_key, 0, sizeof(handle_data.up_key));
            memset(handle_data.left_key, 0, sizeof(handle_data.left_key));
            memset(handle_data.right_key, 0, sizeof(handle_data.right_key));
            memset(handle_data.right_down_key, 0, sizeof(handle_data.right_down_key));
            memset(handle_data.omni_key, 0, sizeof(handle_data.omni_key));
            for (uint8_t i = 0; i < 27U; i++)
            {
                uint8_t pressed = (uint8_t)((data[11U + (i >> 3)] >> (i & 7U)) & 1U);
                handle_key[i].key = pressed;
                if (i < 6U)
                    handle_data.up_key[i] = pressed;        //顶部6个开关
                else if (i < 15U)
                    handle_data.left_key[i - 6U] = pressed; //9个左侧按键
                else if (i < 20U)
                    handle_data.right_key[i - 15U] = pressed; //五个方向键
                else if (i < 22U)
                    handle_data.right_down_key[i - 20U] = pressed; //2个小按钮
                else
                    handle_data.omni_key[i - 22U] = pressed;//全向键
            }

            //肩键四档拆成 6 个键：27~29 左按下/上拨/下拨，30~32 右按下/上拨/下拨
            for (uint8_t i = 0; i < 2U; i++)
            {
                handle_key[27U + i * 3U + 0U].key = (handle_data.shoulder[i] == 1U);//按下
                handle_key[27U + i * 3U + 1U].key = (handle_data.shoulder[i] == 2U);//上拨
                handle_key[27U + i * 3U + 2U].key = (handle_data.shoulder[i] == 3U);//下拨
            }

            if (handle_data.num[0] >= 1000)
                handle_data.num[0] = 0;
            else
                handle_data.num[0]++;

            //遥控器上电或断线重连：头几帧只对齐按键电平，不然会被当成拨了一次开关
            if (handle_last_tick == 0U || HAL_GetTick() - handle_last_tick > 500U)
                handle_sync_num = 5U;
            handle_last_tick = HAL_GetTick();

            if (handle_sync_num != 0U)
            {
                handle_sync_num--;
                for (uint8_t i = 0; i < 27U; i++)
                    handle_key[i].state = handle_key[i].key;
            }
        }
    }
}


// int16_t Rock_trans(uint16_t rock) // 把遥控器数值映射到-1000到1000
// {
//     int16_t value = 0;

//     if (rock >= 2195)
//     {
//         value = (rock - 2195) * 1000.0f / (4095 - 2195);

//         if (value > 1000)
//             value = 1000;
//     }
//     else if (rock < 1900)
//     {
//         value = -(1900 - rock) * 1000.0f / 1900;

//         if (value < -1000)
//             value = -1000;
//     }

//     return value;
// }

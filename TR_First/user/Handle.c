#include "Handle.h"
#include "usart.h"
#include <string.h>

uint8_t receive_handle[17];

static uint8_t handle_stream[64];
static uint16_t handle_stream_len;


HandleData handle_data;
volatile uint8_t wifi_ok;
bool handle_lock;
bool handle_key_control;
static uint8_t handle_lock_switch_last;
static uint8_t handle_key_switch_last;
static uint8_t handle_switch_init;

void Key_Detect(void)
{
    if (handle_switch_init == 0U && handle_data.num[0] != 0)
    {
        handle_lock_switch_last = handle_data.up_key[0];
        handle_key_switch_last = handle_data.up_key[1];
        handle_switch_init = 1U;
    }
    else
    {
        if (handle_data.up_key[0] != handle_lock_switch_last)
        {
            handle_lock = !handle_lock;
            handle_lock_switch_last = handle_data.up_key[0];
        }

        if (handle_data.up_key[1] != handle_key_switch_last)
        {
            handle_key_control = !handle_key_control;
            handle_key_switch_last = handle_data.up_key[1];
        }
    }
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
        HAL_UART_Transmit(&huart9, (uint8_t *)ap_cmd, sizeof(ap_cmd) - 1U, 1000U);
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
        HAL_UART_Transmit(&huart9, (uint8_t *)server_cmd, sizeof(server_cmd) - 1U, 1000U);
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

            if (handle_data.num[0] >= 1000)
                handle_data.num[0] = 0;
            else
                handle_data.num[0]++;
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

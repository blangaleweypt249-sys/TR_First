#include "vofa.h"
#include "WpLib.h"

//调用频率需根据波特率设定
static UART_HandleTypeDef *vofa_huart;

static float_u8 vofa_channel[VOFA_CHANNEL_COUNT];
static uint8_t send_txbuffer[VOFA_FRAME_SIZE];

static const uint8_t vofa_tail[4] = {0x00, 0x00, 0x80, 0x7F};

void Vofa_Init(UART_HandleTypeDef *huart)
{
    vofa_huart = huart;
}

void Vofa_Send(const DJI_Motor *motor)
{
    if ((motor == NULL) || (vofa_huart == NULL))
        return;

    //上一帧 DMA 还没发完就跳过本帧，否则会改坏正在发送的缓冲区
    if (vofa_huart->gState != HAL_UART_STATE_READY)
        return;

    vofa_channel[0].f = motor->target_speed;
    vofa_channel[1].f = motor->Feedback_speed;
    vofa_channel[2].f = motor->target_position;
    vofa_channel[3].f = motor->Feedback_angle;

    for (uint8_t i = 0; i < VOFA_CHANNEL_COUNT; i++)
    {
        for (uint8_t j = 0; j < 4U; j++)
        {
            send_txbuffer[i * 4U + j] = vofa_channel[i].u8[j];
        }
    }

    for (uint8_t i = 0; i < 4U; i++)
    {
        send_txbuffer[VOFA_CHANNEL_COUNT * 4U + i] = vofa_tail[i];
    }

    (void)HAL_UART_Transmit_DMA(vofa_huart, send_txbuffer, VOFA_FRAME_SIZE);
}

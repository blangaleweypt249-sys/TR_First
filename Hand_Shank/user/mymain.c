#include "mymain.h"
#include "tim.h"
#include "usart.h"
#include "WifiConfig.h"
#include <string.h>
#include <math.h>

int16_t adcBuffer[4];
remote_data remoteData;

#define TX_PAYLOAD_SIZE 14U

static uint8_t tx_frame[17]; //0xAA + 14字节数据 + 校验 + 0xBB
static uint8_t send_fail;               //连续发送失败次数
static uint8_t beep_tick;
static uint8_t key_event;
static volatile uint8_t beep_request;
static volatile uint8_t beep_alarm;
static uint8_t beep_count;
static uint8_t beep_phase;

void Beep_Request(uint8_t count)
{
    beep_request = count;
}

void Beep_SetAlarm(uint8_t enable)
{
    beep_alarm = enable;
    if (enable)
    {
        beep_request = 0U;
        beep_phase = 0U;
        beep_tick = 0U;
    }
}

static HAL_StatusTypeDef Wifi_SendRawFrame(const uint8_t *frame, uint16_t size)
{
    const char raw_cmd[] = "AT+SKTSENDRAW=0,17\r\n";
    uint8_t ch;
    uint8_t prompt_count = 0U;
    uint32_t start;

    Wifi_RxClear();//先倒掉上一次残留的 ">>>"/"OK"，不然提示符会数错，数据发不出去

    if (HAL_UART_Transmit(&huart6, (uint8_t *)raw_cmd,
                          sizeof(raw_cmd) - 1U, 100U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 100U)
    {
        if (Wifi_GetByte(&ch, 5U) == 0U)
        {
            continue;
        }
        if (ch == '>')
        {
            prompt_count++;
            if (prompt_count >= 3U)
                break;
        }
        else
        {
            prompt_count = 0U;
        }
    }

    if (prompt_count < 3U)
    {
        return HAL_TIMEOUT;
    }

    if (HAL_UART_Transmit(&huart6, (uint8_t *)frame, size, 100U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 200U)
    {
        if (Wifi_GetByte(&ch, 5U) == 0U)
        {
            continue;
        }
        if (ch == 'K')
            return HAL_OK;
        if (ch == 'R')
            return HAL_ERROR;
    }
    return HAL_TIMEOUT;
}

//消抖

//按键序号和左/右/上/下的对应
//22~26 是五位开关 WS-1001，公共端接 GND，各脚 10K 上拉
//  22=PD3 左  23=PD4 中间按下  24=PD5 下  25=PD6 右  26=PD7 上
static GPIO_TypeDef *key_port[27] = {
    GPIOA, GPIOA, GPIOA, GPIOA, GPIOA, GPIOC,
    GPIOD, GPIOD, GPIOD, GPIOD, GPIOD, GPIOD,
    GPIOA, GPIOA, GPIOA,
    GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE, GPIOE,
    GPIOD, GPIOD, GPIOD, GPIOD, GPIOD
};
static const uint16_t key_pin[27] = {
    GPIO_PIN_6, GPIO_PIN_5, GPIO_PIN_4, GPIO_PIN_1, GPIO_PIN_0, GPIO_PIN_3,
    GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15,
    GPIO_PIN_8, GPIO_PIN_11, GPIO_PIN_12,
    GPIO_PIN_6, GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_2, GPIO_PIN_1, GPIO_PIN_0,
    GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7
};

//上一次的按键状态
static uint8_t key_last[27];
//消抖计数
static uint8_t key_count[27];

static void Key_Scan(void)
{
    for (uint8_t i = 0; i < 27; i++)
    {
        uint8_t level = (HAL_GPIO_ReadPin(key_port[i], key_pin[i]) == GPIO_PIN_RESET);

        if (level != key_last[i])
        {
            key_count[i]++;
            if (key_count[i] >= 5U) //5ms消抖
            {
                if (i < 6U || level)
                    key_event = 1U;
                key_last[i] = level;
                key_count[i] = 0U;
                remoteData.key[i] = level;
            }
            else
            {
                remoteData.key[i] = key_last[i];
            }
        }
        else
        {
            key_count[i] = 0U;
            remoteData.key[i] = level;
        }
    }
}

static void Beep_Start(void)
{
    //设置占空比
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 14482);
    beep_tick = 50U; //50ms
    beep_phase = 1U;
    beep_count = 1U;
}

static void Beep_Task(void)
{
    if (beep_alarm)
    {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 14482);
        beep_phase = 0U;
        beep_tick = 0U;
        return;
    }

    if (beep_phase == 0U)
    {
        if (beep_request == 0U)
            return;

        beep_count = beep_request;
        beep_request = 0U;
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 14482);
        beep_tick = 50U;
        beep_phase = 1U;
        return;
    }

    if (beep_tick == 0U)
        return;

    beep_tick--;
    if (beep_tick == 0U)
    {
        if (beep_phase == 1U)
        {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);
            if (beep_count > 1U)
            {
                beep_count--;
                beep_phase = 2U;
                beep_tick = 50U;
            }
            else
            {
                beep_phase = 0U;
            }
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 14482);
            beep_phase = 1U;
            beep_tick = 50U;
        }
    }
}

//单轴 ADC 归一到 -1~1
static float Rock_Axis(uint16_t rock, uint16_t center, uint16_t min, uint16_t max)
{
    float k;

    if (rock >= center)
    {
        k = (float)(rock - center) / (float)(max - center);
        if (k > 1.0f)
            k = 1.0f;
    }
    else
    {
        k = -(float)(center - rock) / (float)(center - min);
        if (k < -1.0f)
            k = -1.0f;
    }

    return k;
}

//圆形死区 + 三次曲线，输出 -1000~1000
static void Rock_Shape(float x, float y, float *out_x, float *out_y)
{
    const float dead = 0.02f; //死区半径，约 40 个 ADC 码，和原来一样大
    const float expo = 0.3f;  //非线性程度：0 线性，1 全是三次项
    float mag = sqrtf(x * x + y * y);
    float k;

    if (mag <= dead)//斜推也只按圆的半径算，斜角不会再比单轴灵
    {
        *out_x = 0.0f;
        *out_y = 0.0f;
        return;
    }

    k = (mag - dead) / (1.0f - dead);//死区外重新映射到 0~1
    if (k > 1.0f)
        k = 1.0f;

    k = (1.0f - expo) * k + expo * k * k * k;//三次曲线

    *out_x = x / mag * k * 1000.0f;
    *out_y = y / mag * k * 1000.0f;
}

void ScanTask(void *argument)
{
    uint16_t adc_f[4]; //滤波后的摇杆/肩键值

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//启动pwm定时器
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);

    for (uint8_t i = 0; i < 27; i++)
    {
        key_last[i] = (HAL_GPIO_ReadPin(key_port[i], key_pin[i]) == GPIO_PIN_RESET);
        key_count[i] = 0U;
    }

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 4);
    osDelay(2);//等第一次转换完成，不然上电第一帧读到0，摇杆会满量程

    for (uint8_t i = 0; i < 4U; i++)
        adc_f[i] = (uint16_t)adcBuffer[i];

    for (;;)
    {
        Key_Scan();
        if (key_event)
        {
            //按键触发 设置比较值
            key_event = 0U;
            Beep_Start();
        }
        //清空比较值
        Beep_Task();

        //取上一拍已经转完的采样，再做1/4新值滤波，压掉静止噪声
        for (uint8_t i = 0; i < 4U; i++)
            adc_f[i] = (uint16_t)((adc_f[i] * 3U + (uint16_t)adcBuffer[i]) >> 2);

        //ch0 接上下轴，静止 2052；ch1 接左右轴，静止 2069
        Rock_Shape(-Rock_Axis(adc_f[0], 2052, 2, 4095),
                   -Rock_Axis(adc_f[1], 2069, 2, 4095),
                   &remoteData.x, &remoteData.y);

        remoteData.shoulder[0] = adc_f[2] > 700 ? 1 : 0;
        remoteData.shoulder[1] = adc_f[3] > 700 ? 1 : 0;

        if (osMessageQueuePut(ScanToTransHandle, &remoteData, 0U, 0U) != osOK)//队列满就丢旧值，只留最新状态
        {
            osMessageQueueReset(ScanToTransHandle);
            osMessageQueuePut(ScanToTransHandle, &remoteData, 0U, 0U);
        }

        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 4);//启动下一次，下一拍再取

        osDelay(1);
    }
}

void TransmitTask(void *argument)
{
    remote_data data;

    (void)argument;

    for (;;)
    {
        if (!wifi_ready)
        {
            osDelay(10);
            continue;
        }

        if (osMessageQueueGet(ScanToTransHandle, &data, NULL, osWaitForever) == osOK)
        {
            tx_frame[0] = 0xAA;
            memcpy(&tx_frame[1], &data.x, sizeof(data.x));
            memcpy(&tx_frame[5], &data.y, sizeof(data.y));
            memcpy(&tx_frame[9], data.shoulder, sizeof(data.shoulder));
            memset(&tx_frame[11], 0, 4U);
            for (uint8_t i = 0; i < 27U; i++)
            {
                if (data.key[i] != 0U)
                    tx_frame[11U + (i >> 3)] |= (uint8_t)(1U << (i & 7U));
            }

            uint8_t sum = 0;
            for (uint8_t i = 1U; i <= TX_PAYLOAD_SIZE; i++)
            {
                sum += tx_frame[i];
            }
            tx_frame[15] = sum;
            tx_frame[16] = 0xBB;

            if (Wifi_SendRawFrame(tx_frame, 17) != HAL_OK)
            {
                if (++send_fail >= 3U) //连续3帧发不出去才算链路断
                {
                    send_fail = 0U;
                    wifi_ready = 0U;
                }
            }
            else
            {
                send_fail = 0U;
            }
        }

        osDelay(5); //200Hz
     }
}

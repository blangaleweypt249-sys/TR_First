#include "mymain.h"
#include "tim.h"
#include "usart.h"
#include "WifiConfig.h"
#include <string.h>

int16_t adcBuffer[4];
remote_data remoteData;

#define TX_FRAME_SIZE 17U
#define TX_PAYLOAD_SIZE 14U

static uint8_t tx_frame[TX_FRAME_SIZE]; //0xAA + 14字节数据 + 校验 + 0xBB
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
    uint32_t start = HAL_GetTick();

    if (HAL_UART_Transmit(&huart6, (uint8_t *)raw_cmd,
                          sizeof(raw_cmd) - 1U, 1000U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    while ((HAL_GetTick() - start) < 1000U)
    {
        if (HAL_UART_Receive(&huart6, &ch, 1U, 50U) == HAL_OK)
        {
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
    }

    if (prompt_count < 3U)
    {
        return HAL_TIMEOUT;
    }

    if (HAL_UART_Transmit(&huart6, (uint8_t *)frame, size, 1000U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    start = HAL_GetTick();
    while ((HAL_GetTick() - start) < 300U)
    {
        if (HAL_UART_Receive(&huart6, &ch, 1U, 20U) == HAL_OK)
        {
            if (ch == 'K')
                return HAL_OK;
            if (ch == 'R')
                return HAL_ERROR;
        }
    }
    return HAL_TIMEOUT;
}

//消抖

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

//死区控制和映射 -1000到1000
int16_t Rock_trans(uint16_t rock, uint16_t min, uint16_t center, uint16_t max)
{
    int16_t value = 0;

    if (rock >= center + 30)
    {
        value = (rock - center - 30) * 1000.0f / (max - center - 30);

        if (value > 1000)
            value = 1000;
    }
    else if (rock <= center - 30)
    {
        value = -(center - 30 - rock) * 1000.0f / (center - 30 - min);

        if (value < -1000)
            value = -1000;
    }

    return value;
}

void ScanTask(void *argument)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);//启动pwm定时器
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0U);

    for (uint8_t i = 0; i < 27; i++)
    {
        key_last[i] = (HAL_GPIO_ReadPin(key_port[i], key_pin[i]) == GPIO_PIN_RESET);
        key_count[i] = 0U;
    }

    for (;;)
    {
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adcBuffer, 4);
        Key_Scan();
        if (key_event)
        {
            //按键触发 设置比较值
            key_event = 0U;
            Beep_Start();
        }
        //清空比较值
        Beep_Task();
        remoteData.x = -Rock_trans(adcBuffer[0], 2, 2068, 4095);
        remoteData.y = -Rock_trans(adcBuffer[1], 2, 2050, 4095);

        remoteData.shoulder[0] = adcBuffer[2] > 700 ? 1 : 0;
        remoteData.shoulder[1] = adcBuffer[3] > 700 ? 1 : 0;

        osMessageQueuePut(ScanToTransHandle, &remoteData, 0U, 0U);
            
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

            if (Wifi_SendRawFrame(tx_frame, TX_FRAME_SIZE) != HAL_OK)
                wifi_ready = 0U;
        }

        osDelay(5); //200Hz
     }
}

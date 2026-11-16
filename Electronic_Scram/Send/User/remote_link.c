#include "remote_link.h"
#include "usart.h"
#include "lora_config.h"

static uint8_t power_off;       // 0 通电  1 断电
static uint8_t sw_cnt;          // 开关消抖计数
static uint32_t last_tx_tick;

// 发一个字节：'1' 断电，'0' 通电
static void RemoteLink_Send(uint8_t value)
{
    uint8_t data = value ? '1' : '0';

    // AUX 低说明模块正在自检或缓冲满，等下一轮
    if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_RESET)
    {
        return;
    }

    if (HAL_UART_Transmit(&huart1, &data, 1U, 10U) == HAL_OK)
    {
        last_tx_tick = HAL_GetTick();
    }
}

void RemoteLink_Init(void)
{
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, LORA_M0_LEVEL);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, LORA_M1_LEVEL);
    HAL_Delay(LORA_MODE_SETTLE_MS);

    // 上电时开关在哪一边，就按那一边算初始状态
    power_off = (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_SET) ? 1U : 0U;
    sw_cnt = 0U;
    last_tx_tick = 0U;
}

void RemoteLink_Poll(void)
{
    uint8_t raw = (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_SET) ? 1U : 0U;

    // 开关电平稳定 20ms 才算真的切换了
    if (raw != power_off)
    {
        if (++sw_cnt >= 20U)
        {
            sw_cnt = 0U;
            power_off = raw;
            RemoteLink_Send(power_off);
            return;
        }
    }
    else
    {
        sw_cnt = 0U;
    }

    // 每 100ms 重发一次当前状态，丢一包也能补上
    if ((HAL_GetTick() - last_tx_tick) >= 100U)
    {
        RemoteLink_Send(power_off);
    }
}

#include "remote_link.h"
#include "usart.h"
#include "lora_config.h"

uint8_t rx_byte;
static volatile uint8_t power_off;      // 0 通电  1 断电
static uint8_t applied = 0xFFU;

// 8 路输出 + 状态灯：断电时全低
static void RemoteLink_Output(uint8_t off)
{
    GPIO_PinState state = off ? GPIO_PIN_RESET : GPIO_PIN_SET;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, state);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 |
                              GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9, state);
    HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, off ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void RemoteLink_Init(void)
{
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, LORA_M0_LEVEL);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, LORA_M1_LEVEL);
    HAL_Delay(LORA_MODE_SETTLE_MS);

    // 没收到数据之前默认通电
    power_off = 0U;
    RemoteLink_Output(0U);
    applied = 0U;

    HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
}

void RemoteLink_Poll(void)
{
    if (applied != power_off)
    {
        applied = power_off;
        RemoteLink_Output(applied);
    }
}

// '1' 断电，'0' 通电，别的字节不管
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART3)
    {
        return;
    }

    if (rx_byte == '1')
    {
        power_off = 1U;
    }
    else if (rx_byte == '0')
    {
        power_off = 0U;
    }

    HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        HAL_UART_AbortReceive(huart);
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1U);
    }
}

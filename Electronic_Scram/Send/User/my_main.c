/**
 * @file my_main.c
 * @brief 提供急停发送板串口转发初始化和主循环业务入口。
 */

#include "main.h"
#include "my_main.h"
#include "remote_link.h"

/* 将 LoRa 模式引脚 M0、M1 置低，再初始化 USART1 与 USART3 的双向 DMA 转发并启动接收。 */
void My_Init(void)
{
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);

    RemoteLink_Init();
}

/* 周期处理串口故障恢复和两端口双向数据转发，推进尚未完成的 DMA 发送队列。 */
void My_Loop(void)
{
    RemoteLink_Process();
}

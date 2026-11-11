/**
 * @file my_main.c
 * @brief 提供急停接收板输出引脚置位和串口转发业务入口。
 */

#include "main.h"
#include "my_main.h"
#include "remote_link.h"

/* 将 LoRa 模式引脚 M0、M1 置低并将 PB3 至 PB9、PA15 输出置高，再初始化双串口 DMA 转发。 */
void My_Init(void)
{
    HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
    RemoteLink_Init();
}

/* 周期处理串口故障恢复和两端口双向数据转发，推进尚未完成的 DMA 发送队列。 */
void My_Loop(void)
{
    RemoteLink_Process();

}

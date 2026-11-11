/**
 * @file remote_link.h
 * @brief 提供急停发送板 USART3 上游与 USART1 LoRa 链路的 DMA 双向透明转发的类型定义与接口声明。
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* 清空双串口状态并绑定 USART1、USART3，延时后启动两路循环 DMA 接收，失败端口标记为待重启。 */
void RemoteLink_Init(void);
/* 轮询恢复异常端口、推进发送队列并双向转发主机与 LoRa 字节流，再启动新入队数据的发送。 */
void RemoteLink_Process(void);

#ifdef __cplusplus
}
#endif

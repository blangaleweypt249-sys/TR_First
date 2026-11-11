/**
 * @file my_main.h
 * @brief 提供急停发送板串口转发初始化和主循环业务入口的类型定义与接口声明。
 */

#ifndef MY_MAIN_H
#define MY_MAIN_H
/* 将 LoRa 模式引脚 M0、M1 置低，再初始化 USART1 与 USART3 的双向 DMA 转发并启动接收。 */
void My_Init(void);
/* 周期处理串口故障恢复和两端口双向数据转发，推进尚未完成的 DMA 发送队列。 */
void My_Loop(void);
#endif

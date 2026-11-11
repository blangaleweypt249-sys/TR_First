/**
 * @file my_main.h
 * @brief 提供急停接收板输出引脚置位和串口转发业务入口的类型定义与接口声明。
 */

#ifndef MY_MAIN_H
#define MY_MAIN_H
/* 将 LoRa 模式引脚 M0、M1 置低并将 PB3 至 PB9、PA15 输出置高，再初始化双串口 DMA 转发。 */
void My_Init(void);
/* 周期处理串口故障恢复和两端口双向数据转发，推进尚未完成的 DMA 发送队列。 */
void My_Loop(void);
#endif

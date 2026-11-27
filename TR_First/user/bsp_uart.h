#ifndef __BSP_UART_H__
#define __BSP_UART_H__

#include "main.h"
#include "usart.h"

extern uint8_t receive_imu[7];
extern uint8_t receive_dt35[19];
extern uint8_t handle_dma[68];      //UART9 收包缓冲，装得下 4 帧

void Bsp_Uart_Init(void);

#endif

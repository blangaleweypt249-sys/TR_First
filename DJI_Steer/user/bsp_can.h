#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "main.h"

void ALL_CAN_Start(void);
void CAN1_Filter_Init(void);
void CAN2_Filter_Init(void);
HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan, uint8_t type, uint32_t id, uint8_t *data, uint8_t len);
void CAN_Bus_Off_Recover(CAN_HandleTypeDef *hcan);

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

#endif

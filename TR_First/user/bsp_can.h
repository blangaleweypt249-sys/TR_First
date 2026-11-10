#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__

#include "main.h"
#include "fdcan.h"
#include "helm_chassis.h"

void FDCAN1_Init(void);
void FDCAN2_Init(void);
void FDCAN3_Init(void);
void RecoverCANBus(void);

HAL_StatusTypeDef CAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint8_t type, uint32_t id, uint8_t *data, uint8_t len);

#endif

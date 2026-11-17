#ifndef __MYMAIN_H__
#define __MYMAIN_H__

#include "main.h"
#include "stdbool.h"
#include "cmsis_os.h"
#include "usart.h"
#include "100A.h"

void Imu_Init(void);
bool Imu_calibration(void);
void ImuCalcTask(void *argument);
void ImuTransTask(void *argument);
void StateCheckTask(void *argument);

extern osSemaphoreId_t RxsuccessHandle;
extern osSemaphoreId_t ClearYawHandle;
extern osSemaphoreId_t CalcOKHandle;

#endif

#ifndef __MYMAIN_H__
#define __MYMAIN_H__

#include "main.h"
#include <stdbool.h>
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "bsp_can.h"
#include "pid.h"
#include "cmsis_os2.h"
#include "Handle.h"
#include "vesc.h"
#include "DJI.h"
#include "helm_chassis.h"
#include "imu_100A.h"
#include "dt35.h"
#include "STP_23L.h"
#include "w25q128.h"
#include "math.h"


extern HandleData handle_data;
extern float speed_k;
extern uint8_t receive_handle[19];
extern uint8_t receive_imu[7];
extern uint8_t receive_dt35[19];
extern osMessageQueueId_t handleQueueHandle;
extern osMessageQueueId_t imuQueueHandle;
extern osMessageQueueId_t dt35QueueHandle;
extern osSemaphoreId_t ChassisCalcHandle;
extern osSemaphoreId_t ChassisReadyHandle;
void My_init(void);
void Test_HelmMotor(void);
void Handle_Analysis(uint8_t *data);
int16_t Rock_trans(uint16_t rock);

#endif

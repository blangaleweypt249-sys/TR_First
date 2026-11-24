#ifndef __MYMAIN_H__
#define __MYMAIN_H__

#include "main.h"
#include <stdbool.h>
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "init_task.h"
#include "pid.h"
#include "cmsis_os2.h"
#include "Handle.h"
#include "vesc.h"
#include "DJI.h"
#include "helm_chassis.h"
#include "status_task.h"
#include "Leg.h"
#include "imu_100A.h"
#include "dt35.h"
#include "STP_23L.h"
#include "w25q128.h"
#include "math.h"

extern osMessageQueueId_t imuQueueHandle;
extern osMessageQueueId_t dt35QueueHandle;

void Handle_Analysis(uint8_t *data);
int16_t Rock_trans(uint16_t rock);

#endif

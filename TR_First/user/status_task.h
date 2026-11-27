#ifndef __STATUS_TASK_H__
#define __STATUS_TASK_H__

#include "main.h"
#include <stdbool.h>

#define STATE_TIMEOUT_COUNT    20  //大概200ms更新一次

typedef struct
{
    bool imu_state;
    bool w5500_offline; //true：未收到上位机有效探测帧
    bool dt35_state[4];
    bool dji_3508_state[4];
    bool dji_2006_state[4];
    bool vesc_motor_state[4];
} Device_Offline_State_t;

extern Device_Offline_State_t Device_Offline_State;

bool Check_State(int *num, int timeout);
void StatusTask(void *argument);

#endif

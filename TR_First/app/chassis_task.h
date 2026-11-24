#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "main.h"
#include "helm_chassis.h"

extern Speed world_speed;
extern Speed body_speed;
extern float speed_k;

void ChassisTask(void *argument);

#endif

#ifndef __LOCATION_TASK_H__
#define __LOCATION_TASK_H__

#include "main.h"
#include <stdbool.h>
#include "dt35.h"

void Left_Cbf(DT35_Move *dt35, Speed *body_speed);
void LocationTask(void *argument);

#endif

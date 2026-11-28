#ifndef __DT35_H__
#define __DT35_H__

#include "helm_chassis.h"
#include <stdint.h>
#include "WpLib.h"
#include "pid.h"

typedef union 
{
    uint8_t  u8[16];
    float    flo[4];
}U8_to_Flo;

typedef struct 
{
	float actual_dt35_distance;
	float target_dt35_distance;
    int dt35_num[3];
}DT35_Move;

extern DT35_Move dt35_front;
extern DT35_Move dt35_back;
extern DT35_Move dt35_left;
extern DT35_Move dt35_right;
extern PID pid_dt35_left;           //左侧定距用，pid 输出是 mm/s
void DT35_Analysis(uint8_t *data);
void Located_By_Left_DT35(DT35_Move *dt35, Speed *world_speed, float current_angle);

#endif

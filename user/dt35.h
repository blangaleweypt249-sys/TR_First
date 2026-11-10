#ifndef __DT35_H__
#define __DT35_H__

#include "helm_chassis.h"
#include <stdint.h>
#include "WpLib.h"

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
void DT35_Analysis(uint8_t *data);

#endif

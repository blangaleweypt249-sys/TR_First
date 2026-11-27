#ifndef __MY_MAIN_H
#define __MY_MAIN_H

#include "main.h"
#include "dt35.h"

extern DT35 dt35_l;   
extern DT35 dt35_f;   

typedef union
{
    uint8_t u8[16];
    float flo[4];
}DT35_U;


void My_Main_Init(void);
void My_Main_Task(void);

#endif

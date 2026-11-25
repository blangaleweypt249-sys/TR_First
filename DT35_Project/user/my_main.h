#ifndef __MY_MAIN_H
#define __MY_MAIN_H

#include "main.h"
#include "dt35.h"

extern DT35 dt35_l;   //左激光，0x20
extern DT35 dt35_f;   //前激光，0x21

void My_Main_Init(void);
void My_Main_Task(void);

#endif

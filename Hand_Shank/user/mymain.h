#ifndef __MYMAIN_H__
#define __MYMAIN_H__

#include "main.h"
#include "stdint.h"
#include "stdbool.h"
#include "adc.h"
#include "cmsis_os.h"

typedef struct 
{
   float x;
   float y;

   uint8_t key[27];
   uint8_t shoulder[2];

}remote_data;

void Beep_Request(uint8_t count);
void Beep_SetAlarm(uint8_t enable);

extern int16_t adcBuffer[4];
extern osMessageQueueId_t ScanToTransHandle;
extern osSemaphoreId_t myPacksucHandle;
#endif


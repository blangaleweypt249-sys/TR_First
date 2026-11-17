#ifndef __WIFICONFIG_H__
#define __WIFICONFIG_H__

#include "main.h"
#include "cmsis_os.h"

extern osSemaphoreId_t myWIFIsucHandle;
extern volatile uint8_t wifi_ready;

void Wifi_RxStart(void);
uint8_t Wifi_GetByte(uint8_t *ch, uint32_t timeout);
void Wifi_RxClear(void);

#endif

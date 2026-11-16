#ifndef __HANDLE_H__
#define __HANDLE_H__

#include "main.h"
#include "stdint.h"
#include "cmsis_os2.h"
#include <stdbool.h>

#define HANDLE_FRAME_SIZE 17U

typedef struct 
{
  float x;
  float y;
  uint8_t up_key[6];
  uint8_t left_key[9];
  uint8_t right_key[5];
  uint8_t right_down_key[2];
  uint8_t omni_key[5];
  uint8_t shoulder[2];
  int num[3];
}HandleData;

void Handle_Analysis(uint8_t *data);
void Handle_RxFeed(const uint8_t *data, uint16_t size);
void Key_Detect(void);
//int16_t Rock_trans(uint16_t rock);

extern uint8_t receive_handle[17];
extern osMessageQueueId_t handleQueueHandle;
extern HandleData handle_data;
extern volatile uint8_t wifi_ok;
extern bool handle_lock;
extern bool handle_key_control;
extern int16_t handle_value[4]; // 测试遥控数据

#endif

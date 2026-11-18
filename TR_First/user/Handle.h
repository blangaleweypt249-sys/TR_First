#ifndef __HANDLE_H__
#define __HANDLE_H__

#include "main.h"
#include "stdint.h"
#include "cmsis_os2.h"
#include <stdbool.h>

#define HANDLE_FRAME_SIZE 17U

//按键序号就是帧里的位号
typedef enum
{
    eKeyUp1 = 0,
    eKeyUp2,
    eKeyUp3,
    eKeyUp4,
    eKeyUp5,
    eKeyUp6,
    eKeyLeft1 = 6,
    eKeyLeft2,
    eKeyLeft3,
    eKeyLeft4,
    eKeyLeft5,
    eKeyLeft6,
    eKeyLeft7,
    eKeyLeft8,
    eKeyLeft9,
    eKeyRight1 = 15,
    eKeyRight2,
    eKeyRight3,
    eKeyRight4,
    eKeyRight5,
    eKeyDown1 = 20,
    eKeyDown2,
    eKeyOmni1 = 22,
    eKeyOmni2,
    eKeyOmni3,
    eKeyOmni4,
    eKeyOmni5,
}HandleKeyId;

typedef struct
{
    uint8_t key;    //当前电平
    uint8_t state;  //上次处理过的电平
}HandleKey;

extern HandleKey handle_key[27U];

uint8_t Handle_KeyRelease(HandleKeyId id);  //松开那一拍触发一次
uint8_t Handle_KeyChange(HandleKeyId id);   //状态一变触发一次

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
extern int16_t handle_value[4]; // 测试遥控数据

#endif

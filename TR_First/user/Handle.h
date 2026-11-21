#ifndef __HANDLE_H__
#define __HANDLE_H__

#include "main.h"
#include "stdint.h"
#include "cmsis_os2.h"
#include <stdbool.h>

#define HANDLE_FRAME_SIZE 17U

//按键序号就是帧里的位号，行尾是遥控器上对应的引脚
typedef enum
{
    eKeyUp1 = 0,        //PA6
    eKeyUp2,            //PA5
    eKeyUp3,            //PA4
    eKeyUp4,            //PA1
    eKeyUp5,            //PA0
    eKeyUp6,            //PC3
    eKeyLeft1 = 6,      //PD10
    eKeyLeft2,          //PD11
    eKeyLeft3,          //PD12
    eKeyLeft4,          //PD13
    eKeyLeft5,          //PD14
    eKeyLeft6,          //PD15
    eKeyLeft7,          //PA8
    eKeyLeft8,          //PA11
    eKeyLeft9,          //PA12
    eKeyRight1 = 15,    //PE6
    eKeyRight2,         //PE3
    eKeyRight3,         //PE4
    eKeyRight4,         //PE5
    eKeyRight5,         //PE2
    eKeyDown1 = 20,     //PE1
    eKeyDown2,          //PE0
    eKeyOmni1 = 22,     //PD3  五位开关 左
    eKeyOmni2,          //PD4  五位开关 中
    eKeyOmni3,          //PD5  五位开关 下
    eKeyOmni4,          //PD6  五位开关 右
    eKeyOmni5,          //PD7  五位开关 上

    eKeyLeft = eKeyOmni1,
    eKeyOk = eKeyOmni2,
    eKeyDown = eKeyOmni3,
    eKeyRight = eKeyOmni4,
    eKeyUp = eKeyOmni5,
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
extern uint8_t handle_dma[68];      //UART9 收包缓冲，装得下 4 帧
extern osMessageQueueId_t handleQueueHandle;
extern HandleData handle_data;
extern volatile uint8_t wifi_ok;
extern bool handle_lock;
extern int16_t handle_value[4]; // 测试遥控数据

#endif

#ifndef __INTERPOLATION_H__
#define __INTERPOLATION_H__

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float start_pos;
    float target_pos;
    float alltime;
    float timed;
    float actual_pos;
    uint32_t last_update_ms;
    bool state;
}Interpolation_t;

void Interpolation_Init(Interpolation_t *motor);

//enable: 0关闭插值，1开启插值
//run_time_ms: 插值运行时间，单位毫秒
float Interpolation_PositionCommand(Interpolation_t *motor,
                                    float current_command_pos,
                                    float target_pos,
                                    uint8_t enable,
                                    uint32_t run_time_ms);

#endif


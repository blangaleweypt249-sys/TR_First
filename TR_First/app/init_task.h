#ifndef __INIT_TASK_H__
#define __INIT_TASK_H__

#include "main.h"

extern volatile uint8_t boot_done; //1 表示外设和机构初始化完成，其他任务才开始跑

void InitTask(void *argument);

#endif

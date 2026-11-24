#ifndef W5500_H
#define W5500_H

#include <stdbool.h>

extern int w5500_num[3]; //有效探测帧计数、上次计数、离线累积次数

bool W5500_Init(void);
bool W5500_Poll(void);

#endif

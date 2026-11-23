#ifndef W5500_H
#define W5500_H

extern int w5500_num[3]; //有效探测帧计数、上次计数、离线累积次数

void W5500_Task(void *argument);

#endif

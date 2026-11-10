#include "main.h"
#include "my_main.h"
#include "remote_link.h"

void My_Init(void)
{
    RemoteLink_Init();
}

void My_Loop(void)
{
    RemoteLink_Process();
    HAL_Delay(1U);
}

#include "net_task.h"
#include "mymain.h"
#include "init_task.h"
#include "W5500.h"

//W5500：没配好就 100ms 探一次，配好后 5ms 轮询收包
void NetTask(void *argument)
{
    (void)argument;
    bool ready = false;

    while (boot_done == 0U)//等外设和机构初始化完成
        osDelay(10);

    for (;;)
    {
        ready = ready ? W5500_Poll() : W5500_Init(); //IP、端口、socket 配好后正常轮询
        osDelay(ready ? 5U : 100U); //正常最多每 5 ms 处理一包；模块未就绪每 100 ms 再探测
    }
}

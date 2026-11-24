#include "handle_task.h"
#include "mymain.h"

static uint8_t handle_switch_init; //第一帧只对齐按键电平

//锁车开关；上电/重连后第一帧只对齐，不触发
static void Key_Detect(void)
{
    if (handle_switch_init == 0U)
    {
        if (handle_data.num[0] == 0U)
            return;
        for (uint8_t i = 0U; i < 33U; i++)
            handle_key[i].state = handle_key[i].key;
        handle_switch_init = 1U;
        return;
    }

    if (Handle_KeyChange(eKeyUp1))//锁车开关
        handle_lock = !handle_lock;
}

//遥控：解 17 字节帧、刷按键、读锁车开关
void HandleTask(void *argument)
{
    (void)argument;

    while (boot_done == 0U)//等外设和机构初始化完成
        osDelay(10);

    for (;;)
    {
        if (osMessageQueueGet(handleQueueHandle, receive_handle, NULL, 0U) == osOK)
        {
            Handle_Analysis(receive_handle);
        }

        Key_Detect();
        osDelay(1);
    }
}

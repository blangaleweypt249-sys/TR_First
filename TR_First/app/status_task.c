#include "status_task.h"
#include "mymain.h"
#include "init_task.h"
#include "W5500.h"

Device_Offline_State_t Device_Offline_State;

bool Check_State(int *num, int timeout)
{
    if (num[0] == num[1])
        num[2]++;
    else
    {
        num[2]--;
        if (num[2] < 0)
            num[2] = 0;
    }

    num[1] = num[0];

    if (num[2] >= timeout)
    {
        num[2] = timeout - 1;
        return true;
    }

    return false;
}

//CAN 恢复 + 各外设掉线检测
void StatusTask(void *argument)
{
    (void)argument;

    while (boot_done == 0U)//等外设和机构初始化完成
        osDelay(10);

    for (;;)
    {
        RecoverCANBus();
        if (w5500_num[0] != w5500_num[1])
            w5500_num[2] = 0;
        Device_Offline_State.w5500_offline = Check_State(w5500_num, 100);

        Device_Offline_State.imu_state = Check_State(imu_num, STATE_TIMEOUT_COUNT);
        Device_Offline_State.dt35_state[0] = Check_State(dt35_front.dt35_num, STATE_TIMEOUT_COUNT);
        Device_Offline_State.dt35_state[1] = Check_State(dt35_back.dt35_num, STATE_TIMEOUT_COUNT);
        Device_Offline_State.dt35_state[2] = Check_State(dt35_left.dt35_num, STATE_TIMEOUT_COUNT);
        Device_Offline_State.dt35_state[3] = Check_State(dt35_right.dt35_num, STATE_TIMEOUT_COUNT);
        for (uint8_t i = 0; i < 4; i++)
        {
            //0为离线，1为在线
            Device_Offline_State.dji_2006_state[i] = Check_State(helm_motor[i].num, STATE_TIMEOUT_COUNT);
            Device_Offline_State.vesc_motor_state[i] = Check_State(vesc_motor[i].num, STATE_TIMEOUT_COUNT);
        }

        osDelay(5);
    }
}

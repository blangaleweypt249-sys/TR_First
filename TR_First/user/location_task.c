#include "location_task.h"
#include "mymain.h"
#include "init_task.h"

float dis=500;

//左侧 DT35 防撞墙：车体系下 +y 就是往左，120mm 以内不许再靠近
void Left_Cbf(DT35_Move *dt35, Speed *body_speed)
{
    float d = dt35->actual_dt35_distance;
    float allow;

    if (d < 20.0f)//没数据，后、右两路恒为 0
        return;

    allow = 1.5f * (d - 120.0f);//越近允许的靠近速度越小，120mm 处为 0
    if (allow < 0.0f)
        allow = 0.0f;

    if (body_speed->vy > allow)
        body_speed->vy = allow;
}

void LocationTask(void *argument)
{
    (void)argument;

    while (boot_done == 0U)//等外设和机构初始化完成
        osDelay(10);

    for (;;)
    {
        dt35_left.target_dt35_distance =dis;
        Left_Cbf(&dt35_left, &body_speed);
        if (osMessageQueueGet(imuQueueHandle, receive_imu, NULL, 0U) == osOK)
        {
            Imu_Analysis(receive_imu);
        }
        if (osMessageQueueGet(dt35QueueHandle, receive_dt35, NULL, 0U) == osOK)
        {
            DT35_Analysis(receive_dt35);
        }
        osDelay(5);
    }
}

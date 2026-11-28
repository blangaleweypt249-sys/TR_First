#include "climb_task.h"
#include "init_task.h"
#include "Leg.h"
#include "Handle.h"
#include "DJI.h"
#include "helm_chassis.h"
#include "cmsis_os2.h"

float up_distance = 0;

// 小轮点动：PE1 按住前进、PE0 按住后退，1m/s
void ClimbTask(void *argument)
{
    static uint8_t front_out; // 前腿 1 伸出
    static uint8_t mid_out;   // 中腿 1 伸出
    static float speed_cmd;   // 规划后的当前速度
    float speed;              // 按键给的目标速度

    (void)argument;

    while (boot_done == 0U) // 等外设和机构初始化完成
        osDelay(10);

    for (;;)
    {
        // PA1 前腿、PA0 中腿：拨一次翻一次
        if (Handle_KeyChange(eKeyUp4))
        {
            front_out = !front_out;
            Leg_Front(front_out);
        }
        if (Handle_KeyChange(eKeyUp5))
        {
            mid_out = !mid_out;
            Leg_Mid(mid_out);
        }

        // PC3 后腿只在 0 和 40 之间互切
        if (Handle_KeyChange(eKeyUp6))
        {
            if (up_distance > 20.0f)
                up_distance = 0.0f;
            else
                up_distance = 40.0f;
        }

        // PD4 五位开关中间：后腿 18 和 0 互切
        if (Handle_KeyRelease(eKeyOk))
        {
            if (up_distance > 9.0f)
                up_distance = 0.0f;
            else
                up_distance = 15.0f;
        }

        // PE6 前腿伸 + 后腿20
        if (Handle_KeyRelease(eKeyRight1))
        {
            front_out = 1U;
            Leg_Front(1U);
            osDelay(170);
            up_distance = 22.0f;
        }
        // PE3 前腿伸 + 后腿40
        if (Handle_KeyRelease(eKeyRight2))
        {
            front_out = 1U;
            Leg_Front(1U);
            osDelay(170);
            up_distance = 40.5f;
        }
        // PE4 前腿收 + 后腿20
        if (Handle_KeyRelease(eKeyRight3))
        {
            front_out = 0U;
            Leg_Front(0U);
            up_distance = 22.0f;
        }
        // PE5 前腿收 + 后腿0
        if (Handle_KeyRelease(eKeyRight4))
        {
            front_out = 0U;
            Leg_Front(0U);
            up_distance = 0.0f;
        }

        // 后腿伸缩，原来在底盘任务里发，挪到腿这边
        Climb_Leg(up_distance);
        if (front_flag == 1)
        {
            Leg_Front(0); // 前腿收回
        }

        if (handle_key[eKeyOmni5].key) // PD7 五位开关上：前进
            speed = 0.5f;
        else if (handle_key[eKeyOmni3].key) // PD5 五位开关下：后退
            speed = -0.5f;
        else
            speed = 0.0f;

        // 斜率规划：一拍走 0.002 加速、0.004 减速
        speed_cmd = speed_planning(speed, &speed_cmd, 0.002f, 0.004f);

        DJI_Speed(&leg_Left_2006, -speed_cmd * ms_to_rpm);
        DJI_Speed(&leg_Right_2006, speed_cmd * ms_to_rpm); // 镜像导致

        osDelay(5);
    }
}

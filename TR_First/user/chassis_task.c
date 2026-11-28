#include "chassis_task.h"
#include "mymain.h"
#include "location_task.h"
#include "location_task.h"

Speed world_speed;
Speed body_speed;
float speed_k = 0.0045f; // 最大4.5m/s

//肩键：按一次车身目标角叠 90°，左肩 +，右肩 -；只认"按下"那一档
static void car_angle_key(void)
{
    static uint8_t shoulder_last;

    if (handle_data.shoulder[0] == 1U && (shoulder_last & 0x01U) == 0U)
        car_angle.target_angle = wrap_pi(car_angle.target_angle + PI / 2.0f);
    if (handle_data.shoulder[1] == 1U && (shoulder_last & 0x02U) == 0U)
        car_angle.target_angle = wrap_pi(car_angle.target_angle - PI / 2.0f);
    shoulder_last = (uint8_t)((handle_data.shoulder[0] == 1U) | ((handle_data.shoulder[1] == 1U) << 1));
}

//PA5 开关切高低速：默认高速 0.0045，切一次 0.0010，再切回 0.0045
static void chassis_speed_key(void)
{
    static uint8_t low_speed;

    if (Handle_KeyChange(eKeyUp2))
    {
        low_speed = !low_speed;
        speed_k = low_speed ? 0.0010f : 0.0045f;
    }
}

void ChassisTask(void *argument)
{
    static uint32_t tick;

    (void)argument;

    while (boot_done == 0U)//等外设和机构初始化完成
        osDelay(10);

    tick = osKernelGetTickCount();

    for (;;)
    {
        chassis_speed_key();//PA5 高低速

        //锁车或者与遥控失联(PA6锁车)
        if (handle_lock)
        {
            body_speed.vx = 0.0f;
            body_speed.vy = 0.0f;
            body_speed.vw = 0.0f;
        }
        else
        {
             body_speed.vy = handle_value[1] * speed_k;
             body_speed.vx = handle_value[0] * speed_k;
        }
        car_angle.actual_angle = Imu_angle;
        car_angle_key();
        //if (!handle_lock)
            //Located_By_Left_DT35(&dt35_left, &world_speed, Imu_angle);//左侧 DT35 定距 500mm
        //World_to_body(&world_speed, &body_speed, Imu_angle);
        if (!handle_lock)//锁车不跟姿态，helm_calculate 走静止分支
        {
            car_angle_maintain(&body_speed, &car_angle);
        }
        helm_calculate(&body_speed, &helm_chassis);

        //hfdcan1 上只有这 8 台：4 个舵 + 4 个轮，算完立刻发
        for (uint8_t i = 0; i < 4; i++)
        {
            DJI_Position(&helm_motor[i], helm_chassis.set_angle[i]);
        }
        for (uint8_t i = 0; i < 4; i++)
        {
            if (handle_lock)
                vesc_set_rpm(&vesc_motor[i], 0.0f);//锁车直接给0，不等VESC超时
            else
                vesc_set_rpm(&vesc_motor[i], helm_chassis.helm_speed[i] *
                    CosKspeed(helm_chassis.set_angle[i], wrap_angle(helm_motor[i].Feedback_angle)));
        }

        tick += 5U;//200Hz，hfdcan1 一拍发完
        if (osDelayUntil(tick) != osOK)
            tick = osKernelGetTickCount();//落后了重新对齐，别连发
    }
}

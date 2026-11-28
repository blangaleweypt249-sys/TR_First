#include "arm_task.h"
#include "mymain.h"
#include "Up_control.h"

//遥控：按键 → 姿态/动作，状态都在 Up_control 里
static void Arm_Key(void)
{
    if (Handle_KeyRelease(eKeyLeft1)) // PD10 取地面一层，两个臂都动
        Arm_SuckGround1();

    if (Handle_KeyRelease(eKeyLeft2)) // PD11 取地面二层，两个臂都动
    {
        Arm_MoveTo(&left_arm, &layer2_pos[0]);
        Arm_MoveTo(&right_arm, &layer2_pos[4]);
    }

    if (Handle_KeyRelease(eKeyLeft3)) // PD12 取天空块，两个臂都动
    {
        up_state = eUpStoreSky2;
        Arm_MoveTo(&left_arm, &sky_block_pos[0]);
        Arm_MoveTo(&right_arm, &sky_block_pos[4]);
    }

    if (Handle_KeyRelease(eKeyLeft4)) // PD13 取球，只动左臂
        Arm_MoveTo(&left_arm, suck_ball_pos);

    if (Handle_KeyRelease(eKeyLeft5)) // PD14 回零，顺带退出竖排状态
        Arm_HomeAll();

    if (Handle_KeyRelease(eKeyLeft6)) // PD15 取球完毕，只动左臂
        Arm_MoveTo(&left_arm, ball_done_pos);

    if (Handle_KeyRelease(eKeyLeft7)) // PA8 竖直放置等待，两个臂一起动
        Arm_EnterVertical();

    if (Handle_KeyRelease(eKeyLeft8)) // PA11 竖直放置，两个臂一起动
    {
        Arm_PreparePutLeft();
        Arm_PreparePutRight();
    }

    if (Handle_KeyRelease(eKeyLeft9)) // PA12 滑台两档：400 / 0
    {
        if (slide_pos > 200.0f)
            slide_pos = 0.0f;
        else
            slide_pos = 420.0f;
    }

    if (Handle_KeyRelease(eKeyDown1)) // PE1 左臂存块，奇数次存、偶数次取出
        Arm_LeftStore_Toggle();

    if (Handle_KeyRelease(eKeyDown2)) // PE0 右臂存块，第一次进、第二次退
    {
        if (up_state == eUpStoreSky)
            Arm_ExitStoreSky();
        else
            Arm_EnterStoreSky();
    }

    if (Handle_KeyChange(eKeyUp3)) // 顶部3 吸盘切换
        Arm_Suction_Toggle();
}

//机械臂：5010 和 RS 一帧串着发，10ms 一拍
void ArmTask(void *argument)
{
    static uint32_t tick;

    (void)argument;

    while (boot_done == 0U)//等外设和机构初始化完成
        osDelay(10);

    tick = osKernelGetTickCount();

    for (;;)
    {
        if (M5010_pos_flag) //200ms 读一次大臂位置
        {
            M5010_pos_flag = 0U;
            MG5010E_ReadPosition(&left_arm.joint1);
            MG5010E_ReadPosition(&right_arm.joint1);
        }

        Arm_Key();       // 遥控
        Arm_StageTick(); //三个分段状态机

        if (arm_set_home == 0) // 标零模式下不发运动指令
        {
            Arm_Send(&left_arm); //关节 1~4 一串发完，中间不等待
            Arm_Send(&right_arm);
            DJI_Position(&right_arm.slide, slide_pos);
        }

        Arm_Suction_Off();

        tick += 10U;//100Hz
        if (osDelayUntil(tick) != osOK)
            tick = osKernelGetTickCount();//落后了重新对齐，别连发
    }
}

#include "arm_task.h"
#include "mymain.h"
#include "Up_control.h"

//遥控：按键 → 姿态/动作，状态都在 Up_control 里
static void Arm_Key(void)
{
    if (Handle_KeyRelease(eKeyLeft1)) // PD10 吸地面1层块，两个臂都动
        Arm_SuckGround1();

    if (Handle_KeyRelease(eKeyLeft2)) // PD11 回零，同时把竖排的计数清掉
        Arm_HomeAll();

    if (Handle_KeyRelease(eKeyLeft3)) // PD12 吸球，只动左臂
        Arm_MoveTo(&left_arm, suck_ball_pos);

    if (Handle_KeyRelease(eKeyLeft4)) // PD13 取两个天空块，两臂一起动
    {
        up_state = eUpStoreSky2;
        Arm_MoveTo(&left_arm, &sky_block_pos[0]);
        Arm_MoveTo(&right_arm, &sky_block_pos[4]);
    }

    if (Handle_KeyRelease(eKeyLeft5)) // PD14 存天空块，第一次进、第二次退
    {
        if (up_state == eUpStoreSky) // 退出：回存块位吸住，稳了放开上吸盘，再 1.5 秒回零
            Arm_ExitStoreSky();
        else // 进入
            Arm_EnterStoreSky();
    }

    if (Handle_KeyRelease(eKeyLeft6)) // PD15 取球完毕，只动左臂
        Arm_MoveTo(&left_arm, ball_done_pos);

    // PA11 竖直吸取，两个臂一起动
    if (Handle_KeyRelease(eKeyLeft8))
    {
        Arm_MoveTo(&left_arm, &vertical_pos[0]);
        Arm_MoveTo(&right_arm, &vertical_pos[4]);
    }

    // PA5 进入竖直排列预备态：两个臂一起动，末端 3、4 关节走 1 秒插值
    if (Handle_KeyChange(eKeyUp2))
        Arm_EnterVertical();

    // 竖直排列状态下：PE1 左臂竖直放置，PE0 右臂竖直放置
    if (up_state == eUpVertical)
    {
        if (Handle_KeyRelease(eKeyDown1)) // PE1
            Arm_PreparePutLeft();

        if (Handle_KeyRelease(eKeyDown2)) // PE0
            Arm_PreparePutRight();
    }

    // PA12 滑台两档：400 / 0
    if (Handle_KeyRelease(eKeyLeft9))
    {
        if (slide_pos > 200.0f)
            slide_pos = 0.0f;
        else
            slide_pos = 400.0f;
    }

    if (Handle_KeyRelease(eKeyLeft7)) // PA8 吸取二层，两个臂都动
    {
        Arm_MoveTo(&left_arm, &layer2_pos[0]);
        Arm_MoveTo(&right_arm, &layer2_pos[4]);
    }

    if (Handle_KeyRelease(eKeyRight5)) // PE2 左臂取二层
        Arm_MoveTo(&left_arm, left_layer2_pos);

    if (Handle_KeyChange(eKeyUp3))
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

#include "Up_control.h"
#include "Handle.h"
#include "tim.h"

// 标零标志位(1时停止运控手动摆好)
uint8_t arm_set_home;
volatile uint8_t arm_boot_cnt;
// 调试滑台位置用
float slide_pos;

Arm left_arm;
Arm right_arm;
Up_State up_state;

static uint8_t suction_off_flag; // 放下之后等一会全关
static uint32_t suction_off_tick;
static Arm_Stage ground_home[2];                         // 吸地面1层块回零分段：0 左臂 1 右臂
static uint8_t store_sky_suction[2];                     // 进存块前的吸盘状态：0 左臂 1 右臂
static uint8_t vert_gate = 1U;                           // 0 竖直放置等待(等 RS00) 1 竖直放置(等 5010)，初值让第一次按下变 0
static Arm_Stage vert_home;                              // 竖直排列分段：左臂
static Arm_Stage vert_right;                             // 竖直排列分段：右臂
static uint8_t vert_interp_joint;                        // 竖直排列时右臂哪个关节走 1 秒插值：0 无 3 关节3 4 关节4
static const float *vert_pose;                           // 本次竖直排列用的位姿表
static Arm *const arm_list[2] = {&left_arm, &right_arm}; // 按 id 索引

// 吸块位置：前 4 个左臂、后 4 个右臂
// 吸取1层地块(从前面抓)
static const float suck_block_pos[8] =
{
    -142.7f, -0.78f, -0.00f,  0.98f,
    128.5f,  0.92f, -0.08f, 1.41f,
};

// 吸球位置：只有左臂动
static const float suck_ball_pos[4] =
{
    -68.0f, -0.73f, -0.123f, -1.08f,
};

// 取球完毕：只有左臂动 1
static const float ball_done_pos[4] =
{
    -68.0f, -0.73f, -0.123f, 0.00f,
};

// 取两个天空块(横着吸) 1 左臂稍微往右一点吸取
static const float sky_block_pos[8] =
{
    -165.0f, -1.0f, -0.025f, 1.15f,
    148.0f, 1.1f, -0.009f, 1.56f,
};

// 存天空块(只有左臂动) 1
static const float store_sky_pos[4] =
{
    -0.9f, -0.5f, -0.0f, 1.85f,
};

// 竖直放置等待
static const float vertical_pos[8] =
{
    0.0f, -0.5f, 0.0f, -0.5f,
    69.0f, 1.3f, -1.8f, 0.5f,
};

// 竖直放置
static const float vertical_put_pos[8] =
{
    -142.0f, -0.35f, -0.00f, 1.3f,
    120.0f, 2.15f, -1.8f, -1.4f,
};

// 吸取二层地块 1
static const float layer2_pos[8] =
{
    -83.8f, -0.56f, -0.045f, 0.20f,
    69.0f, 0.73f, 0.0f, 0.78f,
};

// 左臂取二层 1
static const float left_layer2_pos[4] =
{
    -83.8f, -0.56f, -0.045f, 0.20f,
};

// TIM7 每 10ms 调一次
void Arm_PollTimer(void)
{
    static uint8_t div;

    // 标零模式下不发运动指令，读位置照发
    if (arm_set_home == 0U)
    {
        MG5010E_SpeedPos(&left_arm.joint1, left_arm.target[0], 20.0f);
        MG5010E_SpeedPos(&right_arm.joint1, right_arm.target[0], 20.0f);
    }

    if (++div < 20U)
        return;

    div = 0U;
    MG5010E_ReadPosition(&left_arm.joint1);
    MG5010E_ReadPosition(&right_arm.joint1);
}

// 按目标下发末端 3 个关节，大臂 5010 由 TIM7 每 10ms 发
static void Arm_Send(Arm *arm)
{
    RS00_Mit(&arm->joint2, 0, arm->target[1], 0, 16, 4, 0, 0);
    osDelay(1);

    // 竖直排列：右臂那个末端关节走 1 秒插值
    RS05_Mit(&arm->joint3, 0, arm->target[2], 0, 8, 2,
             (arm == &right_arm && (vert_interp_joint == 3U)) ? 1U : 0U, 1000U);
    osDelay(1);
    RS05_Mit(&arm->joint4, 0, arm->target[3], 0, 15, 2,
             (arm == &right_arm && (vert_interp_joint == 4U)) ? 1U : 0U, 1000U);
    osDelay(1);

    if (arm == &right_arm) // 插值走完就把开关关掉
    {
        if ((vert_interp_joint == 3U) && !arm->joint3.mit.position_interpolation.state)
            vert_interp_joint = 0U;
        if ((vert_interp_joint == 4U) && !arm->joint4.mit.position_interpolation.state)
            vert_interp_joint = 0U;
    }
}

void Arm_MoveTo(Arm *arm, const float *pos)
{
    arm->target[0] = pos[0];
    arm->target[1] = pos[1];
    arm->target[2] = pos[2];
    arm->target[3] = pos[3];
}

void Arm_Home(Arm *arm)
{
    arm->target[0] = 0.0f;
    arm->target[1] = 0.0f;
    arm->target[2] = 0.0f;
    arm->target[3] = 0.0f;
}

void Arm_SuckBlock(Arm *arm)
{
    Arm_MoveTo(arm, &suck_block_pos[arm->id * 4U]);
}

// 吸取：气泵开、真空阀关
void Arm_Suck(Arm *arm)
{
    HAL_GPIO_WritePin(arm->pump_port, arm->pump_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(arm->vac_port, arm->vac_pin, GPIO_PIN_RESET);
}

// 放下：气泵关、真空阀开
void Arm_Release(Arm *arm)
{
    HAL_GPIO_WritePin(arm->pump_port, arm->pump_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(arm->vac_port, arm->vac_pin, GPIO_PIN_SET);
}

// 全关
void Arm_Off(Arm *arm)
{
    HAL_GPIO_WritePin(arm->pump_port, arm->pump_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(arm->vac_port, arm->vac_pin, GPIO_PIN_RESET);
}

// 0 全关 1 吸取 2 放下
void Arm_Suction(Arm *arm, uint8_t cmd)
{
    arm->suction = cmd;

    if (cmd == 1U)
    {
        Arm_Suck(arm);
    }
    else if (cmd == 2U)
    {
        Arm_Release(arm);
    }
    else
    {
        Arm_Off(arm);
    }
}

// 吸地面1层块状态下回零：5010、RS00 走一半再放末端，两个 5010 都到 5 度内退状态
static void Arm_GroundStage(void)
{
    if (up_state != eUpSuckGround1)
        return;

    for (uint8_t i = 0; i < 2U; i++)
    {
        if (ground_home[i].stage != 1U)
            continue;

        if (fabsf(arm_list[i]->joint1.pos - ground_home[i].joint1_start) >= 0.5f * fabsf(ground_home[i].joint1_start) &&
            fabsf(arm_list[i]->joint2.feedback.pos_fed - ground_home[i].joint2_start) >= 0.5f * fabsf(ground_home[i].joint2_start))
        {
            arm_list[i]->target[2] = 0.0f;
            arm_list[i]->target[3] = 0.0f;
            ground_home[i].stage = 2U;
        }
    }

    if (ground_home[0].stage == 2U && ground_home[1].stage == 2U &&
        fabsf(left_arm.joint1.pos) < 5.0f && fabsf(right_arm.joint1.pos) < 5.0f)
    {
        ground_home[0].stage = 0U;
        ground_home[1].stage = 0U;
        up_state = eUpIdle;
    }
}

// 竖直排列：左臂整条先动，右臂等左臂那个关节走到 90% 再跟；右臂 5010 到位退状态
// 等待：等左臂 4 关节到 90%，右臂整条一起走
// 放置：等左臂 5010 到 90%，右臂 1、2 先走到 80%，末端再走
static void Arm_VerticalStage(void)
{
    if (up_state != eUpVertical)
        return;

    if (vert_home.stage == 1U)
    {
        if (vert_gate == 0U) // 等左臂 4 关节走到 90%
        {
            if (fabsf(left_arm.joint4.feedback.pos_fed - vert_home.joint4_start) >=
                0.9f * fabsf(vert_pose[3] - vert_home.joint4_start))
                vert_home.stage = 3U;
        }
        else // 等左臂 5010 走到 90%
        {
            if (fabsf(left_arm.joint1.pos - vert_home.joint1_start) >=
                0.9f * fabsf(vert_pose[0] - vert_home.joint1_start))
            {
                vert_right.joint1_start = right_arm.joint1.pos; // 右臂 1、2 先走
                vert_right.joint2_start = right_arm.joint2.feedback.pos_fed;
                right_arm.target[0] = vert_pose[4];
                right_arm.target[1] = vert_pose[5];
                vert_home.stage = 2U;
            }
        }
    }

    // 放置：右臂 1、2 走到 80% 再放末端
    if (vert_home.stage == 2U &&
        fabsf(right_arm.joint1.pos - vert_right.joint1_start) >= 0.7f * fabsf(vert_pose[4] - vert_right.joint1_start) &&
        fabsf(right_arm.joint2.feedback.pos_fed - vert_right.joint2_start) >= 0.7f * fabsf(vert_pose[5] - vert_right.joint2_start))
        vert_home.stage = 3U;

    if (vert_home.stage == 3U) // 右臂跟上
    {
        Arm_MoveTo(&right_arm, &vert_pose[4]);
        if (vert_gate == 0U)
            vert_interp_joint = 3U; // 等待：第 3 个电机走 1 秒插值
        else
            vert_interp_joint = 4U; // 放置：第 4 个电机走 1 秒插值
        vert_home.stage = 4U;
    }

    if (vert_home.stage == 4U && fabsf(right_arm.joint1.pos - vert_pose[4]) < 5.0f) // 右臂 5010 到位
    {
        vert_home.stage = 0U;
        up_state = eUpIdle;
    }
}

// 遥控：左侧1 吸块、左侧2 回零、顶部3 吸盘切换
static void Arm_Key(void)
{
    Arm_GroundStage();
    Arm_VerticalStage();

    if (Handle_KeyRelease(eKeyLeft1)) // PD10 吸地面1层块，两个臂都动
    {
        up_state = eUpSuckGround1;
        ground_home[0].stage = 0U;
        ground_home[1].stage = 0U;
        Arm_SuckBlock(&left_arm);
        Arm_SuckBlock(&right_arm);
    }

    if (Handle_KeyRelease(eKeyLeft2)) // PD11 回零，同时把竖排的计数清掉
    {
        if (up_state == eUpSuckGround1) // 末端等 5010 和 RS00 走一半再动
        {
            for (uint8_t i = 0; i < 2U; i++)
            {
                ground_home[i].joint1_start = arm_list[i]->joint1.pos;
                ground_home[i].joint2_start = arm_list[i]->joint2.feedback.pos_fed;
                arm_list[i]->target[0] = 0.0f; // 5010、RS00 先回零
                arm_list[i]->target[1] = 0.0f;
                arm_list[i]->target[2] = arm_list[i]->joint3.feedback.pos_fed; // 末端先保持
                arm_list[i]->target[3] = arm_list[i]->joint4.feedback.pos_fed;
                ground_home[i].stage = 1U;
            }
        }
        else
        {
            Arm_Home(&left_arm);
            Arm_Home(&right_arm);
        }

        if (up_state == eUpStoreSky) // 存块状态回零：关夹爪、退状态，气泵回到进状态前
        {
            HAL_GPIO_WritePin(up_jia_GPIO_Port, up_jia_Pin, GPIO_PIN_RESET);
            Arm_Suction(&left_arm, store_sky_suction[0]);
            Arm_Suction(&right_arm, store_sky_suction[1]);
            suction_off_flag = 0U; // 还原之后不许那个 10 秒全关再动气泵
            up_state = eUpIdle;
        }

        if (up_state == eUpVertical) // 竖直排列也一起退
        {
            vert_home.stage = 0U;
            up_state = eUpIdle;
        }
    }

    if (Handle_KeyRelease(eKeyLeft3)) // PD12 吸球，只动左臂
    {
        Arm_MoveTo(&left_arm, suck_ball_pos);
    }

    if (Handle_KeyRelease(eKeyLeft4)) // PD13 取两个天空块，两个臂都动
    {
        Arm_MoveTo(&left_arm, &sky_block_pos[0]);
        Arm_MoveTo(&right_arm, &sky_block_pos[4]);
    }

    if (Handle_KeyRelease(eKeyLeft5)) // PD14 存天空块，只有左臂动
    {
        up_state = eUpStoreSky;
        store_sky_suction[0] = left_arm.suction;
        store_sky_suction[1] = right_arm.suction;
        Arm_MoveTo(&left_arm, store_sky_pos);
        HAL_GPIO_WritePin(up_jia_GPIO_Port, up_jia_Pin, GPIO_PIN_SET); // PD0 上夹爪打开
    }

    if (Handle_KeyRelease(eKeyLeft6)) // PD15 取球完毕，只动左臂
    {
        Arm_MoveTo(&left_arm, ball_done_pos);
    }

    // PA11 竖直吸取，两个臂一起动
    if (Handle_KeyRelease(eKeyLeft8))
    {
        Arm_MoveTo(&left_arm, &vertical_pos[0]);
        Arm_MoveTo(&right_arm, &vertical_pos[4]);
    }

    // PA5 竖直排列：第一次竖直放置等待，第二次竖直放置，来回切
    if (Handle_KeyChange(eKeyUp2))
    {
        vert_gate = !vert_gate; // 0 等待 1 放置
        if (vert_gate == 0U)
            vert_pose = vertical_pos;
        else
            vert_pose = vertical_put_pos;

        up_state = eUpVertical;
        vert_home.joint1_start = left_arm.joint1.pos;
        vert_home.joint2_start = left_arm.joint2.feedback.pos_fed;
        vert_home.joint3_start = left_arm.joint3.feedback.pos_fed;
        vert_home.joint4_start = left_arm.joint4.feedback.pos_fed;
        vert_home.stage = 1U;

        Arm_MoveTo(&left_arm, vert_pose); // 左臂整条先走

        right_arm.target[0] = right_arm.joint1.pos; // 右臂先锁住
        right_arm.target[1] = right_arm.joint2.feedback.pos_fed;
        right_arm.target[2] = right_arm.joint3.feedback.pos_fed;
        right_arm.target[3] = right_arm.joint4.feedback.pos_fed;
    }

    // PA12 滑台两档：430 / 0
    if (Handle_KeyRelease(eKeyLeft9))
    {
        if (slide_pos > 200.0f)
            slide_pos = 0.0f;
        else
            slide_pos = 380.0f;
    }

    if (Handle_KeyRelease(eKeyLeft7)) // PA8 吸取二层，两个臂都动
    {
        Arm_MoveTo(&left_arm, &layer2_pos[0]);
        Arm_MoveTo(&right_arm, &layer2_pos[4]);
    }

    if (Handle_KeyRelease(eKeyRight5)) // PE2 左臂取二层
    {
        Arm_MoveTo(&left_arm, left_layer2_pos);
    }

    if (Handle_KeyChange(eKeyUp3))
    {
        if (left_arm.suction == 1U) // 吸着 -> 放下
        {
            Arm_Suction(&left_arm, 2U);
            suction_off_flag = 1U;
            suction_off_tick = HAL_GetTick();
        }
        else // 开着或已放下 -> 吸
        {
            Arm_Suction(&left_arm, 1U);
            suction_off_flag = 0U;
        }

        if (up_state != eUpStoreSky) // 存块状态只有左臂动，右臂不跟着切
            Arm_Suction(&right_arm, left_arm.suction);
    }
}

// 按反馈补启动：没回馈的电机重发 主动上报+模式+使能，每台最多 3 次
static void Arm_InitCheck(void)
{
    static uint8_t retry[6];
    static uint8_t cont;
    static RS00 *rs00_motor[2] = {&left_arm.joint2, &right_arm.joint2};
    static RS05 *rs05_motor[4] = {&left_arm.joint3, &left_arm.joint4, &right_arm.joint3, &right_arm.joint4};

    if (++cont < 10U) // 10ms 一轮，100ms 查一次
    {
        return;
    }
    cont = 0;

    for (uint8_t i = 0; i < 2U; i++)
    {
        if (HAL_GetTick() - rs00_motor[i]->last_tick > 200U && retry[i] < 3U)
        {
            RS00_SetActiveReporting(rs00_motor[i]);
            RS00_SetMode(rs00_motor[i], mit);
            RS00_Enable(rs00_motor[i]);
            retry[i]++;
        }
    }
    for (uint8_t i = 0; i < 4U; i++)
    {
        if (HAL_GetTick() - rs05_motor[i]->last_tick > 200U && retry[i + 2U] < 3U)
        {
            RS05_SetActiveReporting(rs05_motor[i]);
            RS05_SetMode(rs05_motor[i], mit);
            RS05_Enable(rs05_motor[i]);
            retry[i + 2U]++;
        }
    }
}

void Up_Init(void)
{
    arm_set_home = 0; // 1: 只标零，把臂摆到零位烧一次，标完改回 0 再烧

    left_arm.id = 0U;
    right_arm.id = 1U;

    // 关节1 大臂第一个电机
    MG5010E_Init(&left_arm.joint1, &hfdcan3, 1, 36.0f);
    MG5010E_Init(&right_arm.joint1, &hfdcan3, 2, 36.0f);

    // 关节2 大臂第二个电机
    RS00_Init(&left_arm.joint2, arm_set_home, &hfdcan3, 23, mit);
    osDelay(1);
    RS00_Init(&right_arm.joint2, arm_set_home, &hfdcan3, 40, mit);
    osDelay(1);

    // 关节3 小臂 / 关节4 腕
    RS05_Init(&left_arm.joint3, arm_set_home, &hfdcan3, 14, mit);
    osDelay(1);
    RS05_Init(&left_arm.joint4, arm_set_home, &hfdcan3, 25, mit);
    osDelay(1);
    RS05_Init(&right_arm.joint3, arm_set_home, &hfdcan3, 22, mit);
    osDelay(1);
    RS05_Init(&right_arm.joint4, arm_set_home, &hfdcan3, 24, mit);
    osDelay(1);

    // 右臂底座 2006 滑台，和腿共用 board 6
    DJI_Init(&right_arm.slide, &hfdcan2, DJI_M2006, 6, 3);

    DJI_SignZero(&right_arm.slide);
    right_arm.has_slide = 1U;

    if (arm_set_home == 1) // MG5010 把当前位置设成原点，断电重上电才生效
    {
        MG5010E_SetOrigin(&left_arm.joint1);
        osDelay(50);
        MG5010E_SetOrigin(&right_arm.joint1);
        osDelay(50);
    }

    // 末端气路
    left_arm.pump_port = GPIOA;
    left_arm.pump_pin = GPIO_PIN_8; // 左气泵
    left_arm.vac_port = GPIOC;
    left_arm.vac_pin = GPIO_PIN_11; // 左真空阀

    right_arm.pump_port = GPIOC;
    right_arm.pump_pin = GPIO_PIN_9; // 右气泵
    right_arm.vac_port = GPIOC;
    right_arm.vac_pin = GPIO_PIN_10; // 右真空阀

    Arm_Off(&left_arm);
    Arm_Off(&right_arm);

    // 上电即回零
    Arm_Home(&left_arm);
    Arm_Home(&right_arm);
}

void PickTask(void *argument)
{
    (void)argument;

    MG5010E_Init(&left_arm.joint1, &hfdcan3, 1, 36.0f);
    MG5010E_Init(&right_arm.joint1, &hfdcan3, 2, 36.0f);

    HAL_TIM_Base_Start_IT(&htim7); // 10ms 喂一次 5010、200ms 读一次位置，上电就开始喂

    osDelay(1500); // 等 RS 电机上电完成，trc 实测最后一条上电报文约 1.6s，晚起的靠 Arm_InitCheck 补

    Up_Init(); // 机构初始化，目标都置 0，上电就回零点

    for (;;)
    {
        Arm_InitCheck(); // 按反馈补启动
        Arm_Key();       // 遥控

        if (arm_set_home == 0)
        {
            Arm_Send(&left_arm);
            Arm_Send(&right_arm);
            DJI_Position(&right_arm.slide, slide_pos);
        }

        if (suction_off_flag && (HAL_GetTick() - suction_off_tick > 10000U)) // 放下 10 秒后全关
        {
            suction_off_flag = 0U;

            if (left_arm.suction == 2U) // 只关还停在放下的臂，别把吸着的关掉
                Arm_Suction(&left_arm, 0U);

            if (right_arm.suction == 2U)
                Arm_Suction(&right_arm, 0U);
        }

        osDelay(10);
    }
}

#include "Up_control.h"
#include "Handle.h"

// 标零标志位(1时停止运控手动摆好)
uint8_t arm_set_home;
volatile uint8_t M5010_pos_flag; //200ms 读一次大臂位置
// 调试滑台位置用
float slide_pos;

Arm left_arm;
Arm right_arm;
Up_State up_state;

static uint8_t suction_off_flag;                         // 放下之后等一会全关
static uint32_t suction_off_tick;
static uint8_t up_suction_flag;                          // 上吸盘状态：0 关 1 吸 2 放
static uint8_t vert_hold[2];                             // 1 = 这一臂在竖直动作，关节2 换大 kp
static Arm_Stage ground_home[2];                         // 吸地面1层块回零分段：0 左臂 1 右臂
static Arm_Stage store_wait;                             // 存天空块分段：0 空闲 1 进入等稳 2 进入等 1 秒 3 进入等 0.5 秒 4 退出等稳 5 退出等 1 秒
static Arm_Stage put_wait;                               // PE0 右臂放置分段：0 空闲 1 等 5010/RS00 稳住
static Arm_Stage left_store;                             // 左臂存块分段：0 空闲 1 等到位 2 存块等 100ms 3 取出等 1 秒 4 回零后等 1 秒
static uint8_t left_store_takeout = 1U;                  // 左臂存块奇偶：0 存 1 取出；初值 1 让第一次按下翻转成存
static void Arm_LeftStoreStage(void);
static Arm *const arm_list[2] = {&left_arm, &right_arm}; // 按 id 索引
// 吸块位置：前 4 个左臂、后 4 个右臂
// 吸取1层地块(从前面抓)
const float suck_block_pos[8] =
{
    -150.0f, -1.08f, -0.00f,  0.95f,
    138.0f,  1.16f, -0.00f, 1.32f,
};

// 吸球位置：只有左臂动
const float suck_ball_pos[4] =
{
    -66.0f, -0.73f, -0.00f, -1.05f,
};

// 取球完毕：只有左臂动 1
const float ball_done_pos[4] =
{
    -66.0f, -0.73f, -0.123f, 0.00f,
};

// 取两个天空块(横着吸) 1 左臂稍微往右一点吸取
const float sky_block_pos[8] =
{
    -163.0f, -1.0f, 0.05f, 1.2f,
    151.0f, 1.05f, -0.009f, 1.55f,
};

// 存天空块(右臂动)
const float store_sky_pos[4] =
{
    -4.0f, 1.15f, 0.0f, 2.07f,
};

// 竖直放置等待
const float vertical_pos[8] =
{
    -143.0f, -0.35f, -0.00f,  1.48f,
    103.0f, 1.58f, -1.4f, 2.00f,
};

// 竖直放置
const float vertical_put_pos[8] =
{
    -143.0f, -0.35f, -0.00f,  1.48f,
    103.0f, 1.58f, -1.6f, -1.34f,
};

// 吸取二层地块 1
const float layer2_pos[8] =
{
    -85.8f, -0.62f, -0.045f, 0.31f,
    72.5f, 0.6f, 0.0f, 0.75f,
};

// 左臂存块位：先写 0 号位，数值待标定
const float left_store_pos[4] =
{
    0.0f, -0.6f, 0.0f, 1.9f,
};

// TIM7 每 10ms 调一次，只置读位置标志，CAN 帧在 ArmTask 里发
void Arm_PollTimer(void)
{
    static uint8_t div;

    if (++div >= 20U) // 200ms 读一次位置
    {
        div = 0U;
        M5010_pos_flag = 1U;
    }
}

// 插值时长按位置误差算：五次多项式峰值速度是 1.875*Δ/T，
// 按 2.7rad/s 算 T(ms)=700*Δ，再限在 0.25~1.5 秒
static uint32_t Arm_InterpMs(float err)
{
    uint32_t t_ms = (uint32_t)(900.0f * fabsf(err));

    if (t_ms < 250U)
        t_ms = 250U;

    if (t_ms > 1500U)
        t_ms = 1500U;

    return t_ms;
}

// 关节当前位置：索引 0~3
static float Arm_JointPos(Arm *arm, uint8_t i)
{
    if (i == 0U)
        return arm->joint1.pos;
    if (i == 1U)
        return arm->joint2.feedback.pos_fed;
    if (i == 2U)
        return arm->joint3.feedback.pos_fed;

    return arm->joint4.feedback.pos_fed;
}

// 只改目标：清掉插值和竖直刚度，不启动插值
static void Arm_SetTarget(Arm *arm, const float *pos)
{
    vert_hold[arm->id] = 0U;
    arm->interp.mask = 0U;
    arm->target[0] = pos[0];
    arm->target[1] = pos[1];
    arm->target[2] = pos[2];
    arm->target[3] = pos[3];
}

// 掩码置位的关节按误差插值到 pos，其余关节直接到位
static void Arm_InterpTo(Arm *arm, const float *pos, uint8_t mask)
{
    float err = 0.0f;

    Arm_SetTarget(arm, pos);

    for (uint8_t i = 0U; i < 4U; i++)
    {
        float cur, e;

        if (((mask >> i) & 0x01U) == 0U)
            continue;

        cur = Arm_JointPos(arm, i);
        Interpolation_Init(&arm->interp.joint[i]);//从现在这一拍重新起曲线
        arm->interp.last[i] = cur;

        e = fabsf(pos[i] - cur);
        if (i == 0U)//关节1 是度，换成弧度和其它关节比
            e *= PI / 180.0f;

        if (e > err)
            err = e;
    }

    arm->interp.ms = Arm_InterpMs(err);
    arm->interp.mask = mask;
}

// 一个臂 4 个关节串着发：先按掩码把目标插值到这一拍的位置，再依次下发
void Arm_Send(Arm *arm)
{
    float cmd[4];
    uint8_t mask = arm->interp.mask;

    for (uint8_t i = 0U; i < 4U; i++)
    {
        cmd[i] = Interpolation_PositionCommand(&arm->interp.joint[i], arm->interp.last[i],
                                               arm->target[i],
                                               (mask >> i) & 0x01U, arm->interp.ms);
        arm->interp.last[i] = cmd[i];

        //这一路走完就把对应的位清掉
        if (((mask >> i) & 0x01U) != 0U && !arm->interp.joint[i].state)
            mask &= (uint8_t)~(1U << i);
    }
    arm->interp.mask = mask;

    MG5010E_SpeedPos(&arm->joint1, cmd[0], 17.0f);

    //竖直动作里右臂关节2 换现在这套 kp/kd，其他动作和回零都用旧参数 27/4
    RS00_Mit(&arm->joint2, 0, cmd[1], 0,
             vert_hold[arm->id] ? 50.0f : 27.0f, vert_hold[arm->id] ? 5.0f : 4.0f, 0, 0);

    RS05_Mit(&arm->joint3, 0, cmd[2], 0, 18, 2, 0, 0);
    RS05_Mit(&arm->joint4, 0, cmd[3], 0, 25, 2, 0, 0);
}

//改目标：直接到位，不插值
void Arm_MoveTo(Arm *arm, const float *pos)
{
    Arm_SetTarget(arm, pos);
}

//回零：直接到 0，不插值
void Arm_Home(Arm *arm)
{
    float zero[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    Arm_SetTarget(arm, zero);
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

// 0 全关 1 吸取 2放下
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

// 上吸盘：0 全关 1 吸取(开气泵关真空阀) 2 放下(关气泵开真空阀)
static void Sky_Suction(uint8_t cmd)
{
    up_suction_flag = cmd;

    if (cmd == 1U)
    {
        HAL_GPIO_WritePin(up_air_GPIO_Port, up_air_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(up_realair_GPIO_Port, up_realair_Pin, GPIO_PIN_RESET);
    }
    else if (cmd == 2U)
    {
        HAL_GPIO_WritePin(up_air_GPIO_Port, up_air_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(up_realair_GPIO_Port, up_realair_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(up_air_GPIO_Port, up_air_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(up_realair_GPIO_Port, up_realair_Pin, GPIO_PIN_RESET);
    }
}

// 等两个关节稳住：5010 看 joint1，RS 看 joint2 或 joint3；
static uint8_t Arm_WaitStill(Arm *arm, Arm_Stage *wait, uint8_t rs_joint, uint8_t check_end)
{
    float rs_pos = (rs_joint == 3U) ? arm->joint3.feedback.pos_fed : arm->joint2.feedback.pos_fed;

    // 5010 单位度、RS00/RS05 单位弧度，一拍变化小于这个只当没动
    if (fabsf(arm->joint1.pos - wait->last1) < 0.05f &&
        fabsf(rs_pos - wait->last2) < 0.002f &&
        (check_end == 0U || arm->interp.mask == 0U))
        wait->still++;
    else
        wait->still = 0U;

    wait->last1 = arm->joint1.pos;
    wait->last2 = rs_pos;

    return (wait->still >= 20U) || ((HAL_GetTick() - wait->tick) > 3000U);
}

// 存天空块：右臂走到位稳住 → 上吸盘吸 → 1 秒 → 右臂吸盘关 → 0.5 秒 → 右臂回零 → 右臂再开成吸取
static void Arm_StoreSkyStage(void)
{
    if (store_wait.stage == 0U)
        return;

    if (store_wait.stage == 1U) // 等右臂 5010、RS00 稳住，末端插值也要走完
    {
        if (Arm_WaitStill(&right_arm, &store_wait, 2U, 1U) == 0U)
            return;

        Sky_Suction(1U); // 上吸盘吸取
        store_wait.tick = HAL_GetTick();
        store_wait.stage = 2U;
        return;
    }

    if (store_wait.stage == 2U) // 1 秒后放开右臂吸盘，再等 100ms 全关
    {
        if (HAL_GetTick() - store_wait.tick < 1000U)
            return;

        Arm_Suction(&right_arm, 2U);
        suction_off_flag = 1U;
        suction_off_tick = HAL_GetTick();

        store_wait.tick = HAL_GetTick();
        store_wait.stage = 3U;
        return;
    }

    if (store_wait.stage == 4U) // 退出：等右臂回到存块位稳住，末端插值也要走完
    {
        if (Arm_WaitStill(&right_arm, &store_wait, 2U, 1U) == 0U)
            return;

        Sky_Suction(2U); // 位置稳了先放开上吸盘，等 100ms 再全关
        suction_off_flag = 1U;
        suction_off_tick = HAL_GetTick();

        store_wait.tick = HAL_GetTick();
        store_wait.stage = 5U;
        return;
    }

    if (store_wait.stage == 5U) // 退出：1.5 秒后回零
    {
        if (HAL_GetTick() - store_wait.tick < 1500U)
            return;

        Arm_Home(&right_arm);
        up_state = eUpIdle;
        store_wait.stage = 0U;
        return;
    }

    if (HAL_GetTick() - store_wait.tick < 1500U) // 1.5 秒后回零位，再开成吸取状态
        return;

    Arm_Home(&right_arm);
    Arm_Suction(&right_arm, 1U);
    store_wait.stage = 0U;
}

// 吸地面1层块、存两个天空块状态下回零：5010、RS00 走稳再放末端，末端也稳了退状态
static void Arm_GroundStage(void)
{
    if (up_state != eUpSuckGround1 && up_state != eUpStoreSky2)
        return;

    for (uint8_t i = 0; i < 2U; i++)
    {
        if (ground_home[i].stage == 0U || ground_home[i].stage == 3U)
            continue;

        if (ground_home[i].stage == 1U) // 连续稳 20 拍或者超时 3 秒就放末端
        {
            if (Arm_WaitStill(arm_list[i], &ground_home[i], 2U, 0U) == 0U)
                continue;

            arm_list[i]->target[2] = 0.0f; // 末端再回零
            arm_list[i]->target[3] = 0.0f;

            ground_home[i].still = 0U;
            ground_home[i].last1 = arm_list[i]->joint1.pos;
            ground_home[i].last2 = arm_list[i]->joint3.feedback.pos_fed;
            ground_home[i].tick = HAL_GetTick();
            ground_home[i].stage = 2U;
            continue;
        }

        // 末端走稳了就算这一臂回零完成
        if (Arm_WaitStill(arm_list[i], &ground_home[i], 3U, 0U) != 0U)
            ground_home[i].stage = 3U;
    }

    if (ground_home[0].stage == 3U && ground_home[1].stage == 3U) // 两臂都稳了退状态
    {
        ground_home[0].stage = 0U;
        ground_home[1].stage = 0U;
        up_state = eUpIdle;
    }
}

// PE0 右臂竖直放置：先让 5010、RS00 走到稳，末端再插值过去
static void Arm_PutStage(void)
{
    if (put_wait.stage == 0U)
        return;

    // 连续稳 20 拍或者超时 3 秒就放末端
    if (Arm_WaitStill(&right_arm, &put_wait, 2U, 0U) == 0U)
        return;

    put_wait.stage = 0U;
    Arm_InterpTo(&right_arm, &vertical_put_pos[4], 0x0CU);//末端 3、4 关节插值过去
}

//三个分段状态机每拍走一次
void Arm_StageTick(void)
{
    Arm_GroundStage();
    Arm_PutStage();
    Arm_StoreSkyStage();
    Arm_LeftStoreStage();
}

void Up_Init(void)
{
    arm_set_home = 0; // 1: 只标零，把臂摆到零位烧一次，标完改回 0 再烧

    left_arm.id = 0U;
    right_arm.id = 1U;

    // 关节1 大臂第一个电机
    MG5010E_Init(&left_arm.joint1, &hfdcan3, 1, 36.0f);
    MG5010E_Init(&right_arm.joint1, &hfdcan3, 2, 36.0f);

    // 上电先清一次电机故障标志
    MG5010E_ClearError(&left_arm.joint1);
    MG5010E_ClearError(&right_arm.joint1);

    // 关节2 大臂第二个电机，防止鬼蓄标零
    RS00_Init(&left_arm.joint2, 1, &hfdcan3, 23, mit);
    RS00_Init(&right_arm.joint2, 1, &hfdcan3, 40, mit);

    // 关节3 小臂 / 关节4 腕
    RS05_Init(&left_arm.joint3, arm_set_home, &hfdcan3, 14, mit);
    RS05_Init(&right_arm.joint3, arm_set_home, &hfdcan3, 22, mit);
    RS05_Init(&left_arm.joint4, arm_set_home, &hfdcan3, 25, mit);
    RS05_Init(&right_arm.joint4, arm_set_home, &hfdcan3, 24, mit);

    // 先固定等 500ms 再使能
    osDelay(500);

    // 0x9B 只清错误，不会把处于关闭状态的电机切回运行状态。
    // 上电阶段重复发送，避免单帧因总线竞争或电机尚未启动而丢失。
    for (uint8_t retry = 0U; retry < 3U; retry++)
    {
        MG5010E_ClearError(&left_arm.joint1);
        osDelay(5);
        MG5010E_ClearError(&right_arm.joint1);
        osDelay(5);
        MG5010E_Enable(&left_arm.joint1);
        osDelay(5);
        MG5010E_Enable(&right_arm.joint1);
        osDelay(5);
    }

    RS00_Enable(&left_arm.joint2);
    osDelay(1);
    RS00_Enable(&right_arm.joint2);
    osDelay(1);
    RS05_Enable(&left_arm.joint3);
    osDelay(1);
    RS05_Enable(&right_arm.joint3);
    osDelay(1);
    RS05_Enable(&left_arm.joint4);
    osDelay(1);
    RS05_Enable(&right_arm.joint4);
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

//放下之后 100ms 把气泵和真空阀全关，只关还停在放下的那一侧
void Arm_Suction_Off(void)
{
    if (suction_off_flag && (HAL_GetTick() - suction_off_tick > 100U)) // 放下 100ms 后全关
    {
        suction_off_flag = 0U;

        if (left_arm.suction == 2U) // 只关还停在放下的臂，别把吸着的关掉
            Arm_Suction(&left_arm, 0U);

        if (right_arm.suction == 2U)
            Arm_Suction(&right_arm, 0U);

        if (up_suction_flag == 2U) // 上吸盘同理
            Sky_Suction(0U);
    }
}

/* ====================================== 按键用到的动作入口 ====================================== */

//吸地面1层块：两臂一起
void Arm_SuckGround1(void)
{
    up_state = eUpSuckGround1;
    ground_home[0].stage = 0U;
    ground_home[1].stage = 0U;
    Arm_SuckBlock(&left_arm);
    Arm_SuckBlock(&right_arm);
}

//回零：动作做到一半时先等 5010、RS00 走稳再放末端
void Arm_HomeAll(void)
{
    if (up_state == eUpSuckGround1 || up_state == eUpStoreSky2)
    {
        for (uint8_t i = 0; i < 2U; i++)
        {
            float pos[4];

            ground_home[i].stage = 1U;
            ground_home[i].still = 0U;
            ground_home[i].last1 = arm_list[i]->joint1.pos;
            ground_home[i].last2 = arm_list[i]->joint2.feedback.pos_fed;
            ground_home[i].tick = HAL_GetTick();

            pos[0] = 0.0f;
            pos[1] = 0.0f;
            pos[2] = arm_list[i]->joint3.feedback.pos_fed;//末端先保持
            pos[3] = arm_list[i]->joint4.feedback.pos_fed;

            Arm_SetTarget(arm_list[i], pos);//回零直接到位，不插值
        }
    }
    else
    {
        Arm_Home(&left_arm);
        Arm_Home(&right_arm);
    }

    if (up_state == eUpStoreSky) // 存块状态回零：停掉分段、退状态，上吸盘保持不动
    {
        store_wait.stage = 0U;
        suction_off_flag = 0U; // 不许那个100毫秒全关再动气泵
        up_state = eUpIdle;
    }
}

//存天空块：进去，等右臂稳住再吸
void Arm_EnterStoreSky(void)
{
    up_state = eUpStoreSky;
    store_wait.stage = 1U;
    store_wait.still = 0U;
    store_wait.last1 = right_arm.joint1.pos;
    store_wait.last2 = right_arm.joint2.feedback.pos_fed;
    store_wait.tick = HAL_GetTick();

    Arm_MoveTo(&right_arm, store_sky_pos);
}

//存天空块：退出，回存块位吸住，稳了放开上吸盘，再 1.5 秒回零
void Arm_ExitStoreSky(void)
{
    store_wait.stage = 4U;
    store_wait.still = 0U;
    store_wait.last1 = right_arm.joint1.pos;
    store_wait.last2 = right_arm.joint2.feedback.pos_fed;
    store_wait.tick = HAL_GetTick();

    Arm_MoveTo(&right_arm, store_sky_pos);
    Arm_Suction(&right_arm, 1U);
}

//竖直排列预备：右臂关节2 和末端都按误差插值，到位后关节2 靠大 kp 保持
void Arm_EnterVertical(void)
{
    Arm_MoveTo(&left_arm, &vertical_pos[0]);
    Arm_InterpTo(&right_arm, &vertical_pos[4], 0x0EU);//右臂 2、3、4 关节一起按误差插值

    vert_hold[right_arm.id] = 1U;
}

//竖直放置：左臂 2、3、4 关节按误差插值
void Arm_PreparePutLeft(void)
{
    Arm_InterpTo(&left_arm, &vertical_put_pos[0], 0x0EU);
}

//竖直放置：右臂先只让 2 关节插值走，末端保持原目标，等 1、2 稳住了再放
void Arm_PreparePutRight(void)
{
    float pos[4];

    pos[0] = vertical_put_pos[4];
    pos[1] = vertical_put_pos[5];
    pos[2] = right_arm.joint3.feedback.pos_fed;//末端先保持
    pos[3] = right_arm.joint4.feedback.pos_fed;

    Arm_InterpTo(&right_arm, pos, 0x02U);//先只让关节2 插值
    vert_hold[right_arm.id] = 1U;

    put_wait.stage = 1U;
    put_wait.still = 0U;
    put_wait.last1 = right_arm.joint1.pos;
    put_wait.last2 = right_arm.joint2.feedback.pos_fed;
    put_wait.tick = HAL_GetTick();
}

//左右气路一起切：吸着就放下，否则吸
void Arm_Suction_Toggle(void)
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

    Arm_Suction(&right_arm, left_arm.suction); // 右臂跟着左臂切，存块状态也要能关
}

//左臂存块/取出：奇数次存、偶数次取出，只动左臂
void Arm_LeftStore_Toggle(void)
{
    left_store_takeout = !left_store_takeout;

    left_store.stage = 1U;
    left_store.still = 0U;
    left_store.last1 = left_arm.joint1.pos;
    left_store.last2 = left_arm.joint2.feedback.pos_fed;
    left_store.tick = HAL_GetTick();

    Arm_MoveTo(&left_arm, left_store_pos);
    HAL_GPIO_WritePin(up_jia_GPIO_Port, up_jia_Pin, GPIO_PIN_SET);//一按下就置高，打开夹爪
}

static void Arm_LeftStoreStage(void)
{
    if (left_store.stage == 0U)
    {
        return;
    }

    if (left_store.stage == 1U)//等左臂到存块位
    {
        if (Arm_WaitStill(&left_arm, &left_store, 2U, 0U) == 0U)
        {
            return;
        }

        if (left_store_takeout == 0U)//存：关吸盘开真空阀放块，再关夹爪
        {
            Arm_Suction(&left_arm, 2U);
            HAL_GPIO_WritePin(up_jia_GPIO_Port, up_jia_Pin, GPIO_PIN_RESET);//低电平关夹爪
            left_store.tick = HAL_GetTick();
            left_store.stage = 2U;
        }
        else//取：开吸盘吸住，再打开夹爪
        {
            Arm_Suction(&left_arm, 1U);
            HAL_GPIO_WritePin(up_jia_GPIO_Port, up_jia_Pin, GPIO_PIN_SET);//高电平打开夹爪
            left_store.tick = HAL_GetTick();
            left_store.stage = 3U;
        }
        return;
    }

    if (left_store.stage == 2U)//存：100ms 后回零，再重新吸住
    {
        if (HAL_GetTick() - left_store.tick < 100U)
        {
            return;
        }

        Arm_Home(&left_arm);
        Arm_Suction(&left_arm, 1U);//回零后重新打开吸盘
        left_store.stage = 0U;
        return;
    }

    if (left_store.stage == 3U)//取出：1 秒后回零
    {
        if (HAL_GetTick() - left_store.tick < 1000U)
        {
            return;
        }

        Arm_Home(&left_arm);
        left_store.tick = HAL_GetTick();
        left_store.stage = 4U;
        return;
    }

    if (left_store.stage == 4U)//取出：回零 1 秒后关 PD0
    {
        if (HAL_GetTick() - left_store.tick < 1000U)
        {
            return;
        }

        HAL_GPIO_WritePin(up_jia_GPIO_Port, up_jia_Pin, GPIO_PIN_RESET);//PD0 低电平关闭夹爪
        left_store.stage = 0U;
    }
}

#include "Up_control.h"
#include "Handle.h"

// 标零标志位(1时停止运控手动摆好)
uint8_t arm_set_home;
volatile uint8_t Motor_cnt;      //RS 电机上电发一帧，6 帧到齐说明 6 台都起来了
volatile uint8_t M5010_pos_flag; //200ms 读一次大臂位置
// 调试滑台位置用
float slide_pos;

Arm left_arm;
Arm right_arm;
Up_State up_state;

static uint8_t suction_off_flag; // 放下之后等一会全关
static uint32_t suction_off_tick;
static uint8_t sky_suction;                              // 上吸盘状态：0 关 1 吸 2 放
static uint8_t end_interp[2];                            // 末端插值开关：0 左臂 1 右臂，bit0 关节3 bit1 关节4
static uint16_t end_interp_ms[2];                        // 末端插值时间，0 左臂 1 右臂
static Arm_Stage ground_home[2];                         // 吸地面1层块回零分段：0 左臂 1 右臂
static Arm_Stage store_wait;                             // 存天空块分段：0 空闲 1 进入等稳 2 进入等 1 秒 3 进入等 0.5 秒 4 退出等稳 5 退出等 1 秒
static Arm_Stage put_wait;                               // PE0 右臂放置分段：0 空闲 1 等 5010/RS00 稳住
static Arm *const arm_list[2] = {&left_arm, &right_arm}; // 按 id 索引
// 吸块位置：前 4 个左臂、后 4 个右臂
// 吸取1层地块(从前面抓)
const float suck_block_pos[8] =
{
    -150.0f, -0.95f, -0.00f,  0.98f,
    138.0f,  1.15f, 0.0f, 1.4f,
};

// 吸球位置：只有左臂动
const float suck_ball_pos[4] =
{
    -66.0f, -0.73f, -0.123f, -1.05f,
};

// 取球完毕：只有左臂动 1
const float ball_done_pos[4] =
{
    -66.0f, -0.73f, -0.123f, 0.00f,
};

// 取两个天空块(横着吸) 1 左臂稍微往右一点吸取
const float sky_block_pos[8] =
{
    -163.0f, -1.0f, 0.05f, 1.18f,
    151.0f, 1.13f, -0.009f, 1.6f,
};

// 存天空块(右臂动)
const float store_sky_pos[4] =
{
    -4.0f, 1.15f, 0.0f, 2.07f,
};

// 竖直放置等待
const float vertical_pos[8] =
{
    -83.0f, -0.5f, -0.045f, 0.32f,
    67.0f, 0.68f, -1.4f, 2.00f,
};

// 竖直放置
const float vertical_put_pos[8] =
{
    -145.0f, -0.5f, -0.00f,  1.31f,
    105.0f, 1.8f, -1.6f, -1.34f,
};

// 吸取二层地块 1
const float layer2_pos[8] =
{
    -83.0f, -0.5f, -0.045f, 0.32f,
    70.0f, 0.65f, 0.0f, 0.75f,
};

// 左臂取二层 1
const float left_layer2_pos[4] =
{
    -81.8f, -0.56f, -0.045f, 0.17f,
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

// 末端插值：joint 传 3 或 4 只插那个关节，传 0 是 3、4 一起，走完自己关
static void Arm_Interp(Arm *arm, uint8_t joint, uint16_t ms)
{
    if (joint == 3U)
        end_interp[arm->id] = 0x01U;
    else if (joint == 4U)
        end_interp[arm->id] = 0x02U;
    else
        end_interp[arm->id] = 0x03U;

    end_interp_ms[arm->id] = ms;
}

// 一个臂 4 个关节串着发，5010 和 RS 中间不等待
void Arm_Send(Arm *arm)
{
    uint8_t mask = end_interp[arm->id];
    uint32_t t_ms = end_interp_ms[arm->id];

    MG5010E_SpeedPos(&arm->joint1, arm->target[0], 17.0f);
    RS00_Mit(&arm->joint2, 0, arm->target[1], 0, 27, 4, 0, 0);

    // 竖直排列：按开关决定末端 3、4 关节走不走插值，别的场合直接下发
    RS05_Mit(&arm->joint3, 0, arm->target[2], 0, 18, 2, (mask & 0x01U) ? 1U : 0U, t_ms);
    RS05_Mit(&arm->joint4, 0, arm->target[3], 0, 25, 2, (mask & 0x02U) ? 1U : 0U, t_ms);

    // 走完就把对应的位关掉
    if ((mask & 0x01U) != 0U && !arm->joint3.mit.position_interpolation.state)
        mask &= 0xFEU;

    if ((mask & 0x02U) != 0U && !arm->joint4.mit.position_interpolation.state)
        mask &= 0xFDU;

    end_interp[arm->id] = mask;
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

// 上吸盘：0 全关 1 吸取(开气泵关真空阀) 2 放下(关气泵开真空阀)
static void Sky_Suction(uint8_t cmd)
{
    sky_suction = cmd;

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
// 连续 20 拍没动算稳，超时 3 秒也算稳，免得卡死
static uint8_t Arm_WaitStill(Arm *arm, Arm_Stage *wait, uint8_t rs_joint, uint8_t check_end)
{
    float rs_pos = (rs_joint == 3U) ? arm->joint3.feedback.pos_fed : arm->joint2.feedback.pos_fed;

    // 5010 单位度、RS00/RS05 单位弧度，一拍变化小于这个只当没动
    if (fabsf(arm->joint1.pos - wait->last1) < 0.05f &&
        fabsf(rs_pos - wait->last2) < 0.002f &&
        (check_end == 0U || !arm->joint4.mit.position_interpolation.state))
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
            Arm_Interp(arm_list[i], 0U, 500U);

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
    right_arm.target[2] = vertical_put_pos[6];
    right_arm.target[3] = vertical_put_pos[7];
    Arm_Interp(&right_arm, 4U, 800U); // 只第 4 关节走 0.8 秒
}

//三个分段状态机每拍走一次
void Arm_StageTick(void)
{
    Arm_GroundStage();
    Arm_PutStage();
    Arm_StoreSkyStage();
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

    // 关节2 大臂第二个电机，左右交错着来
    RS00_Init(&left_arm.joint2, arm_set_home, &hfdcan3, 23, mit);
    RS00_Init(&right_arm.joint2, arm_set_home, &hfdcan3, 40, mit);

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

        if (sky_suction == 2U) // 上吸盘同理
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
            ground_home[i].stage = 1U;
            ground_home[i].still = 0U;
            ground_home[i].last1 = arm_list[i]->joint1.pos;
            ground_home[i].last2 = arm_list[i]->joint2.feedback.pos_fed;
            ground_home[i].tick = HAL_GetTick();

            arm_list[i]->target[0] = 0.0f;
            arm_list[i]->target[1] = 0.0f;
            arm_list[i]->target[2] = arm_list[i]->joint3.feedback.pos_fed; // 末端先保持
            arm_list[i]->target[3] = arm_list[i]->joint4.feedback.pos_fed;
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

    if (up_state == eUpVertical) // 竖直排列回零就退
    {
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
    Arm_Interp(&right_arm, 4U, 800U); // 第 4 关节走 0.8 秒
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
    Arm_Interp(&right_arm, 4U, 800U); // 第 4 关节走 0.8 秒
    Arm_Suction(&right_arm, 1U);
}

//竖直排列预备：两臂末端 3、4 关节走 1 秒插值
void Arm_EnterVertical(void)
{
    up_state = eUpVertical;
    Arm_Interp(&left_arm, 0U, 1000U);
    Arm_Interp(&right_arm, 0U, 1000U);

    Arm_MoveTo(&left_arm, &vertical_pos[0]);
    Arm_MoveTo(&right_arm, &vertical_pos[4]);
}

//竖直放置：左臂直接下发
void Arm_PreparePutLeft(void)
{
    end_interp[0] = 0U;
    Arm_MoveTo(&left_arm, &vertical_put_pos[0]);
}

//竖直放置：右臂先只让 1、2 关节走，末端保持原目标，等 1、2 稳住了再放
void Arm_PreparePutRight(void)
{
    end_interp[1] = 0U;
    put_wait.stage = 1U;
    put_wait.still = 0U;
    put_wait.last1 = right_arm.joint1.pos;
    put_wait.last2 = right_arm.joint2.feedback.pos_fed;
    put_wait.tick = HAL_GetTick();

    right_arm.target[0] = vertical_put_pos[4];
    right_arm.target[1] = vertical_put_pos[5];
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

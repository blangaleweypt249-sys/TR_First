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
static uint8_t sky_suction;                              // 上吸盘状态：0 关 1 吸 2 放
static uint8_t store_stage;                              // 存天空块分段：0 空闲 1 进入等稳 2 进入等 1 秒 3 进入等 0.5 秒 4 退出等稳 5 退出等 1 秒
static uint8_t store_still;                              // 连续稳定计数
static float store_last1;                                // 上一拍 5010 位置
static float store_last2;                                // 上一拍 RS00 位置
static uint32_t store_tick;                              // 分段计时
static uint8_t end_interp[2];                            // 末端插值开关：0 左臂 1 右臂，bit0 关节3 bit1 关节4
static uint16_t end_interp_ms[2];                        // 末端插值时间，0 左臂 1 右臂
static uint8_t put_stage;                                // PE0 右臂放置分段：0 空闲 1 等 5010/RS00 稳住
static uint8_t put_still;                                // 连续稳定计数
static float put_last1;                                  // 上一拍 5010 位置
static float put_last2;                                  // 上一拍 RS00 位置
static uint32_t put_tick;                                // 开始等稳定的时刻
static uint32_t reboot_tick;                             // 5010 重启时刻，0 表示没在重启
static osSemaphoreId_t arm_5010_sem;                     // 二值信号量：有令牌才给 5010 发指令
static Arm *const arm_list[2] = {&left_arm, &right_arm}; // 按 id 索引
static uint8_t Motor_ready;

// 吸块位置：前 4 个左臂、后 4 个右臂
// 吸取1层地块(从前面抓)
static const float suck_block_pos[8] =
{
    -159.0f, -0.98f, -0.00f,  1.05f,
    144.0f,  1.15f, 0.0f, 1.47f,
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
    151.0f, 1.1f, -0.009f, 1.66f,
};

// 存天空块(右臂动)
static const float store_sky_pos[4] =
{
    -6.0f, 1.15f, 0.0f, 2.1f,
};

// 竖直放置等待
static const float vertical_pos[8] =
{
    0.0f, -0.6f, 0.0f, -0.5f,
    69.0f, 1.3f, -1.6f, 0.5f,
};

// 竖直放置
static const float vertical_put_pos[8] =
{
    -144.0f, -0.37f, -0.00f, 1.48f,
    105.0f, 1.8f, -1.6f, -1.30f,
};

// 吸取二层地块 1
static const float layer2_pos[8] =
{
    -83.8f, -0.56f, -0.045f, 0.20f,
    67.0f, 0.68f, 0.0f, 0.65f,
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

    if (osSemaphoreAcquire(Reboot_motorHandle, 0U) == osOK) 
    {
        Motor_ready = 1;//这波其实有点多余了
    }

    if (arm_set_home == 0U && Motor_ready == 1) // 标零模式下不发运动指令，读位置照发
    {
        MG5010E_SpeedPos(&left_arm.joint1, left_arm.target[0], 20.0f);
        MG5010E_SpeedPos(&right_arm.joint1, right_arm.target[0], 20.0f);
    }

    if (++div >= 20U) // 200ms 读一次位置
    {
        div = 0U;
        MG5010E_ReadPosition(&left_arm.joint1);
        MG5010E_ReadPosition(&right_arm.joint1);
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

// 按目标下发末端 3 个关节，大臂 5010 由 TIM7 每 10ms 发
static void Arm_Send(Arm *arm)
{
    uint8_t mask = end_interp[arm->id];
    uint32_t t_ms = end_interp_ms[arm->id];

    RS00_Mit(&arm->joint2, 0, arm->target[1], 0, 27, 4, 0, 0);
    osDelay(1);

    // 竖直排列：按开关决定末端 3、4 关节走不走插值，别的场合直接下发
    RS05_Mit(&arm->joint3, 0, arm->target[2], 0, 18, 2, (mask & 0x01U) ? 1U : 0U, t_ms);
    osDelay(1);
    RS05_Mit(&arm->joint4, 0, arm->target[3], 0, 25, 2, (mask & 0x02U) ? 1U : 0U, t_ms);
    osDelay(1);

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

// 存天空块：右臂走到位稳住 → 上吸盘吸 → 1 秒 → 右臂吸盘关 → 0.5 秒 → 右臂回零 → 右臂再开成吸取
static void Arm_StoreSkyStage(void)
{
    if (store_stage == 0U)
        return;

    if (store_stage == 1U) // 等右臂 5010、RS00 稳住，末端插值也要走完
    {
        if (fabsf(right_arm.joint1.pos - store_last1) < 0.05f &&
            fabsf(right_arm.joint2.feedback.pos_fed - store_last2) < 0.002f &&
            !right_arm.joint4.mit.position_interpolation.state)
            store_still++;
        else
            store_still = 0U;

        store_last1 = right_arm.joint1.pos;
        store_last2 = right_arm.joint2.feedback.pos_fed;

        if (store_still < 20U && (HAL_GetTick() - store_tick) < 3000U)
            return;

        Sky_Suction(1U); // 上吸盘吸取
        store_tick = HAL_GetTick();
        store_stage = 2U;
        return;
    }

    if (store_stage == 2U) // 1 秒后放开右臂吸盘，再等 100ms 全关
    {
        if (HAL_GetTick() - store_tick < 1000U)
            return;

        Arm_Suction(&right_arm, 2U);
        suction_off_flag = 1U;
        suction_off_tick = HAL_GetTick();

        store_tick = HAL_GetTick();
        store_stage = 3U;
        return;
    }

    if (store_stage == 4U) // 退出：等右臂回到存块位稳住，末端插值也要走完
    {
        if (fabsf(right_arm.joint1.pos - store_last1) < 0.05f &&
            fabsf(right_arm.joint2.feedback.pos_fed - store_last2) < 0.002f &&
            !right_arm.joint4.mit.position_interpolation.state)
            store_still++;
        else
            store_still = 0U;

        store_last1 = right_arm.joint1.pos;
        store_last2 = right_arm.joint2.feedback.pos_fed;

        if (store_still < 20U && (HAL_GetTick() - store_tick) < 3000U)
            return;

        Sky_Suction(2U); // 位置稳了先放开上吸盘，等 100ms 再全关
        suction_off_flag = 1U;
        suction_off_tick = HAL_GetTick();

        store_tick = HAL_GetTick();
        store_stage = 5U;
        return;
    }

    if (store_stage == 5U) // 退出：1.5 秒后回零
    {
        if (HAL_GetTick() - store_tick < 1500U)
            return;

        Arm_Home(&right_arm);
        up_state = eUpIdle;
        store_stage = 0U;
        return;
    }

    if (HAL_GetTick() - store_tick < 1500U) // 1.5 秒后回零位，再开成吸取状态
        return;

    Arm_Home(&right_arm);
    Arm_Suction(&right_arm, 1U);
    store_stage = 0U;
}

// 吸地面1层块状态下回零：等每臂的 5010、RS00 走稳了再放末端；两臂 5010 回零后退出
static void Arm_GroundStage(void)
{
    if (up_state != eUpSuckGround1)
        return;

    for (uint8_t i = 0; i < 2U; i++)
    {
        if (ground_home[i].stage != 1U)
            continue;

        // 5010 单位度、RS00 单位弧度，一拍变化小于这个只当没动
        if (fabsf(arm_list[i]->joint1.pos - ground_home[i].last1) < 0.05f &&
            fabsf(arm_list[i]->joint2.feedback.pos_fed - ground_home[i].last2) < 0.002f)
            ground_home[i].still++;
        else
            ground_home[i].still = 0U;

        ground_home[i].last1 = arm_list[i]->joint1.pos;
        ground_home[i].last2 = arm_list[i]->joint2.feedback.pos_fed;

        // 连续稳 20 拍（约 0.4 秒）或者超时 3 秒就放末端
        if (ground_home[i].still < 20U && (HAL_GetTick() - ground_home[i].tick) < 3000U)
            continue;

        arm_list[i]->target[2] = 0.0f;
        arm_list[i]->target[3] = 0.0f;
        Arm_Interp(arm_list[i], 0U, 500U);
        ground_home[i].stage = 2U;
    }

    if (ground_home[0].stage == 2U && ground_home[1].stage == 2U &&
        fabsf(left_arm.joint1.pos) < 5.0f && fabsf(right_arm.joint1.pos) < 5.0f)
    {
        ground_home[0].stage = 0U;
        ground_home[1].stage = 0U;
        up_state = eUpIdle;
    }
}

// PE0 右臂竖直放置：先让 5010、RS00 走到稳，末端再插值 0.5 秒过去
static void Arm_PutStage(void)
{
    if (put_stage == 0U)
        return;

    // 5010 单位度、RS00 单位弧度，一拍变化小于这个只当没动
    if (fabsf(right_arm.joint1.pos - put_last1) < 0.05f &&
        fabsf(right_arm.joint2.feedback.pos_fed - put_last2) < 0.002f)
        put_still++;
    else
        put_still = 0U;

    put_last1 = right_arm.joint1.pos;
    put_last2 = right_arm.joint2.feedback.pos_fed;

    // 连续稳 20 拍（约 0.4 秒）或者超时 3 秒就放末端
    if (put_still < 20U && (HAL_GetTick() - put_tick) < 3000U)
        return;

    put_stage = 0U;
    right_arm.target[2] = vertical_put_pos[6];
    right_arm.target[3] = vertical_put_pos[7];
    Arm_Interp(&right_arm, 4U, 800U); // 只第 4 关节走 0.8 秒
}

// 遥控：左侧1 吸块、左侧2 回零、顶部3 吸盘切换
static void Arm_Key(void)
{
    Arm_GroundStage();
    Arm_PutStage();
    Arm_StoreSkyStage();

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
        if (up_state == eUpSuckGround1) // 末端等 5010 和 RS00 走稳再动
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
            store_stage = 0U;
            suction_off_flag = 0U; // 不许那个100毫秒全关再动气泵
            up_state = eUpIdle;
        }

        if (up_state == eUpVertical) // 竖直排列回零就退
        {
            up_state = eUpIdle;
        }
    }

    if (Handle_KeyRelease(eKeyLeft3)) // PD12 吸球，只动左臂
    {
        Arm_MoveTo(&left_arm, suck_ball_pos);
    }

    if (Handle_KeyRelease(eKeyLeft4)) // PD13 取两个天空块，先不让左臂动了改策略
    {
        Arm_MoveTo(&left_arm, &sky_block_pos[0]);
        Arm_MoveTo(&right_arm, &sky_block_pos[4]);
    }

    if (Handle_KeyRelease(eKeyLeft5)) // PD14 存天空块，第一次进、第二次退
    {
        if (up_state == eUpStoreSky) // 退出：回存块位吸住，稳了放开上吸盘，再 1.5 秒回零
        {
            store_stage = 4U;
            store_still = 0U;
            store_last1 = right_arm.joint1.pos;
            store_last2 = right_arm.joint2.feedback.pos_fed;
            store_tick = HAL_GetTick();

            Arm_MoveTo(&right_arm, store_sky_pos);
            Arm_Interp(&right_arm, 4U, 800U); // 第 4 关节走 0.8 秒
            Arm_Suction(&right_arm, 1U);
        }
        else // 进入
        {
            up_state = eUpStoreSky;
            store_stage = 1U;
            store_still = 0U;
            store_last1 = right_arm.joint1.pos;
            store_last2 = right_arm.joint2.feedback.pos_fed;
            store_tick = HAL_GetTick();

            Arm_MoveTo(&right_arm, store_sky_pos);
            Arm_Interp(&right_arm, 4U, 800U); // 第 4 关节走 0.8 秒
        }
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

    // PA5 进入竖直排列预备态：两个臂一起动，末端 3、4 关节走 1 秒插值
    if (Handle_KeyChange(eKeyUp2))
    {
        up_state = eUpVertical;
        Arm_Interp(&left_arm, 0U, 1000U);
        Arm_Interp(&right_arm, 0U, 1000U);

        Arm_MoveTo(&left_arm, &vertical_pos[0]);
        Arm_MoveTo(&right_arm, &vertical_pos[4]);
    }

    // 竖直排列状态下：PE1 左臂竖直放置，PE0 右臂竖直放置
    if (up_state == eUpVertical)
    {
        if (Handle_KeyRelease(eKeyDown1)) // PE1
        {
            end_interp[0] = 0U; // 左臂直接下发
            Arm_MoveTo(&left_arm, &vertical_put_pos[0]);
        }

        if (Handle_KeyRelease(eKeyDown2)) // PE0
        {
            // 先只让 1、2 关节走，末端保持原目标，等 1、2 稳住了再放
            end_interp[1] = 0U;
            put_stage = 1U;
            put_still = 0U;
            put_last1 = right_arm.joint1.pos;
            put_last2 = right_arm.joint2.feedback.pos_fed;
            put_tick = HAL_GetTick();

            right_arm.target[0] = vertical_put_pos[4];
            right_arm.target[1] = vertical_put_pos[5];
        }
    }

    // PA12 滑台两档：355 / 0
    if (Handle_KeyRelease(eKeyLeft9))
    {
        if (slide_pos > 200.0f)
            slide_pos = 0.0f;
        else
            slide_pos = 355.0f;
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

        Arm_Suction(&right_arm, left_arm.suction); // 右臂跟着左臂切，存块状态也要能关
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

    // 上电先重启一次，清掉电机可能残留的驱动故障
    MG5010E_Reboot(&left_arm.joint1);
    MG5010E_Reboot(&right_arm.joint1);

    osSemaphoreRelease(Reboot_motorHandle);

    reboot_tick = HAL_GetTick(); // 重启满 1 秒再由 Arm_InitCheck 放开，之前一帧都不发

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

    arm_5010_sem = osSemaphoreNew(1U, 0U, NULL); // 二值信号量，初值不给：开机先不发，等重启完再放

    MG5010E_Init(&left_arm.joint1, &hfdcan3, 1, 36.0f);
    MG5010E_Init(&right_arm.joint1, &hfdcan3, 2, 36.0f);

    HAL_TIM_Base_Start_IT(&htim7); //5010用

    osDelay(3000); // 等电机上电完成再初始化，冷启动时电机比主控晚醒

    Up_Init(); // 机构初始化，目标都置 0，上电就回零点

    for (;;)
    {
        Arm_Key();       // 遥控

        if (arm_set_home == 0)
        {
            Arm_Send(&left_arm);
            Arm_Send(&right_arm);
            DJI_Position(&right_arm.slide, slide_pos);
        }

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

        osDelay(10);
    }
}

#include  "Up_control.h"
#include  "Handle.h"
#include  "tim.h"

//标零标志位(1时停止运控手动摆好)
uint8_t arm_set_home;
volatile uint8_t arm_boot_cnt;
//调试滑台位置用
float slide_pos;

Arm left_arm;
Arm right_arm;

static uint8_t suction_off_flag;   //放下之后等一会全关
static uint32_t suction_off_tick;
static uint8_t vert_arm_flag;      //竖排：0 下一个按左臂，1 下一个按右臂
static uint8_t vert_stage;         //右臂分段：1 等关节1/2走到一半
static float vert_j1_start;        //右臂关节1起步位置
static float vert_j2_start;        //右臂关节2起步位置

//吸块位置：前 4 个左臂、后 4 个右臂
//这个是横着吸低的块
static const float suck_block_pos[8] =
{
    -111.6f, -0.74f, -0.05f,  -0.95f,
    102.5f,  1.06f, 0.04f, -0.7f,
};

//吸球位置：只有左臂动
static const float suck_ball_pos[4] =
{
    -68.0f, -0.73f, -0.123f, -1.08f,
};

//取球完毕：只有左臂动
static const float ball_done_pos[4] =
{
    -68.0f, -0.73f, -0.123f, 0.00f,
};

//取两个天空块
static const float sky_block_pos[8] =
{
    -140.5f, -0.88f, 0.03f, -1.6f,
    126.6f, 1.1f, -0.006f, -0.38f,
};

//竖直排列
static const float vertical_pos[8] =
{
    -146.7f, -0.51f, -0.08f, 1.07f,
    125.8f, 1.78f, -1.58f, -1.35f,
};

//吸取二层
static const float layer2_pos[8] =
{
    -83.8f, -0.56f, -0.045f, 0.20f,
    69.0f, 0.73f, 0.0f, 0.78f,
};

//MG5010E 不会主动上报位置，TIM7 每 200ms 调一次，发读位置命令
void Arm_PollTimer(void)
{
    MG5010E_ReadPosition(&left_arm.joint1);
    MG5010E_ReadPosition(&right_arm.joint1);
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

//吸取：气泵开、真空阀关
void Arm_Suck(Arm *arm)
{
    HAL_GPIO_WritePin(arm->pump_port, arm->pump_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(arm->vac_port, arm->vac_pin, GPIO_PIN_RESET);
}

//放下：气泵关、真空阀开
void Arm_Release(Arm *arm)
{
    HAL_GPIO_WritePin(arm->pump_port, arm->pump_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(arm->vac_port, arm->vac_pin, GPIO_PIN_SET);
}

//全关
void Arm_Off(Arm *arm)
{
    HAL_GPIO_WritePin(arm->pump_port, arm->pump_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(arm->vac_port, arm->vac_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(arm->grip_port, arm->grip_pin, GPIO_PIN_RESET);
}

void Arm_Grip(Arm *arm, uint8_t on)
{
    HAL_GPIO_WritePin(arm->grip_port, arm->grip_pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//0 全关 1 吸取 2 放下
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

//按目标下发 4 个关节，MG5010 限速 20rpm
static void Arm_Send(Arm *arm)
{
    MG5010E_SpeedPos(&arm->joint1, arm->target[0], 20.0f);
    osDelay(2);
    RS00_Mit(&arm->joint2, 0, arm->target[1], 0, 16, 4, 0, 0);
    osDelay(1);
    RS05_Mit(&arm->joint3, 0, arm->target[2], 0, 8, 2, 0, 0);
    osDelay(1);
    RS05_Mit(&arm->joint4, 0, arm->target[3], 0, 15, 2, 0, 0);
    osDelay(1);
}

//竖排第一次按：右臂先动关节1、2，走到目标一半再放末端两个
static void Arm_VertStage(void)
{
    if (vert_stage == 0U)
        return;

    if (fabsf(right_arm.joint1.pos - vert_j1_start) >= 0.3f * fabsf(vertical_pos[4] - vert_j1_start) &&
        fabsf(right_arm.joint2.feedback.pos_fed - vert_j2_start) >= 0.3f * fabsf(vertical_pos[5] - vert_j2_start))
    {
        right_arm.target[2] = vertical_pos[6];
        right_arm.target[3] = vertical_pos[7];
        vert_stage = 0U;
    }
}

//遥控：左侧1 吸块、左侧2 回零、顶部3 吸盘切换
static void Arm_Key(void)
{
    Arm_VertStage();

    if (Handle_KeyRelease(eKeyLeft1))
    {
        Arm_SuckBlock(&left_arm);
        Arm_SuckBlock(&right_arm);
    }

    if (Handle_KeyRelease(eKeyLeft2))//PD11 回零，同时把竖排的计数清掉
    {
        Arm_Home(&left_arm);
        Arm_Home(&right_arm);
        vert_arm_flag = 0U;   //下次竖排还是从左臂开始
        vert_stage = 0U;
    }

    if (Handle_KeyRelease(eKeyLeft3))//PD12 吸球，只动左臂
    {
        Arm_MoveTo(&left_arm, suck_ball_pos);
    }

    if (Handle_KeyRelease(eKeyLeft4))//PD13 取两个天空块，两个臂都动
    {
        Arm_MoveTo(&left_arm, &sky_block_pos[0]);
        Arm_MoveTo(&right_arm, &sky_block_pos[4]);
    }

    if (Handle_KeyRelease(eKeyLeft6))//PD15 取球完毕，只动左臂
    {
        Arm_MoveTo(&left_arm, ball_done_pos);
    }

    //竖直排列：PA11 第一次左臂（整条一起动）、第二次右臂（分段）；PA12 滑台两档
    if (Handle_KeyRelease(eKeyLeft8))
    {
        if (vert_arm_flag == 0U)
        {
            Arm_MoveTo(&left_arm, &vertical_pos[0]);
            vert_arm_flag = 1U;
        }
        else
        {
            vert_j1_start = right_arm.joint1.pos;
            vert_j2_start = right_arm.joint2.feedback.pos_fed;
            right_arm.target[0] = vertical_pos[4];   //关节1、2 先走
            right_arm.target[1] = vertical_pos[5];
            right_arm.target[2] = right_arm.joint3.feedback.pos_fed;  //末端先保持
            right_arm.target[3] = right_arm.joint4.feedback.pos_fed;
            vert_stage = 1U;
            vert_arm_flag = 0U;
        }
    }

    if (Handle_KeyRelease(eKeyLeft7))//PA8 吸取二层，两个臂都动
    {
        Arm_MoveTo(&left_arm, &layer2_pos[0]);
        Arm_MoveTo(&right_arm, &layer2_pos[4]);
    }

    if (Handle_KeyChange(eKeyUp3))
    {
        if (left_arm.suction == 1U)//吸着 -> 放下
        {
            Arm_Suction(&left_arm, 2U);
            Arm_Suction(&right_arm, 2U);
            suction_off_flag = 1U;
            suction_off_tick = HAL_GetTick();
        }
        else//开着或已放下 -> 吸
        {
            Arm_Suction(&left_arm, 1U);
            Arm_Suction(&right_arm, 1U);
            suction_off_flag = 0U;
        }
    }
}

//启动电机
static void Arm_InitRetry(void)
{
    static uint16_t retry_cont;
    static uint8_t retry_round;
    static uint8_t retry_step = 3U;

    //电机启动比 MCU 慢，每 500ms 补一轮模式+使能，补到 8 秒；循环 10ms 一轮
    if (++retry_cont >= 50U)
    {
        retry_cont = 0;
        if (retry_round < 16U)
        {
            retry_round++;
            retry_step = 0;
        }
    }

    if (retry_step < 3U)
    {
        if (retry_step == 0)
        {
            RS00_SetActiveReporting(&left_arm.joint2);  RS00_SetMode(&left_arm.joint2, mit);  RS00_Enable(&left_arm.joint2);
            RS00_SetActiveReporting(&right_arm.joint2); RS00_SetMode(&right_arm.joint2, mit); RS00_Enable(&right_arm.joint2);
        }
        else if (retry_step == 1)
        {
            RS05_SetActiveReporting(&left_arm.joint3);  RS05_SetMode(&left_arm.joint3, mit);  RS05_Enable(&left_arm.joint3);
            RS05_SetActiveReporting(&left_arm.joint4);  RS05_SetMode(&left_arm.joint4, mit);  RS05_Enable(&left_arm.joint4);
        }
        else
        {
            RS05_SetActiveReporting(&right_arm.joint3); RS05_SetMode(&right_arm.joint3, mit); RS05_Enable(&right_arm.joint3);
            RS05_SetActiveReporting(&right_arm.joint4); RS05_SetMode(&right_arm.joint4, mit); RS05_Enable(&right_arm.joint4);
        }
        retry_step++;
    }
}

void Up_Init(void)
{
    arm_set_home = 0;//1: 只标零，把臂摆到零位烧一次，标完改回 0 再烧

    left_arm.id = 0U;
    right_arm.id = 1U;

    //关节1 大臂第一个电机
    MG5010E_Init(&left_arm.joint1, &hfdcan3, 1, 36.0f);
    MG5010E_Init(&right_arm.joint1, &hfdcan3, 2, 36.0f);

    //关节2 大臂第二个电机
    RS00_Init(&left_arm.joint2, arm_set_home, &hfdcan3, 23, mit);
    RS00_Init(&right_arm.joint2, arm_set_home, &hfdcan3, 40, mit);

    //关节3 小臂 / 关节4 腕
    RS05_Init(&left_arm.joint3, arm_set_home, &hfdcan3, 14, mit);
    RS05_Init(&left_arm.joint4, arm_set_home, &hfdcan3, 25, mit);
    RS05_Init(&right_arm.joint3, arm_set_home, &hfdcan3, 22, mit);
    RS05_Init(&right_arm.joint4, arm_set_home, &hfdcan3, 24, mit);

    //右臂底座 2006 滑台，和腿共用 board 6
    DJI_Init(&right_arm.slide, &hfdcan2, DJI_M2006, 6, 3);

    DJI_SignZero(&right_arm.slide);
    right_arm.has_slide = 1U;

    if (arm_set_home == 1)//MG5010 把当前位置设成原点，断电重上电才生效
    {
        MG5010E_SetOrigin(&left_arm.joint1);
        osDelay(50);
        MG5010E_SetOrigin(&right_arm.joint1);
        osDelay(50);
    }

    //末端气路
    left_arm.pump_port = GPIOC;     left_arm.pump_pin = GPIO_PIN_9;     //左气泵
    left_arm.vac_port = GPIOC;      left_arm.vac_pin = GPIO_PIN_11;     //左真空阀
    left_arm.grip_port = GPIOA;     left_arm.grip_pin = GPIO_PIN_9;     //左夹爪

    right_arm.pump_port = GPIOA;    right_arm.pump_pin = GPIO_PIN_8;    //右气泵
    right_arm.vac_port = GPIOC;     right_arm.vac_pin = GPIO_PIN_10;    //右真空阀
    right_arm.grip_port = GPIOB;    right_arm.grip_pin = GPIO_PIN_6;    //右夹爪

    Arm_Off(&left_arm);
    Arm_Off(&right_arm);

    //上电即回零
    Arm_Home(&left_arm);
    Arm_Home(&right_arm);

    HAL_TIM_Base_Start_IT(&htim7);//200ms 读一次大臂电机位置
}

void PickTask(void *argument)
{
  (void)argument;
  uint16_t wait_cont = 0;

  while (arm_boot_cnt < 6U && wait_cont < 300U)//等 6 台 RS 电机上电，最多 1.5s
  {
    wait_cont++;
    osDelay(5);
  }

  Up_Init();//机构初始化，目标都置 0，上电就回零点

  for(;;)
  {
    Arm_InitRetry();//补模式+使能
    Arm_Key();      //遥控

    if (arm_set_home == 0)
    {
      Arm_Send(&left_arm);
      Arm_Send(&right_arm);
      DJI_Position(&right_arm.slide, slide_pos);
    }

    if (suction_off_flag && (HAL_GetTick() - suction_off_tick > 10000U))//放下 10 秒后全关
    {
      suction_off_flag = 0U;
      Arm_Suction(&left_arm, 0U);
      Arm_Suction(&right_arm, 0U);
    }

    osDelay(10); 
  }

}

#include  "Up_control.h"
#include  "tim.h"

uint8_t act_flag=0;
uint8_t arm_set_home;//1: 只标零不发运控
volatile uint8_t arm_boot_cnt;

//左机械臂
Arm left_arm;
//右机械臂
Arm right_arm;

//MG5010E 不会主动上报位置，TIM7 每 200ms 调一次，发读位置命令
void Arm_PollTimer(void)
{
    MG5010E_ReadPosition(&left_arm.joint1);
    MG5010E_ReadPosition(&right_arm.joint1);
}

void Up_Init(void)
{
    arm_set_home = 0;//1: 只标零，把臂摆到零位烧一次，标完改回 0 再烧

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

    HAL_TIM_Base_Start_IT(&htim7);//200ms 读一次大臂电机位置
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

//完成：全关
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

void arm_attraction(void)
{
    switch (act_flag)
    {
        case 1:
            Arm_Suck(&left_arm);
            Arm_Suck(&right_arm);
            break;

        case 2:
            Arm_Release(&left_arm);
            Arm_Release(&right_arm);
            break;

        case 3:
            Arm_Off(&left_arm);
            Arm_Off(&right_arm);
            break;

        default:
            break;
    }
}

void PickTask(void *argument)
{
  (void)argument;
  uint16_t wait_cont = 0;
  uint16_t retry_cont = 0;
  uint8_t retry_round = 0;
  uint8_t retry_step = 3U;

  while (arm_boot_cnt < 6U && wait_cont < 300U)//等 6 台 RS 电机上电，最多 1.5s
  {
    wait_cont++;
    osDelay(5);
  }

  Up_Init();//机构初始化

  for(;;)
  {
    if (++retry_cont >= 100U)
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

    arm_attraction();

    if (arm_set_home == 0)//标零那一次不发运控，臂留软的好摆
    {
    //   MG5010E_SpeedPos(&left_arm.joint1, 0.0f, 5.0f);
    //   MG5010E_SpeedPos(&right_arm.joint1, 0.0f, 5.0f);

      RS00_Mit(&left_arm.joint2, 0, 0, 0, 0, 0, 0, 0);
      RS00_Mit(&right_arm.joint2, 0, 0, 0, 0, 0, 0, 0);
      RS05_Mit(&left_arm.joint3, 0, 0, 0, 0, 0, 0, 0);
      RS05_Mit(&left_arm.joint4, 0, 0, 0, 0, 0, 0, 0);
      RS05_Mit(&right_arm.joint3, 0, 0, 0, 0, 0, 0, 0);
      RS05_Mit(&right_arm.joint4, 0, 0, 0, 0, 0, 0, 0);

    //   DJI_Position(&right_arm.slide, 0.0f);
    }

    osDelay(5);//5ms 一轮，RS 200Hz
  }

}

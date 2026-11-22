#include "Leg.h"
#include "Handle.h"
#include "mymain.h"

#define ms_to_rpm  477.46         //小轮直径40mm
#define dis_to_pos 625.0f / 42.5f

uint8_t front_flag=0;

// 后腿电机
DJI_Motor leg_Left;
DJI_Motor leg_Right;
// 后腿电机2006
DJI_Motor leg_Left_2006;
DJI_Motor leg_Right_2006;

void Leg_Init(void)
{
    // 后腿电机初始化
    DJI_Init(&leg_Left, &hfdcan2, DJI_M3508, 5, 1);
    DJI_Init(&leg_Right, &hfdcan2, DJI_M3508, 5, 2);
    DJI_Init(&leg_Left_2006, &hfdcan2, DJI_M2006, 6, 1);
    DJI_Init(&leg_Right_2006, &hfdcan2, DJI_M2006, 6, 2);
}

void Leg_Clear(void)
{
    DJI_SignZero(&leg_Left);
    DJI_SignZero(&leg_Right);
    DJI_SignZero(&leg_Left_2006);
    DJI_SignZero(&leg_Right_2006);
}

//前腿气缸：1 伸出(高电平)，0 收回
void Leg_Front(uint8_t out)
{
    HAL_GPIO_WritePin(front_leg_GPIO_Port, front_leg_Pin, out ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

//中腿气缸：1 伸出(高电平)，0 收回
void Leg_Mid(uint8_t out)
{
    HAL_GPIO_WritePin(med_leg_GPIO_Port, med_leg_Pin, out ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Leg_Advance(float speed)
{
    DJI_Speed(&leg_Left, speed * ms_to_rpm);
    DJI_Speed(&leg_Right, -speed * ms_to_rpm);//镜像导致
}

void Climb_Leg(float distance)
{
    //腿最长距离和最短距离限制
    if(distance < 0.0f)
    {
        distance = 0.0f;
    }
    else if(distance > 42.5f)
    {
        distance = 42.5f;
    }
    //统一传入正值即可
    DJI_Position(&leg_Left, distance * dis_to_pos);
    DJI_Position(&leg_Right, -distance * dis_to_pos);//镜像导致
}

//小轮点动：PE1 按住前进、PE0 按住后退，1m/s
void ClimbTask(void *argument)
{
  static uint8_t front_out;   //前腿 1 伸出
  static uint8_t mid_out;     //中腿 1 伸出
  static float speed_cmd;     //规划后的当前速度
  float speed;                //按键给的目标速度

  (void)argument;
  for(;;)
  {
    //PA1 前腿、PA0 中腿：拨一次翻一次
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
    //PC3 后腿只在 40 和 35 之间互切
    if (Handle_KeyChange(eKeyUp6))
    {
      if (up_distance > 37.0f)
        up_distance = 35.0f;
      else if (up_distance > 32.0f)
        up_distance = 40.0f;
    }

    //PD4 五位开关中间：后腿 18 和 0 互切
    if (Handle_KeyRelease(eKeyOk))
    {
      if (up_distance > 9.0f)
        up_distance = 0.0f;
      else
        up_distance = 14.0f;
    }

    //PE6 前腿伸 + 后腿20
    if (Handle_KeyRelease(eKeyRight1))
    {
      front_out = 1U;
      Leg_Front(1U);
      osDelay(170);
      up_distance = 22.0f;
    }
    //PE3 前腿伸 + 后腿40
    if (Handle_KeyRelease(eKeyRight2))
    {
      front_out = 1U;
      Leg_Front(1U);
      osDelay(170);
      up_distance = 40.5f;
    }
    //PE4 前腿收 + 后腿20
    if (Handle_KeyRelease(eKeyRight3))
    {
      front_out = 0U;
      Leg_Front(0U);
      up_distance = 22.0f;
    }
    //PE5 前腿收 + 后腿0
    if (Handle_KeyRelease(eKeyRight4))
    {
      front_out = 0U;
      Leg_Front(0U);
      up_distance = 0.0f;
    }

    if (handle_key[eKeyDown1].key)
        speed = 0.5f;
    else if (handle_key[eKeyDown2].key)
        speed = -0.5f;
    else
        speed = 0.0f;

    //斜率规划：1kHz 下一拍走 0.001 = 1m/s² 加速，0.004 = 4m/s² 减速
    speed_cmd = speed_planning(speed, &speed_cmd, 0.001f, 0.004f);

    DJI_Speed(&leg_Left_2006, -speed_cmd * ms_to_rpm);
    DJI_Speed(&leg_Right_2006, speed_cmd * ms_to_rpm);//镜像导致

    osDelay(5);
  }
}
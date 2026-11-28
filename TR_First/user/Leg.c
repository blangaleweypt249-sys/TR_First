#include "Leg.h"
#include "Handle.h"
#include "mymain.h"

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


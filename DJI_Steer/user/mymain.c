#include "mymain.h"
#include "can.h"
#include "tim.h"
#include "bsp_can.h"

DJI_Motor dji_motor[DJI_MOTOR_NUM];

static uint16_t feedback_cont;

void My_Init(void)
{
    for (uint8_t i = 0; i < DJI_MOTOR_NUM; i++)
    {
        DJI_Init(&dji_motor[i], &hcan2, BOARD_ID, i + 1U);
        dji_motor[i].last_command_tick = HAL_GetTick();
    }

    CAN1_Filter_Init();
    CAN2_Filter_Init();
    ALL_CAN_Start();

    // TIM1：1kHz
    HAL_TIM_Base_Start_IT(&htim1);

    // 多块板同时上电，反馈时刻错开
    HAL_Delay(BOARD_ID * 3U);
}

// 1kHz
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM1)
    {
        return;
    }

    CAN_Bus_Off_Recover(&hcan1);
    CAN_Bus_Off_Recover(&hcan2);

    for (uint8_t i = 0; i < DJI_MOTOR_NUM; i++)
    {
        DJI_Control(&dji_motor[i]);
    }
    DJI_Transmit(&hcan2);

    feedback_cont++;
    if (feedback_cont >= 10U) //10ms反馈给主控一次
    {
        feedback_cont = 0;
        for (uint8_t i = 0; i < DJI_MOTOR_NUM; i++)
        {
            DJI_Back_To_Master(&dji_motor[i], &hcan1);
        }
    }
}

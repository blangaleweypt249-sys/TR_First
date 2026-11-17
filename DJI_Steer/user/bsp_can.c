#include "bsp_can.h"
#include "DJI.h"
#include "mymain.h"
#include "can.h"
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

void CAN1_Filter_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 1;
    sFilterConfig.FilterIdHigh = 0;
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void CAN2_Filter_Init(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 14;
    sFilterConfig.FilterIdHigh = 0;
    sFilterConfig.FilterIdLow = 0;
    sFilterConfig.FilterMaskIdHigh = 0;
    sFilterConfig.FilterMaskIdLow = 0;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void ALL_CAN_Start(void)
{
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

// 总线关闭后整个重建一遍，别等它自己恢复
void CAN_Bus_Off_Recover(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL || (hcan->Instance->ESR & CAN_ESR_BOFF) == 0U)
    {
        return;
    }

    if (hcan->Instance == CAN1)
    {
        HAL_CAN_Stop(hcan);
        HAL_CAN_DeInit(hcan);
        MX_CAN1_Init();
        CAN1_Filter_Init();
    }
    else if (hcan->Instance == CAN2)
    {
        HAL_CAN_Stop(hcan);
        HAL_CAN_DeInit(hcan);
        MX_CAN2_Init();
        CAN2_Filter_Init();
    }
}

// Header = 0 :标准帧  1 :扩展帧
HAL_StatusTypeDef CAN_Transmit(CAN_HandleTypeDef *hcan, uint8_t type, uint32_t id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_header;
    HAL_StatusTypeDef status;
    uint32_t tx_mailbox;
    uint8_t send_data[8];

    if ((hcan == NULL) || (data == NULL) || (len > 8))
    {
        return HAL_ERROR;
    }
    switch (type)
    {
    case 0:
        tx_header.StdId = id;
        tx_header.ExtId = 0;
        tx_header.IDE = CAN_ID_STD;
        break;

    case 1:
        tx_header.StdId = 0;
        tx_header.ExtId = id;
        tx_header.IDE = CAN_ID_EXT;
        break;

    default:
        return HAL_ERROR;
    }
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = len;
    tx_header.TransmitGlobalTime = DISABLE;
    for (uint8_t i = 0; i < len; i++)
    {
        send_data[i] = data[i];
    }
    status = HAL_CAN_AddTxMessage(hcan, &tx_header, send_data, &tx_mailbox);
    return status;
}

// 主控下来的命令帧
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint8_t mode;
    uint8_t motor_id;
    float target;

    if (hcan != &hcan1)
    {
        return;
    }
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }
    if (rx_header.IDE != CAN_ID_STD || rx_header.DLC != 8U)
    {
        return;
    }
    // 基址 + 本板号
    if ((rx_header.StdId & 0xF0FU) != (DJI_COMMAND_BASE | BOARD_ID))
    {
        return;
    }

    mode = (uint8_t)((rx_header.StdId >> 4) & 0x0FU);
    motor_id = rx_data[0];
    if (mode > DJI_CURRENT || motor_id < 1U || motor_id > DJI_MOTOR_NUM)
    {
        return;
    }

    memcpy(&target, &rx_data[1], sizeof(target));
    DJI_SetCommand(&dji_motor[motor_id - 1U], (DJI_Mode)mode, target);
}

// 电机反馈帧
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (hcan != &hcan2)
    {
        return;
    }
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data) != HAL_OK)
    {
        return;
    }

    for (uint8_t i = 0; i < DJI_MOTOR_NUM; i++)
    {
        DJI_Receive(&dji_motor[i], &rx_header, rx_data);
    }
}

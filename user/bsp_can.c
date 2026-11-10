#include "bsp_can.h"
#include "cmsis_os2.h"

void FDCAN1_Init(void)
{
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
}

void FDCAN2_Init(void)
{
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan2);
}

void FDCAN3_Init(void)
{
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan3);
}

#ifdef STM32H7xx_HAL_H
HAL_StatusTypeDef CAN_Transmit(FDCAN_HandleTypeDef *hfdcan, uint8_t type, uint32_t id, uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx_header;
    HAL_StatusTypeDef status;
    uint8_t send_data[8]; 
    if ((hfdcan == NULL) || (data == NULL) || (len > 8))
    {
        return HAL_ERROR; // 数据长度超过8字节，返回错误
    }

    switch (type)
    {
    case 0:
        tx_header.Identifier = id;
        tx_header.IdType = FDCAN_STANDARD_ID;
        tx_header.TxFrameType = FDCAN_DATA_FRAME;
        tx_header.DataLength = len;
        tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_header.BitRateSwitch = FDCAN_BRS_OFF;
        tx_header.FDFormat = FDCAN_CLASSIC_CAN;
        tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_header.MessageMarker = 0;
        break;
    case 1:
        tx_header.Identifier = id;
        tx_header.IdType = FDCAN_EXTENDED_ID;
        tx_header.TxFrameType = FDCAN_DATA_FRAME;
        tx_header.DataLength = len;
        tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        tx_header.BitRateSwitch = FDCAN_BRS_OFF;
        tx_header.FDFormat = FDCAN_CLASSIC_CAN;
        tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
        tx_header.MessageMarker = 0;
        break;
    default:
        return HAL_ERROR; // 不支持的ID类型，返回错误
    }

    for (uint8_t i = 0; i < len; i++)
    {
        send_data[i] = data[i];
    }

    status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, send_data);
    return status;
}

static HAL_StatusTypeDef FDCAN_ConfigAndStart(FDCAN_HandleTypeDef *hfdcan)
{
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (HAL_FDCAN_ActivateNotification(hfdcan,
                                        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                        0U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_FDCAN_Start(hfdcan);
}

static void RecoverOneFDCAN(FDCAN_HandleTypeDef *hfdcan)
{
    HAL_FDCAN_Stop(hfdcan);
    HAL_FDCAN_DeInit(hfdcan);

    osDelay(5);

    if (HAL_FDCAN_Init(hfdcan) == HAL_OK)
    {
        (void)FDCAN_ConfigAndStart(hfdcan);
    }
}

void RecoverCANBus(void)
{
    FDCAN_HandleTypeDef *cans[] = { &hfdcan1, &hfdcan2, &hfdcan3 };
    const uint32_t n = sizeof(cans) / sizeof(cans[0]);

    for (uint32_t i = 0; i < n; i++) {
        //如果bo位为0则正常，不为0则总线出现问题需要重启
        if ((cans[i]->Instance->PSR & FDCAN_PSR_BO) != 0U) {
            RecoverOneFDCAN(cans[i]);
        }
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
    {
        FDCAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
        {
            if (hfdcan == &hfdcan1)
            {
                if (rx_header.IdType == FDCAN_STANDARD_ID)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        DJI_Analysisdata(&helm_motor[i], &rx_header, rx_data);
                    }
                }
                else
                {
                    for (int i = 0; i < 4; i++)
                    {
                        VESC_Analysisdata(&vesc_motor[i], &rx_header, rx_data);
                    }
                }
            }
            else if(hfdcan == &hfdcan2)
            {

            }
            else if(hfdcan == &hfdcan3)
            {

            }
        }
    }
}
#elif defined(__STM32F4xx_HAL_H) || defined(__STM32F1xx_HAL_H)
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

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                             &rx_header, rx_data) == HAL_OK)
    {
        if (hcan == &hcan1)
        {

        }
        else if (hcan == &hcan2)
        {
            
        }
    }
}

#endif


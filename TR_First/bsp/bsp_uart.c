#include "bsp_uart.h"
#include "mymain.h"
#include "Handle.h"
#include "cmsis_os2.h"

uint8_t receive_imu[7];
uint8_t receive_dt35[19];
uint8_t handle_dma[68];             //UART9 收包缓冲，一次能装 4 帧，不然两帧挤一起就溢出

//三路 DMA 空闲中断接收：遥控、DT35、IMU
void Bsp_Uart_Init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart9, handle_dma, sizeof(handle_dma));
    __HAL_DMA_DISABLE_IT(huart9.hdmarx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, receive_dt35, sizeof(receive_dt35));
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, receive_imu, sizeof(receive_imu));
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART9)
    {
        Handle_RxFeed(handle_dma, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart9, handle_dma, sizeof(handle_dma));
        __HAL_DMA_DISABLE_IT(huart9.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == UART4)
    {
        if (Size == sizeof(receive_dt35))
        {
            osMessageQueuePut(dt35QueueHandle, receive_dt35, 0U, 0U);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, receive_dt35, sizeof(receive_dt35));
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == UART7)
    {
        if (Size == sizeof(receive_imu))
        {
            osMessageQueuePut(imuQueueHandle, receive_imu, 0U, 0U);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, receive_imu, sizeof(receive_imu));
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART9)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart9);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart9, handle_dma, sizeof(handle_dma));
        __HAL_DMA_DISABLE_IT(huart9.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, receive_dt35, sizeof(receive_dt35));
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, receive_imu, sizeof(receive_imu));
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
}

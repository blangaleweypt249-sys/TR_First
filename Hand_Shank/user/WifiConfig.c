#include "WifiConfig.h"
#include "usart.h"
#include "mymain.h"
#include "WpLib.h"
#include <string.h>

volatile uint8_t wifi_ready;

extern DMA_HandleTypeDef hdma_usart6_rx; 

static uint8_t wifi_rx_storage[512];
static uint8_t wifi_dma[128];
static RingBuffer_t wifi_rx;
static uint16_t wifi_dma_last;

//串口收改成循环DMA，不再逐字节轮询，主循环再忙也不会把字节收丢
void Wifi_RxStart(void)
{
    RingBuffer_Init(&wifi_rx, wifi_rx_storage, sizeof(wifi_rx_storage));
    wifi_dma_last = 0U;

    hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
    HAL_DMA_Init(&hdma_usart6_rx);
    HAL_UART_Receive_DMA(&huart6, wifi_dma, sizeof(wifi_dma));
}

//把DMA里新到的字节搬进环形队列
static void Wifi_RxPoll(void)
{
    uint16_t pos = (uint16_t)(sizeof(wifi_dma) - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx));

    if (pos > wifi_dma_last)
        WriteBuffer(&wifi_rx, &wifi_dma[wifi_dma_last], pos - wifi_dma_last);
    else if (pos < wifi_dma_last)
    {
        WriteBuffer(&wifi_rx, &wifi_dma[wifi_dma_last], sizeof(wifi_dma) - wifi_dma_last);
        WriteBuffer(&wifi_rx, wifi_dma, pos);
    }
    wifi_dma_last = pos;
}

uint8_t Wifi_GetByte(uint8_t *ch, uint32_t timeout)
{
    uint32_t start = HAL_GetTick();

    for (;;)
    {
        Wifi_RxPoll();
        if (RingBuffer_Pop(&wifi_rx, ch))
        {
            return 1U;
        }
        if ((HAL_GetTick() - start) >= timeout)
        {
            return 0U;
        }
        osDelay(1);
    }
}

//发失败时清掉残留，免得半截 ">>>" 影响下一帧
void Wifi_RxClear(void)
{
    Wifi_RxPoll();//先把 DMA 里已经到的字节搬进环形队列，再一起清掉
    RingBuffer_Clear(&wifi_rx);
}

static uint16_t Wifi_ReadResponse(uint8_t *buffer, uint16_t capacity, uint32_t timeout)
{
    uint16_t length = 0U;
    uint8_t ch;
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < timeout)
    {
        if (Wifi_GetByte(&ch, 20U) == 0U)
        {
            continue;
        }
        if (length + 1U < capacity)
        {
            buffer[length++] = ch;
            buffer[length] = '\0';
            if (strstr((char *)buffer, "wifi got ip") != NULL)
            {
                break;
            }
        }
    }

    buffer[length] = '\0';
    return length;
}

static void Wifi_SendCommand(const char *command, uint8_t *response,
                             uint16_t response_size, uint32_t timeout)
{
    HAL_UART_Transmit(&huart6, (uint8_t *)command,
                      (uint16_t)strlen(command), 1000U);
    Wifi_ReadResponse(response, response_size, timeout);
}

//只重建socket，不动WiFi，掉线恢复要快得多
static uint8_t Wifi_OpenSocket(void)
{
    const char client_cmd[] = "AT+SKTCLIENT=0,1,,192.168.43.1,6060,,1\r\n";
    const char state_cmd[] = "AT+SKTSTATE\r\n";
    uint8_t response[192];
    const char *socket;

    Wifi_SendCommand("AT+SKTDEL=0\r\n", response, sizeof(response), 500U);
    Wifi_SendCommand(client_cmd, response, sizeof(response), 2000U);
    if (strstr((char *)response, "OK") == NULL)
    {
        return 0U;
    }

    Wifi_SendCommand(state_cmd, response, sizeof(response), 1000U);
    socket = strstr((char *)response, "socket_fd:");
    return (socket != NULL && socket[10] != '-');
}

//整条重连：先连WiFi，再建socket
static uint8_t Wifi_Connect(void)
{
    const char conn_cmd[] = "AT+WLCONN=ssid,UPRE_WIFI,pw,12345678\r\n";
    uint8_t response[256];

    HAL_UART_Transmit(&huart6, (uint8_t *)conn_cmd,
                      sizeof(conn_cmd) - 1U, 1000U);
    Wifi_ReadResponse(response, sizeof(response), 10000U);
    if (strstr((char *)response, "wifi got ip") == NULL)
    {
        return 0U;
    }
    osDelay(2000U);

    Wifi_SendCommand("AT+WLRECONN=auto,1\r\n", response, sizeof(response), 300U); //断线让模块自己重连
    return Wifi_OpenSocket();
}

void WifiTask(void *argument)
{
    uint8_t had_connection = 0U;

    (void)argument;
    Wifi_RxStart();

    for (;;)
    {
        wifi_ready = 0U;
        if ((had_connection && Wifi_OpenSocket()) || Wifi_Connect())
        {
            wifi_ready = 1U;
            had_connection = 1U;
            Beep_SetAlarm(0U);
            Beep_Request(5U);
            while (wifi_ready)
            {
                osDelay(100U);
            }
        }
        else
        {
            if (had_connection)
            {
                Beep_SetAlarm(1U);
            }
            osDelay(1000U);
        }
    }
}

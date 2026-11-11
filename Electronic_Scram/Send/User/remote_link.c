/**
 * @file remote_link.c
 * @brief 实现急停发送板 USART3 上游与 USART1 LoRa 链路的 DMA 双向透明转发。
 */

#include "remote_link.h"
#include "usart.h"
#include <string.h>

#define REMOTE_RX_SIZE 512U
#define REMOTE_TX_SIZE 1024U
#define REMOTE_PORT_COUNT 2U
#define REMOTE_LORA_PORT 0U /* USART1 连接 LoRa 模块，用于无线字节流透传。 */
#define REMOTE_HOST_PORT 1U /* USART3 连接上游数据源，与 LoRa 端口双向转发。 */

typedef struct
{
    UART_HandleTypeDef *uart; /* 端口绑定的 HAL 串口句柄，用于区分 LoRa 端与主机端。 */
    uint8_t rx_buffer[REMOTE_RX_SIZE]; /* 循环 DMA 接收缓冲区，由硬件持续写入、主循环按累计索引消费。 */
    uint8_t tx_buffer[REMOTE_TX_SIZE]; /* 发送环形缓冲区，保存从另一端口转发的原始字节。 */
    volatile uint32_t rx_produced; /* 接收累计写入字节数，由 DMA 位置变化更新并用于检测覆盖。 */
    uint32_t rx_consumed; /* 主循环已经转发的累计字节数，与接收总数之差为待处理量。 */
    volatile uint16_t rx_position; /* 上次统计时 DMA 在接收环形缓冲区中的写入位置。 */
    volatile uint32_t tx_write; /* 发送环形队列累计写入索引，新数据复制完成后才提交。 */
    volatile uint32_t tx_read; /* 发送环形队列累计读索引，仅在对应 DMA 段发送完成后推进。 */
    volatile uint16_t tx_active_size; /* 当前 DMA 正在发送的连续数据段长度，零表示没有在途段。 */
    volatile uint8_t tx_complete; /* 发送完成回调置位，供发送推进逻辑回收在途段。 */
    volatile uint8_t restart_requested; /* 端口需要中止并重建收发状态的请求标志。 */
    volatile uint32_t uart_errors; /* 该端口触发串口错误回调的累计次数。 */
    volatile uint32_t rx_overruns; /* 处理数据前后发现 DMA 已覆盖未消费数据的累计次数。 */
    volatile uint32_t dropped_bytes; /* 因恢复或接收覆盖而主动丢弃的已统计字节数。 */
} remote_port_t;

static remote_port_t ports[REMOTE_PORT_COUNT];

/* 按 UART 句柄查找双串口转发端口，未匹配时返回空指针。 */
static remote_port_t *RemoteLink_FindPort(UART_HandleTypeDef *uart)
{
    uint32_t index;
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) {
        if (ports[index].uart == uart) return &ports[index];
    }
    return NULL;
}

/* 检查接收 DMA 为循环模式、发送 DMA 为普通模式，再启动带空闲检测的持续接收。 */
static HAL_StatusTypeDef RemoteLink_StartReceive(remote_port_t *port)
{
    if (port->uart->hdmarx == NULL || port->uart->hdmatx == NULL) return HAL_ERROR;
    /* 接收 DMA 持续循环运行，通过半满、完成和空闲事件统计环形缓冲区进度。 */
    if (port->uart->hdmarx->Init.Mode != DMA_CIRCULAR ||
        port->uart->hdmatx->Init.Mode != DMA_NORMAL) return HAL_ERROR;
    return HAL_UARTEx_ReceiveToIdle_DMA(port->uart, port->rx_buffer, REMOTE_RX_SIZE);
}

/* 在保留原中断状态的临界区内读取 DMA 剩余计数，累计环形接收进度并返回已接收字节总数。 */
static uint32_t RemoteLink_UpdateReceive(remote_port_t *port)
{
    uint32_t primask = __get_PRIMASK();
    uint32_t produced;
    uint16_t position;
    uint16_t count;
    __disable_irq();
    position = (uint16_t)((REMOTE_RX_SIZE - __HAL_DMA_GET_COUNTER(port->uart->hdmarx)) % REMOTE_RX_SIZE);
    count = (uint16_t)((position + REMOTE_RX_SIZE - port->rx_position) % REMOTE_RX_SIZE);
    port->rx_position = position;
    port->rx_produced += count;
    produced = port->rx_produced;
    __DMB();
    __set_PRIMASK(primask);
    return produced;
}

/* 中止异常串口传输、统计丢弃的待处理字节并清空收发索引，重新启动接收，失败时保留重试请求。 */
static void RemoteLink_Recover(remote_port_t *port)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (HAL_UART_Abort(port->uart) != HAL_OK) {
        __set_PRIMASK(primask);
        return;
    }
    port->dropped_bytes += port->tx_write - port->tx_read;
    port->dropped_bytes += port->rx_produced - port->rx_consumed;
    port->rx_produced = port->rx_consumed = 0U;
    port->rx_position = 0U;
    port->tx_write = port->tx_read = 0U;
    port->tx_active_size = 0U;
    port->tx_complete = 0U;
    port->restart_requested = 0U;
    if (RemoteLink_StartReceive(port) != HAL_OK) port->restart_requested = 1U;
    __set_PRIMASK(primask);
}

/* 在调用者保护的临界区内回收已发送数据并启动下一段连续缓冲区的 DMA 发送，非忙错误触发重启请求。 */
static void RemoteLink_TransmitLocked(remote_port_t *port)
{
    uint32_t available;
    uint16_t offset;
    uint16_t length;
    HAL_StatusTypeDef status;
    if (port->restart_requested != 0U) return;
    if (port->tx_complete != 0U) {
        port->tx_read += port->tx_active_size;
        port->tx_active_size = 0U;
        port->tx_complete = 0U;
    }
    if (port->tx_active_size != 0U) return;
    available = port->tx_write - port->tx_read;
    if (available == 0U) return;
    offset = (uint16_t)(port->tx_read % REMOTE_TX_SIZE);
    length = (uint16_t)(REMOTE_TX_SIZE - offset);
    if (available < length) length = (uint16_t)available;
    port->tx_active_size = length;
    /* 每次 DMA 只发送环形队列中的一段连续数据，发送完成后继续处理下一段。 */
    status = HAL_UART_Transmit_DMA(port->uart, &port->tx_buffer[offset], length);
    if (status != HAL_OK) {
        port->tx_active_size = 0U;
        if (status != HAL_BUSY) port->restart_requested = 1U;
    }
}

/* 保存并屏蔽中断后推进端口发送队列，完成后恢复原中断屏蔽状态。 */
static void RemoteLink_Transmit(remote_port_t *port)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    RemoteLink_TransmitLocked(port);
    __set_PRIMASK(primask);
}

/* 按目标发送队列剩余容量复制源端口数据，复制前后检查 DMA 覆盖，确认有效后提交两端索引。 */
static void RemoteLink_Forward(remote_port_t *source, remote_port_t *target)
{
    uint32_t produced;
    uint32_t available;
    uint32_t space;
    uint32_t count;
    uint32_t index;
    if (source->restart_requested != 0U || target->restart_requested != 0U) return;
    produced = RemoteLink_UpdateReceive(source);
    __DMB();
    available = produced - source->rx_consumed;
    if (available > REMOTE_RX_SIZE) {
        source->rx_overruns++;
        source->dropped_bytes += available;
        source->rx_consumed = produced;
        return;
    }
    space = REMOTE_TX_SIZE - (target->tx_write - target->tx_read);
    count = (available < space) ? available : space;
    for (index = 0U; index < count; index++) {
        target->tx_buffer[(target->tx_write + index) % REMOTE_TX_SIZE] =
            source->rx_buffer[(source->rx_consumed + index) % REMOTE_RX_SIZE];
    }
    __DMB();
    produced = RemoteLink_UpdateReceive(source);
    if (produced - source->rx_consumed > REMOTE_RX_SIZE) {
        source->rx_overruns++;
        source->dropped_bytes += produced - source->rx_consumed;
        source->rx_consumed = produced;
        return;
    }
    if (source->restart_requested != 0U || target->restart_requested != 0U) return;
    target->tx_write += count;
    source->rx_consumed += count;
}

/* 清空双串口状态并绑定 USART1、USART3，延时后启动两路循环 DMA 接收，失败端口标记为待重启。 */
void RemoteLink_Init(void)
{
    uint32_t index;
    memset(ports, 0, sizeof(ports));
    ports[0].uart = &huart1;
    ports[1].uart = &huart3;
    HAL_Delay(10U);
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) {
        if (RemoteLink_StartReceive(&ports[index]) != HAL_OK) {
            ports[index].restart_requested = 1U;
        }
    }
}

/* 轮询恢复异常端口、推进发送队列并双向转发主机与 LoRa 字节流，再启动新入队数据的发送。 */
void RemoteLink_Process(void)
{
    uint32_t index;
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) {
        if (ports[index].restart_requested != 0U) RemoteLink_Recover(&ports[index]);
        RemoteLink_Transmit(&ports[index]);
    }
    /* 将 USART3 上游数据写入 USART1 发送队列，经 DMA 交给 LoRa，同时处理反向回传。 */
    RemoteLink_Forward(&ports[REMOTE_HOST_PORT], &ports[REMOTE_LORA_PORT]);
    RemoteLink_Forward(&ports[REMOTE_LORA_PORT], &ports[REMOTE_HOST_PORT]);
    for (index = 0U; index < REMOTE_PORT_COUNT; index++) RemoteLink_Transmit(&ports[index]);
}

/* 查找触发事件的转发端口，在未请求重启时读取当前 DMA 位置并累计接收进度。 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
    remote_port_t *port = RemoteLink_FindPort(uart);
    (void)size;
    if (port != NULL && port->restart_requested == 0U) {
        (void)RemoteLink_UpdateReceive(port);
    }
}
/* 标记对应端口本段 DMA 发送完成，并立即推进发送队列以续发下一段连续数据。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    remote_port_t *port = RemoteLink_FindPort(uart);
    if (port != NULL) {
        port->tx_complete = 1U;
        RemoteLink_Transmit(port);
    }
}

/* 查找出错串口并累计错误次数，设置重启请求供主循环中止传输并重建收发状态。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    remote_port_t *port = RemoteLink_FindPort(uart);
    if (port != NULL) {
        port->uart_errors++;
        port->restart_requested = 1U;
    }
}


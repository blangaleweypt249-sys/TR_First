#include "W5500.h"
#include "main.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <string.h>

//W5500 数据手册：VDM 控制字 BSB 在位 7:3，RWB 在位 2；Socket 0 占连续三个块
enum
{
    W5500_COMMON = 0x00U, W5500_S0 = 0x08U, W5500_TX = 0x10U, W5500_RX = 0x18U,
    W5500_MR = 0x00U, W5500_SUBR = 0x05U, W5500_SHAR = 0x09U, W5500_SIPR = 0x0FU,
    W5500_PHY = 0x2EU, W5500_VERSION = 0x39U,
    W5500_SN_CR = 0x01U, W5500_SN_IR = 0x02U, W5500_SN_SR = 0x03U,
    W5500_SN_PORT = 0x04U, W5500_SN_DIPR = 0x0CU, W5500_SN_DIPORT = 0x10U,
    W5500_SN_RXBUF = 0x1EU, W5500_SN_TXBUF = 0x1FU,
    W5500_SN_TX_FSR = 0x20U, W5500_SN_TX_WR = 0x24U,
    W5500_SN_RX_RSR = 0x26U, W5500_SN_RX_RD = 0x28U,
    W5500_OPEN = 0x01U, W5500_CLOSE = 0x10U, W5500_SEND = 0x20U, W5500_RECV = 0x40U,
    W5500_SEND_OK = 0x10U, W5500_TIMEOUT = 0x08U,
    W5500_UDP_MODE = 0x02U, W5500_UDP_READY = 0x22U
};

#define W5500_BUF_KIB 2U //每个方向 2 KiB，与 Socket 0 的分配一致
#define W5500_UDP_HEADER 8U //硬件前缀：来源 IPv4、来源端口、载荷长度
#define W5500_SPI_TIMEOUT_MS 2U //最长只搬 8 字节，给 HAL 轮询留余量，不拖住网络任务

#define W5500_FRAME_SIZE 8U //A5、01、4 字节序号、累加和、5A
#define W5500_FRAME_HEAD 0xA5U
#define W5500_FRAME_PROBE 0x01U
#define W5500_FRAME_TAIL 0x5AU
#define W5500_FRAME_SUM 6U //校验覆盖索引 0–5，取低 8 位
#define W5500_RX_SOFT_LIMIT (16U * (W5500_UDP_HEADER + W5500_FRAME_SIZE)) //超 16 个短帧则丢弃积压，不持续回显旧帧

//直连保留厂家例程的地址和端口；02 开头是本地管理的单播 MAC，只给这一块主控使用
static uint8_t w5500_ip[4] = {192U, 168U, 1U, 199U};
static uint8_t w5500_mask[4] = {255U, 255U, 255U, 0U};
static uint8_t w5500_mac[6] = {0x02U, 0x00U, 0x00U, 0x01U, 0x01U, 0xC7U};
static const uint8_t w5500_peer[4] = {192U, 168U, 1U, 50U};
static uint8_t w5500_port[2] = {0x13U, 0x88U}; //5000，大端

static SPI_HandleTypeDef w5500_spi;
static bool w5500_tx_pending;
static volatile uint32_t w5500_rx_overload; //通过 SWD 记录过载时重开 UDP 的次数
int w5500_num[3];
static volatile uint8_t w5500_version_in; //0x04：SPI 读到 W5500，其他值：版本未通过
static volatile uint8_t w5500_phy_in; //PHYCFGR 原值，位 0 表示链路
static volatile uint8_t w5500_socket_in; //Sn_SR 原值，0x22 表示 UDP

static bool W5500_SpiInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI1_CLK_ENABLE();

    //PA15 做软件片选，先拉高再切输出，避免上电误收命令
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_15;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &gpio);

    gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &gpio);

    w5500_spi.Instance = SPI1;
    w5500_spi.Init.Mode = SPI_MODE_MASTER;
    w5500_spi.Init.Direction = SPI_DIRECTION_2LINES;
    w5500_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    w5500_spi.Init.CLKPolarity = SPI_POLARITY_LOW; //W5500 手册支持 SPI 模式 0
    w5500_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    w5500_spi.Init.NSS = SPI_NSS_SOFT;
    w5500_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; //SPI123 标称 137.5 MHz，初次布线联调用约 2.15 MHz
    w5500_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    w5500_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    w5500_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    w5500_spi.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    w5500_spi.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    w5500_spi.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    return HAL_SPI_Init(&w5500_spi) == HAL_OK;
}

static bool W5500_Transfer(uint8_t block, uint16_t addr, uint8_t *data, uint16_t size, bool write)
{
    //单次全双工搬完整个 VDM 帧；拆两次 HAL 调用会在片选未松开时重启 SPI。
    uint8_t tx[3U + W5500_UDP_HEADER] = {0};
    uint8_t rx[sizeof(tx)];
    HAL_StatusTypeDef result;

    if (size > W5500_UDP_HEADER)
        return false;
    tx[0] = (uint8_t)(addr >> 8);
    tx[1] = (uint8_t)addr;
    tx[2] = (uint8_t)(block | (write ? 0x04U : 0U));
    if (write)
        memcpy(&tx[3], data, size);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
    result = HAL_SPI_TransmitReceive(&w5500_spi, tx, rx, (uint16_t)(3U + size), W5500_SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    if (result == HAL_OK && !write)
        memcpy(data, &rx[3], size);
    return result == HAL_OK;
}

static bool W5500_WriteByte(uint8_t block, uint16_t addr, uint8_t value)
{
    return W5500_Transfer(block, addr, &value, 1U, true);
}

static uint16_t W5500_ReadBe16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] << 8) | data[1];
}

static bool W5500_ReadWord(uint16_t addr, uint16_t *value)
{
    uint8_t data[2];
    if (!W5500_Transfer(W5500_S0, addr, data, sizeof(data), false))
        return false;
    *value = W5500_ReadBe16(data);
    return true;
}

static bool W5500_WriteWord(uint16_t addr, uint16_t value)
{
    uint8_t data[] = {(uint8_t)(value >> 8), (uint8_t)value};
    return W5500_Transfer(W5500_S0, addr, data, sizeof(data), true);
}

static bool W5500_ReadCount(uint16_t addr, uint16_t *value)
{
    uint16_t prev, next;
    //收发计数会在 SPI 读到一半时变化；按手册要求读到两次相同，最多比对 3 次
    if (!W5500_ReadWord(addr, &prev))
        return false;
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (!W5500_ReadWord(addr, &next))
            return false;
        if (prev == next)
        {
            *value = next;
            return true;
        }
        prev = next;
    }
    *value = 0U; //本周期不搬，避免用到半更新的计数
    return true;
}

static bool W5500_Command(uint8_t command)
{
    uint8_t state;
    if (!W5500_WriteByte(W5500_S0, W5500_SN_CR, command))
        return false;
    //手册要求命令寄存器自清后才能写下一条；最多读 3 次，不等网络应答
    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (!W5500_Transfer(W5500_S0, W5500_SN_CR, &state, 1U, false))
            return false;
        if (state == 0U)
            return true;
    }
    return false;
}

static bool W5500_Init(void)
{
    uint8_t version;
    //先确认 SPI 能读到 W5500 的版本值 0x04，再走软件复位，不依赖未确认的 RST 引脚
    if (!W5500_Transfer(W5500_COMMON, W5500_VERSION, &version, 1U, false))
        return false;
    w5500_version_in = version;
    if (version != 0x04U || !W5500_WriteByte(W5500_COMMON, W5500_MR, 0x80U))
        return false;
    osDelay(5U); //手册复位后内部稳定需 1 ms，留出 5 个 RTOS tick
    w5500_tx_pending = false;
    return W5500_Transfer(W5500_COMMON, W5500_VERSION, &version, 1U, false) &&
           version == 0x04U &&
           W5500_Transfer(W5500_COMMON, W5500_SUBR, w5500_mask, sizeof(w5500_mask), true) &&
           W5500_Transfer(W5500_COMMON, W5500_SHAR, w5500_mac, sizeof(w5500_mac), true) &&
           W5500_Transfer(W5500_COMMON, W5500_SIPR, w5500_ip, sizeof(w5500_ip), true) &&
           W5500_WriteByte(W5500_S0, W5500_SN_RXBUF, W5500_BUF_KIB) &&
           W5500_WriteByte(W5500_S0, W5500_SN_TXBUF, W5500_BUF_KIB);
}

static bool W5500_Analysis(const uint8_t *frame)
{
    uint8_t sum = 0U;
    if (frame[0] != W5500_FRAME_HEAD || frame[W5500_FRAME_SIZE - 1U] != W5500_FRAME_TAIL)
        return false;
    for (uint8_t i = 0U; i < W5500_FRAME_SUM; i++)
        sum = (uint8_t)(sum + frame[i]);
    return sum == frame[W5500_FRAME_SUM] && frame[1] == W5500_FRAME_PROBE;
}

static bool W5500_OpenSocket(void)
{
    w5500_tx_pending = false;
    return W5500_Command(W5500_CLOSE) &&
           W5500_WriteByte(W5500_S0, W5500_SN_IR, 0x1FU) && //Sn_IR 五个位写 1 清除
           W5500_WriteByte(W5500_S0, 0U, W5500_UDP_MODE) &&
           W5500_Transfer(W5500_S0, W5500_SN_PORT, w5500_port, sizeof(w5500_port), true) &&
           W5500_Command(W5500_OPEN);
}

static bool W5500_Poll(void)
{
    uint8_t phy, state, irq;
    uint16_t queued, rd, room, wr;
    uint8_t head[W5500_UDP_HEADER];
    uint8_t frame[W5500_FRAME_SIZE];

    if (!W5500_Transfer(W5500_COMMON, W5500_PHY, &phy, 1U, false))
        return false;
    w5500_phy_in = phy;
    if ((phy & 1U) == 0U) //PHYCFGR 的位 0 是物理链路
        return true;
    if (!W5500_Transfer(W5500_S0, W5500_SN_SR, &state, 1U, false))
        return false;
    w5500_socket_in = state;
    if (state != W5500_UDP_READY)
        return W5500_OpenSocket();
    if (w5500_tx_pending)
    {
        if (!W5500_Transfer(W5500_S0, W5500_SN_IR, &irq, 1U, false))
            return false;
        irq &= W5500_SEND_OK | W5500_TIMEOUT;
        if (irq == 0U)
            return true;
        if (!W5500_WriteByte(W5500_S0, W5500_SN_IR, irq))
            return false;
        w5500_tx_pending = false;
    }

    if (!W5500_ReadCount(W5500_SN_RX_RSR, &queued))
        return false;
    if (queued > W5500_RX_SOFT_LIMIT)
    {
        //只重开 UDP 清队列，保留 PHY 和 IP；让负载退下去时新包马上能走。
        w5500_rx_overload++;
        return W5500_OpenSocket();
    }
    if (queued < W5500_UDP_HEADER)
        return true;
    if (!W5500_ReadWord(W5500_SN_RX_RD, &rd) ||
        !W5500_Transfer(W5500_RX, rd, head, sizeof(head), false))
        return false;
    uint16_t length = W5500_ReadBe16(&head[6]); //硬件 UDP 前缀的最后两字节是载荷长度
    if (length > W5500_BUF_KIB * 1024U - W5500_UDP_HEADER)
        return false;
    if (queued < W5500_UDP_HEADER + length)
        return true;
    bool echo = length == sizeof(frame) && memcmp(head, w5500_peer, sizeof(w5500_peer)) == 0;
    if (echo && !W5500_Transfer(W5500_RX, (uint16_t)(rd + W5500_UDP_HEADER), frame, sizeof(frame), false))
        return false;

    //每次只消费一包；不认识也要推进读指针，否则坏包会一直挡在队头
    if (!W5500_WriteWord(W5500_SN_RX_RD, (uint16_t)(rd + W5500_UDP_HEADER + length)) ||
        !W5500_Command(W5500_RECV))
        return false;
    if (!echo || !W5500_Analysis(frame))
        return true;

    //网络只回显诊断帧，不写底盘、机械臂或遥控状态
    w5500_num[0] = w5500_num[0] >= 1000 ? 0 : w5500_num[0] + 1; //沿用遥控计数回卷范围
    if (!W5500_ReadCount(W5500_SN_TX_FSR, &room))
        return false;
    if (room < sizeof(frame))
        return true;
    if (!W5500_ReadWord(W5500_SN_TX_WR, &wr) ||
        !W5500_Transfer(W5500_S0, W5500_SN_DIPR, head, sizeof(w5500_peer), true) ||
        !W5500_Transfer(W5500_S0, W5500_SN_DIPORT, &head[4], 2U, true) || //回到来源 UDP 端口
        !W5500_Transfer(W5500_TX, wr, frame, sizeof(frame), true) ||
        !W5500_WriteWord(W5500_SN_TX_WR, (uint16_t)(wr + sizeof(frame))) ||
        !W5500_Command(W5500_SEND))
        return false;
    w5500_tx_pending = true;
    return true;
}

void W5500_Task(void *argument)
{
    (void)argument;
    bool spi_ok = W5500_SpiInit();
    bool ready = false;
    for (;;)
    {
        if (!spi_ok)
            spi_ok = W5500_SpiInit();
        if (spi_ok)
            ready = ready ? W5500_Poll() : W5500_Init();
        osDelay(ready ? 5U : 100U); //正常最多每 5 ms 处理一包；模块未就绪每 100 ms 再探测
    }
}

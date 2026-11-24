#include "W5500.h"
#include "main.h"
#include "spi.h"
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

#define W5500_UDP_HEADER 8U //硬件前缀：来源 IPv4、来源端口、载荷长度
#define W5500_BUF_KIB 2U //每个方向 2 KiB，与 Socket 0 的分配一致

//直连保留厂家例程的地址和端口；02 开头是本地管理的单播 MAC，只给这一块主控使用
static uint8_t w5500_ip[4] = {192U, 168U, 1U, 199U};
static uint8_t w5500_mask[4] = {255U, 255U, 255U, 0U};
static uint8_t w5500_mac[6] = {0x02U, 0x00U, 0x00U, 0x01U, 0x01U, 0xC7U};
static const uint8_t w5500_peer[4] = {192U, 168U, 1U, 50U};
static uint8_t w5500_port[2] = {0x13U, 0x88U}; //5000，大端

static bool w5500_tx_pending;
static volatile uint32_t w5500_rx_overload; //通过 SWD 记录过载时重开 UDP 的次数
int w5500_num[3];
static volatile uint8_t w5500_version_in; //0x04：SPI 读到 W5500，其他值：版本未通过
static volatile uint8_t w5500_phy_in; //PHYCFGR 原值，位 0 表示链路
static volatile uint8_t w5500_socket_in; //Sn_SR 原值，0x22 表示 UDP

//这里只按 VDM 搬数据
static bool W5500_Transfer(uint8_t block, uint16_t addr, uint8_t *data, uint16_t size, bool write)
{
    //单次全双工搬完整个 VDM 帧；拆两次 HAL 调用会在片选未松开时重启 SPI
    uint8_t tx[3U + W5500_UDP_HEADER] = {0};
    uint8_t rx[sizeof(tx)];
    HAL_StatusTypeDef result;

    tx[0] = (uint8_t)(addr >> 8);
    tx[1] = (uint8_t)addr;
    tx[2] = (uint8_t)(block | (write ? 0x04U : 0U));
    if (write)
        memcpy(&tx[3], data, size);

    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_RESET);
    result = HAL_SPI_TransmitReceive(&hspi1, tx, rx, (uint16_t)(3U + size), 2U); //最长搬 8 字节，2ms 足够
    HAL_GPIO_WritePin(W5500_CS_GPIO_Port, W5500_CS_Pin, GPIO_PIN_SET);
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
    //收发计数会在 SPI 读到一半时变；按手册要求读到两次相同，最多比 3 次
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

bool W5500_Init(void)
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

//诊断帧 8 字节：A5、01、4 字节序号、前 6 字节累加和、5A
static bool W5500_Analysis(const uint8_t *frame)
{
    uint8_t sum = 0U;
    if (frame[0] != 0xA5U || frame[7] != 0x5AU)
        return false;
    for (uint8_t i = 0U; i < 6U; i++)
        sum = (uint8_t)(sum + frame[i]);
    return sum == frame[6] && frame[1] == 0x01U;
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

bool W5500_Poll(void)
{
    uint8_t phy, state, irq;
    uint16_t queued, rd, room, wr;
    uint8_t head[W5500_UDP_HEADER];
    uint8_t frame[8];

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
    if (queued > 16U * (W5500_UDP_HEADER + sizeof(frame))) //积压超 16 个短帧就丢，别一直回显旧帧
    {
        //只重开 UDP 清队列，保留 PHY 和 IP；负载退下去时新包马上能走
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




#include "w25q128.h"

W25Q128 w25q128;

static inline void W25Q128_CS_Enable(W25Q128 *flash)
{
    HAL_GPIO_WritePin(flash->W25Q128_CS.GPIOx,
                      flash->W25Q128_CS.GPIO_Pin,
                      GPIO_PIN_RESET);
}

static inline void W25Q128_CS_Disable(W25Q128 *flash)
{
    HAL_GPIO_WritePin(flash->W25Q128_CS.GPIOx,
                      flash->W25Q128_CS.GPIO_Pin,
                      GPIO_PIN_SET);
}

/* 检查地址和长度是否在 Flash 范围内 */
static uint8_t W25Q128_CheckAddress(uint32_t addr, uint32_t size)
{
    if (size == 0U || addr >= W25Q128_CAPACITY)
        return 0U;

    if (size > W25Q128_CAPACITY - addr)
        return 0U;

    return 1U;
}

/* 初始化 */
void W25Q128_Init(W25Q128 *flash,SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
    flash->hspi = hspi;
    flash->W25Q128_CS.GPIOx = GPIOx;
    flash->W25Q128_CS.GPIO_Pin = GPIO_Pin;

    W25Q128_CS_Disable(flash);
}

/* 写使能 */
void W25Q128_WriteEnable(W25Q128 *flash)
{
    uint8_t cmd = CMD_WRITE_ENABLE;

    W25Q128_CS_Enable(flash);
    HAL_SPI_Transmit(flash->hspi, &cmd, 1, 100);
    W25Q128_CS_Disable(flash);
}

/* 写禁止 */
void W25Q128_WriteDisable(W25Q128 *flash)
{
    uint8_t cmd = CMD_WRITE_DISABLE;

    W25Q128_CS_Enable(flash);
    HAL_SPI_Transmit(flash->hspi, &cmd, 1, 100);
    W25Q128_CS_Disable(flash);
}

/* 读取状态寄存器 */
uint8_t W25Q128_ReadStatus(W25Q128 *flash)
{
    uint8_t cmd = CMD_READ_STATUS_REG;
    uint8_t status = 0;

    W25Q128_CS_Enable(flash);

    if (HAL_SPI_Transmit(flash->hspi, &cmd, 1, 100) == HAL_OK)
        HAL_SPI_Receive(flash->hspi, &status, 1, 100);

    W25Q128_CS_Disable(flash);

    return status;
}

/* 等待 Flash 空闲 */
HAL_StatusTypeDef W25Q128_WaitReady(W25Q128 *flash)
{
    uint32_t start = HAL_GetTick();

    while ((HAL_GetTick() - start) < 5000U)
    {
        if ((W25Q128_ReadStatus(flash) & 0x01U) == 0U)
            return HAL_OK;
    }

    return HAL_TIMEOUT;
}

/* 擦除扇区：SectorNum 是扇区编号，从 0 开始 */
HAL_StatusTypeDef W25Q128_EraseSector(W25Q128 *flash, uint32_t SectorNum)
{
    uint32_t Num;
    uint8_t cmd[4];
    HAL_StatusTypeDef ret;

    if (flash == NULL || flash->hspi == NULL ||
        SectorNum >= W25Q128_SECTOR_COUNT)
        return HAL_ERROR;

    Num = SectorNum * W25Q128_SECTOR_SIZE;

    cmd[0] = CMD_SECTOR_ERASE;
    cmd[1] = (uint8_t)(Num >> 16);
    cmd[2] = (uint8_t)(Num >> 8);
    cmd[3] = (uint8_t)Num;

    ret = W25Q128_WaitReady(flash);
    if (ret != HAL_OK)
        return ret;

    W25Q128_WriteEnable(flash);

    W25Q128_CS_Enable(flash);
    ret = HAL_SPI_Transmit(flash->hspi, cmd, 4, 100);
    W25Q128_CS_Disable(flash);

    if (ret != HAL_OK)
        return ret;

    return W25Q128_WaitReady(flash);
}

/* 写入一页内的数据：不能跨页，不负责擦除 */
HAL_StatusTypeDef W25Q128_WritePage(W25Q128 *flash, const uint8_t *pBuffer, uint32_t WriteAddr, uint16_t Size)
{
    uint8_t cmd[4];
    HAL_StatusTypeDef ret;

    if (flash == NULL || flash->hspi == NULL || pBuffer == NULL)
        return HAL_ERROR;

    if (!W25Q128_CheckAddress(WriteAddr, Size))
        return HAL_ERROR;

    if (Size > W25Q128_PAGE_SIZE ||
        (WriteAddr % W25Q128_PAGE_SIZE) + Size > W25Q128_PAGE_SIZE)
        return HAL_ERROR;

    ret = W25Q128_WaitReady(flash);
    if (ret != HAL_OK)
        return ret;

    W25Q128_WriteEnable(flash);

    cmd[0] = CMD_PAGE_PROGRAM;
    cmd[1] = (uint8_t)(WriteAddr >> 16);
    cmd[2] = (uint8_t)(WriteAddr >> 8);
    cmd[3] = (uint8_t)WriteAddr;

    W25Q128_CS_Enable(flash);

    ret = HAL_SPI_Transmit(flash->hspi, cmd, 4, 100);

    if (ret == HAL_OK)
    {
        ret = HAL_SPI_Transmit(flash->hspi, (uint8_t *)pBuffer, Size, 100);
    }

    W25Q128_CS_Disable(flash);

    if (ret != HAL_OK)
        return ret;

    return W25Q128_WaitReady(flash);
}

/* 连续写入：自动拆分页，不负责擦除 */
HAL_StatusTypeDef W25Q128_WriteData(W25Q128 *flash, const uint8_t *pBuffer, uint32_t WriteAddr, uint32_t Size)
{
    uint32_t PageRemain;
    uint16_t WriteSize;
    HAL_StatusTypeDef ret;

    if (flash == NULL || pBuffer == NULL)
        return HAL_ERROR;

    if (!W25Q128_CheckAddress(WriteAddr, Size))
        return HAL_ERROR;

    while (Size > 0U)
    {
        PageRemain = W25Q128_PAGE_SIZE - (WriteAddr % W25Q128_PAGE_SIZE);

        WriteSize = (Size < PageRemain) ? (uint16_t)Size : (uint16_t)PageRemain;

        ret = W25Q128_WritePage(flash, pBuffer, WriteAddr, WriteSize);
        if (ret != HAL_OK)
            return ret;

        pBuffer += WriteSize;
        WriteAddr += WriteSize;
        Size -= WriteSize;
    }

    return HAL_OK;
}

/* 读取连续数据 */
HAL_StatusTypeDef W25Q128_ReadData(W25Q128 *flash, uint8_t *pBuffer, uint32_t ReadAddr, uint32_t Size)
{
    uint8_t cmd[4];
    HAL_StatusTypeDef ret;

    if (flash == NULL || flash->hspi == NULL || pBuffer == NULL)
        return HAL_ERROR;

    if (!W25Q128_CheckAddress(ReadAddr, Size))
        return HAL_ERROR;

    ret = W25Q128_WaitReady(flash);
    if (ret != HAL_OK)
        return ret;

    cmd[0] = CMD_READ_DATA;
    cmd[1] = (uint8_t)(ReadAddr >> 16);
    cmd[2] = (uint8_t)(ReadAddr >> 8);
    cmd[3] = (uint8_t)ReadAddr;

    W25Q128_CS_Enable(flash);

    ret = HAL_SPI_Transmit(flash->hspi, cmd, 4, 100);

    if (ret == HAL_OK)
    {
        ret = HAL_SPI_Receive(flash->hspi, pBuffer, Size, 100);
    }

    W25Q128_CS_Disable(flash);

    return ret;
}


#ifndef __W25Q128_H__
#define __W25Q128_H__

#include "main.h"

// 地址位段	   对应十六进制	     含义	       范围
// bit23 ~ bit16	AA	       块号（Block）  0x00 ~ 0xFF（256 块）
// bit15 ~ bit12  BB的高半字节	块内扇区号	  0x0 ~ 0xF（16 扇区）
// bit11 ~ bit8	  BB的低半字节	扇区内页号	  0x0 ~ 0xF（16 页）
// bit7 ~ bit0	    CC	       页内字节偏移	  0x00 ~ 0xFF（256 字节）

#define W25Q128_CAPACITY       0x01000000UL
#define W25Q128_SECTOR_SIZE    4096U
#define W25Q128_PAGE_SIZE      256U
#define W25Q128_SECTOR_COUNT   4096U

#define CMD_WRITE_ENABLE       0x06
#define CMD_WRITE_DISABLE      0x04
#define CMD_READ_STATUS_REG    0x05
#define CMD_READ_DATA          0x03
#define CMD_PAGE_PROGRAM       0x02
#define CMD_SECTOR_ERASE       0x20
#define CMD_CHIP_ERASE         0xC7
#define CMD_JEDEC_ID           0x9F

typedef struct
{
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
} SPI_CS;

typedef struct
{
    SPI_CS W25Q128_CS;
    SPI_HandleTypeDef *hspi;
} W25Q128;

extern W25Q128 w25q128;

void W25Q128_Init(W25Q128 *flash,SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
void W25Q128_WriteEnable(W25Q128 *flash);
void W25Q128_WriteDisable(W25Q128 *flash);

uint8_t W25Q128_ReadStatus(W25Q128 *flash);
HAL_StatusTypeDef W25Q128_WaitReady(W25Q128 *flash);
HAL_StatusTypeDef W25Q128_EraseSector(W25Q128 *flash,uint32_t SectorNum);
HAL_StatusTypeDef W25Q128_WritePage(W25Q128 *flash,const uint8_t *pBuffer,uint32_t WriteAddr,uint16_t Size);
HAL_StatusTypeDef W25Q128_WriteData(W25Q128 *flash,const uint8_t *pBuffer,uint32_t WriteAddr,uint32_t Size);
HAL_StatusTypeDef W25Q128_ReadData(W25Q128 *flash,uint8_t *pBuffer,uint32_t ReadAddr,uint32_t Size);

#endif

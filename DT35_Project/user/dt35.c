#include "dt35.h"

//INA226 的寄存器，DT35 内部挂的就是它
#define INA226_REG_CONFIG   0x00U
#define INA226_REG_BUS      0x02U
#define INA226_CONFIG       0x4526U

//量程 50~200mm，对应 0~10000mV
#define DT35_NEAR_MM        50U
#define DT35_FAR_MM         200U
#define DT35_NEAR_MV        0U
#define DT35_FAR_MV         10000U

#define DT35_TIMEOUT_MS     20U

static void dt35_write_reg(DT35 *dt35, uint8_t reg, uint16_t value)
{
    uint8_t data[2];

    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
    HAL_I2C_Mem_Write(dt35->hi2c, dt35->addr << 1U, reg,
                      I2C_MEMADD_SIZE_8BIT, data, 2U, DT35_TIMEOUT_MS);
}

static uint8_t dt35_read_reg(DT35 *dt35, uint8_t reg, uint16_t *value)
{
    uint8_t data[2];

    if (HAL_I2C_Mem_Read(dt35->hi2c, dt35->addr << 1U, reg,
                         I2C_MEMADD_SIZE_8BIT, data, 2U,
                         DT35_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    *value = ((uint16_t)data[0] << 8U) | data[1];
    return 1U;
}

void DT35_Init(DT35 *dt35, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    dt35->hi2c = hi2c;
    dt35->addr = addr;
    dt35->distance_mm = 0U;

    dt35->online = (HAL_I2C_IsDeviceReady(hi2c, addr << 1U, 2U,
                                          DT35_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;

    if (dt35->online)
    {
        dt35_write_reg(dt35, INA226_REG_CONFIG, INA226_CONFIG);
    }
}

//电压 mV 线性换算成 mm，超出量程直接给端点值
uint8_t DT35_Read(DT35 *dt35)
{
    uint16_t raw;
    uint32_t voltage_mv;

    if (dt35->online == 0U)
    {
        return 0U;
    }

    if (dt35_read_reg(dt35, INA226_REG_BUS, &raw) == 0U)
    {
        return 0U;
    }

    voltage_mv = ((uint32_t)raw * 5U + 2U) / 4U;

    if (voltage_mv <= DT35_NEAR_MV)
    {
        dt35->distance_mm = DT35_NEAR_MM;
    }
    else if (voltage_mv >= DT35_FAR_MV)
    {
        dt35->distance_mm = DT35_FAR_MM;
    }
    else
    {
        dt35->distance_mm = DT35_NEAR_MM +
                            (uint16_t)((voltage_mv - DT35_NEAR_MV) *
                                       (DT35_FAR_MM - DT35_NEAR_MM) /
                                       (DT35_FAR_MV - DT35_NEAR_MV));
    }

    return 1U;
}

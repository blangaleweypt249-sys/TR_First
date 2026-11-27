#include "100A.h"
#include "usart.h"

// 100A数据更新周期0.5ms
#define GYRO_DT_SEC 0.0005f

ImuData_t imu_100A;
volatile float yaw;

static uint8_t angle_tx[7];
static float last_gyro_speed;
static bool has_last_speed;

bool Imu_ReadData(const uint8_t data[8])
{
    uint32_t orginal_speed;
    uint16_t orginal_temp;

    if ((data[0] != 0x80U) ||
        (((data[1] | data[2] | data[3] | data[4] | data[6] | data[7]) & 0x80U) != 0U) ||
        ((data[1] ^ data[2] ^ data[3] ^ data[4]) != data[5]))
    {
        return false;
    }
    // 解析原始角速度
    orginal_speed = ((uint32_t)(data[1] & 0x7FU) << 21) |
                    ((uint32_t)(data[2] & 0x7FU) << 14) |
                    ((uint32_t)(data[3] & 0x7FU) << 7) |
                    ((uint32_t)(data[4] & 0x7FU));
    if ((orginal_speed & 0x08000000U) != 0U)
    {
        orginal_speed |= 0xF0000000U;
    }

    imu_100A.orginal_speed = (int32_t)orginal_speed;
    imu_100A.gyro_speed = (float)imu_100A.orginal_speed / 14078.5f;
    // 解析原始温度
    orginal_temp = (uint16_t)(((uint16_t)(data[6] & 0x7FU) << 7) |
                              ((uint16_t)(data[7] & 0x7FU)));
    if ((orginal_temp & 0x2000U) != 0U)
    {
        orginal_temp |= 0xC000U;
    }

    imu_100A.orginal_temp = (int16_t)orginal_temp;
    imu_100A.gyro_temp = (float)imu_100A.orginal_temp * 0.0625f;

    if (imu_100A.num[0] >= 1000)
        imu_100A.num[0] = 0;
    else
        imu_100A.num[0]++;
    return true;
}

void Imu_Calc(void)
{
    static float speed_hist[3];
    static uint8_t speed_idx;   //写指针 0..2
    static uint8_t speed_n;     //已经填了几个
    static float still_buf[32];//约 16ms，判静止用
    static uint8_t still_idx;
    static uint8_t still_n;
    float a, b, c, gyro_speed = imu_100A.gyro_speed - imu_100A.basic_speed;

    if (!has_last_speed)
    {
        last_gyro_speed = gyro_speed;
        has_last_speed = true;
        return;
    }

    //3点中值，剔掉单点尖峰
    speed_hist[speed_idx] = gyro_speed;
    speed_idx = (uint8_t)((speed_idx + 1U) % 3U);
    if (speed_n < 3U)
        speed_n++;
    if (speed_n == 3U)
    {
        a = speed_hist[0];
        b = speed_hist[1];
        c = speed_hist[2];
        if (a > b)
            gyro_speed = (b > c) ? b : ((a > c) ? c : a);
        else
            gyro_speed = (a > c) ? a : ((b > c) ? c : b);
    }

    //静止检测：短窗口峰峰值小才跟零偏；微动时窗口被打散，自动不跟
    still_buf[still_idx] = imu_100A.gyro_speed;
    still_idx = (uint8_t)((still_idx + 1U) % 32U);
    if (still_n < 32U)
    {
        still_n++;
    }
    else
    {
        float win_min = still_buf[0];
        float win_max = still_buf[0];
        float win_sum = 0.0f;

        for (uint8_t i = 0U; i < 32U; i++)
        {
            if (still_buf[i] < win_min)
                win_min = still_buf[i];
            if (still_buf[i] > win_max)
                win_max = still_buf[i];
            win_sum += still_buf[i];
        }

        if ((win_max - win_min) < 0.05f)//噪声只有 0.003，峰峰值小于 0.05 才算真没动
            imu_100A.basic_speed += (win_sum / 32.0f - imu_100A.basic_speed) * 0.0005f;//τ 约 1 秒
    }

    if (fabsf(gyro_speed) < 0.03f)
        gyro_speed = 0.0f;
    yaw = wrap_angle(yaw + (gyro_speed + last_gyro_speed) * 0.5f * GYRO_DT_SEC);
    last_gyro_speed = gyro_speed;
}

void Imu_ClearYaw(void)
{
    yaw = 0.0f;
    last_gyro_speed = 0.0f;
    has_last_speed = false;
}

void SendRad(void)
{
    float_u8 angle;

    if (huart3.gState != HAL_UART_STATE_READY)
    {
        return;
    }
    // 角度转弧度
    AngleToRad(yaw, &angle.f);
    angle_tx[0] = 0x1AU;
    for (uint8_t i = 0U; i < 4U; i++)
    {
        angle_tx[i + 1U] = angle.u8[i];
    }
    angle_tx[5] = My_CrcSum(&angle_tx[1], 4U);
    angle_tx[6] = 0xA1U;
    (void)HAL_UART_Transmit_DMA(&huart3, angle_tx, sizeof(angle_tx));
}

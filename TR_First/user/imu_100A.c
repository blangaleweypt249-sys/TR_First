#include "imu_100A.h"
#include "usart.h"

flo_to_u8 imu_data;
float Imu_angle;
int imu_num[3];

static uint8_t imu_clear_flag;

//给 485 小板发清零帧，让它把 yaw 归零
void Imu_ClearYaw(void)
{
  const uint8_t clear_cmd[4] = {0xA5U, 0x5AU, 0x01U, 0xFEU};
  HAL_UART_Transmit_DMA(&huart7, (uint8_t *)clear_cmd, sizeof(clear_cmd));
}

void Imu_Analysis(uint8_t *data)
{
  uint16_t crc = 0;
  if(data[0]==0x1A && data[6]==0xA1)
  {
    for(uint8_t i=1;i<=4;i++)
    {
      crc +=data[i];
    }
    if((crc&0xFF)==data[5])
    {
        if(imu_clear_flag == 0U)//第一帧到了说明小板在线，初始化发一次清零
        {
            Imu_ClearYaw();
            imu_clear_flag = 1U;
        }
        for(uint8_t i=0;i<4;i++)
        {
            imu_data.u8[i] = data[i+1];
        }
        Imu_angle = imu_data.flo;
        if (imu_num[0] >= 1000)
            imu_num[0] = 0;
        else
            imu_num[0]++;
    }
  }
}

#include "imu_100A.h"

flo_to_u8 imu_data;
float Imu_angle;
int imu_num[3];


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

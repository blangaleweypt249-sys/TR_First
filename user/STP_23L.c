#include "STP_23L.h"

STP_Data Stp23l_data[12] = {0};

void STP_23L_ReadData(uint8_t *data, float *stp_distance)
{
    int32_t distance = 0;
    uint16_t crc = 0;
    if (data != NULL)
    {
        if (data[0] == 0xAA && data[1] == 0xAA && data[2] == 0xAA && data[3] == 0xAA && data[5] == 0x02 && data[8] == 0xB8 && data[9] == 0x00)
        {
            crc = My_CrcSum(&data[4], 21);
            if (crc == data[25])
            {
                for (uint8_t i = 0; i < 12; i++)
                {
                    int offest = 10 + 15 * i;
                    
                    Stp23l_data[i].distance = (data[offest + 1] << 8) + data[offest];
                    Stp23l_data[i].noise = (data[offest + 3] << 8) + data[offest + 2];
                    Stp23l_data[i].peak = ((uint32_t)data[offest + 7] << 24) + ((uint32_t)data[offest + 6] << 16) + ((uint32_t)data[offest + 5] << 8) + data[offest + 4];
                    Stp23l_data[i].confidence = data[offest + 8];
                    Stp23l_data[i].intg = ((uint32_t)data[offest + 12] << 24) + ((uint32_t)data[offest + 11] << 16) + ((uint32_t)data[offest + 10] << 8) + data[offest + 9];
                    Stp23l_data[i].reftof = (data[offest + 14] << 8) + data[offest + 13];
                    distance += Stp23l_data[i].distance;
                }

                //测量单位为mm
                *stp_distance = distance / 12.0f;
            }
        }
    }
}
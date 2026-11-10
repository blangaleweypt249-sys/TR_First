#include "dt35.h"

U8_to_Flo distance;

DT35_Move dt35_front;
DT35_Move dt35_back;
DT35_Move dt35_left;
DT35_Move dt35_right;

PID pid_dt35_left;
PID pid_dt35_back;
PID pid_dt35_right;
PID pid_dt35_front;

DT35_Move *dt35_list[4] =
{
    &dt35_front,
    &dt35_back,
    &dt35_left,
    &dt35_right
};

void DT35_Analysis(uint8_t *data)
{
    uint16_t crc = 0;
    if (data[0] == 0x35 && data[18] == 0x53)
    {
        for (uint8_t i = 1; i <= 16; i++)
        {
            crc += data[i];
        }
        if ((crc & 0xFF) == data[17])
        {
            for (uint8_t i = 1; i <= 16; i++)
            {
                distance.u8[i - 1] = data[i];
            }
            dt35_front.actual_dt35_distance = distance.flo[3];
            dt35_back.actual_dt35_distance = distance.flo[0];
            dt35_left.actual_dt35_distance = distance.flo[1];
            dt35_right.actual_dt35_distance = distance.flo[2];

            // 检查外设连接状态
            for (uint8_t i = 0; i < 4; i++)
            {
                if (dt35_list[i]->dt35_num[0] >= 1000)
                    dt35_list[i]->dt35_num[0] = 0;
                else
                    dt35_list[i]->dt35_num[0]++;
            }
        }
    }
}

void Located_By_Left_DT35(DT35_Move *dt35,Speed  *world_speed,float current_angle)
{
	
	if(current_angle > PI)
	{
		current_angle -= 2 * PI;
	}
	else if(current_angle < -PI)
	{
		current_angle += 2 * PI;
	}
	
	if(fabsf(current_angle - 0) < 0.05){      //车身朝前
		world_speed->vy = -calc_pid(&pid_dt35_left,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - PI / 2) < 0.05)   //车向左转90度
	{
		 world_speed->vx = calc_pid(&pid_dt35_left,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	else if(fabsf(current_angle - PI) < 0.05)       //车向后
	{ 
		 world_speed->vy = calc_pid(&pid_dt35_left,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - (-PI / 2)) < 0.05)  //车向右转90度
	{
		 world_speed->vx = -calc_pid(&pid_dt35_left,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	else if(fabsf(current_angle - (-PI)) < 0.05)    //车向后
	{
		 world_speed->vy = calc_pid(&pid_dt35_left,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
}

void Located_By_Right_DT35(DT35_Move *dt35,Speed *world_speed,float current_angle)
{
	
	if(current_angle > PI)
	{
		current_angle -= 2 * PI;
	}
	else if(current_angle < -PI)
	{
		current_angle += 2 * PI;
	}
	
	if(fabsf(current_angle - 0) < 0.05){      //车身朝前
		world_speed->vy = calc_pid(&pid_dt35_right,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - PI / 2) < 0.05)   //车向左转90度
	{
		 world_speed->vx = -calc_pid(&pid_dt35_right,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	else if(fabsf(current_angle - PI) < 0.05)       //车向后
	{
		 world_speed->vy = -calc_pid(&pid_dt35_right,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - (-PI / 2)) < 0.05)  //车向右转90度
	{
		 world_speed->vx = calc_pid(&pid_dt35_right,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	else if(fabsf(current_angle - (-PI)) < 0.05)    //车向后
	{
		 world_speed->vy = -calc_pid(&pid_dt35_right,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
}

void Located_By_Back_DT35(DT35_Move *dt35,Speed *world_speed,float current_angle)
{
	if(current_angle > PI)
	{
		current_angle -= 2 * PI;
	}
	else if(current_angle < -PI)
	{
		current_angle += 2 * PI;
	}
	
	if(fabsf(current_angle - 0) < 0.05){        //车身朝前
		world_speed->vx = calc_pid(&pid_dt35_back,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - PI / 2) < 0.05)   //车向左转90度
	{
		 world_speed->vy = calc_pid(&pid_dt35_back,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - PI) < 0.05)       //车向后
	{
		 world_speed->vx = -calc_pid(&pid_dt35_back,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - (-PI / 2)) < 0.05)   //车向右转90度
	{
		 world_speed->vy = -calc_pid(&pid_dt35_back,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - (-PI)) < 0.05)     //车向后
	{
		 world_speed->vx = -calc_pid(&pid_dt35_back,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
}

void Located_By_Front_DT35(DT35_Move *dt35,Speed *world_speed,float current_angle)
{
	if(current_angle > PI)
	{
		current_angle -= 2 * PI;
	}
	else if(current_angle < -PI)
	{
		current_angle += 2 * PI;
	}
	
	if(fabsf(current_angle - 0) < 0.05){        //车身朝前
		world_speed->vx = -calc_pid(&pid_dt35_front,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - PI / 2) < 0.05)   //车向左转90度
	{
		 world_speed->vy = -calc_pid(&pid_dt35_front,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - PI) < 0.05)       //车向后
	{
		 world_speed->vx = calc_pid(&pid_dt35_front,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - (-PI / 2)) < 0.05)   //车向右转90度
	{
		 world_speed->vy = calc_pid(&pid_dt35_front,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
	
	else if(fabsf(current_angle - (-PI)) < 0.05)     //车向后
	{
		 world_speed->vx = calc_pid(&pid_dt35_front,dt35->actual_dt35_distance,dt35->target_dt35_distance) / 1000.0f;
	}
}

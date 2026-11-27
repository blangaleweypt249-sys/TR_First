#include "init_task.h"
#include "mymain.h"
#include "Up_control.h"
#include "Leg.h"
#include "tim.h"
#include "usart.h"

volatile uint8_t boot_done;

//外设：电机结构体、三路 CAN、串口、flash
static void Periph_Init(void)
{
    Vesc_Init(&vesc_motor[0], &hfdcan1, 52);
    Vesc_Init(&vesc_motor[1], &hfdcan1, 51);
    Vesc_Init(&vesc_motor[2], &hfdcan1, 53);
    Vesc_Init(&vesc_motor[3], &hfdcan1, 54);

    // 舵初始化，4块小板各带1个2006
    DJI_Init(&helm_motor[0], &hfdcan1, DJI_M2006, 1, 1);
    DJI_Init(&helm_motor[1], &hfdcan1, DJI_M2006, 2, 1);
    DJI_Init(&helm_motor[2], &hfdcan1, DJI_M2006, 3, 1);
    DJI_Init(&helm_motor[3], &hfdcan1, DJI_M2006, 4, 1);

    Leg_Init();

    pid_init(&pid_CarAnale, 0.0f, 8.5f, 0.00f, 0.0f, 5.0f, 0.0f, 0.01f);

    W25Q128_Init(&w25q128, &hspi6, SPI6_NSS_GPIO_Port, SPI6_NSS_Pin);

    FDCAN1_Init();
    FDCAN2_Init();
    FDCAN3_Init();

    Leg_Clear();

    //三路串口：遥控、DT35、IMU
    Bsp_Uart_Init();
}

//遥控链路：上电把 WiFi 配成 AP + TCP server，收到 'O' 算通
static void Wifi_Init(void)
{
    const char ap_cmd[] = "AT+WLSTARTAP=ssid,UPRE_WIFI,ch,6,sec,wpa2,pw,12345678\r\n";
    const char server_cmd[] = "AT+SKTSERVER=0,1,,6060,1\r\n";
    for (uint8_t retry = 0; retry < 3; retry++)
    {
        wifi_ok = 0;
        HAL_UART_Transmit_DMA(&huart9, (uint8_t *)ap_cmd, sizeof(ap_cmd) - 1U);
        for (uint8_t wait = 0; wait < 100; wait++)
        {
            if (wifi_ok)
                break;
            osDelay(10);
        }
        if (wifi_ok)
            break;
    }

    for (uint8_t retry = 0; retry < 3; retry++)
    {
        wifi_ok = 0;
        HAL_UART_Transmit_DMA(&huart9, (uint8_t *)server_cmd, sizeof(server_cmd) - 1U);
        for (uint8_t wait = 0; wait < 100; wait++)
        {
            if (wifi_ok)
                break;
            osDelay(10);
        }
        if (wifi_ok)
            break;
    }
}

//初始化任务
void InitTask(void *argument)
{
    (void)argument;

    Periph_Init();

    osDelay(2000);//等电机上电完成，冷启动时电机比主控晚醒

    Up_Init();//机构零点、清错、使能

    Helm_Calibration();//舵轮回零，找光电门

    HAL_TIM_Base_Start_IT(&htim7);//5010 200ms 读一次位置

    boot_done = 1U;//电机和机构都好了，其余任务可以跑，WiFi 在后台继续配

    Wifi_Init();

    osThreadExit();
}

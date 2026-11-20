#include "mymain.h"
#include "usart.h"

/*-----------------全局变量--------------*/
uint8_t receive_imu[7];
uint8_t receive_dt35[19];
float last_target_speed[4] = {0.0f};

/*-----------------chassis相关--------------*/
Speed world_speed;
Speed body_speed;
Helm_chassis helm_chassis;
float speed_k = 0.0025f; // 最大2.5m/s
float speed_w = 2.0f;

float up_distance = 0;


uint8_t shangxipan;
// uint8_t rx_data[8]; //vesc电机反馈
// FDCAN_RxHeaderTypeDef rx_header;

void ChassisTask(void *argument)
{
    Helm_Calibration();

    for (;;)
    {
        // 测试舵轮电机
        //(void)Test_HelmMotor();
        Climb_Leg(up_distance);
        if (front_flag == 1)
        {
            Leg_Front(0);//前腿收回
        }
        Key_Detect();

        //锁车或者与遥控失联(PA6锁车)
        if (handle_lock)
        {
            world_speed.vx = 0.0f;
            world_speed.vy = 0.0f;
            world_speed.vw = 0.0f;
        }
        else
        {
            world_speed.vy = handle_value[1] * speed_k;
            world_speed.vx = handle_value[0] * speed_k;
        }
        car_angle.actual_angle = Imu_angle;
        car_angle_key();
        if (osMessageQueueGet(imuQueueHandle, receive_imu, NULL, 0U) == osOK)
        {
            Imu_Analysis(receive_imu);
        }
        if (osMessageQueueGet(dt35QueueHandle, receive_dt35, NULL, 0U) == osOK)
        {
            DT35_Analysis(receive_dt35);
        }
        World_to_body(&world_speed, &body_speed, Imu_angle);
        car_angle_maintain(&body_speed, &car_angle);
        helm_calculate(&body_speed, &helm_chassis);
        osSemaphoreRelease(ChassisCalcHandle);
        osDelay(1);
    }
}

void Chassis_canTask(void *argument)
{
    bool drive_started = false;

    // 等待底盘就绪信号
    if (osSemaphoreAcquire(ChassisReadyHandle, osWaitForever) != osOK)
    {
        return;
    }

    for (;;)
    {
        // 等待底盘计算信号
        if (osSemaphoreAcquire(ChassisCalcHandle, osWaitForever) != osOK)
        {
            continue;
        }
        // 舵轮电机相关can发送
        float set_angle[4];
        float helm_speed[4];
        for (uint8_t i = 0; i < 4; i++)
        {
            set_angle[i] = helm_chassis.set_angle[i];
            helm_speed[i] = helm_chassis.helm_speed[i];
        }
        for (uint8_t i = 0; i < 4; i++)
        {
            helm_motor[i].angle_cos = wrap_angle(helm_motor[i].Feedback_angle);
            helm_motor[i].speed_cosk = CosKspeed(set_angle[i], helm_motor[i].angle_cos);
            DJI_Position(&helm_motor[i], set_angle[i]);
        }
        for (uint8_t i = 0; i < 4; i++)
        {
            //float planned_speed = speed_planning(helm_speed[i], &last_target_speed[i], 300.0f, 1500.0f);
            vesc_set_rpm(&vesc_motor[i], helm_speed[i]* helm_motor[i].speed_cosk);
        }

        osDelay(1);
    }
}

void My_init(void)
{
    // 轮初始化
    Vesc_Init(&vesc_motor[0], &hfdcan1, 52);
    Vesc_Init(&vesc_motor[1], &hfdcan1, 51);
    Vesc_Init(&vesc_motor[2], &hfdcan1, 53);
    Vesc_Init(&vesc_motor[3], &hfdcan1, 54);

    // 舵初始化，4块小板各带1个2006
    DJI_Init(&helm_motor[0], &hfdcan1, DJI_M2006, 1, 1);
    DJI_Init(&helm_motor[1], &hfdcan1, DJI_M2006, 2, 1);
    DJI_Init(&helm_motor[2], &hfdcan1, DJI_M2006, 3, 1);
    DJI_Init(&helm_motor[3], &hfdcan1, DJI_M2006, 4, 1);

    // 后腿初始化
    Leg_Init();

    // 车身保持pid初始化
    pid_init(&pid_CarAnale, 0.0f, 6.0f, 0.020f, 0.08f, 4.0f, 1.5f, 0.02f);

    W25Q128_Init(&w25q128, &hspi6, SPI6_NSS_GPIO_Port, SPI6_NSS_Pin);

    FDCAN1_Init();
    FDCAN2_Init();
    FDCAN3_Init();

    Leg_Clear();

    HAL_UARTEx_ReceiveToIdle_DMA(&huart9, receive_handle, sizeof(receive_handle));
    __HAL_DMA_DISABLE_IT(huart9.hdmarx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, receive_dt35, sizeof(receive_dt35));
    __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, receive_imu, sizeof(receive_imu));
    __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
}

void Test_HelmMotor(void)
{
    // for (uint8_t i = 0; i < 4; i++) // 测试用
    // {
    vesc_set_rpm(&vesc_motor[2], 400.0f);
    // }
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     DJI_Speed(&helm_motor[3], 5.0f);
    // }
    // for (uint8_t i = 0; i < 4; i++)
    // {
    //     DJI_Position(&helm_motor[1], 0.0f);
    // }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

    if (huart->Instance == UART9)
    {
        Handle_RxFeed(receive_handle, Size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart9, receive_handle, sizeof(receive_handle));
        __HAL_DMA_DISABLE_IT(huart9.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == UART4)
    {
        if (Size == sizeof(receive_dt35))
        {
            osMessageQueuePut(dt35QueueHandle, receive_dt35, 0U, 0U);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, receive_dt35, sizeof(receive_dt35));
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == UART7)
    {
        if (Size == sizeof(receive_imu))
        {
            osMessageQueuePut(imuQueueHandle, receive_imu, 0U, 0U);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, receive_imu, sizeof(receive_imu));
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART9)
    {
        __HAL_UART_CLEAR_OREFLAG(&huart9);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart9, receive_handle, sizeof(receive_handle));
        __HAL_DMA_DISABLE_IT(huart9.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, receive_dt35, sizeof(receive_dt35));
        __HAL_DMA_DISABLE_IT(huart4.hdmarx, DMA_IT_HT);
    }

    if (huart->Instance == UART7)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart7, receive_imu, sizeof(receive_imu));
        __HAL_DMA_DISABLE_IT(huart7.hdmarx, DMA_IT_HT);
    }
}

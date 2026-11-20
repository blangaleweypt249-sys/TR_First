#include "mymain.h"
#include "WpLib.h"

static uint8_t imu_rx[8];
// 环形缓冲区
static RingBuffer_t imu_ring_buffer;
static uint8_t imu_ring_storage[128];

static uint8_t clear_rx[4];

bool Check_State(int *num, int timeout);

void ImuCalcTask(void *argument)
{
    uint8_t frame[8];
    bool has_new_angle;

    (void)argument;
    uint32_t last_tick_pa2 = 0; // 记录PA2上一次翻转的时间
    for (;;)
    {
        if (osSemaphoreAcquire(RxsuccessHandle, osWaitForever) == osOK)
        {
            has_new_angle = false;
            if (osSemaphoreAcquire(ClearYawHandle, 0U) == osOK)
            {
                Imu_ClearYaw();
                has_new_angle = true;
            }
            while (FindTrueData(&imu_ring_buffer, frame) != 0U)
            {
                if (Imu_ReadData(frame))
                {
                    // 解析正常闪烁
                    if (osKernelGetTickCount() - last_tick_pa2 >= 500)
                    {
                        //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
                        last_tick_pa2 = osKernelGetTickCount();
                    }
                    if (Imu_calibration()) // 采样校准
                    {
                        Imu_Calc();
                        has_new_angle = true;
                    }
                }
            }
            if (has_new_angle)
            {
                osSemaphoreRelease(CalcOKHandle);
            }
        }
        osDelay(1);
    }
}

void ImuTransTask(void *argument)
{
    (void)argument;
    uint32_t last_tick_pa1 = 0; // 记录PA1上一次翻转的时间
    for (;;)
    {
        if (osSemaphoreAcquire(CalcOKHandle, osWaitForever) == osOK)
        {
            SendRad();
            // 发送正常闪烁
            if (osKernelGetTickCount() - last_tick_pa1 >= 500)
            {
                //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);
                last_tick_pa1 = osKernelGetTickCount();
            }
        }
        osDelay(1);
    }
}

void StateCheckTask(void *argument)
{
    bool state;

    (void)argument;
    uint32_t last_tick_pa0 = 0; // 记录PA0上一次翻转的时间
    for (;;)
    {
        // 1表示离线 0表示在线
        state = Check_State(imu_100A.num, 10);
        if (state)
        {
            // 离线则熄灭
            //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
        }
        else
        {
            // 在线则闪烁
            if (osKernelGetTickCount() - last_tick_pa0 >= 500)
            {
                //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
                last_tick_pa0 = osKernelGetTickCount();
            }
        }
        osDelay(5);
    }
}

void Imu_Init(void)
{
    // 初始化环形缓冲区
    RingBuffer_Init(&imu_ring_buffer, imu_ring_storage, sizeof(imu_ring_storage));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, imu_rx, sizeof(imu_rx));
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, clear_rx, sizeof(clear_rx));
    __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
}

bool Check_State(int *num, int timeout)
{
    if (num[0] == num[1])
        num[2]++;
    else
    {
        num[2]--;
        if (num[2] < 0)
            num[2] = 0;
    }

    num[1] = num[0];

    if (num[2] >= timeout)
    {
        num[2] = timeout - 1;
        return true;
    }

    return false;
}

bool Imu_calibration(void)
{
    static bool started;
    static bool finished;
    static uint32_t start_tick;
    static uint32_t sample_count;
    static float speed_sum;
    static float speed_min;
    static float speed_max;
    if (finished)
    {
        return true;
    }

    if (!started)
    {
        started = true;
        start_tick = osKernelGetTickCount();
        speed_sum = 0.0f;
        sample_count = 0U;
        speed_min = 100.0f;
        speed_max = -100.0f;
    }

    speed_sum += imu_100A.gyro_speed;
    sample_count++;
    if (imu_100A.gyro_speed < speed_min)
        speed_min = imu_100A.gyro_speed;
    if (imu_100A.gyro_speed > speed_max)
        speed_max = imu_100A.gyro_speed;
    if ((osKernelGetTickCount() - start_tick) < 3000U)
    {
        return false;
    }

    if ((speed_max - speed_min) > 0.5f) //标定期间动了，重来
    {
        started = false;
        return false;
    }

    imu_100A.basic_speed = speed_sum / (float)sample_count;
    finished = true;
    Imu_ClearYaw();
    return true;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        if (Size > 0)
        {
            (void)WriteBuffer(&imu_ring_buffer, imu_rx, Size);
            osSemaphoreRelease(RxsuccessHandle);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, imu_rx, sizeof(imu_rx));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == USART3)
    {
        // 接收主控清零
        if ((Size == sizeof(clear_rx)) &&
            (clear_rx[0] == 0xA5U) && (clear_rx[1] == 0x5AU) &&
            (clear_rx[2] == 0x01U) && (clear_rx[3] == 0xFEU))
        {
            osSemaphoreRelease(ClearYawHandle);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, clear_rx, sizeof(clear_rx));
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, imu_rx, sizeof(imu_rx));
        __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, clear_rx, sizeof(clear_rx));
        __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
    }
}

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for Imu_calc */
osThreadId_t Imu_calcHandle;
const osThreadAttr_t Imu_calc_attributes = {
  .name = "Imu_calc",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Imu_trans */
osThreadId_t Imu_transHandle;
const osThreadAttr_t Imu_trans_attributes = {
  .name = "Imu_trans",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for State_check */
osThreadId_t State_checkHandle;
const osThreadAttr_t State_check_attributes = {
  .name = "State_check",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal7,
};
/* Definitions for ImuQueue */
osMessageQueueId_t ImuQueueHandle;
const osMessageQueueAttr_t ImuQueue_attributes = {
  .name = "ImuQueue"
};
/* Definitions for ClearYaw */
osSemaphoreId_t ClearYawHandle;
const osSemaphoreAttr_t ClearYaw_attributes = {
  .name = "ClearYaw"
};
/* Definitions for CalcOK */
osSemaphoreId_t CalcOKHandle;
const osSemaphoreAttr_t CalcOK_attributes = {
  .name = "CalcOK"
};
/* Definitions for Rxsuccess */
osSemaphoreId_t RxsuccessHandle;
const osSemaphoreAttr_t Rxsuccess_attributes = {
  .name = "Rxsuccess"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ImuCalcTask(void *argument);
void ImuTransTask(void *argument);
void StateCheckTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of ClearYaw */
  ClearYawHandle = osSemaphoreNew(1, 0, &ClearYaw_attributes);

  /* creation of CalcOK */
  CalcOKHandle = osSemaphoreNew(1, 0, &CalcOK_attributes);

  /* creation of Rxsuccess */
  RxsuccessHandle = osSemaphoreNew(1, 0, &Rxsuccess_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ImuQueue */
  ImuQueueHandle = osMessageQueueNew (5, 7, &ImuQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Imu_calc */
  Imu_calcHandle = osThreadNew(ImuCalcTask, NULL, &Imu_calc_attributes);

  /* creation of Imu_trans */
  Imu_transHandle = osThreadNew(ImuTransTask, NULL, &Imu_trans_attributes);

  /* creation of State_check */
  State_checkHandle = osThreadNew(StateCheckTask, NULL, &State_check_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ImuCalcTask */
/**
  * @brief  Function implementing the Imu_calc thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ImuCalcTask */
__weak void ImuCalcTask(void *argument)
{
  /* USER CODE BEGIN ImuCalcTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ImuCalcTask */
}

/* USER CODE BEGIN Header_ImuTransTask */
/**
* @brief Function implementing the Imu_trans thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImuTransTask */
__weak void ImuTransTask(void *argument)
{
  /* USER CODE BEGIN ImuTransTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ImuTransTask */
}

/* USER CODE BEGIN Header_StateCheckTask */
/**
* @brief Function implementing the State_check thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StateCheckTask */
__weak void StateCheckTask(void *argument)
{
  /* USER CODE BEGIN StateCheckTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StateCheckTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


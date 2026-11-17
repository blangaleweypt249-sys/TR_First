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
#include "mymain.h"

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
/* Definitions for myScanTask */
osThreadId_t myScanTaskHandle;
const osThreadAttr_t myScanTask_attributes = {
  .name = "myScanTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myTransmitTask */
osThreadId_t myTransmitTaskHandle;
const osThreadAttr_t myTransmitTask_attributes = {
  .name = "myTransmitTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myWifiTask */
osThreadId_t myWifiTaskHandle;
const osThreadAttr_t myWifiTask_attributes = {
  .name = "myWifiTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ScanToTrans */
osMessageQueueId_t ScanToTransHandle;
const osMessageQueueAttr_t ScanToTrans_attributes = {
  .name = "ScanToTrans"
};
/* Definitions for myWIFIsuc */
osSemaphoreId_t myWIFIsucHandle;
const osSemaphoreAttr_t myWIFIsuc_attributes = {
  .name = "myWIFIsuc"
};
/* Definitions for myPacksuc */
osSemaphoreId_t myPacksucHandle;
const osSemaphoreAttr_t myPacksuc_attributes = {
  .name = "myPacksuc"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void ScanTask(void *argument);
void TransmitTask(void *argument);
void WifiTask(void *argument);

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
  /* creation of myWIFIsuc */
  myWIFIsucHandle = osSemaphoreNew(1, 0, &myWIFIsuc_attributes);

  /* creation of myPacksuc */
  myPacksucHandle = osSemaphoreNew(1, 0, &myPacksuc_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of ScanToTrans */
  ScanToTransHandle = osMessageQueueNew (1, sizeof(remote_data), &ScanToTrans_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of myScanTask */
  myScanTaskHandle = osThreadNew(ScanTask, NULL, &myScanTask_attributes);

  /* creation of myTransmitTask */
  myTransmitTaskHandle = osThreadNew(TransmitTask, NULL, &myTransmitTask_attributes);

  /* creation of myWifiTask */
  myWifiTaskHandle = osThreadNew(WifiTask, NULL, &myWifiTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_ScanTask */
/**
  * @brief  Function implementing the myScanTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ScanTask */
__weak void ScanTask(void *argument)
{
  /* USER CODE BEGIN ScanTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ScanTask */
}

/* USER CODE BEGIN Header_TransmitTask */
/**
* @brief Function implementing the myTransmitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TransmitTask */
__weak void TransmitTask(void *argument)
{
  /* USER CODE BEGIN TransmitTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TransmitTask */
}

/* USER CODE BEGIN Header_WifiTask */
/**
* @brief Function implementing the myWifiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_WifiTask */
__weak void WifiTask(void *argument)
{
  /* USER CODE BEGIN WifiTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END WifiTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


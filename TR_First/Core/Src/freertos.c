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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

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
/* Definitions for myInitTask */
osThreadId_t myInitTaskHandle;
const osThreadAttr_t myInitTask_attributes = {
  .name = "myInitTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for myChassisTask */
osThreadId_t myChassisTaskHandle;
const osThreadAttr_t myChassisTask_attributes = {
  .name = "myChassisTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for myHandleTask */
osThreadId_t myHandleTaskHandle;
const osThreadAttr_t myHandleTask_attributes = {
  .name = "myHandleTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myClimbTask */
osThreadId_t myClimbTaskHandle;
const osThreadAttr_t myClimbTask_attributes = {
  .name = "myClimbTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for myArmTask */
osThreadId_t myArmTaskHandle;
const osThreadAttr_t myArmTask_attributes = {
  .name = "myArmTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myStatusTask */
osThreadId_t myStatusTaskHandle;
const osThreadAttr_t myStatusTask_attributes = {
  .name = "myStatusTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for myLocationTask */
osThreadId_t myLocationTaskHandle;
const osThreadAttr_t myLocationTask_attributes = {
  .name = "myLocationTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for myNetTask */
osThreadId_t myNetTaskHandle;
const osThreadAttr_t myNetTask_attributes = {
  .name = "myNetTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for handleQueue */
osMessageQueueId_t handleQueueHandle;
const osMessageQueueAttr_t handleQueue_attributes = {
  .name = "handleQueue"
};
/* Definitions for imuQueue */
osMessageQueueId_t imuQueueHandle;
const osMessageQueueAttr_t imuQueue_attributes = {
  .name = "imuQueue"
};
/* Definitions for dt35Queue */
osMessageQueueId_t dt35QueueHandle;
const osMessageQueueAttr_t dt35Queue_attributes = {
  .name = "dt35Queue"
};
/* Definitions for Reboot_motor */
osSemaphoreId_t Reboot_motorHandle;
const osSemaphoreAttr_t Reboot_motor_attributes = {
  .name = "Reboot_motor"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void InitTask(void *argument);
void ChassisTask(void *argument);
void HandleTask(void *argument);
void ClimbTask(void *argument);
void ArmTask(void *argument);
void StatusTask(void *argument);
void LocationTask(void *argument);
void NetTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
}
/* USER CODE END 5 */

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
  /* creation of Reboot_motor */
  Reboot_motorHandle = osSemaphoreNew(1, 0, &Reboot_motor_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of handleQueue */
  handleQueueHandle = osMessageQueueNew (1, sizeof(receive_handle), &handleQueue_attributes);

  /* creation of imuQueue */
  imuQueueHandle = osMessageQueueNew (1, sizeof(receive_imu), &imuQueue_attributes);

  /* creation of dt35Queue */
  dt35QueueHandle = osMessageQueueNew (1, 19, &dt35Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of myInitTask */
  myInitTaskHandle = osThreadNew(InitTask, NULL, &myInitTask_attributes);

  /* creation of myChassisTask */
  myChassisTaskHandle = osThreadNew(ChassisTask, NULL, &myChassisTask_attributes);

  /* creation of myHandleTask */
  myHandleTaskHandle = osThreadNew(HandleTask, NULL, &myHandleTask_attributes);

  /* creation of myClimbTask */
  myClimbTaskHandle = osThreadNew(ClimbTask, NULL, &myClimbTask_attributes);

  /* creation of myArmTask */
  myArmTaskHandle = osThreadNew(ArmTask, NULL, &myArmTask_attributes);

  /* creation of myStatusTask */
  myStatusTaskHandle = osThreadNew(StatusTask, NULL, &myStatusTask_attributes);

  /* creation of myLocationTask */
  myLocationTaskHandle = osThreadNew(LocationTask, NULL, &myLocationTask_attributes);

  /* creation of myNetTask */
  myNetTaskHandle = osThreadNew(NetTask, NULL, &myNetTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_InitTask */
/**
  * @brief  Function implementing the myInitTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_InitTask */
__weak void InitTask(void *argument)
{
  /* USER CODE BEGIN InitTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END InitTask */
}

/* USER CODE BEGIN Header_ChassisTask */
/**
  * @brief  Function implementing the myChassisTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ChassisTask */
__weak void ChassisTask(void *argument)
{
  /* USER CODE BEGIN ChassisTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChassisTask */
}

/* USER CODE BEGIN Header_HandleTask */
/**
* @brief Function implementing the myHandleTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HandleTask */
__weak void HandleTask(void *argument)
{
  /* USER CODE BEGIN HandleTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END HandleTask */
}

/* USER CODE BEGIN Header_ClimbTask */
/**
* @brief Function implementing the myClimbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ClimbTask */
__weak void ClimbTask(void *argument)
{
  /* USER CODE BEGIN ClimbTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ClimbTask */
}

/* USER CODE BEGIN Header_ArmTask */
/**
* @brief Function implementing the myArmTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ArmTask */
__weak void ArmTask(void *argument)
{
  /* USER CODE BEGIN ArmTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ArmTask */
}

/* USER CODE BEGIN Header_StatusTask */
/**
* @brief Function implementing the myStatusTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StatusTask */
__weak void StatusTask(void *argument)
{
  /* USER CODE BEGIN StatusTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StatusTask */
}

/* USER CODE BEGIN Header_LocationTask */
/**
* @brief Function implementing the myLocationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LocationTask */
__weak void LocationTask(void *argument)
{
  /* USER CODE BEGIN LocationTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LocationTask */
}

/* USER CODE BEGIN Header_NetTask */
/**
* @brief Function implementing the myNetTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_NetTask */
__weak void NetTask(void *argument)
{
  /* USER CODE BEGIN NetTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END NetTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


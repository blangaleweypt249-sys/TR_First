/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI6_NSS_Pin GPIO_PIN_4
#define SPI6_NSS_GPIO_Port GPIOA
#define up_air_Pin GPIO_PIN_8
#define up_air_GPIO_Port GPIOC
#define left_air_Pin GPIO_PIN_9
#define left_air_GPIO_Port GPIOC
#define right_air_Pin GPIO_PIN_8
#define right_air_GPIO_Port GPIOA
#define up_jia_Pin GPIO_PIN_9
#define up_jia_GPIO_Port GPIOA
#define right_realair_Pin GPIO_PIN_10
#define right_realair_GPIO_Port GPIOC
#define left_realair_Pin GPIO_PIN_11
#define left_realair_GPIO_Port GPIOC
#define left_jiazhua_Pin GPIO_PIN_0
#define left_jiazhua_GPIO_Port GPIOD
#define four_helm_Pin GPIO_PIN_1
#define four_helm_GPIO_Port GPIOD
#define three_helm_Pin GPIO_PIN_3
#define three_helm_GPIO_Port GPIOD
#define one_helm_Pin GPIO_PIN_4
#define one_helm_GPIO_Port GPIOD
#define two_helm_Pin GPIO_PIN_5
#define two_helm_GPIO_Port GPIOD
#define up_realair_Pin GPIO_PIN_6
#define up_realair_GPIO_Port GPIOD
#define med_leg_Pin GPIO_PIN_7
#define med_leg_GPIO_Port GPIOD
#define right_jia_Pin GPIO_PIN_6
#define right_jia_GPIO_Port GPIOB
#define front_leg_Pin GPIO_PIN_7
#define front_leg_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

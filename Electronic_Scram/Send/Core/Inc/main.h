/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* 发送板引脚 ----------------------------------------------------------------*/
#define SW_Pin              GPIO_PIN_0      // 急停开关
#define SW_GPIO_Port        GPIOA
#define LORA_AUX_Pin        GPIO_PIN_8      // 模块 AUX，高电平表示可以收串口数据
#define LORA_AUX_GPIO_Port  GPIOA
#define LORA_M0_Pin         GPIO_PIN_14     // 模块 MD0
#define LORA_M0_GPIO_Port   GPIOB
#define LORA_M1_Pin         GPIO_PIN_15     // 模块 MD1
#define LORA_M1_GPIO_Port   GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

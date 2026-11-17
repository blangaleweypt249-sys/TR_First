#ifndef __LORA_CONFIG_H__
#define __LORA_CONFIG_H__

// 无线模块配置：当前用 A39-T400A21D1a
// A39 正常/透传模式：MD0=1、MD1=0，串口 115200 8N1
// 换 E32/E22 时改成 M0=RESET、M1=RESET
#define LORA_UART_BAUD_RATE  115200U
#define LORA_M0_LEVEL        GPIO_PIN_SET      // A39: MD0=1
#define LORA_M1_LEVEL        GPIO_PIN_RESET    // A39: MD1=0
#define LORA_MODE_SETTLE_MS  20U

#endif

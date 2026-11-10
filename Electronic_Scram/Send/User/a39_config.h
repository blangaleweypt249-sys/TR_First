#ifndef A39_CONFIG_H
#define A39_CONFIG_H
/* A39-T400A21D1a: externally provision both modules for transparent UART
 * and matching RF settings. These values retain project configuration,
 * not factory defaults. Check mode levels with the supplied manual. */
#define A39_UART_BAUD_RATE 115200U
#define A39_MD0_LEVEL GPIO_PIN_SET
#define A39_MD1_LEVEL GPIO_PIN_RESET
#define A39_MODE_SETTLE_MS 10U
#endif

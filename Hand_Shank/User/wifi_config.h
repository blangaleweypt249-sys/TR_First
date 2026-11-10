#ifndef HAND_WIFI_CONFIG_H
#define HAND_WIFI_CONFIG_H
/* Preconfigured UART-to-Wi-Fi transparent bridge:
 * USART6 PC6(TX) -> module RX, PC7(RX) <- module TX.
 * Set SSID/security, peer address/port and binary transparent forwarding
 * on the module before use. No model-specific AT commands are sent. */
#define WIFI_UART_HANDLE huart6
#define WIFI_UART_BAUD_RATE 115200U
#define WIFI_TX_PERIOD_MS 20U
#endif

#include "WifiConfig.h"
#include "usart.h"
#include "mymain.h"
#include <string.h>

volatile uint8_t wifi_ready;

static uint16_t Wifi_ReadResponse(uint8_t *buffer, uint16_t capacity,
                                  uint32_t timeout)
{
  uint16_t length = 0U;
  uint8_t ch;
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < timeout)
  {
    if (HAL_UART_Receive(&huart6, &ch, 1U, 50U) == HAL_OK)
    {
      if (length + 1U < capacity)
      {
        buffer[length++] = ch;
        buffer[length] = '\0';
        if (strstr((char *)buffer, "wifi got ip") != NULL)
          break;
      }
    }
  }

  if (capacity > 0U)
    buffer[length < capacity ? length : capacity - 1U] = '\0';
  return length;
}

static void Wifi_SendCommand(const char *command, uint8_t *response,
                             uint16_t response_size, uint32_t timeout)
{
  HAL_UART_Transmit(&huart6, (uint8_t *)command,
                    (uint16_t)strlen(command), 1000U);
  Wifi_ReadResponse(response, response_size, timeout);
}

static uint8_t Wifi_Connect(void)
{
  const char conn_cmd[] = "AT+WLCONN=ssid,UPRE_WIFI,pw,12345678\r\n";
  const char client_cmd[] = "AT+SKTCLIENT=0,1,,192.168.43.1,6060,,1\r\n";
  const char delete_cmd[] = "AT+SKTDEL=0\r\n";
  const char state_cmd[] = "AT+SKTSTATE\r\n";
  uint8_t response[256];
  const char *socket;

  Wifi_SendCommand(delete_cmd, response, sizeof(response), 500U);
  HAL_UART_Transmit(&huart6, (uint8_t *)conn_cmd,
                    sizeof(conn_cmd) - 1U, 1000U);
  Wifi_ReadResponse(response, sizeof(response), 10000U);
  if (strstr((char *)response, "wifi got ip") == NULL)
    return 0U;
  osDelay(2000U);

  Wifi_SendCommand(client_cmd, response, sizeof(response), 2000U);
  if (strstr((char *)response, "OK") == NULL)
    return 0U;

  Wifi_SendCommand(state_cmd, response, sizeof(response), 1000U);
  socket = strstr((char *)response, "socket_fd:");
  return (socket != NULL && socket[10] != '-');
}

void WifiTask(void *argument)
{
  (void)argument;
  uint8_t had_connection = 0U;

  for (;;)
  {
    wifi_ready = 0U;
    if (Wifi_Connect())
    {
      wifi_ready = 1U;
      had_connection = 1U;
      Beep_SetAlarm(0U);
      Beep_Request(5U);
      while (wifi_ready)
        osDelay(100U);
    }
    else
    {
      if (had_connection)
        Beep_SetAlarm(1U);
      osDelay(1000U);
    }
  }
}

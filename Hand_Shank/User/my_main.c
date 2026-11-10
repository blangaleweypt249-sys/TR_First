#include "my_main.h"
#include "remote.h"

/* GPIO, ADC1 and USART6 are initialized by main.c. */
void My_Init(void)
{
    Remote_Init();
}

/* Send the current controls through the Wi-Fi UART bridge. */
void My_Loop(void)
{
    Remote_Loop();
}

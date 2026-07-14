#if PLATFORM_OPL1000

#include "hal_dbg_uart.h"
#include "../hal_uart.h"

void HAL_UART_SendByte(byte b)
{
    Hal_DbgUart_DataSend((uint32_t)b);
}

int HAL_UART_Init(int baud, int parity, bool hwflowc, int txOverride, int rxOverride)
{
    (void)baud;
    (void)parity;
    (void)hwflowc;
    (void)txOverride;
    (void)rxOverride;
    return 0;
}

#endif

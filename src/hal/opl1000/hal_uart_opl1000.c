#if PLATFORM_OPL1000

#include <stdint.h>
#include <stdio.h>

#include "hal_dbg_uart.h"
#include "../hal_uart.h"

static void OpenOPL1000_UART_Rx(uint32_t data)
{
    UART_AppendByteToReceiveRingBuffer((int)(data & 0xffu));
}

void HAL_UART_SendByte(byte b)
{
    if (Hal_DbgUart_DataSend != NULL)
    {
        Hal_DbgUart_DataSend((uint32_t)b);
        return;
    }

    putchar((int)b);
}

int HAL_UART_Init(int baud, int parity, bool hwflowc, int txOverride, int rxOverride)
{
    (void)parity;
    (void)hwflowc;
    (void)txOverride;
    (void)rxOverride;

    if (baud <= 0)
    {
        baud = 115200;
    }

    if (Hal_DbgUart_Init != NULL)
    {
        Hal_DbgUart_Init((uint32_t)baud);
    }

    if (Hal_DbgUart_RxCallBackFuncSet != NULL)
    {
        Hal_DbgUart_RxCallBackFuncSet(OpenOPL1000_UART_Rx);
    }

    if (Hal_DbgUart_RxIntEn != NULL)
    {
        Hal_DbgUart_RxIntEn(1);
    }

    return 0;
}

#endif

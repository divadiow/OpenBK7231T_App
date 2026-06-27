#if PLATFORM_OPL1000

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "../../new_pins.h"
#include "../hal_pins.h"
#include "hal_pin.h"
#include "hal_pin_def.h"
#include "hal_vic.h"

#define OPENOPL1000_GPIO_MAX 24

static uint32_t s_outputMask;

static int OpenOPL1000_PinValid(int index)
{
    return index >= 0 && index < OPENOPL1000_GPIO_MAX;
}

static int OpenOPL1000_PinGpioCapable(int index)
{
    return (index >= 2 && index <= 11) || (index >= 16 && index <= 23);
}

static E_GpioIdx_t OpenOPL1000_PinToGpio(int index)
{
    return (E_GpioIdx_t)index;
}

static const char *OpenOPL1000_GetPinLabel(int index)
{
    switch (index)
    {
    case 0:  return "IO0 (DBG_TX)";
    case 1:  return "IO1 (DBG_RX)";
    case 2:  return "IO2 (AUX2/UART0_TX)";
    case 3:  return "IO3 (AUX3/UART0_RX)";
    case 4:  return "IO4 (AUX4/UART1_TX)";
    case 5:  return "IO5 (AUX5/UART1_RX)";
    case 6:  return "IO6 (AUX6/UART0_TX)";
    case 7:  return "IO7 (AUX7/UART0_RX)";
    case 8:  return "IO8 (UART1_TX/AUX8)";
    case 9:  return "IO9 (UART1_RX/AUX9)";
    case 10: return "IO10 (AUX10/UART0_TX)";
    case 11: return "IO11 (AUX11/UART0_RX)";
    case 12: return "IO12 (SPI0_CS/AUX12)";
    case 13: return "IO13 (SPI0_CLK/AUX13)";
    case 14: return "IO14 (SPI0_IO0/AUX14)";
    case 15: return "IO15 (SPI0_IO1/AUX15)";
    case 16: return "IO16 (SPI0_IO2)";
    case 17: return "IO17 (SPI0_IO3)";
    case 18: return "IO18 (PWM5)";
    case 19: return "IO19 (PWM4)";
    case 20: return "IO20 (PWM3/ICE_M3_DAT)";
    case 21: return "IO21 (PWM2/ICE_M3_CLK)";
    case 22: return "IO22 (PWM1)";
    case 23: return "IO23 (PWM0)";
    default: return "";
    }
}

void HAL_PIN_SetOutputValue(int index, int iVal)
{
    E_GpioLevel_t level;

    if (!OpenOPL1000_PinGpioCapable(index)) return;

    level = iVal ? GPIO_LEVEL_HIGH : GPIO_LEVEL_LOW;
    if ((s_outputMask & (1u << index)) == 0)
    {
        Hal_Pin_ConfigSet((uint8_t)index,
                          iVal ? PIN_TYPE_GPIO_OUTPUT_HIGH : PIN_TYPE_GPIO_OUTPUT_LOW,
                          PIN_DRIVING_FLOAT);
        s_outputMask |= (1u << index);
        return;
    }

    Hal_Vic_GpioOutput(OpenOPL1000_PinToGpio(index), level);
}

int HAL_PIN_ReadDigitalInput(int index)
{
    if (!OpenOPL1000_PinGpioCapable(index)) return 0;
    return Hal_Vic_GpioInput(OpenOPL1000_PinToGpio(index)) == GPIO_LEVEL_HIGH;
}

void HAL_PIN_Setup_Input_Pulldown(int index)
{
    if (!OpenOPL1000_PinGpioCapable(index)) return;
    s_outputMask &= ~(1u << index);
    Hal_Pin_ConfigSet((uint8_t)index, PIN_TYPE_GPIO_INPUT, PIN_DRIVING_LOW);
}

void HAL_PIN_Setup_Input_Pullup(int index)
{
    if (!OpenOPL1000_PinGpioCapable(index)) return;
    s_outputMask &= ~(1u << index);
    Hal_Pin_ConfigSet((uint8_t)index, PIN_TYPE_GPIO_INPUT, PIN_DRIVING_HIGH);
}

void HAL_PIN_Setup_Input(int index)
{
    if (!OpenOPL1000_PinGpioCapable(index)) return;
    s_outputMask &= ~(1u << index);
    Hal_Pin_ConfigSet((uint8_t)index, PIN_TYPE_GPIO_INPUT, PIN_DRIVING_FLOAT);
}

void HAL_PIN_Setup_Output(int index)
{
    if (!OpenOPL1000_PinGpioCapable(index)) return;
    Hal_Pin_ConfigSet((uint8_t)index, PIN_TYPE_GPIO_OUTPUT_LOW, PIN_DRIVING_FLOAT);
    s_outputMask |= (1u << index);
}

void HAL_PIN_PWM_Stop(int index)
{
    (void)index;
}

void HAL_PIN_PWM_Start(int index, int freq)
{
    (void)index;
    (void)freq;
}

void HAL_PIN_PWM_Update(int index, float value)
{
    (void)index;
    (void)value;
}

int HAL_PIN_CanThisPinBePWM(int index)
{
    (void)index;
    return 0;
}

const char *HAL_PIN_GetPinNameAlias(int index)
{
    if (!OpenOPL1000_PinValid(index)) return "";
    return OpenOPL1000_GetPinLabel(index);
}

int HAL_PIN_Find(const char *name)
{
    int pin;
    if (name == 0) return -1;
    if (sscanf(name, "IO%d", &pin) == 1 && OpenOPL1000_PinValid(pin)) return pin;
    if (sscanf(name, "P%d", &pin) == 1 && OpenOPL1000_PinValid(pin)) return pin;
    return -1;
}

void HAL_AttachInterrupt(int pinIndex, OBKInterruptType mode, OBKInterruptHandler function)
{
    (void)pinIndex;
    (void)mode;
    (void)function;
}

void HAL_DetachInterrupt(int pinIndex)
{
    (void)pinIndex;
}

unsigned int HAL_GetGPIOPin(int index)
{
    return OpenOPL1000_PinValid(index) ? (unsigned int)index : 0;
}

#endif

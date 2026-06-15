#if PLATFORM_OPL1000

#include <stdio.h>
#include <string.h>

#include "../../new_pins.h"
#include "../hal_pins.h"

#define OPENOPL1000_GPIO_MAX 24

static unsigned char s_pinState[OPENOPL1000_GPIO_MAX];
static unsigned char s_pinOutput[OPENOPL1000_GPIO_MAX];
static char s_pinAlias[OPENOPL1000_GPIO_MAX][8];

static int OpenOPL1000_PinValid(int index)
{
    return index >= 0 && index < OPENOPL1000_GPIO_MAX;
}

void HAL_PIN_SetOutputValue(int index, int iVal)
{
    if (!OpenOPL1000_PinValid(index)) return;
    s_pinOutput[index] = 1;
    s_pinState[index] = iVal ? 1 : 0;
    printf("[OpenOPL1000] GPIO%d stub output=%d\r\n", index, s_pinState[index]);
}

int HAL_PIN_ReadDigitalInput(int index)
{
    if (!OpenOPL1000_PinValid(index)) return 0;
    return s_pinState[index];
}

void HAL_PIN_Setup_Input_Pulldown(int index)
{
    if (!OpenOPL1000_PinValid(index)) return;
    s_pinOutput[index] = 0;
}

void HAL_PIN_Setup_Input_Pullup(int index)
{
    if (!OpenOPL1000_PinValid(index)) return;
    s_pinOutput[index] = 0;
    s_pinState[index] = 1;
}

void HAL_PIN_Setup_Input(int index)
{
    if (!OpenOPL1000_PinValid(index)) return;
    s_pinOutput[index] = 0;
}

void HAL_PIN_Setup_Output(int index)
{
    if (!OpenOPL1000_PinValid(index)) return;
    s_pinOutput[index] = 1;
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
    snprintf(s_pinAlias[index], sizeof(s_pinAlias[index]), "IO%d", index);
    return s_pinAlias[index];
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

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
static char s_pinAlias[6];

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
    snprintf(s_pinAlias, sizeof(s_pinAlias), "IO%d", index);
    return s_pinAlias;
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

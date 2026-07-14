#include "opl1000_libc.h"

#include <limits.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#ifndef OPENOPL1000_LIBC_HOST_TEST
#include "hal_dbg_uart.h"
#endif

typedef enum
{
    OPENOPL1000_LENGTH_DEFAULT,
    OPENOPL1000_LENGTH_HH,
    OPENOPL1000_LENGTH_H,
    OPENOPL1000_LENGTH_L,
    OPENOPL1000_LENGTH_LL
} OpenOPL1000_Length;

static int OpenOPL1000_IsSpace(char ch)
{
    return ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r' || ch == '\f' || ch == '\v';
}

static int OpenOPL1000_DigitValue(char ch)
{
    if (ch >= '0' && ch <= '9')
    {
        return ch - '0';
    }
    if (ch >= 'a' && ch <= 'f')
    {
        return ch - 'a' + 10;
    }
    if (ch >= 'A' && ch <= 'F')
    {
        return ch - 'A' + 10;
    }
    return -1;
}

static const char *OpenOPL1000_ParseUnsigned(const char *text, int base, int width,
                                              unsigned long long *value, int *digitCount)
{
    const char *p = text;
    unsigned long long result = 0;
    int digits = 0;
    int remaining = width > 0 ? width : INT_MAX;

    if (remaining >= 2 && p[0] == '0' && (p[1] == 'x' || p[1] == 'X') && (base == 0 || base == 16))
    {
        base = 16;
        p += 2;
        remaining -= 2;
    }
    else if (base == 0)
    {
        base = p[0] == '0' ? 8 : 10;
    }

    while (remaining > 0)
    {
        int digit = OpenOPL1000_DigitValue(*p);
        if (digit < 0 || digit >= base)
        {
            break;
        }
        if (result > (ULLONG_MAX - (unsigned int)digit) / (unsigned int)base)
        {
            result = ULLONG_MAX;
        }
        else
        {
            result = result * (unsigned int)base + (unsigned int)digit;
        }
        p++;
        remaining--;
        digits++;
    }

    *value = result;
    *digitCount = digits;
    return p;
}

static double OpenOPL1000_ScalePower10(double value, int exponent)
{
    double scale = 10.0;
    unsigned int power;

    if (exponent < 0)
    {
        power = (unsigned int)(-exponent);
        while (power != 0)
        {
            if (power & 1u)
            {
                value /= scale;
            }
            power >>= 1;
            if (power != 0)
            {
                scale *= scale;
            }
        }
    }
    else
    {
        power = (unsigned int)exponent;
        while (power != 0)
        {
            if (power & 1u)
            {
                value *= scale;
            }
            power >>= 1;
            if (power != 0)
            {
                scale *= scale;
            }
        }
    }
    return value;
}

double OpenOPL1000_ParseDouble(const char *text, char **endptr)
{
    const char *start = text;
    const char *p = text;
    const char *exponentStart;
    double value = 0.0;
    double fractionScale = 1.0;
    int negative = 0;
    int digits = 0;
    int exponent = 0;
    int exponentNegative = 0;
    int exponentDigits = 0;

    while (OpenOPL1000_IsSpace(*p))
    {
        p++;
    }
    if (*p == '+' || *p == '-')
    {
        negative = *p == '-';
        p++;
    }

    while (*p >= '0' && *p <= '9')
    {
        value = value * 10.0 + (double)(*p - '0');
        p++;
        digits++;
    }
    if (*p == '.')
    {
        p++;
        while (*p >= '0' && *p <= '9')
        {
            fractionScale *= 0.1;
            value += (double)(*p - '0') * fractionScale;
            p++;
            digits++;
        }
    }

    if (digits == 0)
    {
        if (endptr != NULL)
        {
            *endptr = (char *)start;
        }
        return 0.0;
    }

    exponentStart = p;
    if (*p == 'e' || *p == 'E')
    {
        p++;
        if (*p == '+' || *p == '-')
        {
            exponentNegative = *p == '-';
            p++;
        }
        while (*p >= '0' && *p <= '9')
        {
            if (exponent < 4096)
            {
                exponent = exponent * 10 + (*p - '0');
                if (exponent > 4096)
                {
                    exponent = 4096;
                }
            }
            p++;
            exponentDigits++;
        }
        if (exponentDigits == 0)
        {
            p = exponentStart;
            exponent = 0;
        }
    }

    if (exponentNegative)
    {
        exponent = -exponent;
    }
    value = OpenOPL1000_ScalePower10(value, exponent);
    if (negative)
    {
        value = -value;
    }
    if (endptr != NULL)
    {
        *endptr = (char *)p;
    }
    return value;
}

static void OpenOPL1000_StoreSigned(va_list *args, OpenOPL1000_Length length, long long value)
{
    switch (length)
    {
        case OPENOPL1000_LENGTH_HH: *va_arg(*args, signed char *) = (signed char)value; break;
        case OPENOPL1000_LENGTH_H:  *va_arg(*args, short *) = (short)value; break;
        case OPENOPL1000_LENGTH_L:  *va_arg(*args, long *) = (long)value; break;
        case OPENOPL1000_LENGTH_LL: *va_arg(*args, long long *) = value; break;
        default:                    *va_arg(*args, int *) = (int)value; break;
    }
}

static void OpenOPL1000_StoreUnsigned(va_list *args, OpenOPL1000_Length length, unsigned long long value)
{
    switch (length)
    {
        case OPENOPL1000_LENGTH_HH: *va_arg(*args, unsigned char *) = (unsigned char)value; break;
        case OPENOPL1000_LENGTH_H:  *va_arg(*args, unsigned short *) = (unsigned short)value; break;
        case OPENOPL1000_LENGTH_L:  *va_arg(*args, unsigned long *) = (unsigned long)value; break;
        case OPENOPL1000_LENGTH_LL: *va_arg(*args, unsigned long long *) = value; break;
        default:                    *va_arg(*args, unsigned int *) = (unsigned int)value; break;
    }
}

/* The linked OpenOPL1000 feature set needs integer/float conversions, strings,
 * characters, field widths, assignment suppression, and hh/h/l/ll lengths.
 * Add new conversions here only when a newly enabled OPL1000 module needs them.
 */
static int OpenOPL1000_VScan(const char *text, const char *format, va_list argsIn)
{
    const char *input = text;
    const char *fmt = format;
    int assigned = 0;
    va_list args;

    va_copy(args, argsIn);
    while (*fmt != '\0')
    {
        int suppress = 0;
        int width = 0;
        OpenOPL1000_Length length = OPENOPL1000_LENGTH_DEFAULT;
        char conversion;

        if (OpenOPL1000_IsSpace(*fmt))
        {
            while (OpenOPL1000_IsSpace(*fmt))
            {
                fmt++;
            }
            while (OpenOPL1000_IsSpace(*input))
            {
                input++;
            }
            continue;
        }
        if (*fmt != '%')
        {
            if (*input != *fmt)
            {
                break;
            }
            input++;
            fmt++;
            continue;
        }

        fmt++;
        if (*fmt == '%')
        {
            if (*input != '%')
            {
                break;
            }
            input++;
            fmt++;
            continue;
        }
        if (*fmt == '*')
        {
            suppress = 1;
            fmt++;
        }
        while (*fmt >= '0' && *fmt <= '9')
        {
            width = width * 10 + (*fmt - '0');
            fmt++;
        }
        if (*fmt == 'h')
        {
            fmt++;
            length = OPENOPL1000_LENGTH_H;
            if (*fmt == 'h')
            {
                fmt++;
                length = OPENOPL1000_LENGTH_HH;
            }
        }
        else if (*fmt == 'l')
        {
            fmt++;
            length = OPENOPL1000_LENGTH_L;
            if (*fmt == 'l')
            {
                fmt++;
                length = OPENOPL1000_LENGTH_LL;
            }
        }

        conversion = *fmt++;
        if (conversion == 'c')
        {
            int count = width > 0 ? width : 1;
            int i;
            if (*input == '\0')
            {
                break;
            }
            if (suppress)
            {
                for (i = 0; i < count && *input != '\0'; i++)
                {
                    input++;
                }
            }
            else
            {
                char *target = va_arg(args, char *);
                for (i = 0; i < count && *input != '\0'; i++)
                {
                    target[i] = *input++;
                }
                if (i != count)
                {
                    break;
                }
                assigned++;
            }
            continue;
        }
        if (conversion == 's')
        {
            int count = 0;
            char *target = suppress ? NULL : va_arg(args, char *);
            while (OpenOPL1000_IsSpace(*input))
            {
                input++;
            }
            while (*input != '\0' && !OpenOPL1000_IsSpace(*input) && (width == 0 || count < width))
            {
                if (!suppress)
                {
                    target[count] = *input;
                }
                input++;
                count++;
            }
            if (count == 0)
            {
                break;
            }
            if (!suppress)
            {
                target[count] = '\0';
                assigned++;
            }
            continue;
        }

        while (OpenOPL1000_IsSpace(*input))
        {
            input++;
        }
        if (conversion == 'f' || conversion == 'F' || conversion == 'e' || conversion == 'E' ||
            conversion == 'g' || conversion == 'G')
        {
            char *end;
            double value = OpenOPL1000_ParseDouble(input, &end);
            if (end == input || (width > 0 && end - input > width))
            {
                break;
            }
            input = end;
            if (!suppress)
            {
                if (length == OPENOPL1000_LENGTH_L)
                {
                    *va_arg(args, double *) = value;
                }
                else
                {
                    *va_arg(args, float *) = (float)value;
                }
                assigned++;
            }
            continue;
        }
        if (conversion == 'd' || conversion == 'i' || conversion == 'u' ||
            conversion == 'x' || conversion == 'X')
        {
            const char *numberStart = input;
            unsigned long long value;
            int negative = 0;
            int digits;
            int remaining = width;
            int base = conversion == 'i' ? 0 : ((conversion == 'x' || conversion == 'X') ? 16 : 10);

            if (*input == '+' || *input == '-')
            {
                negative = *input == '-';
                input++;
                if (remaining > 0)
                {
                    remaining--;
                }
            }
            input = OpenOPL1000_ParseUnsigned(input, base, remaining, &value, &digits);
            if (digits == 0)
            {
                input = numberStart;
                break;
            }
            if (!suppress)
            {
                if (conversion == 'd' || conversion == 'i')
                {
                    long long signedValue = negative ? -(long long)value : (long long)value;
                    OpenOPL1000_StoreSigned(&args, length, signedValue);
                }
                else
                {
                    if (negative)
                    {
                        value = 0u - value;
                    }
                    OpenOPL1000_StoreUnsigned(&args, length, value);
                }
                assigned++;
            }
            continue;
        }

        break;
    }
    va_end(args);
    return assigned;
}

int OpenOPL1000_Scan(const char *text, const char *format, ...)
{
    int result;
    va_list args;

    va_start(args, format);
    result = OpenOPL1000_VScan(text, format, args);
    va_end(args);
    return result;
}

#ifndef OPENOPL1000_LIBC_HOST_TEST
double strtod(const char *text, char **endptr)
{
    return OpenOPL1000_ParseDouble(text, endptr);
}

double atof(const char *text)
{
    return OpenOPL1000_ParseDouble(text, NULL);
}

long atol(const char *text)
{
    return strtol(text, NULL, 10);
}

char *strtok(char *text, const char *delimiters)
{
    static char *state;
    return strtok_r(text, delimiters, &state);
}

int sscanf(const char *text, const char *format, ...)
{
    int result;
    va_list args;

    va_start(args, format);
    result = OpenOPL1000_VScan(text, format, args);
    va_end(args);
    return result;
}
#endif

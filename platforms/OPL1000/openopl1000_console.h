#ifndef OPENOPL1000_CONSOLE_H
#define OPENOPL1000_CONSOLE_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void OpenOPL1000_ConsoleExecute(const char *command, char *output, size_t outputLen);

#ifdef __cplusplus
}
#endif

#endif /* OPENOPL1000_CONSOLE_H */

#ifndef TEST_STORAGE_ENV_H
#define TEST_STORAGE_ENV_H
#define __NEW_COMMON_H__
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
typedef unsigned char byte;
typedef void *xSemaphoreHandle;
#define pdTRUE 1
#define portMAX_DELAY 0xffffffffu
xSemaphoreHandle xSemaphoreCreateMutex(void);
int xSemaphoreTake(xSemaphoreHandle, unsigned);
int xSemaphoreGive(xSemaphoreHandle);
void *test_malloc(size_t);
#define os_malloc test_malloc
#define os_free free
#endif

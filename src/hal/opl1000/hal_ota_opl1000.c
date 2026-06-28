#if PLATFORM_OPL1000

#include <stdio.h>
#include <string.h>
#include "../hal_ota.h"
#include "../../httpserver/new_http.h"
#include "../../logging/logging.h"

void otarequest(const char *urlin)
{
    printf("OTA request ignored for now: %s\r\n", urlin ? urlin : "");
}

int HAL_FlashRead(char *buffer, int readlen, int startaddr)
{
    (void)startaddr;
    if (buffer && readlen > 0)
    {
        memset(buffer, 0xFF, readlen);
    }
    return 0;
}

int http_rest_post_flash(http_request_t* request, int startaddr, int maxaddr)
{
    (void)request;
    (void)startaddr;
    (void)maxaddr;
    ADDLOG_ERROR(LOG_FEATURE_OTA, "OTA flash write is not implemented yet");
    return -1;
}

#endif

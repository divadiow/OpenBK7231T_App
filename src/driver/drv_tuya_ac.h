#ifndef __DRV_TUYA_AC_H__
#define __DRV_TUYA_AC_H__

#include "../new_common.h"

void TuyaAC_Init(void);
void TuyaAC_RunEverySecond(void);
void TuyaAC_AppendInformationToHTTPIndexPage(http_request_t *request);

#endif // __DRV_TUYA_AC_H__

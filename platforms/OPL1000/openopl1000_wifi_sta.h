#ifndef OPENOPL1000_WIFI_STA_H
#define OPENOPL1000_WIFI_STA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void OpenOPL1000_WifiStaInit(void);
bool OpenOPL1000_WifiHasIp(void);
bool OpenOPL1000_WifiIsConnected(void);
void OpenOPL1000_WifiRequestScan(void);
void OpenOPL1000_WifiRequestReconnect(void);
void OpenOPL1000_WifiFormatStatus(char *buffer, size_t bufferLen);
void OpenOPL1000_WifiFormatScanResults(char *buffer, size_t bufferLen);
const char *OpenOPL1000_WifiGetTargetSsid(void);
uint32_t OpenOPL1000_GetUptimeSeconds(void);

#ifdef __cplusplus
}
#endif

#endif /* OPENOPL1000_WIFI_STA_H */

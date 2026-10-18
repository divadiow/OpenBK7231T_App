#ifndef OBK_SV6X66_PLATFORM_H
#define OBK_SV6X66_PLATFORM_H
#include <stdint.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "osal.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/sys.h"

typedef unsigned int UINT32;
typedef int OSStatus;
typedef void *beken_thread_arg_t;
typedef TaskHandle_t beken_thread_t;
typedef void (*beken_thread_function_t)(beken_thread_arg_t);
#define kNoErr 0
#define BEKEN_DEFAULT_WORKER_PRIORITY 2
#define BEKEN_APPLICATION_PRIORITY 2
#define ASSERT(x) configASSERT(x)
#define bk_printf SV6X66_Printf
#define rtos_delay_milliseconds OS_MsDelay
#define delay_ms OS_MsDelay
#define os_malloc SV6X66_Malloc
#define os_free SV6X66_Free
#define os_realloc SV6X66_Realloc
#ifndef close
#define close(fd) lwip_close(fd)
#endif
#define lwip_close_force(fd) lwip_close(fd)
#define OBK_OTA_EXTENSION ".sv6ota"
#ifndef LWIP_CONST_CAST
#define LWIP_CONST_CAST(type,value) ((type)(value))
#endif
#ifndef LWIP_DNS_ADDRTYPE_IPV4
#define LWIP_DNS_ADDRTYPE_IPV4 0
#endif
err_t SV6X66_DnsLookup(const char *,ip_addr_t *);
#define dns_gethostbyname_addrtype(name,address,callback,arg,type) SV6X66_DnsLookup(name,address)
#include "sv6x66_alloc.h"
#define GLOBAL_INT_DECLARATION() OS_DeclareCritical()
#define GLOBAL_INT_DISABLE() OS_EnterCritical()
#define GLOBAL_INT_RESTORE() OS_ExitCritical()

#define SV6X66_FLASH_BASE 0x30000000u
#define SV6X66_FLASH_SIZE 0x00200000u
#define SV6X66_BANK_A 0x0000B000u
#define SV6X66_BANK_B 0x000A6000u
#define SV6X66_BANK_SIZE 0x0009B000u
#define SV6X66_SELECTOR 0x00004000u

OSStatus rtos_create_thread(beken_thread_t *,uint8_t,const char *,beken_thread_function_t,uint32_t,beken_thread_arg_t);
OSStatus rtos_delete_thread(beken_thread_t *);
OSStatus rtos_suspend_thread(beken_thread_t *);
int SV6X66_Printf(const char *,...);
void SV6X66_LogEnable(int);
int SV6X66_LogUartSetBaud(void);
void SV6X66_Reboot(void);
uint32_t SV6X66_RunningBank(void);
int SV6X66_PinAvailable(int pin);
int SV6X66_PinRoleSupported(int pin,int role);
int SV6X66_ProfileValid(void);
int SV6X66_FactoryWindow(void);
void SV6X66_WifiInit(void);
void SV6X66_WifiPoll(void);
struct netif;
/* Static callbacks execute on TCPIP; cleanup also runs before SDK netif free. */
void SV6X66_MDNS_Dispatch(void (*callback)(void),void (*cleanup)(void),int start);
struct netif *SV6X66_MDNS_GetStaNetif(void);
int SV6X66_MDNS_BeforeWifiStop(void);
#define SV6X66_WIFI_SCAN_MAX 20
typedef struct {
	char ssid[33];
	uint8_t channel;
	int16_t rssi;
	const char *security;
} SV6X66_WifiScanResult;
typedef enum {
	SV6X66_SCAN_OK,SV6X66_SCAN_BUSY,SV6X66_SCAN_TIMEOUT,
	SV6X66_SCAN_UNSUPPORTED,SV6X66_SCAN_FAILED
} SV6X66_WifiScanStatus;
SV6X66_WifiScanStatus SV6X66_WifiScan(SV6X66_WifiScanResult *,unsigned int,unsigned int *);
void SV6X66_StorageInit(void);
#endif

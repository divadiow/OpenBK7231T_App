#ifndef OBK_SV6X66_PLATFORM_H
#define OBK_SV6X66_PLATFORM_H
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "osal.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "lwip/err.h"
#include "lwip/dns.h"
#include "lwip/tcpip.h"
typedef unsigned int UINT32;
typedef void *beken_thread_arg_t;
typedef xTaskHandle beken_thread_t;
typedef void (*beken_thread_function_t)(void *);
typedef int OSStatus;
#define kNoErr 0
#define ASSERT configASSERT
// Application workers run below SDK TCP/IP (3) and the timer daemon (4).
#define BEKEN_DEFAULT_WORKER_PRIORITY 1
#define BEKEN_APPLICATION_PRIORITY 1
#define bk_printf printf
#define os_malloc OS_MemAlloc
#define os_free OS_MemFree
#define malloc OS_MemAlloc
#define free OS_MemFree
#define realloc OS_MemRealloc
#define delay_ms OS_MsDelay
#define rtos_delay_milliseconds OS_MsDelay
#define lwip_close_force(x) lwip_close(x)
#define GLOBAL_INT_DECLARATION()
#define GLOBAL_INT_DISABLE() taskENTER_CRITICAL()
#define GLOBAL_INT_RESTORE() taskEXIT_CRITICAL()
#define OBK_OTA_EXTENSION ".ota"
#define sockaddr_storage sockaddr
#define ss_family sa_family
#define ip4_addr ip_addr
#define LWIP_IPV4 1
// This lwIP 1.4 socket API takes timeouts as integer milliseconds.
#define LWIP_SO_SNDRCVTIMEO_NONSTANDARD 1
#define IPADDR4_INIT(value) { value }
int SV6X66_NetInit(void);
err_t SV6X66_DNSLookup(const char *, ip_addr_t *, dns_found_callback, void *);
#define LWIP_CONST_CAST(type, value) ((type)(value))
OSStatus rtos_create_thread(beken_thread_t *, uint8_t, const char *, beken_thread_function_t, uint32_t, void *);
OSStatus rtos_delete_thread(beken_thread_t *);
OSStatus rtos_suspend_thread(beken_thread_t *);
#endif

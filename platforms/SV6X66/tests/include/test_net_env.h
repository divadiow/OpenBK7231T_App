#ifndef SV6X66_TEST_NET_ENV_H
#define SV6X66_TEST_NET_ENV_H

/* Force-include this header when compiling mqtt_dispatch.c for host tests. */
#define OBK_SV6X66_PLATFORM_H
#define LWIP_HDR_APPS_MQTT_CLIENT_H

#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>

typedef uint8_t u8_t;
typedef uint16_t u16_t;
typedef uint32_t u32_t;
typedef int err_t;
typedef pthread_t OsTaskHandle;

#define ERR_OK 0
#define ERR_MEM (-1)
#define ERR_IF (-2)
#define ERR_ARG (-3)
#define ERR_INPROGRESS (-4)
#define portMAX_DELAY UINT32_MAX
#define pdTRUE 1
#define pdFALSE 0

typedef struct { uint32_t addr; } ip_addr_t;
typedef struct mqtt_client_t {
    int id;
    int connected;
} mqtt_client_t;

struct mqtt_connect_client_info_t {
    const char *client_id;
    const char *client_user;
    const char *client_pass;
    u16_t keep_alive;
};

typedef enum {
    MQTT_CONNECT_ACCEPTED = 0,
    MQTT_CONNECT_DISCONNECTED = 256
} mqtt_connection_status_t;
typedef void (*mqtt_connection_cb_t)(mqtt_client_t *, void *, mqtt_connection_status_t);
typedef void (*mqtt_incoming_publish_cb_t)(void *, const char *, u32_t);
typedef void (*mqtt_incoming_data_cb_t)(void *, const u8_t *, u16_t, u8_t);
typedef void (*mqtt_request_cb_t)(void *, err_t);
typedef void (*dns_found_callback)(const char *, const ip_addr_t *, void *);
typedef void (*tcpip_callback_fn)(void *);

struct test_sem;
typedef struct test_sem *xSemaphoreHandle;

OsTaskHandle OS_TaskGetCurrHandle(void);
xSemaphoreHandle xSemaphoreCreateBinary(void);
int xSemaphoreGive(xSemaphoreHandle sem);
int xSemaphoreTake(xSemaphoreHandle sem, uint32_t timeout);
void vSemaphoreDelete(xSemaphoreHandle sem);
err_t tcpip_callback_with_block(tcpip_callback_fn callback, void *arg, u8_t block);
err_t dns_gethostbyname(const char *name, ip_addr_t *address,
                        dns_found_callback callback, void *arg);

#endif
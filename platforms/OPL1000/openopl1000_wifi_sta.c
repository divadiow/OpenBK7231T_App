#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"
#include "event_loop.h"
#include "lwip_helper.h"
#include "openopl1000_wifi_sta.h"
#include "sys_os_config.h"
#include "wifi_api.h"
#include "wifi_event.h"
#include "wifi_event_handler.h"

#ifndef OPENOPL1000_WIFI_SSID
#define OPENOPL1000_WIFI_SSID "test"
#endif

#ifndef OPENOPL1000_WIFI_PASSWORD
#define OPENOPL1000_WIFI_PASSWORD "1234abcd"
#endif

#define OPENOPL1000_WIFI_READY_DELAY_MS 2000

static osThreadId s_wifiThread;
static bool s_seenScanComplete;
static bool s_haveIp;

static void OpenOPL1000_WifiWaitReady(void)
{
    osDelay(OPENOPL1000_WIFI_READY_DELAY_MS);
}

static void OpenOPL1000_WifiScan(void)
{
    wifi_scan_config_t scanConfig;
    memset(&scanConfig, 0, sizeof(scanConfig));

    printf("[OpenOPL1000] scanning for SSID '%s'\r\n", OPENOPL1000_WIFI_SSID);
    wifi_scan_start(&scanConfig, NULL);
}

static int OpenOPL1000_WifiConnectFromScan(void)
{
    wifi_config_t wifiConfig;
    wifi_scan_list_t *scanList;
    int matched = 0;

    memset(&wifiConfig, 0, sizeof(wifiConfig));
    wifi_get_config(WIFI_MODE_STA, &wifiConfig);

    scanList = (wifi_scan_list_t *)malloc(sizeof(wifi_scan_list_t));
    if (scanList == NULL)
    {
        printf("[OpenOPL1000] failed to allocate scan list\r\n");
        return -1;
    }

    memset(scanList, 0, sizeof(wifi_scan_list_t));
    wifi_scan_get_ap_list(scanList);

    printf("[OpenOPL1000] scan complete, ap_count=%u\r\n", scanList->num);

    for (int i = 0; i < scanList->num; i++)
    {
        if ((scanList->ap_record[i].ssid_length == wifiConfig.sta_config.ssid_length) &&
            (memcmp(scanList->ap_record[i].ssid,
                    wifiConfig.sta_config.ssid,
                    wifiConfig.sta_config.ssid_length) == 0))
        {
            matched = 1;
            printf("[OpenOPL1000] matched SSID '%s', connecting\r\n", OPENOPL1000_WIFI_SSID);
            break;
        }
    }

    free(scanList);

    if (!matched)
    {
        printf("[OpenOPL1000] SSID '%s' not found, rescanning\r\n", OPENOPL1000_WIFI_SSID);
        OpenOPL1000_WifiScan();
        return -1;
    }

    wifi_connection_connect(&wifiConfig);
    return 0;
}

static int OpenOPL1000_DefaultStaConnectedHandler(wifi_event_t event, uint8_t *data, uint32_t length)
{
    (void)event;
    (void)length;
    return wifi_station_connected_event_handler(data);
}

static int OpenOPL1000_DefaultStaDisconnectedHandler(wifi_event_t event, uint8_t *data, uint32_t length)
{
    (void)event;
    (void)length;
    return wifi_station_disconnected_event_handler(data);
}

static int OpenOPL1000_WifiEventHandler(wifi_event_id_t eventId, void *data, uint16_t length)
{
    (void)data;
    (void)length;

    switch (eventId)
    {
        case WIFI_EVENT_STA_START:
            printf("[OpenOPL1000] Wi-Fi STA started\r\n");
            OpenOPL1000_WifiWaitReady();
            OpenOPL1000_WifiScan();
            break;

        case WIFI_EVENT_SCAN_COMPLETE:
            s_seenScanComplete = true;
            OpenOPL1000_WifiConnectFromScan();
            break;

        case WIFI_EVENT_STA_CONNECTED:
            printf("[OpenOPL1000] connected to '%s'\r\n", OPENOPL1000_WIFI_SSID);
            lwip_net_start(WIFI_MODE_STA);
            break;

        case WIFI_EVENT_STA_GOT_IP:
            s_haveIp = true;
            printf("[OpenOPL1000] got IP\r\n");
            lwip_get_ip_info("st1");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            s_haveIp = false;
            printf("[OpenOPL1000] disconnected, rescanning\r\n");
            OpenOPL1000_WifiWaitReady();
            OpenOPL1000_WifiScan();
            break;

        case WIFI_EVENT_STA_CONNECTION_FAILED:
            s_haveIp = false;
            printf("[OpenOPL1000] connection failed, rescanning\r\n");
            OpenOPL1000_WifiWaitReady();
            OpenOPL1000_WifiScan();
            break;

        default:
            printf("[OpenOPL1000] Wi-Fi event %d\r\n", eventId);
            break;
    }

    return 0;
}

static void OpenOPL1000_WifiThread(void *args)
{
    (void)args;

    lwip_network_init(WIFI_MODE_STA);
    lwip_net_ready();

    while (1)
    {
        if (!s_haveIp && s_seenScanComplete)
        {
            /* The event handler handles retries; this thread just keeps the
             * network task alive and gives us a low-noise periodic status. */
        }
        osDelay(5000);
    }
}

void OpenOPL1000_WifiStaInit(void)
{
    wifi_init_config_t initConfig = {
        .event_handler = (wifi_event_notify_cb_t)&wifi_event_loop_send,
        .magic = 0x1F2F3F4F
    };
    wifi_config_t wifiConfig;
    osThreadDef_t threadDef;

    printf("[OpenOPL1000] first-stage STA bring-up\r\n");
    printf("[OpenOPL1000] target SSID='%s' password='%s'\r\n",
           OPENOPL1000_WIFI_SSID,
           OPENOPL1000_WIFI_PASSWORD);

    memset(&wifiConfig, 0, sizeof(wifiConfig));
    strncpy((char *)wifiConfig.sta_config.ssid,
            OPENOPL1000_WIFI_SSID,
            sizeof(wifiConfig.sta_config.ssid));
    strncpy((char *)wifiConfig.sta_config.password,
            OPENOPL1000_WIFI_PASSWORD,
            sizeof(wifiConfig.sta_config.password));
    wifiConfig.sta_config.ssid_length = strlen(OPENOPL1000_WIFI_SSID);
    wifiConfig.sta_config.password_length = strlen(OPENOPL1000_WIFI_PASSWORD);

    wifi_register_event_handler(WIFI_EVENT_STA_CONNECTED, OpenOPL1000_DefaultStaConnectedHandler);
    wifi_register_event_handler(WIFI_EVENT_STA_DISCONNECTED, OpenOPL1000_DefaultStaDisconnectedHandler);

    wifi_event_loop_init((wifi_event_cb_t)OpenOPL1000_WifiEventHandler);
    wifi_init(&initConfig, NULL);
    wifi_set_config(WIFI_MODE_STA, &wifiConfig);
    wifi_start();

    threadDef.name = "openopl1000_wifi";
    threadDef.stacksize = OS_TASK_STACK_SIZE_APP;
    threadDef.tpriority = OS_TASK_PRIORITY_APP;
    threadDef.instances = 0;
    threadDef.pthread = OpenOPL1000_WifiThread;

    s_wifiThread = osThreadCreate(&threadDef, NULL);
    if (s_wifiThread == NULL)
    {
        printf("[OpenOPL1000] Wi-Fi task create failed\r\n");
    }
    else
    {
        printf("[OpenOPL1000] Wi-Fi task created\r\n");
    }
}

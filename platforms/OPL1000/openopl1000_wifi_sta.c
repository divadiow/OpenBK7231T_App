#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"
#include "event_loop.h"
#include "lwip_helper.h"
#include "openopl1000_http_server.h"
#include "openopl1000_wifi_sta.h"
#include "sys_os_config.h"
#include "wifi_api.h"
#include "wifi_event.h"
#include "wifi_event_handler.h"

#ifndef OPENOPL1000_WIFI_SSID
#define OPENOPL1000_WIFI_SSID "OPL1000_AP"
#endif

#ifndef OPENOPL1000_WIFI_PASSWORD
#define OPENOPL1000_WIFI_PASSWORD "1234abcd"
#endif

#define OPENOPL1000_WIFI_READY_DELAY_MS 2000
#define OPENOPL1000_SCAN_CACHE_LINES 16
#define OPENOPL1000_SCAN_CACHE_LINE_LEN 96

static osThreadId s_wifiThread;
static bool s_seenScanComplete;
static bool s_staConnected;
static bool s_haveIp;
static bool s_scanInProgress;
static uint32_t s_scanCount;
static uint32_t s_connectAttemptCount;
static uint32_t s_gotIpCount;
static wifi_event_id_t s_lastEvent = WIFI_EVENT_NONE;
static char s_lastEventText[48] = "not started";
static char s_lastScanSummary[OPENOPL1000_SCAN_CACHE_LINES][OPENOPL1000_SCAN_CACHE_LINE_LEN];
static int s_lastScanCount;

static void OpenOPL1000_WifiWaitReady(void)
{
    osDelay(OPENOPL1000_WIFI_READY_DELAY_MS);
}

uint32_t OpenOPL1000_GetUptimeSeconds(void)
{
    if (osKernelSysTick == NULL)
    {
        return 0;
    }

    return osKernelSysTick() / osKernelSysTickFrequency;
}

const char *OpenOPL1000_WifiGetTargetSsid(void)
{
    return OPENOPL1000_WIFI_SSID;
}

bool OpenOPL1000_WifiHasIp(void)
{
    return s_haveIp;
}

bool OpenOPL1000_WifiIsConnected(void)
{
    return s_staConnected;
}

static const char *OpenOPL1000_AuthModeName(wifi_auth_mode_t authMode)
{
    switch (authMode)
    {
        case WIFI_AUTH_OPEN:
            return "OPEN";
        case WIFI_AUTH_WEP:
            return "WEP";
        case WIFI_AUTH_WPA_PSK:
            return "WPA";
        case WIFI_AUTH_WPA2_PSK:
            return "WPA2";
        case WIFI_AUTH_WPA_WPA2_PSK:
            return "WPA/WPA2";
        case WIFI_AUTH_WPA2_ENTERPRISE:
            return "WPA2-ENT";
        default:
            return "UNKNOWN";
    }
}

static void OpenOPL1000_WifiCacheScanResults(void)
{
    wifi_scan_list_t *scanList;

    scanList = (wifi_scan_list_t *)malloc(sizeof(wifi_scan_list_t));
    if (scanList == NULL)
    {
        s_lastScanCount = 0;
        snprintf(s_lastScanSummary[0], OPENOPL1000_SCAN_CACHE_LINE_LEN,
                 "scan result unavailable: allocation failed");
        return;
    }

    memset(scanList, 0, sizeof(wifi_scan_list_t));
    wifi_scan_get_ap_list(scanList);

    s_lastScanCount = scanList->num;
    if (s_lastScanCount > OPENOPL1000_SCAN_CACHE_LINES)
    {
        s_lastScanCount = OPENOPL1000_SCAN_CACHE_LINES;
    }

    for (int i = 0; i < s_lastScanCount; i++)
    {
        char ssid[WIFI_MAX_LENGTH_OF_SSID];
        int ssidLen = scanList->ap_record[i].ssid_length;

        if (ssidLen < 0)
        {
            ssidLen = 0;
        }
        if (ssidLen >= WIFI_MAX_LENGTH_OF_SSID)
        {
            ssidLen = WIFI_MAX_LENGTH_OF_SSID - 1;
        }

        memset(ssid, 0, sizeof(ssid));
        memcpy(ssid, scanList->ap_record[i].ssid, ssidLen);
        ssid[ssidLen] = '\0';

        snprintf(s_lastScanSummary[i], OPENOPL1000_SCAN_CACHE_LINE_LEN,
                 "%02d: ssid='%s' rssi=%d channel=%u auth=%s",
                 i + 1,
                 ssidLen ? ssid : "<hidden>",
                 scanList->ap_record[i].rssi,
                 scanList->ap_record[i].channel,
                 OpenOPL1000_AuthModeName(scanList->ap_record[i].auth_mode));
    }

    if (s_lastScanCount == 0)
    {
        snprintf(s_lastScanSummary[0], OPENOPL1000_SCAN_CACHE_LINE_LEN,
                 "no APs found in last scan");
    }

    free(scanList);
}

static void OpenOPL1000_WifiScan(void)
{
    wifi_scan_config_t scanConfig;
    memset(&scanConfig, 0, sizeof(scanConfig));

    s_scanInProgress = true;
    printf("[OpenOPL1000] scanning for SSID '%s'\r\n", OPENOPL1000_WIFI_SSID);
    wifi_scan_start(&scanConfig, false);
}

void OpenOPL1000_WifiRequestScan(void)
{
    if (s_scanInProgress)
    {
        printf("[OpenOPL1000] scan already in progress\r\n");
        return;
    }

    printf("[OpenOPL1000] manual scan requested\r\n");
    OpenOPL1000_WifiScan();
}

void OpenOPL1000_WifiRequestReconnect(void)
{
    printf("[OpenOPL1000] reconnect requested\r\n");
    s_haveIp = false;
    s_staConnected = false;
    wifi_connection_disconnect_ap();
    OpenOPL1000_WifiWaitReady();
    OpenOPL1000_WifiScan();
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

    s_connectAttemptCount++;
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

static void OpenOPL1000_SetLastEvent(wifi_event_id_t eventId, const char *text)
{
    s_lastEvent = eventId;
    snprintf(s_lastEventText, sizeof(s_lastEventText), "%s", text);
}

static int OpenOPL1000_WifiEventHandler(wifi_event_id_t eventId, void *data, uint16_t length)
{
    (void)data;
    (void)length;

    switch (eventId)
    {
        case WIFI_EVENT_STA_START:
            OpenOPL1000_SetLastEvent(eventId, "STA_START");
            printf("[OpenOPL1000] Wi-Fi STA started\r\n");
            OpenOPL1000_WifiWaitReady();
            OpenOPL1000_WifiScan();
            break;

        case WIFI_EVENT_SCAN_COMPLETE:
            OpenOPL1000_SetLastEvent(eventId, "SCAN_COMPLETE");
            s_seenScanComplete = true;
            s_scanInProgress = false;
            s_scanCount++;
            OpenOPL1000_WifiCacheScanResults();
            OpenOPL1000_WifiConnectFromScan();
            break;

        case WIFI_EVENT_STA_CONNECTED:
            OpenOPL1000_SetLastEvent(eventId, "STA_CONNECTED");
            s_staConnected = true;
            printf("[OpenOPL1000] connected to '%s'\r\n", OPENOPL1000_WIFI_SSID);
            lwip_net_start(WIFI_MODE_STA);
            break;

        case WIFI_EVENT_STA_GOT_IP:
            OpenOPL1000_SetLastEvent(eventId, "STA_GOT_IP");
            s_haveIp = true;
            s_gotIpCount++;
            printf("[OpenOPL1000] got IP\r\n");
            lwip_get_ip_info("st1");
            OpenOPL1000_HttpServerStart();
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            OpenOPL1000_SetLastEvent(eventId, "STA_DISCONNECTED");
            s_haveIp = false;
            s_staConnected = false;
            printf("[OpenOPL1000] disconnected, rescanning\r\n");
            OpenOPL1000_WifiWaitReady();
            OpenOPL1000_WifiScan();
            break;

        case WIFI_EVENT_STA_CONNECTION_FAILED:
            OpenOPL1000_SetLastEvent(eventId, "STA_CONNECTION_FAILED");
            s_haveIp = false;
            s_staConnected = false;
            printf("[OpenOPL1000] connection failed, rescanning\r\n");
            OpenOPL1000_WifiWaitReady();
            OpenOPL1000_WifiScan();
            break;

        default:
            OpenOPL1000_SetLastEvent(eventId, "OTHER");
            printf("[OpenOPL1000] Wi-Fi event %d\r\n", eventId);
            break;
    }

    return 0;
}

void OpenOPL1000_WifiFormatStatus(char *buffer, size_t bufferLen)
{
    int8_t rssi = 0;
    int rssiResult = -1;
    uint8_t mac[WIFI_MAC_ADDRESS_LENGTH] = {0};
    int macResult;

    if ((buffer == NULL) || (bufferLen == 0))
    {
        return;
    }

    if (s_staConnected)
    {
        rssiResult = wifi_connection_get_rssi(&rssi);
    }

    macResult = wifi_config_get_mac_address(WIFI_MODE_STA, mac);

    snprintf(buffer, bufferLen,
             "OpenOPL1000 status\r\n"
             "uptime_s: %lu\r\n"
             "target_ssid: %s\r\n"
             "connected: %s\r\n"
             "got_ip: %s\r\n"
             "scan_in_progress: %s\r\n"
             "scan_count: %lu\r\n"
             "connect_attempts: %lu\r\n"
             "got_ip_count: %lu\r\n"
             "last_event: %d %s\r\n"
             "rssi: %s%d\r\n"
             "sta_mac: %s%02X:%02X:%02X:%02X:%02X:%02X\r\n",
             (unsigned long)OpenOPL1000_GetUptimeSeconds(),
             OPENOPL1000_WIFI_SSID,
             s_staConnected ? "yes" : "no",
             s_haveIp ? "yes" : "no",
             s_scanInProgress ? "yes" : "no",
             (unsigned long)s_scanCount,
             (unsigned long)s_connectAttemptCount,
             (unsigned long)s_gotIpCount,
             (int)s_lastEvent,
             s_lastEventText,
             (rssiResult == 0) ? "" : "unavailable ",
             (rssiResult == 0) ? rssi : 0,
             (macResult == 0) ? "" : "unavailable ",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void OpenOPL1000_WifiFormatScanResults(char *buffer, size_t bufferLen)
{
    size_t used = 0;

    if ((buffer == NULL) || (bufferLen == 0))
    {
        return;
    }

    buffer[0] = '\0';
    used += snprintf(buffer + used, bufferLen - used,
                     "last_scan_count: %d\r\n", s_lastScanCount);

    if (s_lastScanCount <= 0)
    {
        if (used < bufferLen)
        {
            snprintf(buffer + used, bufferLen - used, "%s\r\n", s_lastScanSummary[0][0] ? s_lastScanSummary[0] : "no cached scan yet");
        }
        return;
    }

    for (int i = 0; i < s_lastScanCount; i++)
    {
        if (used >= bufferLen)
        {
            break;
        }

        used += snprintf(buffer + used, bufferLen - used, "%s\r\n", s_lastScanSummary[i]);
    }
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

    printf("[OpenOPL1000] STA bring-up with web console\r\n");
    printf("[OpenOPL1000] target SSID='%s' password='%s'\r\n",
           OPENOPL1000_WIFI_SSID,
           OPENOPL1000_WIFI_PASSWORD);

    memset(&wifiConfig, 0, sizeof(wifiConfig));
    strncpy((char *)wifiConfig.sta_config.ssid,
            OPENOPL1000_WIFI_SSID,
            sizeof(wifiConfig.sta_config.ssid) - 1);
    strncpy((char *)wifiConfig.sta_config.password,
            OPENOPL1000_WIFI_PASSWORD,
            sizeof(wifiConfig.sta_config.password) - 1);
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

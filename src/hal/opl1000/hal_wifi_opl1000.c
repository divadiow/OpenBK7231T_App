#if PLATFORM_OPL1000

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../new_common.h"
#include "../../logging/logging.h"
#include "../hal_wifi.h"

#include "cmsis_os.h"
#include "controller_wifi_com.h"
#include "event_loop.h"
#include "lwip_helper.h"
#include "lwip/ip4_addr.h"
#include "lwip/netif.h"
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

/* Keep this small. The real OBK runtime leaves very little heap on OPL1000 A2
 * during early bring-up, so the SDK demo's malloc(sizeof(wifi_scan_list_t))
 * can fail before association. Use a fixed small scan-record buffer instead.
 */
#define OPENOPL1000_WIFI_READY_DELAY_MS 2000
#define OPENOPL1000_SCAN_RECORD_LIMIT   8

typedef size_t (*OpenOPL1000HeapFn)(void);
extern size_t xPortGetFreeHeapSize(void);

static void (*g_wifiStatusCallback)(int code);
static bool g_wifiInitialised;
static bool g_lwipThreadStarted;
static osThreadId g_lwipThread;
static bool g_haveIp;
static bool g_seenScanComplete;
static char g_ssid[WIFI_MAX_LENGTH_OF_SSID];
static char g_password[WIFI_LENGTH_PASSPHRASE + 1];
static uint8_t g_bssid[WIFI_MAC_ADDRESS_LENGTH];
static uint8_t g_channel;
static char g_ip[16] = "0.0.0.0";
static char g_gw[16] = "0.0.0.0";
static char g_mask[16] = "0.0.0.0";
static char g_dns[16] = "0.0.0.0";
static wifi_scan_info_t g_scanRecords[OPENOPL1000_SCAN_RECORD_LIMIT];

static unsigned int OpenOPL1000_GetFreeHeap(void)
{
    return (unsigned int)xPortGetFreeHeapSize();
}

static void OpenOPL1000_ReportStatus(int status)
{
    if (g_wifiStatusCallback)
    {
        g_wifiStatusCallback(status);
    }
}

static void OpenOPL1000_UpdateNetifStrings(void)
{
    struct netif *netif = netif_find("st1");

    if (netif)
    {
        ip4addr_ntoa_r(ip_2_ip4(&netif->ip_addr), g_ip, sizeof(g_ip));
        ip4addr_ntoa_r(ip_2_ip4(&netif->gw), g_gw, sizeof(g_gw));
        ip4addr_ntoa_r(ip_2_ip4(&netif->netmask), g_mask, sizeof(g_mask));
    }
    else
    {
        strcpy(g_ip, "0.0.0.0");
        strcpy(g_gw, "0.0.0.0");
        strcpy(g_mask, "0.0.0.0");
    }

    /* DNS is not directly exposed by the Opulinks helper path used here yet. */
    strcpy(g_dns, g_gw);
}

static void OpenOPL1000_LwipThread(void *args)
{
    (void)args;

    /* This mirrors the vendor examples. lwip_net_ready() blocks until Wi-Fi is
     * connected and DHCP has completed, so it must not run in the OBK init task
     * before wifi_start() has returned. v11 called it synchronously and stalled
     * before wifi_set_config()/wifi_start().
     */
    printf("[OpenOPL1000] lwIP task: network init start, heap=%u\r\n", OpenOPL1000_GetFreeHeap());
    lwip_network_init(WIFI_MODE_STA);
    printf("[OpenOPL1000] lwIP task: net_ready wait, heap=%u\r\n", OpenOPL1000_GetFreeHeap());
    lwip_net_ready();
    printf("[OpenOPL1000] lwIP task: net_ready returned, heap=%u\r\n", OpenOPL1000_GetFreeHeap());

    while (1)
    {
        if (g_haveIp)
        {
            OpenOPL1000_UpdateNetifStrings();
        }
        osDelay(10000);
    }
}

static void OpenOPL1000_LwipStartTaskOnce(void)
{
    osThreadDef_t threadDef;

    if (g_lwipThreadStarted)
    {
        return;
    }

    memset(&threadDef, 0, sizeof(threadDef));
    threadDef.name = "opl_lwip";
    threadDef.stacksize = OS_TASK_STACK_SIZE_APP;
    threadDef.tpriority = OS_TASK_PRIORITY_APP;
    threadDef.instances = 0;
    threadDef.pthread = OpenOPL1000_LwipThread;

    g_lwipThread = osThreadCreate(&threadDef, NULL);
    g_lwipThreadStarted = (g_lwipThread != NULL);

    printf("[OpenOPL1000] lwIP task create %s, heap=%u\r\n",
           g_lwipThreadStarted ? "ok" : "failed",
           OpenOPL1000_GetFreeHeap());
}

static void OpenOPL1000_WifiWaitReady(void)
{
    osDelay(OPENOPL1000_WIFI_READY_DELAY_MS);
}

static void OpenOPL1000_WifiScan(void)
{
    int rc;
    wifi_scan_config_t scanConfig;
    memset(&scanConfig, 0, sizeof(scanConfig));

    printf("[OpenOPL1000] scanning for SSID '%s', heap=%u\r\n", g_ssid, OpenOPL1000_GetFreeHeap());
    rc = wifi_scan_start(&scanConfig, false);
    printf("[OpenOPL1000] wifi_scan_start rc=%d\r\n", rc);
}

static int OpenOPL1000_WifiConnectDirect(const char *reason)
{
    int rc;
    wifi_config_t wifiConfig;

    memset(&wifiConfig, 0, sizeof(wifiConfig));
    wifi_get_config(WIFI_MODE_STA, &wifiConfig);

    printf("[OpenOPL1000] direct connect attempt (%s), ssid='%s', heap=%u\r\n",
           reason ? reason : "no reason",
           g_ssid,
           OpenOPL1000_GetFreeHeap());

    OpenOPL1000_ReportStatus(WIFI_STA_CONNECTING);
    rc = wifi_connection_connect(&wifiConfig);
    printf("[OpenOPL1000] wifi_connection_connect rc=%d\r\n", rc);
    return rc;
}

static int OpenOPL1000_WifiConnectFromScan(void)
{
    wifi_config_t wifiConfig;
    uint16_t recordCount = OPENOPL1000_SCAN_RECORD_LIMIT;
    int matched = 0;
    int rc;

    memset(&wifiConfig, 0, sizeof(wifiConfig));
    wifi_get_config(WIFI_MODE_STA, &wifiConfig);

    memset(g_scanRecords, 0, sizeof(g_scanRecords));
    rc = wifi_scan_get_ap_records(&recordCount, g_scanRecords);
    printf("[OpenOPL1000] scan records rc=%d count=%u heap=%u\r\n",
           rc,
           (unsigned int)recordCount,
           OpenOPL1000_GetFreeHeap());

    if ((rc != 0) || (recordCount == 0))
    {
        return OpenOPL1000_WifiConnectDirect("no scan records");
    }

    for (uint16_t i = 0; i < recordCount; i++)
    {
        const wifi_scan_info_t *rec = &g_scanRecords[i];

        if ((rec->ssid_length == wifiConfig.sta_config.ssid_length) &&
            (memcmp(rec->ssid,
                    wifiConfig.sta_config.ssid,
                    wifiConfig.sta_config.ssid_length) == 0))
        {
            matched = 1;
            memcpy(g_bssid, rec->bssid, sizeof(g_bssid));
            g_channel = rec->channel;
            memcpy(wifiConfig.sta_config.bssid, rec->bssid, sizeof(wifiConfig.sta_config.bssid));
            wifiConfig.sta_config.bssid_present = 1;
            printf("[OpenOPL1000] matched SSID '%s' bssid=" MACSTR " ch=%u rssi=%d\r\n",
                   g_ssid,
                   MAC2STR(rec->bssid),
                   (unsigned int)rec->channel,
                   rec->rssi);
            break;
        }
    }

    if (!matched)
    {
        printf("[OpenOPL1000] SSID '%s' not found in first %u records, rescanning\r\n",
               g_ssid,
               (unsigned int)recordCount);
        OpenOPL1000_WifiScan();
        return -1;
    }

    wifi_set_config(WIFI_MODE_STA, &wifiConfig);
    OpenOPL1000_ReportStatus(WIFI_STA_CONNECTING);
    rc = wifi_connection_connect(&wifiConfig);
    printf("[OpenOPL1000] wifi_connection_connect rc=%d\r\n", rc);
    return rc;
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
    (void)length;

    switch (eventId)
    {
        case WIFI_EVENT_STA_START:
            printf("[OpenOPL1000] Wi-Fi STA started\r\n");
            OpenOPL1000_WifiWaitReady();
            OpenOPL1000_WifiScan();
            break;

        case WIFI_EVENT_SCAN_COMPLETE:
            g_seenScanComplete = true;
            printf("[OpenOPL1000] scan complete event\r\n");
            OpenOPL1000_WifiConnectFromScan();
            break;

        case WIFI_EVENT_STA_CONNECTED:
            if (data)
            {
                wifi_event_sta_connected_t *ev = (wifi_event_sta_connected_t *)data;
                memcpy(g_bssid, ev->bssid, sizeof(g_bssid));
                g_channel = ev->channel;
            }
            printf("[OpenOPL1000] connected to '%s', starting lwIP\r\n", g_ssid);
            OpenOPL1000_ReportStatus(WIFI_STA_CONNECTING);
            lwip_net_start(WIFI_MODE_STA);
            break;

        case WIFI_EVENT_STA_GOT_IP:
            g_haveIp = true;
            OpenOPL1000_UpdateNetifStrings();
            printf("[OpenOPL1000] got IP %s\r\n", g_ip);
            lwip_get_ip_info("st1");
            OpenOPL1000_ReportStatus(WIFI_STA_CONNECTED);
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            g_haveIp = false;
            printf("[OpenOPL1000] disconnected, rescanning\r\n");
            OpenOPL1000_ReportStatus(WIFI_STA_DISCONNECTED);
            OpenOPL1000_WifiWaitReady();
            OpenOPL1000_WifiScan();
            break;

        case WIFI_EVENT_STA_CONNECTION_FAILED:
            g_haveIp = false;
            printf("[OpenOPL1000] connection failed, rescanning\r\n");
            OpenOPL1000_ReportStatus(WIFI_STA_AUTH_FAILED);
            OpenOPL1000_WifiWaitReady();
            OpenOPL1000_WifiScan();
            break;

        default:
            printf("[OpenOPL1000] Wi-Fi event %d\r\n", eventId);
            break;
    }

    return 0;
}

static void OpenOPL1000_WifiInitOnce(void)
{
    wifi_init_config_t initConfig = {
        .event_handler = (wifi_event_notify_cb_t)&wifi_event_loop_send,
        .magic = 0x1F2F3F4F
    };

    if (g_wifiInitialised)
    {
        return;
    }

    printf("[OpenOPL1000] Wi-Fi init start, heap=%u\r\n", OpenOPL1000_GetFreeHeap());

    wifi_register_event_handler(WIFI_EVENT_STA_CONNECTED, OpenOPL1000_DefaultStaConnectedHandler);
    wifi_register_event_handler(WIFI_EVENT_STA_DISCONNECTED, OpenOPL1000_DefaultStaDisconnectedHandler);

    wifi_event_loop_init((wifi_event_cb_t)OpenOPL1000_WifiEventHandler);
    wifi_init(&initConfig, NULL);

    g_wifiInitialised = true;
    printf("[OpenOPL1000] Wi-Fi init done, heap=%u\r\n", OpenOPL1000_GetFreeHeap());
}

void HAL_WiFi_SetupStatusCallback(void (*cb)(int code))
{
    g_wifiStatusCallback = cb;
}

void HAL_ConnectToWiFi(const char *oob_ssid, const char *connect_key, obkStaticIP_t *ip)
{
    int rc;
    wifi_config_t wifiConfig;
    (void)ip;

    if (oob_ssid && *oob_ssid)
    {
        strncpy(g_ssid, oob_ssid, sizeof(g_ssid) - 1);
    }
    else
    {
        strncpy(g_ssid, OPENOPL1000_WIFI_SSID, sizeof(g_ssid) - 1);
    }

    if (connect_key && *connect_key)
    {
        strncpy(g_password, connect_key, sizeof(g_password) - 1);
    }
    else
    {
        strncpy(g_password, OPENOPL1000_WIFI_PASSWORD, sizeof(g_password) - 1);
    }

    g_ssid[sizeof(g_ssid) - 1] = 0;
    g_password[sizeof(g_password) - 1] = 0;

    printf("[OpenOPL1000] HAL_ConnectToWiFi ssid='%s' heap=%u\r\n", g_ssid, OpenOPL1000_GetFreeHeap());
    OpenOPL1000_ReportStatus(WIFI_STA_CONNECTING);

    OpenOPL1000_WifiInitOnce();

    memset(&wifiConfig, 0, sizeof(wifiConfig));
    strncpy((char *)wifiConfig.sta_config.ssid, g_ssid, sizeof(wifiConfig.sta_config.ssid) - 1);
    strncpy((char *)wifiConfig.sta_config.password, g_password, sizeof(wifiConfig.sta_config.password));
    wifiConfig.sta_config.ssid_length = strlen(g_ssid);
    wifiConfig.sta_config.password_length = strlen(g_password);

    rc = wifi_set_config(WIFI_MODE_STA, &wifiConfig);
    printf("[OpenOPL1000] wifi_set_config rc=%d ssid_len=%u pass_len=%u\r\n",
           rc,
           (unsigned int)wifiConfig.sta_config.ssid_length,
           (unsigned int)wifiConfig.sta_config.password_length);

    rc = wifi_start();
    printf("[OpenOPL1000] wifi_start rc=%d heap=%u\r\n", rc, OpenOPL1000_GetFreeHeap());

    OpenOPL1000_LwipStartTaskOnce();
}

void HAL_FastConnectToWiFi(const char *oob_ssid, const char *connect_key, obkStaticIP_t *ip)
{
    HAL_ConnectToWiFi(oob_ssid, connect_key, ip);
}

int HAL_SetupWiFiOpenAccessPoint(const char *ssid)
{
    printf("[OpenOPL1000] SoftAP requested for '%s' but OPL1000 SDK supports STA only\r\n", ssid ? ssid : "");
    OpenOPL1000_ReportStatus(WIFI_AP_FAILED);
    HAL_ConnectToWiFi(OPENOPL1000_WIFI_SSID, OPENOPL1000_WIFI_PASSWORD, NULL);
    return -1;
}

void HAL_DisableEnhancedFastConnect(void)
{
}

void HAL_DisconnectFromWifi(void)
{
    g_haveIp = false;
    wifi_stop();
    OpenOPL1000_ReportStatus(WIFI_STA_DISCONNECTED);
}

const char *HAL_GetMyIPString(void)
{
    OpenOPL1000_UpdateNetifStrings();
    return g_ip;
}

const char *HAL_GetMyGatewayString(void)
{
    OpenOPL1000_UpdateNetifStrings();
    return g_gw;
}

const char *HAL_GetMyDNSString(void)
{
    OpenOPL1000_UpdateNetifStrings();
    return g_dns;
}

const char *HAL_GetMyMaskString(void)
{
    OpenOPL1000_UpdateNetifStrings();
    return g_mask;
}

void WiFI_GetMacAddress(char *mac)
{
    static const uint8_t fallback[6] = {0x02, 0x4F, 0x50, 0x10, 0x00, 0x01};

    if (mac == NULL)
    {
        return;
    }

    memcpy(mac, fallback, sizeof(fallback));

    if (wifi_get_mac_addr)
    {
        wifi_get_mac_addr((uint8_t *)mac);
    }
}

int WiFI_SetMacAddress(char *mac)
{
    (void)mac;
    return 0;
}

const char *HAL_GetMACStr(char *macstr)
{
    char mac[6];
    WiFI_GetMacAddress(mac);
    sprintf(macstr, MACSTR, MAC2STR((uint8_t *)mac));
    return macstr;
}

char *HAL_GetWiFiBSSID(char *bssid)
{
    if (bssid)
    {
        sprintf(bssid, MACSTR, MAC2STR(g_bssid));
    }
    return bssid;
}

uint8_t HAL_GetWiFiChannel(uint8_t *chan)
{
    if (chan)
    {
        *chan = g_channel;
    }
    return g_channel;
}

void HAL_PrintNetworkInfo(void)
{
    printf("[OpenOPL1000] IP=%s GW=%s MASK=%s\r\n", HAL_GetMyIPString(), HAL_GetMyGatewayString(), HAL_GetMyMaskString());
}

int HAL_GetWifiStrength(void)
{
    return 0;
}

#endif

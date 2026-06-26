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

#define OPENOPL1000_WIFI_READY_DELAY_MS       5000
#define OPENOPL1000_SCAN_RESULT_WAIT_MS       4000
#define OPENOPL1000_DIRECT_ASSOC_WAIT_SECONDS 12
#define OPENOPL1000_ASSOC_WAIT_SECONDS        30
#define OPENOPL1000_SCAN_RETRY_DELAY_MS       10000
#define OPENOPL1000_SCAN_PASS_COUNT           4
#define OPENOPL1000_WIFI_VERBOSE_SCAN         0
#define OPENOPL1000_NETINFO_TRACE             0

extern size_t xPortGetFreeHeapSize(void);

static void (*g_wifiStatusCallback)(int code);
static bool g_wifiInitialised;
static bool g_wifiStarted;
static bool g_workerStarted;
static bool g_associated;
static bool g_haveIp;
static bool g_lwipStarted;
static osThreadId g_workerThread;
static char g_ssid[WIFI_MAX_LENGTH_OF_SSID];
static char g_password[WIFI_LENGTH_PASSPHRASE + 1];
static uint8_t g_bssid[WIFI_MAC_ADDRESS_LENGTH];
static uint8_t g_channel;
static char g_ip[16] = "0.0.0.0";
static char g_gw[16] = "0.0.0.0";
static char g_mask[16] = "0.0.0.0";
static char g_dns[16] = "0.0.0.0";

static unsigned int OpenOPL1000_GetFreeHeap(void)
{
    return (unsigned int)xPortGetFreeHeapSize();
}

static bool OpenOPL1000_IsUsableMac(const uint8_t *mac)
{
    bool anyNonZero = false;
    bool anyNonFf = false;

    if (mac == NULL)
    {
        return false;
    }

    for (int i = 0; i < 6; i++)
    {
        if (mac[i] != 0x00)
        {
            anyNonZero = true;
        }
        if (mac[i] != 0xFF)
        {
            anyNonFf = true;
        }
    }

    return anyNonZero && anyNonFf;
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

    strcpy(g_dns, g_gw);
}

static bool OpenOPL1000_IsAssociated(void)
{
    wifi_ap_record_t apInfo;
    int rc;

    memset(&apInfo, 0, sizeof(apInfo));
    rc = wifi_sta_get_ap_info(&apInfo);

    if (rc == 0 && apInfo.ssid_length > 0)
    {
        memcpy(g_bssid, apInfo.bssid, sizeof(g_bssid));
        g_channel = apInfo.channel;
        if (!g_associated)
        {
            printf("[OpenOPL1000] associated with '%.*s' bssid=" MACSTR " ch=%u rssi=%d heap=%u\r\n",
                   apInfo.ssid_length,
                   apInfo.ssid,
                   MAC2STR(apInfo.bssid),
                   (unsigned int)apInfo.channel,
                   apInfo.rssi,
                   OpenOPL1000_GetFreeHeap());
        }
        return true;
    }

    return false;
}

static bool OpenOPL1000_UpdateBestApFromReport(scan_report_t *report)
{
    wifi_config_t wifiConfig;
    const scan_info_t *best = NULL;

    if (report == NULL || report->pScanInfo == NULL || report->uScanApNum == 0)
    {
        printf("[OpenOPL1000] worker: raw scan report not ready; will retry\r\n");
        return false;
    }

    memset(&wifiConfig, 0, sizeof(wifiConfig));
    wifi_get_config(WIFI_MODE_STA, &wifiConfig);

    for (uint32_t i = 0; i < report->uScanApNum; i++)
    {
        const scan_info_t *rec = &report->pScanInfo[i];
        unsigned int ssidLen = (unsigned int)rec->ssid_len;

        if (ssidLen > WIFI_MAX_LENGTH_OF_SSID)
        {
            ssidLen = WIFI_MAX_LENGTH_OF_SSID;
        }

#if OPENOPL1000_WIFI_VERBOSE_SCAN
        printf("[OpenOPL1000] worker: raw[%u] ssid='%.*s' len=%u bssid=" MACSTR " rssi=%d ch=%u caps=0x%04x\r\n",
               (unsigned int)i,
               ssidLen,
               rec->ssid,
               ssidLen,
               MAC2STR(rec->bssid),
               rec->rssi,
               (unsigned int)rec->ap_channel,
               (unsigned int)rec->capabilities);
#endif

        if ((ssidLen == wifiConfig.sta_config.ssid_length) &&
            (memcmp(rec->ssid,
                    wifiConfig.sta_config.ssid,
                    wifiConfig.sta_config.ssid_length) == 0))
        {
            if (best == NULL || rec->rssi > best->rssi)
            {
                best = rec;
            }
        }
    }

    if (best == NULL)
    {
        return false;
    }

    memcpy(g_bssid, best->bssid, sizeof(g_bssid));
    g_channel = best->ap_channel;
    printf("[OpenOPL1000] worker: best SSID '%s' bssid=" MACSTR " ch=%u rssi=%d; using SDK no-BSSID connect path\r\n",
           g_ssid,
           MAC2STR(best->bssid),
           (unsigned int)best->ap_channel,
           best->rssi);
    return true;
}

static int OpenOPL1000_TryDirectConnectWithoutBssid(void)
{
    wifi_config_t wifiConfig;
    int rc;

    memset(&wifiConfig, 0, sizeof(wifiConfig));
    wifi_get_config(WIFI_MODE_STA, &wifiConfig);

    wifiConfig.sta_config.bssid_present = 0;
    memset(wifiConfig.sta_config.bssid, 0, sizeof(wifiConfig.sta_config.bssid));
    wifiConfig.sta_config.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifiConfig.sta_config.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifiConfig.sta_config.threshold.authmode = WIFI_AUTH_OPEN;
    wifiConfig.sta_config.threshold.rssi = -127;

    rc = wifi_set_config(WIFI_MODE_STA, &wifiConfig);
    printf("[OpenOPL1000] worker: direct no-BSSID connect set_config rc=%d heap=%u\r\n",
           rc,
           OpenOPL1000_GetFreeHeap());

    OpenOPL1000_ReportStatus(WIFI_STA_CONNECTING);
    rc = wifi_connection_connect(&wifiConfig);
    printf("[OpenOPL1000] worker: direct no-BSSID wifi_connection_connect rc=%d heap=%u\r\n",
           rc,
           OpenOPL1000_GetFreeHeap());
    return rc;
}

static bool OpenOPL1000_WaitForAssociation(unsigned int seconds)
{
    for (unsigned int i = 0; i < seconds; i++)
    {
        osDelay(1000);
        if (OpenOPL1000_IsAssociated())
        {
            g_associated = true;
            return true;
        }
    }

    return false;
}

static int OpenOPL1000_DoScanAndConnect(void)
{
    wifi_scan_config_t scanConfig;
    scan_report_t *report;
    uint16_t apCount = 0;
    int rc = -1;

    for (int pass = 0; pass < OPENOPL1000_SCAN_PASS_COUNT; pass++)
    {
        bool targeted = (pass == 0);

        memset(&scanConfig, 0, sizeof(scanConfig));

        if (targeted)
        {
            scanConfig.ssid = (uint8_t *)g_ssid;
        }
        scanConfig.channel = 0;
        scanConfig.show_hidden = true;
        scanConfig.scan_type = WIFI_SCAN_TYPE_MIX;
        scanConfig.scan_time.active.min = 100;
        scanConfig.scan_time.active.max = 300;
        scanConfig.scan_time.passive = 150;

        printf("[OpenOPL1000] worker: %s scan pass %d/%d for SSID '%s' wait=%ums heap=%u\r\n",
               targeted ? "targeted" : "broad fallback",
               pass + 1,
               OPENOPL1000_SCAN_PASS_COUNT,
               g_ssid,
               (unsigned int)OPENOPL1000_SCAN_RESULT_WAIT_MS,
               OpenOPL1000_GetFreeHeap());

        /* In crowded RF environments the SDK's retained all-SSID scan list is
         * small and RSSI-sorted.  Filtering the scan request to the desired
         * SSID is more useful than asking for a larger all-SSID list: it stops
         * unrelated nearby APs from displacing the weak phone hotspot result.
         */
        rc = wifi_scan_start(&scanConfig, false);
        printf("[OpenOPL1000] worker: wifi_scan_start(%s) rc=%d heap=%u\r\n",
               targeted ? "targeted" : "broad",
               rc,
               OpenOPL1000_GetFreeHeap());

        if (rc != 0)
        {
            osDelay(1000);
            continue;
        }

        osDelay(OPENOPL1000_SCAN_RESULT_WAIT_MS);

        rc = wifi_scan_get_ap_num(&apCount);
#if OPENOPL1000_WIFI_VERBOSE_SCAN
        printf("[OpenOPL1000] worker: public scan count rc=%d ap_count=%u heap=%u\r\n",
               rc,
               (unsigned int)apCount,
               OpenOPL1000_GetFreeHeap());
#endif

        report = wifi_get_scan_result ? wifi_get_scan_result() : NULL;
#if OPENOPL1000_WIFI_VERBOSE_SCAN
        printf("[OpenOPL1000] worker: raw scan report ptr=%p count=%u heap=%u\r\n",
               report,
               report ? (unsigned int)report->uScanApNum : 0,
               OpenOPL1000_GetFreeHeap());
#endif

        if (OpenOPL1000_UpdateBestApFromReport(report))
        {
            rc = OpenOPL1000_TryDirectConnectWithoutBssid();
            if (OpenOPL1000_WaitForAssociation(OPENOPL1000_DIRECT_ASSOC_WAIT_SECONDS))
            {
                return 0;
            }
        }

        printf("[OpenOPL1000] worker: SSID '%s' not associated after %s scan pass %d\r\n",
               g_ssid,
               targeted ? "targeted" : "broad",
               pass + 1);

        osDelay(1000);
    }

    return rc;
}

static void OpenOPL1000_DoConnectCycle(void)
{
    int rc;

    /* The SDK station demos scan first, then connect. v43 logs showed the
     * pre-scan direct connect returned rc=-1 and only succeeded after a scan
     * had populated the SDK's internal AP table, so keep scan-first as the
     * normal path and use direct connect as the last fallback.
     */
    rc = OpenOPL1000_DoScanAndConnect();
    if (OpenOPL1000_IsAssociated())
    {
        return;
    }

    printf("[OpenOPL1000] worker: scan-first connect returned %d, trying final direct connect heap=%u\r\n",
           rc,
           OpenOPL1000_GetFreeHeap());

    rc = OpenOPL1000_TryDirectConnectWithoutBssid();
    if (rc == 0 || rc == -1)
    {
        if (OpenOPL1000_WaitForAssociation(OPENOPL1000_DIRECT_ASSOC_WAIT_SECONDS))
        {
            return;
        }

        printf("[OpenOPL1000] worker: final direct connect did not associate after %u seconds heap=%u\r\n",
               (unsigned int)OPENOPL1000_DIRECT_ASSOC_WAIT_SECONDS,
               OpenOPL1000_GetFreeHeap());
    }
    else
    {
        printf("[OpenOPL1000] worker: final direct connect returned %d heap=%u\r\n",
               rc,
               OpenOPL1000_GetFreeHeap());
    }
}

static bool OpenOPL1000_StartLwipAndPollForIp(void)
{
    if (!g_lwipStarted)
    {
        g_lwipStarted = true;
        printf("[OpenOPL1000] worker: lwIP network init start, heap=%u\r\n", OpenOPL1000_GetFreeHeap());
        lwip_network_init(WIFI_MODE_STA);
        printf("[OpenOPL1000] worker: lwIP net_start, heap=%u\r\n", OpenOPL1000_GetFreeHeap());
        lwip_net_start(WIFI_MODE_STA);

        /* Do not call lwip_net_ready() here. On OPL1000 A2 it blocks the
         * temporary Wi-Fi worker indefinitely after DHCP succeeds, so the
         * worker stack is never returned. v24 proved DHCP/IP works via
         * lwip_net_start() plus the SDK event path; v25 polls the netif
         * directly and terminates the worker once a non-zero IPv4 address
         * appears.
         */
        printf("[OpenOPL1000] worker: polling for DHCP address, heap=%u\r\n", OpenOPL1000_GetFreeHeap());
    }

    for (int i = 0; i < 40; i++)
    {
        OpenOPL1000_UpdateNetifStrings();
        if (strcmp(g_ip, "0.0.0.0") != 0)
        {
            g_haveIp = true;
            printf("[OpenOPL1000] worker: got IP %s GW=%s MASK=%s heap=%u\r\n",
                   g_ip,
                   g_gw,
                   g_mask,
                   OpenOPL1000_GetFreeHeap());
            lwip_get_ip_info("st1");
            OpenOPL1000_ReportStatus(WIFI_STA_CONNECTED);
            return true;
        }
        osDelay(500);
    }

    printf("[OpenOPL1000] worker: DHCP address not ready yet, heap=%u\r\n",
           OpenOPL1000_GetFreeHeap());
    return false;
}

static void OpenOPL1000_WifiWorker(void *args)
{
    (void)args;

    printf("[OpenOPL1000] worker: started, heap=%u\r\n", OpenOPL1000_GetFreeHeap());
    osDelay(OPENOPL1000_WIFI_READY_DELAY_MS);

    while (1)
    {
        if (!g_associated)
        {
            OpenOPL1000_DoConnectCycle();
            OpenOPL1000_WaitForAssociation(OPENOPL1000_ASSOC_WAIT_SECONDS);

            if (!g_associated)
            {
                printf("[OpenOPL1000] worker: not associated after %u seconds, retrying heap=%u\r\n",
                       (unsigned int)OPENOPL1000_ASSOC_WAIT_SECONDS,
                       OpenOPL1000_GetFreeHeap());
                /* Do not call wifi_connection_disconnect_ap() here. The v13 log
                 * WDTed in the SDK opl_event_loop shortly after a retry. Let the
                 * controller settle, then re-scan/re-connect from the worker.
                 */
                osDelay(OPENOPL1000_SCAN_RETRY_DELAY_MS);
                continue;
            }
        }

        if (g_associated && !g_haveIp)
        {
            OpenOPL1000_StartLwipAndPollForIp();
        }

        if (g_haveIp)
        {
            OpenOPL1000_UpdateNetifStrings();
            if (strcmp(g_ip, "0.0.0.0") == 0)
            {
                g_haveIp = false;
                g_lwipStarted = false;
                printf("[OpenOPL1000] worker: lost IP, restarting network path\r\n");
            }
            else
            {
                /* Once DHCP has completed the worker has no more critical work.
                 * Terminate it so its stack is returned before HTTP requests
                 * arrive. v24 showed lwip_net_ready() did not return here;
                 * v25 avoids that blocking waiter and reclaims this stack.
                 */
                printf("[OpenOPL1000] worker: network ready, terminating worker to free stack, heap=%u\r\n",
                       OpenOPL1000_GetFreeHeap());
                g_workerStarted = false;
                g_workerThread = NULL;
                osThreadTerminate(osThreadGetId());
                while (1)
                {
                    osDelay(10000);
                }
            }
        }

        osDelay(10000);
    }
}

static void OpenOPL1000_StartWorkerOnce(void)
{
    osThreadDef_t threadDef;

    if (g_workerStarted)
    {
        return;
    }

    memset(&threadDef, 0, sizeof(threadDef));
    threadDef.name = "opl_wifi";
    threadDef.stacksize = OS_TASK_STACK_SIZE_APP;
    threadDef.tpriority = OS_TASK_PRIORITY_APP;
    threadDef.instances = 0;
    threadDef.pthread = OpenOPL1000_WifiWorker;

    g_workerThread = osThreadCreate(&threadDef, NULL);
    g_workerStarted = (g_workerThread != NULL);

    printf("[OpenOPL1000] worker task create %s, heap=%u\r\n",
           g_workerStarted ? "ok" : "failed",
           OpenOPL1000_GetFreeHeap());
}

static int32_t OpenOPL1000_DefaultStaConnectedHandler(wifi_event_t event, uint8_t *data, uint32_t length)
{
    (void)event;
    (void)length;
    return wifi_station_connected_event_handler(data);
}

static int32_t OpenOPL1000_DefaultStaDisconnectedHandler(wifi_event_t event, uint8_t *data, uint32_t length)
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
            printf("[OpenOPL1000] event: STA_START\r\n");
            break;

        case WIFI_EVENT_SCAN_COMPLETE:
            printf("[OpenOPL1000] event: SCAN_COMPLETE\r\n");
            break;

        case WIFI_EVENT_STA_CONNECTED:
            printf("[OpenOPL1000] event: STA_CONNECTED\r\n");
            g_associated = true;
            OpenOPL1000_ReportStatus(WIFI_STA_CONNECTING);
            break;

        case WIFI_EVENT_STA_GOT_IP:
            printf("[OpenOPL1000] event: STA_GOT_IP\r\n");
            g_haveIp = true;
            OpenOPL1000_UpdateNetifStrings();
            printf("[OpenOPL1000] event: got IP %s\r\n", g_ip);
            OpenOPL1000_ReportStatus(WIFI_STA_CONNECTED);
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            printf("[OpenOPL1000] event: STA_DISCONNECTED\r\n");
            g_associated = false;
            g_haveIp = false;
            g_lwipStarted = false;
            OpenOPL1000_ReportStatus(WIFI_STA_DISCONNECTED);
            break;

        case WIFI_EVENT_STA_CONNECTION_FAILED:
            printf("[OpenOPL1000] event: STA_CONNECTION_FAILED\r\n");
            g_associated = false;
            g_haveIp = false;
            g_lwipStarted = false;
            OpenOPL1000_ReportStatus(WIFI_STA_AUTH_FAILED);
            break;

        default:
            printf("[OpenOPL1000] event: %d\r\n", eventId);
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
    wifiConfig.sta_config.bssid_present = 0;
    wifiConfig.sta_config.scan_method = WIFI_ALL_CHANNEL_SCAN;
    wifiConfig.sta_config.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    wifiConfig.sta_config.threshold.authmode = WIFI_AUTH_OPEN;
    wifiConfig.sta_config.threshold.rssi = -127;

    rc = wifi_set_config(WIFI_MODE_STA, &wifiConfig);
    printf("[OpenOPL1000] wifi_set_config rc=%d ssid_len=%u pass_len=%u\r\n",
           rc,
           (unsigned int)wifiConfig.sta_config.ssid_length,
           (unsigned int)wifiConfig.sta_config.password_length);

    if (!g_wifiStarted)
    {
        rc = wifi_start();
        g_wifiStarted = (rc == 0);
        printf("[OpenOPL1000] wifi_start rc=%d heap=%u\r\n", rc, OpenOPL1000_GetFreeHeap());
    }
    else
    {
        printf("[OpenOPL1000] wifi_start skipped; already started, heap=%u\r\n", OpenOPL1000_GetFreeHeap());
    }

    OpenOPL1000_StartWorkerOnce();
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
    g_associated = false;
    g_haveIp = false;
    g_lwipStarted = false;
    wifi_connection_disconnect_ap();
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
    uint8_t candidate[6];

    if (mac == NULL)
    {
        return;
    }

    memcpy(mac, fallback, sizeof(fallback));

    memset(candidate, 0, sizeof(candidate));
    if (wifi_config_get_mac_address(WIFI_MODE_STA, candidate) == 0 &&
        OpenOPL1000_IsUsableMac(candidate))
    {
        memcpy(mac, candidate, sizeof(candidate));
        return;
    }

    if (wifi_get_mac_addr)
    {
        memset(candidate, 0, sizeof(candidate));
        wifi_get_mac_addr(candidate);
        if (OpenOPL1000_IsUsableMac(candidate))
        {
            memcpy(mac, candidate, sizeof(candidate));
        }
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
#if OPENOPL1000_NETINFO_TRACE
    printf("[OpenOPL1000] IP=%s GW=%s MASK=%s\r\n", HAL_GetMyIPString(), HAL_GetMyGatewayString(), HAL_GetMyMaskString());
#endif
}

int HAL_GetWifiStrength(void)
{
    int8_t rssi = 0;
    wifi_ap_record_t apInfo;

    if (wifi_connection_get_rssi(&rssi) == 0)
    {
        return rssi;
    }

    memset(&apInfo, 0, sizeof(apInfo));
    if (wifi_sta_get_ap_info(&apInfo) == 0 && apInfo.ssid_length > 0)
    {
        return apInfo.rssi;
    }

    return -127;
}

#endif

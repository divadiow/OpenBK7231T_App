#if PLATFORM_MT7687

#include "hal.h"
#include "top.h"
#include "mt7687.h"
#include "system_mt7687.h"
#include "wifi_api.h"
#include "lwip/inet.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "lwip/dns.h"
#include "lwip/tcpip.h"
#include "connsys_profile.h"
#include "ethernetif.h"
#include "dhcp.h"
#include "dhcpd.h"
#include "wifi_inband.h"
#include "wifi_scan.h"
#include "event_groups.h"
#include "semphr.h"
#include <easyflash.h>
#include "../../new_common.h"
#include "../../new_cfg.h"
#include "../../new_pins.h"
#include "../hal_wifi.h"
#include "../../logging/logging.h"

extern struct netif sta_if, ap_if;
extern uint8_t g_mac[6];
static void (*g_wifiStatusCallback)(int code);
static bool g_bAccessPointMode = false;
static EventGroupHandle_t g_wifiEvents;
static SemaphoreHandle_t g_scanLock;
static SemaphoreHandle_t g_scanDone;
#define MT_WIFI_READY_BIT (1U << 0)
#define MT_WIFI_PORT_SECURE_BIT (1U << 1)

static int32_t wifi_init_done(wifi_event_t event, uint8_t* payload, uint32_t length)
{
	xEventGroupSetBits(g_wifiEvents, MT_WIFI_READY_BIT);
	return 1;
}

static int32_t wifi_scan_done(wifi_event_t event, uint8_t* payload, uint32_t length)
{
	xSemaphoreGive(g_scanDone);
	return 1;
}

void HAL_MT7687_RegisterWiFiReady(void)
{
	g_wifiEvents = xEventGroupCreate();
	if(g_wifiEvents == NULL ||
		wifi_connection_register_event_handler(WIFI_EVENT_IOT_INIT_COMPLETE, wifi_init_done) < 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "WiFi: could not register ready event");
	}
	g_scanLock = xSemaphoreCreateMutex();
	g_scanDone = xSemaphoreCreateBinary();
	if(g_scanLock != NULL && g_scanDone != NULL &&
		wifi_connection_register_event_handler(WIFI_EVENT_IOT_SCAN_COMPLETE, wifi_scan_done) < 0)
	{
		vSemaphoreDelete(g_scanDone);
		g_scanDone = NULL;
	}
}

static bool wait_for_wifi_ready(void)
{
	// Let the supplicant task finish before issuing Wi-Fi commands.
	if(g_wifiEvents != NULL &&
		(xEventGroupWaitBits(g_wifiEvents, MT_WIFI_READY_BIT, pdFALSE, pdTRUE,
			pdMS_TO_TICKS(5000)) & MT_WIFI_READY_BIT))
	{
		return true;
	}
	ADDLOG_ERROR(LOG_FEATURE_GENERAL, "WiFi: SDK initialization timed out");
	return false;
}
static char g_ipStr[16] = "unknown";
static char g_gwStr[16] = "unknown";
static char g_maskStr[16] = "unknown";
static char g_dnsStr[16] = "unknown";
static obkStaticIP_t g_staticIP;

#define MT_FAST_CONNECT_KEY "mt_wifi_ap"
#define MT_FAST_CONNECT_TIMEOUT 8

typedef struct {
	char ssid[WIFI_MAX_LENGTH_OF_SSID + 1];
	uint8_t bssid[6];
} mtFastConnect_t;

static mtFastConnect_t g_cachedAP;
static char g_currentSSID[WIFI_MAX_LENGTH_OF_SSID + 1];
static bool g_cacheLoaded;
static bool g_fastConnectTried;
static int g_fastConnectTimeout;

extern void InitEasyFlashIfNeeded(void);
extern void Main_ConnectToWiFiNow(void);

static void load_cached_ap(void)
{
	if(!g_cacheLoaded)
	{
		InitEasyFlashIfNeeded();
		if(ef_get_env_blob(MT_FAST_CONNECT_KEY, &g_cachedAP, sizeof(g_cachedAP), NULL) != sizeof(g_cachedAP))
		{
			memset(&g_cachedAP, 0, sizeof(g_cachedAP));
		}
		g_cachedAP.ssid[WIFI_MAX_LENGTH_OF_SSID] = '\0';
		g_cacheLoaded = true;
	}
}

static bool valid_bssid(const uint8_t* bssid)
{
	static const uint8_t zero[6];
	return !(bssid[0] & 1) && memcmp(bssid, zero, sizeof(zero)) != 0;
}

void HAL_DisableEnhancedFastConnect(void)
{
	g_fastConnectTried = true;
	g_fastConnectTimeout = 0;
}

void HAL_MT7687_WiFiTick(void)
{
	static bool saved;
	if(g_bAccessPointMode || !Main_IsConnectedToWiFi())
	{
		saved = false;
		if(!g_bAccessPointMode && g_fastConnectTimeout > 0 &&
			!(xEventGroupGetBits(g_wifiEvents) & MT_WIFI_PORT_SECURE_BIT) && --g_fastConnectTimeout == 0)
		{
			// Retry without the cached BSSID if the AP has moved or disappeared.
			Main_ConnectToWiFiNow();
		}
		return;
	}

	g_fastConnectTimeout = 0;
	if(saved || !CFG_HasFlag(OBK_FLAG_WIFI_ENHANCED_FAST_CONNECT) || !g_currentSSID[0]) return;

	uint8_t bssid[6] = { 0 };
	if(wifi_config_get_bssid(bssid) >= 0 && valid_bssid(bssid))
	{
		load_cached_ap();
		if(strcmp(g_currentSSID, g_cachedAP.ssid) != 0 || memcmp(bssid, g_cachedAP.bssid, sizeof(bssid)) != 0)
		{
			mtFastConnect_t updated = { 0 };
			strcpy(updated.ssid, g_currentSSID);
			memcpy(updated.bssid, bssid, sizeof(bssid));
			if(ef_set_env_blob(MT_FAST_CONNECT_KEY, &updated, sizeof(updated)) == EF_NO_ERR)
			{
				g_cachedAP = updated;
			}
		}
		saved = true;
	}
}

static void apply_sta_ip(void* arg)
{
	if(g_bAccessPointMode) return;

	netif_set_link_up(&sta_if);
	if(g_staticIP.localIPAddr[0] || g_staticIP.localIPAddr[1] ||
		g_staticIP.localIPAddr[2] || g_staticIP.localIPAddr[3])
	{
		ip4_addr_t address, mask, gateway;
		ip_addr_t dns;
		IP4_ADDR(&address, g_staticIP.localIPAddr[0], g_staticIP.localIPAddr[1],
			g_staticIP.localIPAddr[2], g_staticIP.localIPAddr[3]);
		IP4_ADDR(&mask, g_staticIP.netMask[0], g_staticIP.netMask[1],
			g_staticIP.netMask[2], g_staticIP.netMask[3]);
		IP4_ADDR(&gateway, g_staticIP.gatewayIPAddr[0], g_staticIP.gatewayIPAddr[1],
			g_staticIP.gatewayIPAddr[2], g_staticIP.gatewayIPAddr[3]);
		IP_ADDR4(&dns, g_staticIP.dnsServerIpAddr[0], g_staticIP.dnsServerIpAddr[1],
			g_staticIP.dnsServerIpAddr[2], g_staticIP.dnsServerIpAddr[3]);
		for(uint8_t i = 0; i < DNS_MAX_SERVERS; i++)
		{
			dns_setserver(i, i == 0 ? &dns : NULL);
		}
		netif_set_addr(&sta_if, &address, &mask, &gateway);
	}
	else if(dhcp_start(&sta_if) != ERR_OK)
	{
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "WiFi: could not start DHCP");
		if(g_wifiStatusCallback != NULL) g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
	}
}

int HAL_MT7687_Scan(wifi_scan_list_item_t* results, uint32_t count)
{
	if(g_scanLock == NULL || g_scanDone == NULL || !wait_for_wifi_ready() ||
		xSemaphoreTake(g_scanLock, 0) != pdTRUE)
	{
		return -1;
	}

	uint8_t opmode;
	int result = -1;
	if(wifi_config_get_opmode(&opmode) >= 0 &&
		(opmode == WIFI_MODE_STA_ONLY || opmode == WIFI_MODE_AP_ONLY))
	{
		wifi_connection_stop_scan();
		xSemaphoreTake(g_scanDone, 0);
		if(wifi_connection_scan_init(results, count) >= 0)
		{
			// Partial scans return to the SoftAP channel between channel groups.
			if(wifi_connection_start_scan(NULL, 0, NULL, opmode == WIFI_MODE_AP_ONLY, 0) == 0)
			{
				result = 0;
				if(xSemaphoreTake(g_scanDone, pdMS_TO_TICKS(15000)) != pdTRUE)
				{
					wifi_connection_stop_scan();
					result = -2;
				}
			}
			wifi_connection_scan_deinit();
		}
	}
	xSemaphoreGive(g_scanLock);
	return result;
}

void WiFI_GetMacAddress(char* mac)
{
	memcpy(mac, g_mac, 6);
}

const char* HAL_GetMyIPString()
{
	struct netif* netif = g_bAccessPointMode ? &ap_if : &sta_if;
	strcpy(g_ipStr, ip4addr_ntoa(&netif->ip_addr));
	strcpy(g_maskStr, ip4addr_ntoa(&netif->netmask));
	strcpy(g_gwStr, ip4addr_ntoa(&netif->gw));
	return g_ipStr;
}

const char* HAL_GetMyGatewayString()
{
	return g_gwStr;
}

const char* HAL_GetMyMaskString()
{
	return g_maskStr;
}

const char* HAL_GetMyDNSString()
{
	if(g_bAccessPointMode) return "unknown";
	const ip_addr_t* dns = dns_getserver(0);
	return ipaddr_ntoa_r(dns, g_dnsStr, sizeof(g_dnsStr));
}

const char* HAL_GetMACStr(char* macstr)
{
	sprintf(macstr, MACSTR, MAC2STR(g_mac));
	return macstr;
}

int HAL_GetWifiStrength()
{
	int8_t rssi;
	wifi_connection_get_rssi(&rssi);
	return rssi;
}

void HAL_WiFi_SetupStatusCallback(void (*cb)(int code))
{
	g_wifiStatusCallback = cb;
}

int32_t sta_evt(wifi_event_t event, uint8_t* payload, uint32_t length)
{
	switch(event)
	{
		case WIFI_EVENT_IOT_CONNECTED:
			ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "WiFi connected");
			break;
		case WIFI_EVENT_IOT_PORT_SECURE:
			ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "WiFi handshake finished");
			if(g_wifiEvents != NULL) xEventGroupSetBits(g_wifiEvents, MT_WIFI_PORT_SECURE_BIT);
			if(!g_bAccessPointMode && tcpip_callback(apply_sta_ip, NULL) != ERR_OK)
			{
				ADDLOG_ERROR(LOG_FEATURE_GENERAL, "WiFi: could not schedule IP setup");
				if(g_wifiStatusCallback != NULL) g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
			}
			break;
		case WIFI_EVENT_IOT_DISCONNECTED:
			if(g_wifiEvents != NULL) xEventGroupClearBits(g_wifiEvents, MT_WIFI_PORT_SECURE_BIT);
			netifapi_dhcp_stop(&sta_if);
			netifapi_netif_set_link_down(&sta_if);
			netifapi_netif_set_addr(&sta_if, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY);
			if(!g_bAccessPointMode && g_wifiStatusCallback != NULL)
			{
				g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
			}
			break;
		case WIFI_EVENT_IOT_CONNECTION_FAILED:
		{
			if(payload == NULL || length < 3 || payload[0] != WIFI_PORT_STA)
			{
				break;
			}
			uint16_t reason_code = payload[1] | (payload[2] << 8);
			if(g_wifiStatusCallback != NULL)
			{
				if(reason_code == WIFI_REASON_CODE_PREV_AUTH_INVALID || reason_code == WIFI_REASON_CODE_PASSWORD_ERROR)
				{
					g_wifiStatusCallback(WIFI_STA_AUTH_FAILED);
				}
				else
				{
					g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
				}
			}
			break;
		}
	}
	return 1;
}

static void ip_ready_callback(struct netif* netif)
{
	if(!g_bAccessPointMode && netif_is_link_up(netif) &&
		!ip4_addr_isany_val(netif->ip_addr) && g_wifiStatusCallback != NULL)
	{
		g_wifiStatusCallback(WIFI_STA_CONNECTED);
	}
}

static void install_sta_status_callback(struct netif* netif)
{
	netif_set_status_callback(netif, ip_ready_callback);
}

static void connect_to_wifi(const char* ssid, const char* pwd, obkStaticIP_t* ip, uint8_t* bssid)
{
	g_fastConnectTimeout = 0;
	if(g_wifiEvents != NULL) xEventGroupClearBits(g_wifiEvents, MT_WIFI_PORT_SECURE_BIT);
	snprintf(g_currentSSID, sizeof(g_currentSSID), "%s", ssid);
	if(g_wifiStatusCallback != 0)
	{
		g_wifiStatusCallback(WIFI_STA_CONNECTING);
	}
	if(!wait_for_wifi_ready())
	{
		if(g_wifiStatusCallback != NULL) g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
		return;
	}
	if(g_bAccessPointMode)
	{
		dhcpd_stop();
		netifapi_netif_set_link_down(&ap_if);
	}
	g_bAccessPointMode = false;
	netifapi_netif_set_default(&sta_if);
	netifapi_dhcp_stop(&sta_if);
	netifapi_netif_set_link_down(&sta_if);
	if(ip != NULL) g_staticIP = *ip;
	else memset(&g_staticIP, 0, sizeof(g_staticIP));
	wifi_connection_register_event_handler(WIFI_EVENT_IOT_CONNECTED, sta_evt);
	wifi_connection_register_event_handler(WIFI_EVENT_IOT_PORT_SECURE, sta_evt);
	wifi_connection_register_event_handler(WIFI_EVENT_IOT_DISCONNECTED, sta_evt);
	wifi_connection_register_event_handler(WIFI_EVENT_IOT_CONNECTION_FAILED, sta_evt);
	uint8_t pwd_len = (uint8_t)strlen(pwd);
	netifapi_netif_set_addr(&sta_if, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY);
	int32_t result = netifapi_netif_common(&sta_if, install_sta_status_callback, NULL);
	if(result == ERR_OK) result = wifi_config_set_opmode(WIFI_MODE_STA_ONLY);
	if(result >= 0 && wifi_5g_support() >= 0)
	{
		int32_t phy_result = wifi_config_set_wireless_mode(WIFI_PORT_STA, WIFI_PHY_11ABGN_MIXED);
		if(phy_result < 0)
		{
			ADDLOG_ERROR(LOG_FEATURE_GENERAL, "WiFi: dual-band STA mode failed (%d)", (int)phy_result);
		}
	}
	if(result < 0 ||
		wifi_config_set_ssid(WIFI_PORT_STA, (uint8_t*)ssid, strlen(ssid)) < 0 ||
		(pwd_len > 0 && wifi_config_set_wpa_psk_key(WIFI_PORT_STA, (uint8_t*)pwd, pwd_len) < 0))
	{
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "WiFi: STA configuration failed");
		if(g_wifiStatusCallback != NULL) g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
		return;
	}
	if(bssid != NULL && wifi_config_set_bssid(bssid) >= 0)
	{
		g_fastConnectTimeout = MT_FAST_CONNECT_TIMEOUT;
	}
	if(wifi_config_reload_setting() < 0)
	{
		g_fastConnectTimeout = 0;
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "WiFi: STA reload failed");
		if(g_wifiStatusCallback != NULL) g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
	}
}

void HAL_ConnectToWiFi(const char* ssid, const char* pwd, obkStaticIP_t* ip)
{
	connect_to_wifi(ssid, pwd, ip, NULL);
}

void HAL_FastConnectToWiFi(const char* ssid, const char* pwd, obkStaticIP_t* ip)
{
	uint8_t* bssid = NULL;
	if(!g_fastConnectTried)
	{
		load_cached_ap();
		if(strcmp(ssid, g_cachedAP.ssid) == 0 && valid_bssid(g_cachedAP.bssid))
		{
			bssid = g_cachedAP.bssid;
			g_fastConnectTried = true;
		}
	}
	connect_to_wifi(ssid, pwd, ip, bssid);
}

void HAL_DisconnectFromWifi()
{
	g_fastConnectTimeout = 0;
	wifi_connection_disconnect_ap();
}

int HAL_SetupWiFiOpenAccessPoint(const char* ssid)
{
	if(!wait_for_wifi_ready()) return -1;
	g_bAccessPointMode = true;
	netifapi_dhcp_stop(&sta_if);
	netifapi_netif_set_link_down(&sta_if);
	netifapi_netif_set_addr(&sta_if, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY);

	if(wifi_config_set_opmode(WIFI_MODE_AP_ONLY) < 0 ||
		wifi_config_set_security_mode(WIFI_PORT_AP, WIFI_AUTH_MODE_OPEN, WIFI_ENCRYPT_TYPE_WEP_DISABLED) < 0 ||
		wifi_config_set_channel(WIFI_PORT_AP, 1) < 0 ||
		wifi_config_set_ssid(WIFI_PORT_AP, (uint8_t*)ssid, strlen(ssid)) < 0 ||
		wifi_config_reload_setting() < 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "WiFi: SoftAP configuration failed");
		g_bAccessPointMode = false;
		return -1;
	}
	dhcpd_settings_t dhcpd_settings = { 0 };
	strcpy(dhcpd_settings.dhcpd_server_address, "192.168.4.1");
	strcpy(dhcpd_settings.dhcpd_gateway, "192.168.4.1");
	strcpy(dhcpd_settings.dhcpd_netmask, "255.255.255.0");
	strcpy(dhcpd_settings.dhcpd_primary_dns, "192.168.4.1");
	strcpy(dhcpd_settings.dhcpd_secondary_dns, "0.0.0.0");
	strcpy(dhcpd_settings.dhcpd_ip_pool_start, "192.168.4.100");
	strcpy(dhcpd_settings.dhcpd_ip_pool_end, "192.168.4.110");
	netifapi_netif_set_default(&ap_if);
	ip4_addr_t ipaddr;
	ip4_addr_t netmask;
	ip4_addr_t gateway;

	IP4_ADDR(&ipaddr, 192, 168, 4, 1);
	IP4_ADDR(&netmask, 255, 255, 255, 0);
	IP4_ADDR(&gateway, 192, 168, 4, 1);

	netifapi_netif_set_addr(&ap_if, &ipaddr, &netmask, &gateway);
	netifapi_netif_set_link_up(&ap_if);
	return dhcpd_start(&dhcpd_settings);
}

void HAL_PrintNetworkInfo()
{
	uint8_t mac[6];
	WiFI_GetMacAddress((char*)mac);
	ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "+--------------- net device info ------------+");
	ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "|netif type    : %-16s            |", g_bAccessPointMode == false ? "STA" : "AP");
	ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "|netif rssi    = %-16i            |", HAL_GetWifiStrength());
	ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "|netif ip      = %-16s            |", HAL_GetMyIPString());
	ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "|netif mask    = %-16s            |", HAL_GetMyMaskString());
	ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "|netif gateway = %-16s            |", HAL_GetMyGatewayString());
	ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "|netif mac     : "MACSTR" %-6s    |", MAC2STR(g_mac), "");
	ADDLOG_DEBUG(LOG_FEATURE_GENERAL, "+--------------------------------------------+");
}

#endif

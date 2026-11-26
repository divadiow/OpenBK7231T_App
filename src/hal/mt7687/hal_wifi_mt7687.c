#if PLATFORM_MT7687

#include "hal.h"
#include "top.h"
#include "mt7687.h"
#include "system_mt7687.h"
#include "wifi_api.h"
#include "lwip/inet.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "connsys_profile.h"
#include "ethernetif.h"
#include "dhcp.h"
#include "dhcpd.h"
#include "wifi_api.h"
#include "wifi_scan.h"
#include "event_groups.h"
#include "../hal_wifi.h"
#include "../../logging/logging.h"

extern struct netif sta_if, ap_if;
extern uint8_t g_mac[6];
static void (*g_wifiStatusCallback)(int code);
bool g_bAccessPointMode = false;
static EventGroupHandle_t g_wifiEvents;
#define MT_WIFI_READY_BIT (1U << 0)

static int32_t wifi_init_done(wifi_event_t event, uint8_t* payload, uint32_t length)
{
	xEventGroupSetBits(g_wifiEvents, MT_WIFI_READY_BIT);
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
	return g_dnsStr;
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
			if(!g_bAccessPointMode)
			{
				netifapi_netif_set_link_up(&sta_if);
				if(netifapi_dhcp_start(&sta_if) != ERR_OK)
				{
					ADDLOG_ERROR(LOG_FEATURE_GENERAL, "WiFi: could not start DHCP");
					if(g_wifiStatusCallback != NULL) g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
				}
			}
			break;
		case WIFI_EVENT_IOT_DISCONNECTED:
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

void HAL_ConnectToWiFi(const char* ssid, const char* pwd, obkStaticIP_t* ip)
{
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
	wifi_connection_register_event_handler(WIFI_EVENT_IOT_CONNECTED, sta_evt);
	wifi_connection_register_event_handler(WIFI_EVENT_IOT_PORT_SECURE, sta_evt);
	wifi_connection_register_event_handler(WIFI_EVENT_IOT_DISCONNECTED, sta_evt);
	wifi_connection_register_event_handler(WIFI_EVENT_IOT_CONNECTION_FAILED, sta_evt);
	uint8_t pwd_len = (uint8_t)strlen(pwd);
	netifapi_netif_set_addr(&sta_if, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY);
	if(netifapi_netif_common(&sta_if, install_sta_status_callback, NULL) != ERR_OK ||
		wifi_config_set_opmode(WIFI_MODE_STA_ONLY) < 0 ||
		wifi_config_set_ssid(WIFI_PORT_STA, (uint8_t*)ssid, strlen(ssid)) < 0 ||
		(pwd_len > 0 && wifi_config_set_wpa_psk_key(WIFI_PORT_STA, (uint8_t*)pwd, pwd_len) < 0) ||
		wifi_config_reload_setting() < 0)
	{
		ADDLOG_ERROR(LOG_FEATURE_GENERAL, "WiFi: STA configuration failed");
		if(g_wifiStatusCallback != NULL) g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
	}
}

void HAL_DisconnectFromWifi()
{
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

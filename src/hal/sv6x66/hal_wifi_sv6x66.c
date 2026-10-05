#if defined(PLATFORM_SV6X66)

#include "../hal_wifi.h"

#include <stdio.h>
#include <string.h>

#include "wifi_api.h"
#include "wificonf.h"
#include "softap_func.h"
#include "netstack.h"

/* The SDK implementation exports this, but its public header omits it. */
extern int softap_set_custom_conf(SOFTAP_CUSTOM_CONFIG *config);
extern IEEE80211STATUS gwifistatus;

static void (*g_wifiStatusCallback)(int code);
static int g_apMode;
static char g_ipString[16], g_gatewayString[16], g_maskString[16], g_dnsString[16];

static u32 ip_to_sdk(const unsigned char bytes[4])
{
	u32 value;
	memcpy(&value, bytes, sizeof(value));
	return value;
}

static void format_ip(char *dst, const u8 bytes[4])
{
	snprintf(dst, 16, "%u.%u.%u.%u", bytes[0], bytes[1], bytes[2], bytes[3]);
}

static int get_interface_config(const char *name, u8 *dhcp, u32 *ip, u32 *mask,
								u32 *gateway, u32 *dns, u8 *mac)
{
	return get_if_config_2((char *)name, dhcp, ip, mask, gateway, dns, mac, 6);
}

static void update_network_strings(void)
{
	u8 dhcp = 0, mac[6];
	u32 ip = 0, mask = 0, gateway = 0, dns = 0;
	union { u32 value; u8 octet[4]; } a, b, c, d;

	if (get_interface_config(g_apMode ? IF1_NAME : IF0_NAME, &dhcp,
							 &ip, &mask, &gateway, &dns, mac) != 0)
	{
		strcpy(g_ipString, "0.0.0.0");
		strcpy(g_maskString, "0.0.0.0");
		strcpy(g_gatewayString, "0.0.0.0");
		strcpy(g_dnsString, "0.0.0.0");
		return;
	}
	a.value = ip; b.value = mask; c.value = gateway; d.value = dns;
	format_ip(g_ipString, a.octet);
	format_ip(g_maskString, b.octet);
	format_ip(g_gatewayString, c.octet);
	format_ip(g_dnsString, d.octet);
}

static void wifi_response_callback(WIFI_RSP *response)
{
	int status;
	if (response == NULL)
		return;
	if (response->wifistatus == 1)
	{
		status = WIFI_STA_CONNECTED;
		update_network_strings();
	}
	else if (response->reason == WRONG_PASSPHRASE || response->reason == UNSUPPORT_ENCRYPT)
		status = WIFI_STA_AUTH_FAILED;
	else
		status = WIFI_STA_DISCONNECTED;
	if (g_wifiStatusCallback != NULL)
		g_wifiStatusCallback(status);
}

static void softap_station_callback(STAINFO *station)
{
	if (station == NULL || g_wifiStatusCallback == NULL)
		return;
	g_wifiStatusCallback(station->stastatus == STA_CONN ? WIFI_AP_CONNECTED : WIFI_AP_FAILED);
}

static void notify_ap_failure(void)
{
	if (g_wifiStatusCallback != NULL)
		g_wifiStatusCallback(WIFI_AP_FAILED);
}

int HAL_SetupWiFiOpenAccessPoint(const char *ssid)
{
	SOFTAP_CUSTOM_CONFIG config;
	size_t length;
	int result;
	if (ssid == NULL)
	{
		notify_ap_failure();
		return -1;
	}
	length = strlen(ssid);
	if (length == 0 || length > sizeof(config.ssid))
	{
		notify_ap_failure();
		return -1;
	}

	memset(&config, 0, sizeof(config));
	config.start_ip = 0xC0A80402; /* 192.168.4.2 in the SDK's CLI format */
	config.max_sta_num = 4;
	/* The SDK rejects a DHCP range larger than its lease allocation. */
	config.end_ip = config.start_ip + config.max_sta_num - 1;
	config.gw = 0xC0A80401;
	config.subnet = 0xFFFFFF00;
	config.encryt_mode = 0;
	config.channel = 1;
	config.beacon_interval = 100;
	config.ssid_length = (s8)length;
	memcpy(config.ssid, ssid, length);
	result = softap_set_custom_conf(&config);
	if (result != 0)
	{
		notify_ap_failure();
		return result;
	}
	result = wifi_register_softap_cb(softap_station_callback);
	if (result != 0)
	{
		notify_ap_failure();
		return result;
	}
	result = DUT_wifi_start(DUT_AP);
	if (result != 0)
	{
		notify_ap_failure();
		return result;
	}
	g_apMode = 1;
	update_network_strings();
	return 0;
}

void HAL_ConnectToWiFi(const char *ssid, const char *key, obkStaticIP_t *ip)
{
	size_t ssid_length, key_length;
	u8 dhcp, ssid_len, key_len;
	u32 ipaddr = 0, mask = 0, gateway = 0, dns = 0;
	int result;
	if (ssid == NULL)
	{
		if (g_wifiStatusCallback != NULL)
			g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
		return;
	}
	ssid_length = strlen(ssid);
	key_length = key != NULL ? strlen(key) : 0;
	if (ssid_length == 0 || ssid_length > 32 || key_length > 64)
	{
		if (g_wifiStatusCallback != NULL)
			g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
		return;
	}
	ssid_len = (u8)ssid_length;
	key_len = (u8)key_length;
	if (ip != NULL)
	{
		dhcp = 0;
		ipaddr = ip_to_sdk(ip->localIPAddr);
		mask = ip_to_sdk(ip->netMask);
		gateway = ip_to_sdk(ip->gatewayIPAddr);
		dns = ip_to_sdk(ip->dnsServerIpAddr);
	}
	else
		dhcp = 1;
	g_apMode = 0;
	if (get_DUT_wifi_mode() != DUT_STA && DUT_wifi_start(DUT_STA) != 0)
	{
		if (g_wifiStatusCallback != NULL)
			g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
		return;
	}
	if (set_if_config_2(0, dhcp, ipaddr, mask, gateway, dns) != 0)
	{
		if (g_wifiStatusCallback != NULL)
			g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
		return;
	}
	if (g_wifiStatusCallback != NULL)
		g_wifiStatusCallback(WIFI_STA_CONNECTING);
	result = wifi_connect_active_5((u8 *)ssid, ssid_len, (u8 *)key, key_len,
								  NET80211_CRYPT_UNKNOWN, 0, NULL, 0, 0,
								  wifi_response_callback);
	if (result != 0 && g_wifiStatusCallback != NULL)
		g_wifiStatusCallback(WIFI_STA_DISCONNECTED);
}

void HAL_FastConnectToWiFi(const char *ssid, const char *key, obkStaticIP_t *ip)
{
	HAL_ConnectToWiFi(ssid, key, ip);
}

void HAL_DisableEnhancedFastConnect(void) { }
void HAL_DisconnectFromWifi(void) { wifi_disconnect(wifi_response_callback); }
void HAL_WiFi_SetupStatusCallback(void (*cb)(int code)) { g_wifiStatusCallback = cb; }

const char *HAL_GetMyIPString(void) { update_network_strings(); return g_ipString; }
const char *HAL_GetMyGatewayString(void) { update_network_strings(); return g_gatewayString; }
const char *HAL_GetMyDNSString(void) { update_network_strings(); return g_dnsString; }
const char *HAL_GetMyMaskString(void) { update_network_strings(); return g_maskString; }

void WiFI_GetMacAddress(char *mac)
{
	u8 dhcp = 0, address[6] = {0};
	u32 ip, mask, gateway, dns;
	if (mac == NULL)
		return;
	if (get_interface_config(g_apMode ? IF1_NAME : IF0_NAME, &dhcp,
							 &ip, &mask, &gateway, &dns, address) == 0)
		memcpy(mac, address, sizeof(address));
	else
		memset(mac, 0, 6);
}

const char *HAL_GetMACStr(char *macstr)
{
	u8 mac[6];
	if (macstr == NULL)
		return NULL;
	WiFI_GetMacAddress((char *)mac);
	snprintf(macstr, 18, MACSTR, MAC2STR(mac));
	return macstr;
}

char *HAL_GetWiFiBSSID(char *bssid)
{
	u8 ssid[32], ssid_len = sizeof(ssid), mac[6], rssi, channel;
	if (bssid == NULL)
		return NULL;
	bssid[0] = '\0';
	if (g_apMode || get_wifi_status() != 1)
		return bssid;
	if (get_connectap_info(0, ssid, &ssid_len, mac, (u8)sizeof(mac), &rssi, &channel) == 0)
		snprintf(bssid, 18, MACSTR, MAC2STR(mac));
	return bssid;
}

uint8_t HAL_GetWiFiChannel(uint8_t *channel)
{
	if (channel == NULL)
		return 0;
	if (g_apMode)
		*channel = gwifistatus.run_channel;
	else
	{
		u8 ssid[32], ssid_len = sizeof(ssid), mac[6], rssi;
		*channel = 0;
		if (get_wifi_status() == 1)
			(void)get_connectap_info(0, ssid, &ssid_len, mac, (u8)sizeof(mac), &rssi, channel);
	}
	return *channel;
}

int WiFI_SetMacAddress(char *mac) { (void)mac; return -1; }
void HAL_PrintNetworkInfo(void) { (void)HAL_GetMyIPString(); }
int HAL_GetWifiStrength(void)
{
	u8 ssid[32], ssid_len = sizeof(ssid), mac[6], rssi, channel;
	if (g_apMode || get_wifi_status() != 1)
		return 0;
	if (get_connectap_info(0, ssid, &ssid_len, mac, (u8)sizeof(mac), &rssi, &channel) != 0)
		return 0;
	return -(int)rssi;
}

#endif /* PLATFORM_SV6X66 */

#include "sv6x66_platform.h"
#include "new_common.h"
#include "hal/hal_wifi.h"
#include "wifi_api.h"
#include "netstack.h"
#include "sta_func.h"
#include "softap_func.h"
#include "security/drv_security.h"
#include "rf/rf_api.h"
#include <string.h>
#include <stdio.h>

/* Exported by SDK libssvradio; its public SOFTAP_CUSTOM_CONFIG is also used
	* by SDK mac_atcmd CLI. No vendor function body is copied here. */
extern int softap_set_custom_conf(SOFTAP_CUSTOM_CONFIG *);
static void (*status_callback)(int);
static int access_point,connected=-1,network_ready;
enum {SV_STA_IDLE,SV_STA_PENDING,SV_STA_ACTIVE,SV_STA_FAILED};
static volatile unsigned int sta_state;
enum {SCAN_IDLE,SCAN_WAITING,SCAN_DONE,SCAN_ABANDONED};
static volatile unsigned int scan_state;
static void scan_complete(void *unused)
{
	OS_DeclareCritical();
	(void)unused;
	OS_EnterCritical();
	if(scan_state==SCAN_WAITING)scan_state=SCAN_DONE;
	else if(scan_state==SCAN_ABANDONED)scan_state=SCAN_IDLE;
	OS_ExitCritical();
}
SV6X66_WifiScanStatus SV6X66_WifiScan(SV6X66_WifiScanResult *results,
	unsigned int capacity,unsigned int *count)
{
	static const char *security_names[]={"Open","WEP","WPA","WPA2","WPA/WPA2"};
	TAG_AP_INFO records[SV6X66_WIFI_SCAN_MAX];
	uint32_t deadline;
	unsigned int state,i;
	u8 available;
	WIFI_OPMODE mode;
	SV6X66_WifiScanStatus status=SV6X66_SCAN_FAILED;
	OS_DeclareCritical();
	if(!results || !count || !capacity || capacity>SV6X66_WIFI_SCAN_MAX)return status;
	*count=0;
	if(!network_ready)return status;
	mode=get_DUT_wifi_mode();
	if(mode!=DUT_STA && mode!=DUT_CONCURRENT)return SV6X66_SCAN_UNSUPPORTED;
	OS_EnterCritical();
	state=scan_state;
	if(state==SCAN_IDLE)scan_state=SCAN_WAITING;
	OS_ExitCritical();
	if(state!=SCAN_IDLE)return SV6X66_SCAN_BUSY;
	if(scan_AP(scan_complete))goto done;
	deadline=OS_GetSysTick()+OS_MS2TICK(10000);
	for(;;) {
		int expired=(int32_t)(OS_GetSysTick()-deadline)>=0;
		OS_EnterCritical();
		if(scan_state==SCAN_WAITING && expired)scan_state=SCAN_ABANDONED;
		state=scan_state;
		OS_ExitCritical();
		if(state==SCAN_DONE)break;
		/* A late completion owns the outstanding SDK scan, never this buffer. */
		if(state==SCAN_ABANDONED)return SV6X66_SCAN_TIMEOUT;
		OS_MsDelay(20);
	}
	available=(u8)capacity;
	if(get_ap_lsit_total_num()==0) {status=SV6X66_SCAN_OK;goto done;}
	memset(records,0,sizeof(records));
	if(get_ap_list(records,&available) || available>capacity)goto done;
	for(i=0;i<available;i++) {
		unsigned int length=records[i].name_len;
		if(length>32)length=32;
		memcpy(results[i].ssid,records[i].name,length);
		results[i].ssid[length]=0;
		results[i].channel=records[i].channel;
		results[i].rssi=-(int16_t)records[i].rssi;
		results[i].security=records[i].security_type<=WPAWPA2?
			security_names[records[i].security_type]:"Unknown";
	}
	*count=available;status=SV6X66_SCAN_OK;
done:
	OS_EnterCritical();scan_state=SCAN_IDLE;OS_ExitCritical();
	return status;
}
static int ap_start_seen;
static s32 ap_start_result;
extern s32 __real_softap_start(void);
/* DUT_wifi_start discards this SDK result. Observe its one existing call;
	* never invoke AP startup a second time just to discover the outcome. */
s32 __wrap_softap_start(void)
{
	s32 result=__real_softap_start();
	ap_start_result=result;ap_start_seen=1;
	return result;
}
static u32 ip,mask,gateway,dns;
static char ip_text[16],mask_text[16],gateway_text[16],dns_text[16];
static void format_ip(char *text,u32 value)
{
	const unsigned char *p=(const unsigned char *)&value;
	snprintf(text,16,"%u.%u.%u.%u",p[0],p[1],p[2],p[3]);
}
static void refresh(void)
{
	u8 dhcp;
	if(!access_point && get_if_config(&dhcp,&ip,&mask,&gateway,&dns))ip=mask=gateway=dns=0;
	format_ip(ip_text,ip);format_ip(mask_text,mask);format_ip(gateway_text,gateway);format_ip(dns_text,dns);
}
void SV6X66_WifiPoll(void)
{
	int now,linked;
	OS_DeclareCritical();
	if(!network_ready)return;
	refresh();
	now=1;
	if(!access_point) {
		linked=get_wifi_status();
		OS_EnterCritical();
		now=(sta_state==SV_STA_PENDING || sta_state==SV_STA_ACTIVE) && linked && ip!=0;
		if(now)sta_state=SV_STA_ACTIVE;
		else if(sta_state==SV_STA_PENDING)now=-1;
		else if(sta_state==SV_STA_ACTIVE)sta_state=SV_STA_FAILED;
		OS_ExitCritical();
	}
	if(now<0)return;
	if(now!=connected) {connected=now;if(status_callback)status_callback(access_point?WIFI_AP_CONNECTED:now?WIFI_STA_CONNECTED:WIFI_STA_DISCONNECTED);}
}
static void wifi_response(WIFI_RSP *response)
{
	/* Vendor callback only records state; OBK callback runs in application
		* polling task, after netstack reports an address, not in radio context. */
	OS_DeclareCritical();
	if(!response || response->id!=0)return;
	OS_EnterCritical();
	if(sta_state!=SV_STA_IDLE) {
		if(response->wifistatus!=1)sta_state=SV_STA_FAILED;
		else if(sta_state==SV_STA_FAILED)sta_state=SV_STA_PENDING;
	}
	OS_ExitCritical();
}
void SV6X66_WifiInit(void)
{
	extern int SV6X66_IdentityInit(void);
	extern struct st_rf_table ssv_rf_table;
	typedef char temperature_layout_must_be_18[(sizeof(struct st_tempe_table)==18)?1:-1];
	typedef char rf_gain_offset_must_be_54[(offsetof(struct st_rf_table,rf_gain)==54)?1:-1];
	typedef char rate_b_offset_must_be_55[(offsetof(struct st_rf_table,rate_gain_b)==55)?1:-1];
	typedef char boundary_offset_must_be_68[(offsetof(struct st_rf_table,low_boundary)==68)?1:-1];
	(void)sizeof(temperature_layout_must_be_18);(void)sizeof(rf_gain_offset_must_be_54);
	(void)sizeof(rate_b_offset_must_be_55);(void)sizeof(boundary_offset_must_be_68);
	/* Stock read at3006F6AC copies128 bytes from5000, but SDK struct has a
		* smaller ABI. Only independently mapped2.4GHz fields0..69 are reused;
		* unproved5GHz/defaultfields remain SDK values. Never write factory RF. */
	if(SV6X66_IdentityInit()) {SV6X66_Printf("Factory MAC identity invalid; network unavailable\r\n");return;}
	if(build_default_rf_table(&ssv_rf_table)) {SV6X66_Printf("RF defaults initialization failed; network unavailable\r\n");return;}
	memcpy(&ssv_rf_table,(const void *)(SV6X66_FLASH_BASE+0x5000),70);
	ssv_rf_table.boot_flag=EN_NOT_FIRST_BOOT;ssv_rf_table.work_mode=EN_WORK_NOMAL;
	if(load_rf_table_to_mac(&ssv_rf_table)) {SV6X66_Printf("RF calibration initialization failed; network unavailable\r\n");return;}
	WIFI_INIT();
	if(netstack_init(NULL)) {SV6X66_Printf("netstack initialization failed; network unavailable\r\n");return;}
	init_sta_conf();init_softap_conf();refresh();network_ready=1;
}
void HAL_WiFi_SetupStatusCallback(void (*callback)(int)) {status_callback=callback;}
void HAL_ConnectToWiFi(const char *ssid,const char *key,obkStaticIP_t *config)
{
	u32 local=0,netmask=0,router=0,server=0;
	size_t ssid_length,key_length;
	OS_DeclareCritical();
	if(!ssid || !key || !network_ready) {if(status_callback)status_callback(WIFI_STA_DISCONNECTED);return;}
	ssid_length=strlen(ssid);key_length=strlen(key);
	if(ssid_length>32 || key_length>63)goto failed;
	if(!SV6X66_MDNS_BeforeWifiStop())goto failed;
	OS_EnterCritical();sta_state=SV_STA_IDLE;OS_ExitCritical();
	DUT_wifi_stop();access_point=0;connected=-1;
	ip=mask=gateway=dns=0;format_ip(ip_text,ip);format_ip(mask_text,mask);format_ip(gateway_text,gateway);format_ip(dns_text,dns);
	if(config) {memcpy(&local,config->localIPAddr,4);memcpy(&netmask,config->netMask,4);memcpy(&router,config->gatewayIPAddr,4);memcpy(&server,config->dnsServerIpAddr,4);}
	if(DUT_wifi_start(DUT_STA) || set_if_config(!local,local,netmask,router,server))goto failed;
	OS_EnterCritical();sta_state=SV_STA_PENDING;OS_ExitCritical();
	if(wifi_connect_active((u8 *)ssid,ssid_length,(u8 *)key,key_length,wifi_response))goto failed;
	if(status_callback)status_callback(WIFI_STA_CONNECTING);
	return;
failed:
	OS_EnterCritical();sta_state=SV_STA_FAILED;OS_ExitCritical();
	connected=0;
	if(status_callback)status_callback(WIFI_STA_AUTH_FAILED);
}
void HAL_DisconnectFromWifi(void)
{
	OS_DeclareCritical();
	if(!SV6X66_MDNS_BeforeWifiStop())return;
	OS_EnterCritical();sta_state=SV_STA_IDLE;OS_ExitCritical();
	wifi_disconnect(NULL);DUT_wifi_stop();connected=-1;
}
int HAL_SetupWiFiOpenAccessPoint(const char *ssid)
{
	SOFTAP_CUSTOM_CONFIG config;
	size_t n;
	int result;
	OS_DeclareCritical();
	if(!ssid || !network_ready) {SV6X66_Printf("AP request rejected: network_ready=%d\r\n",network_ready);return -1;}
	n=strlen(ssid);
	if(!n || n>32)return -1;
	if(!SV6X66_MDNS_BeforeWifiStop())return -1;
	OS_EnterCritical();sta_state=SV_STA_IDLE;OS_ExitCritical();
	DUT_wifi_stop();access_point=0;connected=-1;
	ip=mask=gateway=dns=0;format_ip(ip_text,ip);format_ip(mask_text,mask);format_ip(gateway_text,gateway);format_ip(dns_text,dns);
	memset(&config,0,sizeof(config));
	config.start_ip=0xc0a80464;config.end_ip=0xc0a80467;config.gw=0xc0a80401;config.subnet=0xffffff00;
	config.max_sta_num=4;config.channel=1;config.beacon_interval=100;
	config.ssid_length=n;memcpy(config.ssid,ssid,n);
	result=softap_set_custom_conf(&config);
	if(result) {
		SV6X66_Printf("AP config failed: rc=%d mode=%d\r\n",result,(int)get_DUT_wifi_mode());
		if(status_callback)status_callback(WIFI_AP_FAILED);return result;
	}
	ap_start_seen=0;ap_start_result=0;
	result=DUT_wifi_start(DUT_AP);
	SV6X66_Printf("AP start: dut_rc=%d observed=%d sdk_rc=%ld mode=%d\r\n",result,ap_start_seen,(long)ap_start_result,(int)get_DUT_wifi_mode());
	if(result || !ap_start_seen || ap_start_result) {
		if(status_callback)status_callback(WIFI_AP_FAILED);
		return result?result:ap_start_seen?(int)ap_start_result:-1;
	}
	access_point=1;connected=-1;
	ip=inet_addr("192.168.4.1");mask=inet_addr("255.255.255.0");gateway=ip;dns=ip;refresh();
	return 0;
}
const char *HAL_GetMyIPString(void) {return ip_text;}
const char *HAL_GetMyGatewayString(void) {return gateway_text;}
const char *HAL_GetMyDNSString(void) {return dns_text;}
const char *HAL_GetMyMaskString(void) {return mask_text;}
void WiFI_GetMacAddress(char *mac)
{
	/* OBK device identity uses the primary interface, including in AP mode. */
	int result=get_local_mac(0,(u8 *)mac,6);
	if(result) {memset(mac,0,6);SV6X66_Printf("Primary MAC query failed: rc=%d\r\n",result);}
}
int WiFI_SetMacAddress(char *mac)
{(void)mac;SV6X66_Printf("MAC override unsupported; factory identity retained\r\n");return 0;}
const char *HAL_GetMACStr(char *text)
{unsigned char mac[6];WiFI_GetMacAddress((char *)mac);snprintf(text,18,MACSTR,MAC2STR(mac));return text;}
int HAL_GetWifiStrength(void)
{
	u8 ssid[32],ssidlen=32,mac[6],rssi=0,channel=0;
	if(get_connectap_info(0,ssid,&ssidlen,mac,6,&rssi,&channel))return -127;
	/* SDK reports an unsigned dBm magnitude, as its AT status formatter shows. */
	return -(int)rssi;
}
char *HAL_GetWiFiBSSID(char *text)
{
	u8 ssid[32],ssidlen=32,mac[6],rssi=0,channel=0;
	if(get_connectap_info(0,ssid,&ssidlen,mac,6,&rssi,&channel))text[0]=0;
	else snprintf(text,18,MACSTR,MAC2STR(mac));return text;
}
uint8_t HAL_GetWiFiChannel(uint8_t *channel)
{
	u8 ssid[32],ssidlen=32,mac[6],rssi=0;
	if(get_connectap_info(0,ssid,&ssidlen,mac,6,&rssi,channel))*channel=0;
	return *channel;
}
void HAL_PrintNetworkInfo(void)
{char mac[18];SV6X66_Printf("WiFi %s IP=%s MAC=%s RSSI=%d\r\n",access_point?"AP":"STA",ip_text,HAL_GetMACStr(mac),HAL_GetWifiStrength());}

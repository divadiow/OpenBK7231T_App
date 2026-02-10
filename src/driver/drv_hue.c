#include "../new_common.h"
#include "../new_pins.h"
#include "../new_cfg.h"
// Commands register, execution API and cmd tokenizer
#include "../cmnds/cmd_public.h"
#include "../mqtt/new_mqtt.h"
#include "../logging/logging.h"
#include "../hal/hal_pins.h"
#include "../hal/hal_wifi.h"
#include "drv_public.h"
#include "drv_local.h"
#include "drv_ssdp.h"
#include "../httpserver/new_http.h"

// Hue driver for Alexa, requiring btsimonh's SSDP to work
// Based on Tasmota approach
// The procedure is following:
// 1. first MSEARCH over UDP is done
// 2. then obk replies to MSEARCH with page details
// 3. then alexa accesses our XML pages here with GET
// 4. and can change the binary state (0 or 1) with POST

static char *buffer_out = 0;
static char *g_serial = 0;
static char *g_userID = 0;
static char *g_uid = 0;
static char *g_bridgeID = 0;
static int outBufferLen = 0;
static int stat_searchesReceived = 0;
static int stat_setupXMLVisits = 0;
// stubs
static int stat_metaServiceXMLVisits = 0;
static int stat_eventsReceived = 0;
static int stat_eventServiceXMLVisits = 0;

static int HUE_startsWith(const char* base, const char* prefix) {
	while (*prefix) {
		if (*base != *prefix)
			return 0;
		if (*base == 0)
			return 0;
		base++;
		prefix++;
	}
	return 1;
}

static int HUE_IsOn() {
	return LED_GetEnableAll() ? 1 : 0;
}
static int HUE_GetBri() {
	int bri = (int)(LED_GetDimmer() * 254.0f / 100.0f);
	if (bri < 1)
		bri = 1;
	if (bri > 254)
		bri = 254;
	return bri;
}
static int HUE_GetHue() {
	int h = (int)(LED_GetHue() * 65535.0f / 360.0f);
	if (h < 0)
		h = 0;
	if (h > 65535)
		h = 65535;
	return h;
}
static int HUE_GetSat() {
	int sat = (int)(LED_GetSaturation() * 254.0f / 100.0f);
	if (sat < 0)
		sat = 0;
	if (sat > 254)
		sat = 254;
	return sat;
}
// Hue CT is mireds 153..500. We keep conversion linear and clamp to that range.
static int HUE_GetCt() {
	float f = LED_GetTemperature0to1Range();
	int ct = 500 - (int)(f * (500.0f - 153.0f));
	if (ct < 153)
		ct = 153;
	if (ct > 500)
		ct = 500;
	return ct;
}
static int HUE_GetPWMCount() {
	int pwmCount = 0;
	PIN_get_Relay_PWM_Count(0, &pwmCount, 0);
	return pwmCount;
}
static int HUE_IsHueSatSupported() {
	int pwmCount = HUE_GetPWMCount();
	if (CFG_HasFlag(OBK_FLAG_LED_FORCESHOWRGBCWCONTROLLER))
		return 1;
	if (LED_IsLedDriverChipRunning())
		return 1;
	return pwmCount > 2;
}
static int HUE_IsCtSupported() {
	int pwmCount = HUE_GetPWMCount();
	if (CFG_HasFlag(OBK_FLAG_LED_FORCESHOWRGBCWCONTROLLER))
		return 1;
	if (LED_IsLedDriverChipRunning())
		return 1;
	return pwmCount == 2 || pwmCount > 3;
}
static const char *HUE_GetTypeString() {
	if (HUE_IsHueSatSupported() && HUE_IsCtSupported())
		return "Extended color light";
	if (HUE_IsHueSatSupported())
		return "Color light";
	if (HUE_IsCtSupported())
		return "Color temperature light";
	return "Dimmable light";
}
static const char *HUE_GetColorModeString() {
	if (HUE_IsHueSatSupported())
		return "hs";
	if (HUE_IsCtSupported())
		return "ct";
	return "none";
}

static int HUE_ExtractBool(const char* body, const char* key, int* outBool) {
	char pat[48];
	const char* p;
	snprintf(pat, sizeof(pat), "\"%s\"", key);
	p = strstr(body, pat);
	if (!p)
		return 0;
	p += strlen(pat);
	while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == ':')
		p++;
	if (!strncmp(p, "true", 4)) {
		*outBool = 1;
		return 1;
	}
	if (!strncmp(p, "false", 5)) {
		*outBool = 0;
		return 1;
	}
	return 0;
}
static int HUE_ExtractInt(const char* body, const char* key, int* outInt) {
	char pat[48];
	const char* p;
	char* end;
	long v;
	snprintf(pat, sizeof(pat), "\"%s\"", key);
	p = strstr(body, pat);
	if (!p)
		return 0;
	p += strlen(pat);
	while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n' || *p == ':')
		p++;
	v = strtol(p, &end, 10);
	if (end == p)
		return 0;
	*outInt = (int)v;
	return 1;
}
static int HUE_UserMatches(const char* userStart, const char* userEnd) {
	int len;
	if (g_userID == 0)
		return 0;
	len = userEnd - userStart;
	if (len <= 0)
		return 0;
	if ((int)strlen(g_userID) != len)
		return 0;
	return !my_strnicmp(userStart, g_userID, len);
}

static void HUE_PostCurrentState(http_request_t* request) {
	const char *typeStr;
	const char *modeStr;
	typeStr = HUE_GetTypeString();
	modeStr = HUE_GetColorModeString();
	hprintf255(request,
		"{\"state\":{\"on\":%s,\"bri\":%i,\"hue\":%i,\"sat\":%i,\"ct\":%i,\"colormode\":\"%s\",\"alert\":\"none\",\"effect\":\"none\",\"reachable\":true},"
		"\"type\":\"%s\",\"name\":\"OpenBeken Light\",\"modelid\":\"LCT015\",\"manufacturername\":\"OpenBeken\",\"productname\":\"OpenBeken\",\"swversion\":\"1.0\"}",
		HUE_IsOn() ? "true" : "false",
		HUE_GetBri(), HUE_GetHue(), HUE_GetSat(), HUE_GetCt(), modeStr, typeStr);
}
static void HUE_PostSuccessBool(http_request_t* request, int lightIdx, const char* key, int v) {
	hprintf255(request, "[{\"success\":{\"/lights/%i/state/%s\":%s}}]", lightIdx, key, v ? "true" : "false");
}
static void HUE_PostSuccessInt(http_request_t* request, int lightIdx, const char* key, int v) {
	hprintf255(request, "[{\"success\":{\"/lights/%i/state/%s\":%i}}]", lightIdx, key, v);
}
static void HUE_SetBri254(int bri254) {
	int dimmer;
	if (bri254 < 1)
		bri254 = 1;
	if (bri254 > 254)
		bri254 = 254;
	dimmer = (int)(bri254 * 100.0f / 254.0f);
	if (dimmer < 1)
		dimmer = 1;
	if (dimmer > 100)
		dimmer = 100;
	LED_SetDimmer(dimmer);
}
static void HUE_SetHue65535(int hue65535) {
	char tmp[64];
	int hue360;
	if (hue65535 < 0)
		hue65535 = 0;
	if (hue65535 > 65535)
		hue65535 = 65535;
	hue360 = (int)(hue65535 * 360.0f / 65535.0f);
	if (hue360 < 0)
		hue360 = 0;
	if (hue360 > 360)
		hue360 = 360;
	snprintf(tmp, sizeof(tmp), "%i", hue360);
	CMD_ExecuteCommandArgs("HSBColor1", tmp, 0);
}
static void HUE_SetSat254(int sat254) {
	char tmp[64];
	int sat100;
	if (sat254 < 0)
		sat254 = 0;
	if (sat254 > 254)
		sat254 = 254;
	sat100 = (int)(sat254 * 100.0f / 254.0f);
	if (sat100 < 0)
		sat100 = 0;
	if (sat100 > 100)
		sat100 = 100;
	snprintf(tmp, sizeof(tmp), "%i", sat100);
	CMD_ExecuteCommandArgs("HSBColor2", tmp, 0);
}
static void HUE_SetCt(int ct) {
	float f;
	if (ct < 153)
		ct = 153;
	if (ct > 500)
		ct = 500;
	f = (500.0f - ct) / (500.0f - 153.0f);
	if (f < 0)
		f = 0;
	if (f > 1)
		f = 1;
	LED_SetTemperature0to1Range(f);
}


// ARGUMENTS: first IP, then bridgeID
const  char *hue_resp = "HTTP/1.1 200 OK\r\n"
   "HOST: 239.255.255.250:1900\r\n"
   "CACHE-CONTROL: max-age=100\r\n"
   "EXT:\r\n"
   "LOCATION: http://%s:80/description.xml\r\n"
   "SERVER: Linux/3.14.0 UPnP/1.0 IpBridge/1.24.0\r\n"  // was 1.17
   "hue-bridgeid: %s\r\n";

// ARGUMENTS: uuid
const  char *hue_resp1 = "ST: upnp:rootdevice\r\n"
  "USN: uuid:%s::upnp:rootdevice\r\n"
  "\r\n";

// ARGUMENTS: uuid and uuid
const  char *hue_resp2 =  "ST: uuid:%s\r\n"
   "USN: uuid:%s\r\n"
   "\r\n";

// ARGUMENTS: uuid
const  char *hue_resp3 = "ST: urn:schemas-upnp-org:device:basic:1\r\n"
   "USN: uuid:%s\r\n"
   "\r\n";

void DRV_HUE_Send_Advert_To(struct sockaddr_in *addr) {
	//const char *useType;

	if (g_uid == 0) {
		// not running
		return;
	}

	stat_searchesReceived++;

	if (buffer_out == 0) {
		outBufferLen = strlen(hue_resp) + 256;
		buffer_out = (char*)malloc(outBufferLen);
	}
	{
		// ARGUMENTS: first IP, then bridgeID
		snprintf(buffer_out, outBufferLen, hue_resp, HAL_GetMyIPString(), g_bridgeID);

		addLogAdv(LOG_ALL, LOG_FEATURE_HTTP, "HUE - Sending[0] %s", buffer_out);
		DRV_SSDP_SendReply(addr, buffer_out);
	}
	{
		// ARGUMENTS: uuid
		snprintf(buffer_out, outBufferLen, hue_resp1, g_uid);

		addLogAdv(LOG_ALL, LOG_FEATURE_HTTP, "HUE - Sending[1] %s", buffer_out);
		DRV_SSDP_SendReply(addr, buffer_out);
	}
	{
		// ARGUMENTS: uuid and uuid
		snprintf(buffer_out, outBufferLen, hue_resp2, g_uid, g_uid);

		addLogAdv(LOG_ALL, LOG_FEATURE_HTTP, "HUE - Sending[2] %s", buffer_out);
		DRV_SSDP_SendReply(addr, buffer_out);
	}
	{
		// ARGUMENTS: uuid
		snprintf(buffer_out, outBufferLen, hue_resp3, g_uid);

		addLogAdv(LOG_ALL, LOG_FEATURE_HTTP, "HUE - Sending[3] %s", buffer_out);
		DRV_SSDP_SendReply(addr, buffer_out);
	}
}


void HUE_AppendInformationToHTTPIndexPage(http_request_t* request, int bPreState) {
	if(bPreState)
		return;
	hprintf255(request, "<h4>HUE: searches %i, setup %i, events %i, mService %i, event %i </h4>",
		stat_searchesReceived, stat_setupXMLVisits, stat_eventsReceived, stat_metaServiceXMLVisits, stat_eventServiceXMLVisits);

}

const char *g_hue_setup_1 = "<?xml version=\"1.0\"?>"
"<root xmlns=\"urn:schemas-upnp-org:device-1-0\">"
"<specVersion>"
"<major>1</major>"
"<minor>0</minor>"
"</specVersion>"
"<URLBase>http://";
// IP ADDR HERE
const char *g_hue_setup_2 = ":80/</URLBase>"
"<device>"
"<deviceType>urn:schemas-upnp-org:device:Basic:1</deviceType>"
"<friendlyName>Amazon-Echo-HA-Bridge (";
// IP ADDR HERE
const char *g_hue_setup_3 = ")</friendlyName>"
"<manufacturer>Royal Philips Electronics</manufacturer>"
"<manufacturerURL>http://www.philips.com</manufacturerURL>"
"<modelDescription>Philips hue Personal Wireless Lighting</modelDescription>"
"<modelName>Philips hue bridge 2012</modelName>"
"<modelNumber>929000226503</modelNumber>"
"<serialNumber>";
// SERIAL here
const char *g_hue_setup_4 = "</serialNumber>"
"<UDN>uuid:";
// UID HERE
const char *g_hue_setup_5 = "</UDN>"
   "</device>"
   "</root>\r\n"
   "\r\n";

static int HUE_Setup(http_request_t* request) {
	http_setup(request, httpMimeTypeXML);
	poststr(request, g_hue_setup_1);
	poststr(request, HAL_GetMyIPString());
	poststr(request, g_hue_setup_2);
	poststr(request, HAL_GetMyIPString());
	poststr(request, g_hue_setup_3);
	poststr(request, g_serial);
	poststr(request, g_hue_setup_4);
	poststr(request, g_uid);
	poststr(request, g_hue_setup_5);
	poststr(request, NULL);

	stat_setupXMLVisits++;

	return 0;
}
static int HUE_NotImplemented(http_request_t* request) {

	http_setup(request, httpMimeTypeJson);
	poststr(request, "{}");
	poststr(request, NULL);

	return 0;
}
static int HUE_Authentication(http_request_t* request) {

	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "[{\"success\":{\"username\":\"%s\"}}]",g_userID);
	poststr(request, NULL);

	return 0;
}
static int HUE_Config_Internal(http_request_t* request) {

	poststr(request, "{\"name\":\"Philips hue\",\"mac\":\"");
	poststr(request, g_serial);
	poststr(request, "\",\"dhcp\":true,\"ipaddress\":\"");
	poststr(request, HAL_GetMyIPString());
	poststr(request, "\",\"netmask\":\"");
	// TODO: mask
	poststr(request, "\",\"gateway\":\"");
	// TODO: gw
	poststr(request, "\",\"proxyaddress\":\"none\",\"proxyport\":0,\"bridgeid\":\"");
	poststr(request, g_bridgeID);
	poststr(request, "\",\"UTC\":\"{dt\",\"whitelist\":{\"");
	poststr(request, g_userID);
	poststr(request, "\":{\"last use date\":\"");
	// TODO: date
	poststr(request, "\",\"create date\":\"");
	// TODO: date
	poststr(request, "\",\"name\":\"Remote\"}},\"swversion\":\"01041302\",\"apiversion\":\"1.17.0\",\"swupdate\":{\"updatestate\":0,\"url\":\"\",\"text\":\"\",\"notify\": false},\"linkbutton\":false,\"portalservices\":false}");
	poststr(request, NULL);

	return 0;
}




static int HUE_GlobalConfig(http_request_t* request) {

	http_setup(request, httpMimeTypeJson);
	poststr(request, "{\"lights\":{\"1\":");
	HUE_PostCurrentState(request);
	poststr(request, "},\"groups\":{},\"schedules\":{},\"config\":");
	HUE_Config_Internal(request);
	poststr(request, "}");
	poststr(request, NULL);

	return 0;
}


// http://192.168.0.213/api/username/lights/1/state
// http://192.168.0.213/description.xml
int HUE_APICall(http_request_t* request) {
	const char* api;
	const char* rem;
	const char* userEnd;
	char body[512];
	int bodyCopyLen;
	int lightId;
	int b;
	if (g_uid == 0) {
		// not running
		return 0;
	}
	if (!HUE_startsWith(request->url, "api")) {
		return 0;
	}
	if (!strcmp(request->url, "api") && request->method == HTTP_POST) {
		HUE_Authentication(request);
		return 1;
	}
	if (!HUE_startsWith(request->url, "api/")) {
		return 0;
	}

	api = request->url + 4;
	rem = strchr(api, '/');
	if (!rem) {
		// allow other OpenBeken REST API paths (e.g. /api/channels) to work untouched
		return 0;
	}
	userEnd = rem;
	if (!HUE_UserMatches(api, userEnd)) {
		return 0;
	}
	rem++;

	if (!strcmp(rem, "config") && request->method == HTTP_GET) {
		http_setup(request, httpMimeTypeJson);
		HUE_Config_Internal(request);
		return 1;
	}
	if (!strcmp(rem, "lights") && request->method == HTTP_GET) {
		http_setup(request, httpMimeTypeJson);
		poststr(request, "{\"1\":");
		HUE_PostCurrentState(request);
		poststr(request, "}");
		poststr(request, NULL);
		return 1;
	}
	if (!strncmp(rem, "lights/", 7) && request->method == HTTP_GET) {
		char *endPtr;
		lightId = strtol(rem + 7, &endPtr, 10);
		if (*endPtr != 0) {
			HUE_NotImplemented(request);
			return 1;
		}
		if (lightId != 1) {
			HUE_NotImplemented(request);
			return 1;
		}
		http_setup(request, httpMimeTypeJson);
		HUE_PostCurrentState(request);
		poststr(request, NULL);
		return 1;
	}
	if (!strncmp(rem, "lights/", 7) && request->method == HTTP_PUT) {
		char *endPtr;
		lightId = strtol(rem + 7, &endPtr, 10);
		if (strcmp(endPtr, "/state")) {
			HUE_NotImplemented(request);
			return 1;
		}
		if (lightId != 1) {
			HUE_NotImplemented(request);
			return 1;
		}

		http_setup(request, httpMimeTypeJson);
		if (request->bodystart == 0 || request->bodylen <= 0) {
			poststr(request, "[{\"error\":{\"type\":2,\"description\":\"body required\"}}]");
			poststr(request, NULL);
			return 1;
		}
		bodyCopyLen = request->bodylen;
		if (bodyCopyLen >= (int)sizeof(body))
			bodyCopyLen = sizeof(body) - 1;
		memcpy(body, request->bodystart, bodyCopyLen);
		body[bodyCopyLen] = 0;

		if (HUE_ExtractBool(body, "on", &b)) {
			LED_SetEnableAll(b);
			HUE_PostSuccessBool(request, 1, "on", b);
			poststr(request, NULL);
			return 1;
		}
		if (HUE_ExtractInt(body, "bri", &b)) {
			HUE_SetBri254(b);
			HUE_PostSuccessInt(request, 1, "bri", HUE_GetBri());
			poststr(request, NULL);
			return 1;
		}
		if (HUE_ExtractInt(body, "hue", &b)) {
			if (!HUE_IsHueSatSupported()) {
				poststr(request, "[{\"error\":{\"type\":6,\"description\":\"hue not supported in current mode\"}}]");
				poststr(request, NULL);
				return 1;
			}
			HUE_SetHue65535(b);
			HUE_PostSuccessInt(request, 1, "hue", HUE_GetHue());
			poststr(request, NULL);
			return 1;
		}
		if (HUE_ExtractInt(body, "sat", &b)) {
			if (!HUE_IsHueSatSupported()) {
				poststr(request, "[{\"error\":{\"type\":6,\"description\":\"sat not supported in current mode\"}}]");
				poststr(request, NULL);
				return 1;
			}
			HUE_SetSat254(b);
			HUE_PostSuccessInt(request, 1, "sat", HUE_GetSat());
			poststr(request, NULL);
			return 1;
		}
		if (HUE_ExtractInt(body, "ct", &b)) {
			if (!HUE_IsCtSupported()) {
				poststr(request, "[{\"error\":{\"type\":6,\"description\":\"ct not supported in current mode\"}}]");
				poststr(request, NULL);
				return 1;
			}
			HUE_SetCt(b);
			HUE_PostSuccessInt(request, 1, "ct", HUE_GetCt());
			poststr(request, NULL);
			return 1;
		}

		poststr(request, "[{\"error\":{\"type\":6,\"description\":\"parameter not available\"}}]");
		poststr(request, NULL);
		return 1;
	}

	HUE_NotImplemented(request);
	return 1;
}
// backlog startDriver SSDP; startDriver HUE
// 
void HUE_Init() {
	char tmp[64];
	unsigned char mac[8];

	WiFI_GetMacAddress((char*)mac);
	// username - 
	snprintf(tmp, sizeof(tmp), "%02X%02X%02X",  mac[3], mac[4], mac[5]);
	g_userID = strdup(tmp);
	// SERIAL - as in Tas, full 12 chars of MAC, so 5c cf 7f 13 9f 3d
	snprintf(tmp, sizeof(tmp), "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	g_serial = strdup(tmp);
	// BridgeID - as in Tas, full 12 chars of MAC with FFFE inside, so 5C CF 7F FFFE 13 9F 3D
	snprintf(tmp, sizeof(tmp), "%02X%02X%02XFFFE%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
	g_bridgeID = strdup(tmp);
	// uuid
	snprintf(tmp, sizeof(tmp), "f6543a06-da50-11ba-8d8f-%s", g_serial);
	g_uid = strdup(tmp);


	//HTTP_RegisterCallback("/api", HTTP_ANY, HUE_APICall);
	HTTP_RegisterCallback("/description.xml", HTTP_GET, HUE_Setup, 0);
}

#include "sv6x66_platform.h"
#include "cJSON.h"
#include <string.h>

static uint8_t factory_mac[2][6];
static int identity_valid;
static cJSON *read_factory_json(void)
{
	const char *source=(const char *)(SV6X66_FLASH_BASE+0x6000);
	cJSON *json;size_t length=0;char *copy;
	while(length<4096 && source[length] && (uint8_t)source[length]!=0xff)length++;
	if(length<2 || length==4096 || source[0]!='{' || source[length-1]!='}')return NULL;
	copy=malloc(length+1);if(!copy)return NULL;
	memcpy(copy,source,length);copy[length]=0;
	json=cJSON_Parse(copy);free(copy);return json;
}
static int hex(char c)
{if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;if(c>='A'&&c<='F')return c-'A'+10;return -1;}
static int parse_mac(const cJSON *json,const char *name,uint8_t *mac)
{
	const cJSON *item=cJSON_GetObjectItem(json,name);
	const char *text;unsigned int i,nonzero=0;
	if(!item || !item->valuestring || strlen(item->valuestring)!=17)return -1;
	text=item->valuestring;
	for(i=0;i<6;i++) {
		int hi=hex(text[i*3]),lo=hex(text[i*3+1]);
		if(hi<0||lo<0||(i<5&&text[i*3+2]!=':'))return -1;
		mac[i]=(hi<<4)|lo;nonzero|=mac[i];
	}
	return !nonzero || (mac[0]&1)?-1:0;
}
int SV6X66_IdentityInit(void)
{
	cJSON *json=read_factory_json();
	if(!json)return -1;
	identity_valid=!parse_mac(json,"addr1",factory_mac[0])&&!parse_mac(json,"addr2",factory_mac[1]);
	cJSON_Delete(json);return identity_valid?0:-1;
}
/* SDK opaque handle ABI is preserved. All device values stay on target; no
	* factory config strings or private data are copied into repository sources. */
void *__wrap_wifi_cfg_init(void) {return identity_valid?read_factory_json():NULL;}
void __wrap_wifi_cfg_get_addr1(const void *handle,char address[6])
{if(handle && identity_valid)memcpy(address,factory_mac[0],6);else memset(address,0,6);}
void __wrap_wifi_cfg_get_addr2(const void *handle,char address[6])
{if(handle && identity_valid)memcpy(address,factory_mac[1],6);else memset(address,0,6);}
void __wrap_wifi_cfg_deinit(void *handle) {if(handle)cJSON_Delete(handle);}
void __wrap_wifi_cfg_write_cfg(const void *handle)
{(void)handle;SV6X66_Printf("Factory identity writes refused\r\n");}
void __wrap_wifi_cfg_replace_mem_addr1(const void *handle,char *address)
{(void)handle;(void)address;SV6X66_Printf("Factory identity override refused\r\n");}
void __wrap_wifi_cfg_replace_mem_addr2(const void *handle,char *address)
{(void)handle;(void)address;SV6X66_Printf("Factory identity override refused\r\n");}

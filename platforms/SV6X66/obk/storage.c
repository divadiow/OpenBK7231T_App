#include "sv6x66_platform.h"
#include "flash.h"
#include "spiffs_interface.h"
#include "new_common.h"
#include "hal/hal_flashConfig.h"
#include "hal/hal_flashVars.h"
#include <string.h>
#include <stdio.h>

#define FS_START 0x141000u
#define FS_BYTES 0xbf000u
static spiffs filesystem;
static uint8_t work[512],descriptors[768],cache[2048];
static int mounted,writable;
/* Read-only diagnostics: retain the original filesystem operation sequence. */
static void storage_diagnostic(const char *stage,const char *name,s32_t result)
{
	SV6X66_Printf("SPIFFS %s file=%s rc=%ld errno=%ld mounted=%d writable=%d\r\n",
		stage,name?name:"-",(long)result,(long)prvSPIFFS_errno(&filesystem),mounted,writable);
}
/* SDK consumers may reference this symbol; no opaque FS_init, formatting or
	* vendor OTA cleanup is invoked. OBK owns only its named files below. */
spiffs *fs_handle;
static int in_fs(uint32_t address,uint32_t length)
{return address>=FS_START && length<=FS_BYTES && address<=FS_START+FS_BYTES-length;}
static s32_t read_flash(u32_t address,u32_t length,u8_t *data)
{
	if(!in_fs(address,length))return SPIFFS_ERR_INTERNAL;
	memcpy(data,(const void *)(SV6X66_FLASH_BASE+address),length);return 0;
}
static s32_t write_flash(u32_t address,u32_t length,u8_t *data)
{
	if(!writable || !in_fs(address,length))return SPIFFS_ERR_NOT_WRITABLE;
	return SV6X66_FlashWrite(address,data,length)?SPIFFS_ERR_INTERNAL:0;
}
static s32_t erase_flash(u32_t address,u32_t length)
{
	uint32_t offset;
	if(!writable || (address&4095) || (length&4095) || !in_fs(address,length))return SPIFFS_ERR_NOT_WRITABLE;
	for(offset=0;offset<length;offset+=4096)if(SV6X66_FlashErase(address+offset))return SPIFFS_ERR_ERASE_FAIL;
	return 0;
}
void SV6X66_StorageInit(void)
{
	spiffs_config config;
	s32_t result;
	uint32_t block,valid=0;
	memset(&config,0,sizeof(config));
	config.phys_addr=FS_START;config.phys_size=FS_BYTES;config.phys_erase_block=4096;
	config.log_block_size=4096;config.log_page_size=256;
	config.hal_read_f=read_flash;config.hal_write_f=write_flash;config.hal_erase_f=erase_flash;
	/* Existing CKW04 SPIFFS magic includes remaining block count. Accept only
		* known geometry; erased blocks are allowed, contradictory magic is not. */
	for(block=0;block<191;block++) {
		const uint16_t *magic=(const uint16_t *)(SV6X66_FLASH_BASE+FS_START+block*4096+252);
		uint16_t expected=(uint16_t)(0x20140529u^256u^(191-block));
		if(*magic==expected)valid++;
		else if(*magic!=0xffff) {SV6X66_Printf("SPIFFS geometry mismatch; persistence unavailable\r\n");return;}
	}
	if(!valid) {SV6X66_Printf("SPIFFS mount skipped: no matching geometry markers\r\n");return;}
	if(SV6X66_FlashLock()) {SV6X66_Printf("SPIFFS mount skipped: flash busy\r\n");return;}
	writable=0;
	result=prvSPIFFS_mount(&filesystem,&config,work,descriptors,sizeof(descriptors),cache,sizeof(cache),NULL);
	mounted=result==0;
	if(mounted) {writable=1;fs_handle=&filesystem;}
	storage_diagnostic("mount",NULL,result);
	SV6X66_FlashUnlock();
	SV6X66_Printf("CKW04 SPIFFS mount %s; format is never attempted\r\n",mounted?"OK":"failed/read-only");
}
static uint32_t crc32(const uint8_t *p,uint32_t length)
{
	uint32_t c=UINT32_MAX,i;
	while(length--) {c^=*p++;for(i=0;i<8;i++)c=(c>>1)^(0xedb88320u&(0u-(c&1)));}
	return c^UINT32_MAX;
}
typedef struct {uint32_t magic,sequence,length,crc;} record_header;
static int read_slot(const char *name,void *data,uint32_t length,uint32_t *sequence,int diagnose)
{
	record_header h;
	spiffs_file f=prvSPIFFS_open(&filesystem,name,SPIFFS_RDONLY,0);
	int result=-1;s32_t count;
	if(f<0) {if(diagnose)storage_diagnostic("readback-open",name,f);return -1;}
	count=prvSPIFFS_read(&filesystem,f,&h,sizeof(h));
	if(count==sizeof(h) && h.magic==0x314b424f && h.length==length) {
		count=prvSPIFFS_read(&filesystem,f,data,length);
		if(count==length && crc32(data,length)==h.crc) {*sequence=h.sequence;result=0;}
		else if(diagnose)storage_diagnostic(count==length?"readback-crc":"readback-payload",name,count);
	}
	else if(diagnose)storage_diagnostic(count==sizeof(h)?"readback-header-fields":"readback-header",name,count);
	count=prvSPIFFS_close(&filesystem,f);
	if(count<0) {if(diagnose)storage_diagnostic("readback-close",name,count);result=-1;}
	return result;
}
static int load_record(const char *key,void *data,uint32_t length)
{
	char name[24];uint32_t sequence[2]={0};int good[2],selected=-1;
	void *candidate;
	if(!mounted || SV6X66_FlashLock())return -1;
	candidate=malloc(length);
	if(!candidate) {SV6X66_FlashUnlock();return -1;}
	snprintf(name,sizeof(name),"obk.%s0",key);good[0]=read_slot(name,data,length,&sequence[0],0)==0;
	snprintf(name,sizeof(name),"obk.%s1",key);good[1]=read_slot(name,candidate,length,&sequence[1],0)==0;
	if(good[0])selected=0;
	if(good[1] && (selected<0 || (int32_t)(sequence[1]-sequence[0])>0)) {memcpy(data,candidate,length);selected=1;}
	free(candidate);SV6X66_FlashUnlock();return selected<0?-1:0;
}
static int save_record(const char *key,const void *data,uint32_t length)
{
	char name[24];uint32_t seq[2]={0},checkseq;int good[2],slot,result=-1;
	record_header h;spiffs_file f;void *check;
	s32_t count;
	if(!mounted || !writable) {storage_diagnostic("save-unavailable",key,-1);return -1;}
	if(SV6X66_FlashLock()) {storage_diagnostic("save-flash-busy",key,-1);return -1;}
	check=malloc(length);
	if(!check) {storage_diagnostic("save-allocation",key,-1);goto done;}
	snprintf(name,sizeof(name),"obk.%s0",key);good[0]=read_slot(name,check,length,&seq[0],0)==0;
	snprintf(name,sizeof(name),"obk.%s1",key);good[1]=read_slot(name,check,length,&seq[1],0)==0;
	slot=good[0] && (!good[1] || (int32_t)(seq[0]-seq[1])>=0)?1:0;
	h.magic=0x314b424f;h.sequence=(slot?seq[0]:seq[1])+1;h.length=length;h.crc=crc32(data,length);
	snprintf(name,sizeof(name),"obk.%s%d",key,slot);
	f=prvSPIFFS_open(&filesystem,name,SPIFFS_CREAT|SPIFFS_TRUNC|SPIFFS_RDWR|SPIFFS_DIRECT,0);
	if(f<0) {storage_diagnostic("save-open",name,f);goto release;}
	count=prvSPIFFS_write(&filesystem,f,&h,sizeof(h));
	if(count!=sizeof(h)) {storage_diagnostic("save-header-write",name,count);goto close_failure;}
	count=prvSPIFFS_write(&filesystem,f,(void *)data,length);
	if(count!=length) {storage_diagnostic("save-payload-write",name,count);goto close_failure;}
	count=prvSPIFFS_fflush(&filesystem,f);
	if(count<0) {storage_diagnostic("save-flush",name,count);goto close_failure;}
	count=prvSPIFFS_close(&filesystem,f);
	if(count<0) {storage_diagnostic("save-close",name,count);goto release;}
	if(!read_slot(name,check,length,&checkseq,1)) {
		if(checkseq==h.sequence && !memcmp(check,data,length))result=0;
		else storage_diagnostic("readback-sequence-or-data",name,-1);
	}
	goto release;
close_failure:
	count=prvSPIFFS_close(&filesystem,f);
	if(count<0)storage_diagnostic("save-cleanup-close",name,count);
release:free(check);
done:SV6X66_FlashUnlock();return result;
}
int HAL_Configuration_ReadConfigMemory(void *target,int bytes)
{
	if(bytes<=0)return 0;
	/* A missing record must not leave partially read garbage in default config. */
	void *copy=malloc(bytes);int result=0;
	if(copy && !load_record("cfg",copy,bytes)) {memcpy(target,copy,bytes);result=bytes;}
	free(copy);return result;
}
int HAL_Configuration_SaveConfigMemory(void *source,int bytes)
{
	if(bytes<=0 || save_record("cfg",source,bytes)) {SV6X66_Printf("OBK configuration save failed\r\n");return 0;}
	return bytes;
}
static FLASH_VARS_STRUCTURE vars;
static int loaded;
static void load_vars(void)
{if(!loaded){FLASH_VARS_STRUCTURE copy;memset(&vars,0,sizeof(vars));if(!load_record("vars",&copy,sizeof(copy)))vars=copy;loaded=1;}}
static int save_vars(void)
{int result=save_record("vars",&vars,sizeof(vars));if(result)SV6X66_Printf("OBK retained-state save failed\r\n");return result;}
void HAL_FlashVars_IncreaseBootCount(void) {load_vars();vars.boot_count++;save_vars();}
void HAL_FlashVars_SaveBootComplete(void) {load_vars();vars.boot_success_count=vars.boot_count;save_vars();}
int HAL_FlashVars_GetBootFailures(void) {load_vars();return (uint16_t)(vars.boot_count-vars.boot_success_count);}
int HAL_FlashVars_GetBootCount(void) {load_vars();return vars.boot_count;}
int HAL_FlashVars_GetChannelValue(int ch) {load_vars();return ch>=0&&ch<MAX_RETAIN_CHANNELS?vars.savedValues[ch]:0;}
void HAL_FlashVars_SaveChannel(int ch,int value)
{load_vars();if(ch>=0&&ch<MAX_RETAIN_CHANNELS){vars.savedValues[ch]=value;save_vars();}}
void HAL_FlashVars_SaveLED(byte mode,short brightness,short temperature,byte r,byte g,byte b,byte all)
{
	load_vars();vars.savedValues[MAX_RETAIN_CHANNELS-4]=all;vars.savedValues[MAX_RETAIN_CHANNELS-3]=mode;
	vars.savedValues[MAX_RETAIN_CHANNELS-2]=temperature;vars.savedValues[MAX_RETAIN_CHANNELS-1]=brightness;
	vars.rgb[0]=r;vars.rgb[1]=g;vars.rgb[2]=b;save_vars();
}
void HAL_FlashVars_ReadLED(byte *mode,short *brightness,short *temperature,byte *rgb,byte *all)
{
	load_vars();*all=vars.savedValues[MAX_RETAIN_CHANNELS-4];*mode=vars.savedValues[MAX_RETAIN_CHANNELS-3];
	*temperature=vars.savedValues[MAX_RETAIN_CHANNELS-2];*brightness=vars.savedValues[MAX_RETAIN_CHANNELS-1];memcpy(rgb,vars.rgb,3);
}
int HAL_GetEnergyMeterStatus(ENERGY_METERING_DATA *data)
{if(!data)return -1;load_vars();*data=vars.emetering;return 0;}
int HAL_SetEnergyMeterStatus(ENERGY_METERING_DATA *data)
{if(!data)return -1;load_vars();vars.emetering=*data;return save_vars();}
void HAL_FlashVars_SaveTotalConsumption(float value) {load_vars();vars.emetering.TotalConsumption=value;save_vars();}

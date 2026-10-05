#include "test_storage_env.h"
#include "fsal.h"
#include "../sv6x66_storage.h"
#include "../../../src/hal/hal_flashConfig.h"
#include <assert.h>
#include <stdio.h>
SSV_FS fs_handle = (void *)1;
static int fs_error, fault, write_count, locked;
static unsigned held, mutex_count, read_open_count;
static struct { uint8_t data[512]; unsigned len, position; int exists, writing; } disk[4];
void *test_malloc(size_t n) { return fault == 1 ? NULL : malloc(n); }
xSemaphoreHandle xSemaphoreCreateMutex(void) { return (void *)(uintptr_t)(++mutex_count); }
int xSemaphoreTake(xSemaphoreHandle s,unsigned t) { (void)t;unsigned bit=1u<<(uintptr_t)s;assert(!(held&bit));held|=bit;locked++;return 1; }
int xSemaphoreGive(xSemaphoreHandle s) { unsigned bit=1u<<(uintptr_t)s;assert(held&bit);held&=~bit;locked--;return 1; }
#ifndef TEST_FLASHVARS
void SV6X66_FlashVarsInit(void) { }
#endif
int test_fs_errno(SSV_FS f) { (void)f;return fs_error; }
SSV_FILE FS_open(SSV_FS f,const char *name,uint32_t flags,uint32_t mode) {
 (void)f;(void)mode;int n=!strcmp(name,"obk_cfg0")?0:!strcmp(name,"obk_cfg1")?1:!strcmp(name,"obk_vars0")?2:3;
 if(!(flags&SPIFFS_WRONLY)) read_open_count++;
 if((fault==2 || (fault==6 && read_open_count==3)) && !(flags&SPIFFS_WRONLY)) {fs_error=-123;return -1;}
 if(!disk[n].exists && !(flags&SPIFFS_CREAT)) {fs_error=SPIFFS_ERR_NOT_FOUND;return -1;}
 disk[n].exists=1;disk[n].position=0;disk[n].writing=!!(flags&SPIFFS_WRONLY);if(flags&SPIFFS_TRUNC)disk[n].len=0;
 return n;
}
int32_t FS_fstat(SSV_FS f,SSV_FILE n,SSV_FILE_STAT *st) { (void)f;st->size=disk[n].len;return 0; }
int32_t FS_read(SSV_FS f,SSV_FILE n,void *p,uint32_t len) {
 (void)f;unsigned available=disk[n].len-disk[n].position;if(len>available)len=available;
 memcpy(p,disk[n].data+disk[n].position,len);disk[n].position+=len;return len;
}
int32_t FS_write(SSV_FS f,SSV_FILE n,void *p,uint32_t len) {
 (void)f;assert(locked);write_count++;if(fault==3 && write_count==2)len--;
 assert(disk[n].position+len<=sizeof(disk[n].data));memcpy(disk[n].data+disk[n].position,p,len);disk[n].position+=len;disk[n].len=disk[n].position;return len;
}
int32_t FS_close(SSV_FS f,SSV_FILE n) { (void)f;if(disk[n].writing && fault==4){disk[n].data[16]^=1;return -1;} if(disk[n].writing && fault==5)disk[n].data[20]^=1;disk[n].writing=0;return 0; }
#ifdef TEST_FLASHVARS
void test_storage_fault(int value) { fault = value; write_count = 0; }
int test_storage_writes(void) { return write_count; }
#define main test_records_main
#endif
int main(void) {
 uint8_t a[64],b[64],out[64],old[512];memset(a,0x11,sizeof(a));memset(b,0x22,sizeof(b));SV6X66_StorageInit();
 assert(!SV6X66_StorageReadRecord(1,out,sizeof(out)));assert(SV6X66_StorageWriteRecord(1,a,sizeof(a))==sizeof(a));assert(SV6X66_StorageReadRecord(1,out,sizeof(out))==sizeof(out));assert(!memcmp(a,out,sizeof(a)));
 for(int f=1;f<=6;f++){if(f>2 && f<6)continue;fault=f;read_open_count=0;memset(out,0x33,sizeof(out));assert(SV6X66_StorageReadRecord(1,out,sizeof(out))==-1);for(unsigned n=0;n<sizeof(out);n++)assert(out[n]==0x33);fault=0;}
 memcpy(old,disk[0].data,sizeof(old));
 for(int f=1;f<=5;f++){fault=f;write_count=0;assert(!SV6X66_StorageWriteRecord(1,b,sizeof(b)));assert(!memcmp(old,disk[0].data,sizeof(old)));fault=0;assert(SV6X66_StorageReadRecord(1,out,sizeof(out))==sizeof(out));assert(!memcmp(a,out,sizeof(a)));}
 assert(SV6X66_StorageWriteRecord(1,b,sizeof(b))==sizeof(b));assert(SV6X66_StorageReadRecord(1,out,sizeof(out))==sizeof(out));assert(!memcmp(b,out,sizeof(b)));
 disk[1].data[8]^=1;assert(SV6X66_StorageReadRecord(1,out,sizeof(out))==sizeof(out));assert(!memcmp(a,out,sizeof(a))); // sequence corruption rejected
 assert(SV6X66_StorageWriteRecord(2,b,sizeof(b))==sizeof(b));assert(SV6X66_StorageReadRecord(1,out,sizeof(out))==sizeof(out));assert(!memcmp(a,out,sizeof(a)));
 fault=1;assert(!HAL_Configuration_ReadConfigMemory(out,sizeof(out)));fault=0;write_count=0;assert(!HAL_Configuration_SaveConfigMemory(b,sizeof(b)));assert(write_count==0);assert(HAL_Configuration_ReadConfigMemory(out,sizeof(out))==sizeof(out));assert(!memcmp(a,out,sizeof(a)));assert(HAL_Configuration_SaveConfigMemory(b,sizeof(b)));
 puts("Persistent storage fault tests passed");return 0;
}

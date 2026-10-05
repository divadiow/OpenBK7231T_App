// Exercise the production mount against the SDK's actual SPIFFS implementation.
#include "../stock_mount.h"
#include "../sv6x66_layout.h"
#include "spiffs_nucleus.h"
#include "sys/flash.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
static unsigned char *image;
static unsigned programs, erases;
unsigned int flash_init(void) { return 1; }
void flash_sector_erase(unsigned int addr) {
    assert(addr >= SV6X66_FS_START && addr <= SV6X66_FLASH_SIZE - 4096);
    memset(image + addr, 255, 4096); erases++;
}
void flash_page_program(unsigned int addr, unsigned int len, unsigned char *src) {
    assert(addr >= SV6X66_FS_START && len <= SV6X66_FLASH_SIZE - addr);
    for (unsigned i=0; i<len; i++) image[addr+i] &= src[i];
    programs++;
}
static s32_t read_image(u32_t addr,u32_t len,u8_t *dst) { memcpy(dst,image+addr,len); return 0; }
static s32_t write_image(u32_t addr,u32_t len,u8_t *src) { flash_page_program(addr,len,src); return 0; }
static s32_t erase_image(u32_t addr,u32_t len) { while(len){flash_sector_erase(addr);addr+=4096;len-=4096;}return 0; }
int main(void) {
    image=mmap((void *)0x30000000u,SV6X66_FLASH_SIZE,PROT_READ|PROT_WRITE,
               MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0);
    assert(image==(void *)0x30000000u);
    memset(image,255,SV6X66_FLASH_SIZE);
    spiffs fixture={0}; spiffs_config cfg={0};
    unsigned char work[512],fds[768],cache[sizeof(spiffs_cache)+8*(sizeof(spiffs_cache_page)+256)];
    cfg.phys_addr=SV6X66_FS_START;cfg.phys_size=SV6X66_FLASH_SIZE-SV6X66_FS_START;
    cfg.phys_erase_block=4096;cfg.log_block_size=4096;cfg.log_page_size=256;
    cfg.hal_read_f=read_image;cfg.hal_write_f=write_image;cfg.hal_erase_f=erase_image;
    assert(prvSPIFFS_mount(&fixture,&cfg,work,fds,sizeof(fds),cache,sizeof(cache),NULL)<0);
    assert(!prvSPIFFS_format(&fixture)); // Only the synthetic in-memory fixture is formatted.
    programs=erases=0;
    SSV_FS mounted=SV6X66_StockMount();assert(mounted && !programs && !erases);
    prvSPIFFS_unmount(mounted);
    image[SPIFFS_MAGIC_PADDR(mounted,0)]^=1; // One bad block triggers the SDK's mount repair.
    unsigned char *before=malloc(SV6X66_FLASH_SIZE);assert(before);
    memcpy(before,image,SV6X66_FLASH_SIZE);
    for(int attempt=0;attempt<2;attempt++) {
        assert(!SV6X66_StockMount());
        assert(!programs && !erases && !memcmp(before,image,SV6X66_FLASH_SIZE));
    }
    free(before);assert(!munmap(image,SV6X66_FLASH_SIZE));
    puts("Real SPIFFS mount repair blocked; damaged image unchanged across retries");
    return 0;
}

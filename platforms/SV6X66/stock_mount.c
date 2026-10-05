#include "stock_mount.h"
#include "sv6x66_layout.h"
#include "spiffs_nucleus.h"
#include "sys/flash.h"
#include <stdint.h>
#include <string.h>
#if defined(SV6X66_STOCK_CKW04)
#define PAGE_SIZE 256u
#define BLOCK_SIZE 4096u
#define CACHE_SIZE (sizeof(spiffs_cache) + 8u * (sizeof(spiffs_cache_page) + PAGE_SIZE))
static spiffs fs;
static int writes_enabled;
static spiffs_config config;
static uint8_t work[2u * PAGE_SIZE] __attribute__((aligned(4)));
static uint8_t descriptors[768] __attribute__((aligned(4)));
static uint8_t cache[CACHE_SIZE] __attribute__((aligned(4)));
static int in_range(u32_t addr, u32_t len)
{
    return addr >= SV6X66_FS_START && addr <= SV6X66_FLASH_SIZE && len <= SV6X66_FLASH_SIZE - addr;
}
static s32_t read_flash(u32_t addr, u32_t len, u8_t *dst)
{
    if ((!dst && len) || !in_range(addr, len)) return SPIFFS_ERR_INTERNAL;
    // N10 startup keeps D-cache disabled; XIP reads depend on that policy.
    if (len) memcpy(dst, (const void *)(uintptr_t)(0x30000000u + addr), len);
    return SPIFFS_OK;
}
static s32_t write_flash(u32_t addr, u32_t len, u8_t *src)
{
    if (!writes_enabled) return SPIFFS_ERR_NOT_WRITABLE;
    if ((!src && len) || !in_range(addr, len)) return SPIFFS_ERR_INTERNAL;
    while (len) {
        u32_t chunk = PAGE_SIZE - (addr & (PAGE_SIZE - 1u));
        if (chunk > len) chunk = len;
        flash_page_program(addr, chunk, src);
        addr += chunk; src += chunk; len -= chunk;
    }
    return SPIFFS_OK;
}
static s32_t erase_flash(u32_t addr, u32_t len)
{
    if (!writes_enabled) return SPIFFS_ERR_NOT_WRITABLE;
    if (!in_range(addr, len) || addr % BLOCK_SIZE || len % BLOCK_SIZE) return SPIFFS_ERR_INTERNAL;
    while (len) { flash_sector_erase(addr); addr += BLOCK_SIZE; len -= BLOCK_SIZE; }
    return SPIFFS_OK;
}
static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
int SV6X66_StockHeaderValid(const uint8_t header[40])
{
    return header && read_le32(header) == 0x00500400u && read_le32(header + 16) == SV6X66_FLASH_SIZE &&
        read_le32(header + 20) == SV6X66_RAW_SIZE && read_le32(header + 24) == SV6X66_MAIN_SIZE;
}
SSV_FS SV6X66_StockMount(void)
{
    if (prvSPIFFS_mounted(&fs)) return &fs;
    writes_enabled = 0;
    if (!flash_init()) return NULL;
    memset(&config, 0, sizeof(config));
    config.hal_read_f = read_flash; config.hal_write_f = write_flash; config.hal_erase_f = erase_flash;
    config.phys_addr = SV6X66_FS_START;
    config.phys_size = SV6X66_FLASH_SIZE - SV6X66_FS_START;
    config.phys_erase_block = BLOCK_SIZE; config.log_block_size = BLOCK_SIZE; config.log_page_size = PAGE_SIZE;
    // SPIFFS may repair blocks during mount. Deny writes until mount succeeds.
    if (prvSPIFFS_mount(&fs, &config, work, descriptors, sizeof(descriptors), cache, sizeof(cache), NULL) < 0)
        return NULL;
    writes_enabled = 1;
    return &fs;
}
#endif

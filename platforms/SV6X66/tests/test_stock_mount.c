#include "../stock_mount.h"
#include "../sv6x66_layout.h"
#include "spiffs_nucleus.h"
#include "sys/flash.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static int flash_ready, mounts, programs, erases, mount_result;
static spiffs_config observed;
unsigned int flash_init(void) { return flash_ready; }
void flash_sector_erase(unsigned int addr) { assert(addr >= SV6X66_FS_START && addr + 4096 <= SV6X66_FLASH_SIZE && !(addr % 4096)); erases++; }
void flash_page_program(unsigned int addr, unsigned int len, unsigned char *src)
{
    assert(src && addr >= SV6X66_FS_START && len <= SV6X66_FLASH_SIZE - addr);
    assert(len && len <= 256 && (addr % 256) + len <= 256); programs++;
}
u8_t prvSPIFFS_mounted(spiffs *fs) { (void)fs; return 0; }
s32_t prvSPIFFS_mount(spiffs *fs, spiffs_config *cfg, u8_t *work, u8_t *fds, u32_t fdlen,
                      void *cache, u32_t cachelen, spiffs_check_callback cb)
{
    assert(fs && work && fds && cache && !cb && fdlen == 768);
    assert(cachelen == sizeof(spiffs_cache) + 8 * (sizeof(spiffs_cache_page) + 256));
    observed = *cfg; mounts++;
    uint8_t data = 0;
    assert(cfg->hal_write_f(0xBA000, 1, &data) == SPIFFS_ERR_NOT_WRITABLE);
    assert(cfg->hal_erase_f(0xBA000, 4096) == SPIFFS_ERR_NOT_WRITABLE);
    assert(!programs && !erases);
    return mount_result;
}
int main(void)
{
    uint8_t header[40] = {0}, data[48] = {0};
    uint32_t fields[10] = {0x00500400, 0, 1, 1, 0x200000, 0x2000, 0xAF000};
    memcpy(header, fields, sizeof(header));
    assert(SV6X66_StockHeaderValid(header));
    header[24] ^= 1; assert(!SV6X66_StockHeaderValid(header));
    assert(!SV6X66_StockHeaderValid(NULL));
    assert(!SV6X66_StockMount() && mounts == 0);
    flash_ready = 1; mount_result = SPIFFS_ERR_NOT_A_FS;
    assert(!SV6X66_StockMount() && mounts == 1 && !programs && !erases);
    mount_result = 0; assert(SV6X66_StockMount());
    assert(observed.phys_addr == 0xBA000 && observed.phys_size == 0x146000);
    assert(observed.log_page_size == 256 && observed.phys_erase_block == 4096 && observed.log_block_size == 4096);
    assert(observed.hal_read_f(0x8000, 1, data) < 0);
    assert(observed.hal_read_f(0x1FFFFF, 2, data) < 0);
    assert(observed.hal_read_f(0x200000, 0, NULL) == 0);
    assert(observed.hal_write_f(0xB000, sizeof(data), data) < 0);
    assert(observed.hal_write_f(0x1FFFFF, sizeof(data), data) < 0);
    assert(observed.hal_write_f(0xFFFFFFFF, sizeof(data), data) < 0);
    assert(observed.hal_write_f(0xBA0F0, sizeof(data), data) == 0 && programs == 2);
    assert(observed.hal_erase_f(0xB000, 4096) < 0);
    assert(observed.hal_erase_f(0xBA001, 4096) < 0);
    assert(observed.hal_erase_f(0xBA000, 4095) < 0);
    assert(observed.hal_erase_f(0xBA000, 4096) == 0 && erases == 1);
    puts("Stock filesystem mount never formats; flash callbacks enforce partition bounds");
    return 0;
}

#include "../hal_ota.h"

#include "../../new_common.h"
#include "../../new_cfg.h"
#include "typedef.h"
#include "flash_pub.h"
#include "BkDriverFlash.h"
#include <easyflash.h>
//#include "flash.h"
#include "../../logging/logging.h"
#include "../../httpclient/http_client.h"
#include "../../httpserver/new_http.h"
#include "../../driver/drv_public.h"
#include "../../driver/drv_bl_shared.h"
#include "../../littlefs/our_lfs.h"

static unsigned char *sector = (void *)0;
static int sectorlen = 0;
static unsigned int addr = 0xff000;
static unsigned int ota_write_end = 0;
static int ota_flash_locked = 0;
#define SECTOR_SIZE 0x1000
static int store_sector(unsigned int write_addr, unsigned char *data);
extern void flash_protection_op(UINT8 mode,PROTECT_TYPE type);

// from wlan_ui.c
void bk_reboot(void);

// from flash.c
extern UINT32 flash_read(char *user_buf, UINT32 count, UINT32 address);
extern UINT32 flash_write(char *user_buf, UINT32 count, UINT32 address);
extern UINT32 flash_ctrl(UINT32 cmd, void *parm);

int HAL_FlashRead(char*buffer, int readlen, int startaddr) {
	int res;
	res = flash_read((char*)buffer, readlen, startaddr);
	return res;
}

static void release_ota_lock(void)
{
    if (ota_flash_locked)
    {
        ota_flash_locked = 0;
        hal_flash_unlock();
    }
}

static void release_ota_writer(int release_lock)
{
    if (sector)
    {
        os_free(sector);
        sector = (void *)0;
    }
    sectorlen = 0;
    ota_write_end = 0;
	flash_protection_op(FLASH_XTX_16M_SR_WRITE_ENABLE, FLASH_UNPROTECT_LAST_BLOCK);
    if (release_lock)
    {
        release_ota_lock();
    }
}

static void abort_ota(void)
{
    release_ota_writer(1);
}

static int init_ota(unsigned int startaddr, unsigned int endaddr){
    if (startaddr > 0xff000 && endaddr > startaddr){
        if (sector || ota_flash_locked){
            addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"aborting OTA, flash writer is busy");
            return 0;
        }
        hal_flash_lock();
        if (sector || ota_flash_locked)
        {
            hal_flash_unlock();
            addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"aborting OTA, flash writer became busy");
            return 0;
        }
        ota_flash_locked = 1;
        flash_init();
		flash_protection_op(FLASH_XTX_16M_SR_WRITE_ENABLE, FLASH_PROTECT_NONE);
        sector = os_malloc(SECTOR_SIZE);
        if (!sector)
        {
            addLogAdv(LOG_ERROR, LOG_FEATURE_OTA,"aborting OTA, sector allocation failed");
            release_ota_writer(1);
            return 0;
        }
        sectorlen = 0;
        addr = startaddr;
        ota_write_end = endaddr;
        addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"init OTA, range 0x%x-0x%x", startaddr, endaddr);
        return 1;
    }
    addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"aborting OTA, invalid range 0x%x-0x%x", startaddr, endaddr);
    return 0;
}

static int close_ota_internal(int release_lock){
    int result = 1;

    if (!sector)
    {
        return 0;
    }
    addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"");
    if (sectorlen){
        addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"close OTA, additional 0x%x FF added", SECTOR_SIZE - sectorlen);
        memset(sector+sectorlen, 0xff, SECTOR_SIZE - sectorlen);
        sectorlen = SECTOR_SIZE;
        result = store_sector(addr, sector);
        addr += SECTOR_SIZE;
        sectorlen = 0;
    }
    addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"close OTA, addr 0x%x", addr);

    release_ota_writer(release_lock);
    return result;
}

static int close_ota(){
    return close_ota_internal(1);
}

static int add_otadata(unsigned char *data, int len)
{
    if (!sector || len < 0) return 0;
    if ((addr + sectorlen > ota_write_end) || ((unsigned int)len > ota_write_end - (addr + sectorlen)))
    {
        addLogAdv(LOG_ERROR, LOG_FEATURE_OTA,"OTA write exceeds end 0x%x", ota_write_end);
        return 0;
    }
    //addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"OTA DataRxed start: %02.2x %02.2x len %d", data[0], data[1], len);
    while (len > 0)
    {
        // force it to sleep...  we MUST have some idle task processing
	    // else task memory doesn't get freed

        if (sectorlen < SECTOR_SIZE)
        {
            int lenstore = SECTOR_SIZE - sectorlen;
            if (lenstore > len) 
                lenstore = len;
            memcpy(sector + sectorlen, data, lenstore);
            data += lenstore;
            len -= lenstore;
            sectorlen += lenstore;
            //addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"OTA sector start: %02.2x %02.2x len %d", sector[0], sector[1], sectorlen);
        }

        if (sectorlen == SECTOR_SIZE){
            if (!store_sector(addr, sector))
            {
                return 0;
            }
            addr += SECTOR_SIZE;
            sectorlen = 0;
        } else {
            //addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"OTA sectorlen 0x%x not yet 0x%x", sectorlen, SECTOR_SIZE);
            rtos_delay_milliseconds(10);
        }
    }
    return 1;
}

static int store_sector(unsigned int write_addr, unsigned char *data){
    if (ota_write_end < SECTOR_SIZE || write_addr > ota_write_end - SECTOR_SIZE)
    {
        addLogAdv(LOG_ERROR, LOG_FEATURE_OTA,"OTA sector 0x%x exceeds end 0x%x", write_addr, ota_write_end);
        return 0;
    }
    //if (!(addr % 0x4000))
    {
      addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"%x", write_addr);
    }
    //addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"writing OTA, addr 0x%x", addr);
    if (flash_ctrl(CMD_FLASH_WRITE_ENABLE, (void *)0) != FLASH_SUCCESS ||
        flash_ctrl(CMD_FLASH_ERASE_SECTOR, &write_addr) != FLASH_SUCCESS ||
        flash_ctrl(CMD_FLASH_WRITE_ENABLE, (void *)0) != FLASH_SUCCESS ||
        flash_write((char *)data , SECTOR_SIZE, write_addr) != FLASH_SUCCESS)
    {
        addLogAdv(LOG_ERROR, LOG_FEATURE_OTA,"OTA flash write failed at 0x%x", write_addr);
        return 0;
    }
    OTA_IncrementProgress(SECTOR_SIZE);
    return 1;
}

#if PLATFORM_BK7238
#define BK7238_OTA_START                 0x132000U
#define BK7238_STANDARD_OTA_END          0x1E0000U
#define BK7238_TUYA_OTA_END              0x1E1000U
#define BK7238_TUYA_JOURNAL_START        0x1E1000U
#define BK7238_TUYA_JOURNAL_END          0x1E3000U
#define BK7238_MAX_RAW_APP_SIZE          0x110000U
#define BK7238_RBL_HEADER_SIZE           0x60U
#define BK7238_PCZL_CHECK_SIZE           0xD6U
#define BK7238_TUYA_MANAGEMENT_SIZE      0x40U
#define BK7238_FLASH_CLEANUP_RETRIES     3

typedef enum
{
    BK7238_OTA_OWNER_NONE = 0,
    BK7238_OTA_OWNER_POST,
    BK7238_OTA_OWNER_URL,
    BK7238_OTA_OWNER_FLASH
} bk7238_ota_owner_t;

typedef struct
{
    int active;
    int failed;
    int header_validated;
    int staging_touched;
    int lfs_blocked;
    int cleanup_failed;
    int error_code;
    const char *error_message;
    bk7238_ota_owner_t owner;
    bk_flash_boot_profile_t profile;
    unsigned int expected_size;
    unsigned int received_size;
    unsigned int header_required;
    unsigned int header_size;
    unsigned int staging_end;
    unsigned char header[BK7238_PCZL_CHECK_SIZE];
} bk7238_ota_stream_t;

static bk7238_ota_stream_t bk7238_ota_stream;
static volatile bk7238_ota_owner_t bk7238_ota_owner;
static volatile int bk7238_ota_activation_pending;
static volatile int bk7238_ota_flash_quarantined;
static int bk7238_url_ota_failed;

static int bk7238_ota_acquire_owner(bk7238_ota_owner_t owner)
{
    int acquired = 0;

    if (owner == BK7238_OTA_OWNER_NONE || bk7238_ota_activation_pending ||
        bk7238_ota_flash_quarantined || bk7238_ota_owner != BK7238_OTA_OWNER_NONE ||
        bk7238_ota_stream.active || sector || ota_flash_locked)
    {
        return 0;
    }

    rtos_enter_critical();
    if (!bk7238_ota_activation_pending && !bk7238_ota_flash_quarantined &&
        bk7238_ota_owner == BK7238_OTA_OWNER_NONE &&
        !bk7238_ota_stream.active && !sector && !ota_flash_locked)
    {
        bk7238_ota_owner = owner;
        acquired = 1;
    }
    rtos_exit_critical();
    return acquired;
}

static int bk7238_ota_is_owner(bk7238_ota_owner_t owner)
{
    return owner != BK7238_OTA_OWNER_NONE && bk7238_ota_owner == owner;
}

static void bk7238_ota_release_owner(bk7238_ota_owner_t owner)
{
    if (!bk7238_ota_is_owner(owner))
    {
        return;
    }

    rtos_enter_critical();
    if (!bk7238_ota_activation_pending && !bk7238_ota_flash_quarantined &&
        bk7238_ota_owner == owner)
    {
        bk7238_ota_owner = BK7238_OTA_OWNER_NONE;
    }
    rtos_exit_critical();
}

static UINT32 read_le32(const unsigned char *data)
{
    return ((UINT32)data[0]) |
        ((UINT32)data[1] << 8) |
        ((UINT32)data[2] << 16) |
        ((UINT32)data[3] << 24);
}

static void write_le32(unsigned char *data, UINT32 value)
{
    data[0] = (unsigned char)value;
    data[1] = (unsigned char)(value >> 8);
    data[2] = (unsigned char)(value >> 16);
    data[3] = (unsigned char)(value >> 24);
}

static int erase_flash_sector(unsigned int erase_addr)
{
    return flash_ctrl(CMD_FLASH_WRITE_ENABLE, (void *)0) == FLASH_SUCCESS &&
        flash_ctrl(CMD_FLASH_ERASE_SECTOR, &erase_addr) == FLASH_SUCCESS;
}

static int bk7238_flash_region_is_erased(unsigned int read_addr, unsigned int len)
{
    unsigned char buffer[64];
    unsigned int i;
    unsigned int read_len;

    while (len)
    {
        read_len = len > sizeof(buffer) ? sizeof(buffer) : len;
        if (flash_read((char *)buffer, read_len, read_addr) != FLASH_SUCCESS)
        {
            return 0;
        }
        for (i = 0; i < read_len; i++)
        {
            if (buffer[i] != 0xFF)
            {
                return 0;
            }
        }
        read_addr += read_len;
        len -= read_len;
    }
    return 1;
}

static int bk7238_clear_tuya_journal_unprotected(void)
{
    unsigned int erase_addr;
    int attempt;

    for (erase_addr = BK7238_TUYA_JOURNAL_START;
        erase_addr < BK7238_TUYA_JOURNAL_END; erase_addr += SECTOR_SIZE)
    {
        for (attempt = 0; attempt < BK7238_FLASH_CLEANUP_RETRIES; attempt++)
        {
            if (erase_flash_sector(erase_addr) &&
                bk7238_flash_region_is_erased(erase_addr, SECTOR_SIZE))
            {
                break;
            }
        }
        if (attempt == BK7238_FLASH_CLEANUP_RETRIES)
        {
            return 0;
        }
    }
    return 1;
}

static int bk7238_invalidate_staging(void)
{
    unsigned int erase_addr = BK7238_OTA_START;
    int result;
    int attempt;

    if (!ota_flash_locked)
    {
        return 0;
    }

    flash_init();
    flash_protection_op(FLASH_XTX_16M_SR_WRITE_ENABLE, FLASH_PROTECT_NONE);
    result = 0;
    for (attempt = 0; attempt < BK7238_FLASH_CLEANUP_RETRIES; attempt++)
    {
        if (erase_flash_sector(erase_addr) &&
            bk7238_flash_region_is_erased(erase_addr, SECTOR_SIZE))
        {
            result = 1;
            break;
        }
    }
    flash_protection_op(FLASH_XTX_16M_SR_WRITE_ENABLE, FLASH_UNPROTECT_LAST_BLOCK);
    return result;
}

static int bk7238_ota_fail(bk7238_ota_owner_t owner, int error_code, const char *message)
{
    int cleanup_ok;

    if (!bk7238_ota_is_owner(owner) || bk7238_ota_stream.owner != owner)
    {
        return 0;
    }

    addLogAdv(LOG_ERROR, LOG_FEATURE_OTA, "BK7238 OTA failed: %s", message);
    bk7238_ota_stream.failed = 1;
    bk7238_ota_stream.error_code = error_code;
    bk7238_ota_stream.error_message = message;
    cleanup_ok = !bk7238_ota_stream.cleanup_failed;
    if (sector || ota_flash_locked)
    {
        release_ota_writer(0);
    }
    if (bk7238_ota_stream.staging_touched)
    {
        if (!bk7238_invalidate_staging())
        {
            addLogAdv(LOG_ERROR, LOG_FEATURE_OTA,
                "BK7238 OTA staging header invalidation failed");
            cleanup_ok = 0;
        }
        bk7238_ota_stream.staging_touched = 0;
    }
    if (!cleanup_ok)
    {
        bk7238_ota_flash_quarantined = 1;
        addLogAdv(LOG_ERROR, LOG_FEATURE_OTA,
            "BK7238 OTA flash cleanup could not be verified; flash writers remain disabled until reboot");
    }
    release_ota_lock();
#if ENABLE_LITTLEFS
    if (bk7238_ota_stream.lfs_blocked)
    {
        LFS_EndFlashAccessBlock(cleanup_ok);
        if (cleanup_ok)
        {
            bk7238_ota_stream.lfs_blocked = 0;
        }
    }
#endif
    bk7238_ota_stream.active = 0;
    return 0;
}

static int bk7238_validate_rbl_header(const unsigned char *header, unsigned int content_length)
{
    UINT32 package_size;
    UINT32 raw_size;

    if (memcmp(header, "RBL\0", 4) != 0)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 400,
            "standard Beken bootloader requires an RBL file");
    }
    if (ef_calc_crc32(0, header, 0x5C) != read_le32(header + 0x5C))
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 400, "invalid RBL header CRC");
    }

    package_size = read_le32(header + 0x58);
    raw_size = read_le32(header + 0x54);
    if (package_size != content_length - BK7238_RBL_HEADER_SIZE)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 400,
            "RBL package size does not match upload size");
    }
    if (raw_size == 0 || raw_size > BK7238_MAX_RAW_APP_SIZE)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 400,
            "RBL application size is invalid");
    }
    return 1;
}

static int bk7238_validate_pczl_header(const unsigned char *header, unsigned int content_length)
{
    static const unsigned char pczl_magic[8] = { 'M', 'M', 'M', 0, 'P', 'C', 'Z', 'L' };
    static const unsigned char xz_magic[6] = { 0xFD, '7', 'z', 'X', 'Z', 0 };
    UINT32 raw_crc;
    UINT32 raw_size;

    if (memcmp(header, pczl_magic, sizeof(pczl_magic)) != 0)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 400,
            "Tuya T1 bootloader requires a PCZL UG file");
    }
    if (memcmp(header + 0xD0, xz_magic, sizeof(xz_magic)) != 0)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 400, "PCZL XZ stream is missing");
    }
    if (read_le32(header + 0x18) != 0x40 ||
        read_le32(header + 0x3C) != 0x90 ||
        read_le32(header + 0x44) != 0x40)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 400,
            "unsupported PCZL header layout");
    }
    if (read_le32(header + 0x20) != content_length - 0x40)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 400,
            "PCZL region size does not match upload size");
    }
    if (read_le32(header + 0x2C) != 0x10000 || read_le32(header + 0x30) != 0x20000)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 400,
            "unsupported PCZL application layout");
    }

    raw_crc = read_le32(header + 0x08);
    raw_size = read_le32(header + 0x10);
    if (raw_crc != read_le32(header + 0x0C) ||
        raw_size != read_le32(header + 0x14) ||
        raw_size != read_le32(header + 0x48) ||
        raw_size != read_le32(header + 0x4C))
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 400,
            "PCZL duplicate fields do not agree");
    }
    if (raw_size == 0 || raw_size > BK7238_MAX_RAW_APP_SIZE)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 400,
            "PCZL application size is invalid");
    }
    return 1;
}

static int bk7238_validate_ota_header(void)
{
    if (bk7238_ota_stream.profile == BK_FLASH_BOOT_PROFILE_STANDARD)
    {
        return bk7238_validate_rbl_header(bk7238_ota_stream.header,
            bk7238_ota_stream.expected_size);
    }
    return bk7238_validate_pczl_header(bk7238_ota_stream.header,
        bk7238_ota_stream.expected_size);
}

static int bk7238_ota_begin(bk7238_ota_owner_t owner, unsigned int content_length)
{
    if (!bk7238_ota_is_owner(owner) || bk7238_ota_stream.active || sector || ota_flash_locked)
    {
        return 0;
    }
    memset(&bk7238_ota_stream, 0, sizeof(bk7238_ota_stream));
    bk7238_ota_stream.active = 1;
    bk7238_ota_stream.owner = owner;
    bk7238_ota_stream.expected_size = content_length;
    bk7238_ota_stream.profile = bk_flash_get_boot_profile();

    if (bk7238_ota_stream.profile == BK_FLASH_BOOT_PROFILE_STANDARD)
    {
        bk7238_ota_stream.header_required = BK7238_RBL_HEADER_SIZE;
        bk7238_ota_stream.staging_end = BK7238_STANDARD_OTA_END;
    }
    else if (bk7238_ota_stream.profile == BK_FLASH_BOOT_PROFILE_TUYA_T1)
    {
        bk7238_ota_stream.header_required = BK7238_PCZL_CHECK_SIZE;
        bk7238_ota_stream.staging_end = BK7238_TUYA_OTA_END;
    }
    else
    {
        return bk7238_ota_fail(owner, 409, "bootloader profile is unknown; OTA is disabled");
    }

    if (content_length < bk7238_ota_stream.header_required)
    {
        return bk7238_ota_fail(owner, 400, "OTA file is too small");
    }
    if (content_length > bk7238_ota_stream.staging_end - BK7238_OTA_START)
    {
        return bk7238_ota_fail(owner, 413,
            "OTA file exceeds the active bootloader staging area");
    }
    return 1;
}

static int bk7238_ota_feed(bk7238_ota_owner_t owner, const unsigned char *data, unsigned int len)
{
    unsigned int copy_len;
    unsigned int original_len = len;

    if (!bk7238_ota_is_owner(owner) || bk7238_ota_stream.owner != owner ||
        !bk7238_ota_stream.active || bk7238_ota_stream.failed)
    {
        return 0;
    }
    if (len > bk7238_ota_stream.expected_size - bk7238_ota_stream.received_size)
    {
        return bk7238_ota_fail(owner, 400, "received more OTA data than declared");
    }

    if (!bk7238_ota_stream.header_validated)
    {
        copy_len = bk7238_ota_stream.header_required - bk7238_ota_stream.header_size;
        if (copy_len > len)
        {
            copy_len = len;
        }
        memcpy(bk7238_ota_stream.header + bk7238_ota_stream.header_size, data, copy_len);
        bk7238_ota_stream.header_size += copy_len;
        data += copy_len;
        len -= copy_len;

        if (bk7238_ota_stream.header_size == bk7238_ota_stream.header_required)
        {
            if (!bk7238_validate_ota_header())
            {
                return 0;
            }
#if ENABLE_LITTLEFS
            if (!LFS_BeginFlashAccessBlock())
            {
                return bk7238_ota_fail(owner, 500,
                    "could not quiesce LittleFS for OTA");
            }
            bk7238_ota_stream.lfs_blocked = 1;
#endif
            if (!init_ota(BK7238_OTA_START, bk7238_ota_stream.staging_end))
            {
                return bk7238_ota_fail(owner, 500, "could not initialize flash writer");
            }
            bk7238_ota_stream.staging_touched = 1;
            if (bk7238_ota_stream.profile == BK_FLASH_BOOT_PROFILE_TUYA_T1 &&
                !bk7238_clear_tuya_journal_unprotected())
            {
                bk7238_ota_stream.cleanup_failed = 1;
                return bk7238_ota_fail(owner, 500, "could not clear Tuya OTA journal");
            }
            if (!add_otadata(bk7238_ota_stream.header, bk7238_ota_stream.header_size))
            {
                return bk7238_ota_fail(owner, 500, "could not write OTA header");
            }
            bk7238_ota_stream.header_validated = 1;
        }
    }

    if (len && !add_otadata((unsigned char *)data, len))
    {
        return bk7238_ota_fail(owner, 500, "could not write OTA data");
    }

    bk7238_ota_stream.received_size += original_len;
    OTA_SetTotalBytes(bk7238_ota_stream.received_size);
    return 1;
}

static int bk7238_flash_crc32(unsigned int read_addr, unsigned int len, UINT32 *result)
{
    unsigned char buffer[256];
    UINT32 crc = 0;
    unsigned int read_len;

    while (len)
    {
        read_len = len > sizeof(buffer) ? sizeof(buffer) : len;
        if (flash_read((char *)buffer, read_len, read_addr) != FLASH_SUCCESS)
        {
            return 0;
        }
        crc = ef_calc_crc32(crc, buffer, read_len);
        read_addr += read_len;
        len -= read_len;
    }
    *result = crc;
    return 1;
}

static int bk7238_verify_staged_ota(void)
{
    unsigned char header[BK7238_PCZL_CHECK_SIZE];
    UINT32 actual_crc;
    UINT32 expected_crc;
    unsigned int crc_offset;
    unsigned int crc_length;

    if (flash_read((char *)header, bk7238_ota_stream.header_required,
        BK7238_OTA_START) != FLASH_SUCCESS ||
        memcmp(header, bk7238_ota_stream.header, bk7238_ota_stream.header_required) != 0)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 500,
            "OTA header readback failed");
    }

    if (bk7238_ota_stream.profile == BK_FLASH_BOOT_PROFILE_STANDARD)
    {
        crc_offset = BK7238_RBL_HEADER_SIZE;
        expected_crc = read_le32(header + 0x4C);
    }
    else
    {
        crc_offset = 0x40;
        expected_crc = read_le32(header + 0x24);
    }
    crc_length = bk7238_ota_stream.expected_size - crc_offset;
    if (!bk7238_flash_crc32(BK7238_OTA_START + crc_offset, crc_length, &actual_crc) ||
        actual_crc != expected_crc)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 500,
            "OTA payload CRC readback failed");
    }
    return 1;
}

static int bk7238_commit_tuya_journal(void)
{
    unsigned char record[BK7238_TUYA_MANAGEMENT_SIZE];
    unsigned char verify[BK7238_TUYA_MANAGEMENT_SIZE];
    UINT32 checksum = 0;
    unsigned int i;
    int result = 1;

    memset(record, 0, sizeof(record));
    write_le32(record + 0x00, 0xABCDDCBA);
    write_le32(record + 0x08, bk7238_ota_stream.expected_size);
    write_le32(record + 0x0C, 0);
    write_le32(record + 0x10, 0);
    write_le32(record + 0x14, 0);
    record[0x18] = 0;
    record[0x19] = 1;
    record[0x1A] = 1;
    record[0x1B] = 0;
    write_le32(record + 0x1C, 0x10000);
    write_le32(record + 0x20, 0x1D2000);
    write_le32(record + 0x24, 0);
    write_le32(record + 0x28, BK7238_OTA_START);
    for (i = 8; i < sizeof(record); i++)
    {
        checksum += record[i];
    }
    write_le32(record + 0x04, checksum);

    flash_init();
    flash_protection_op(FLASH_XTX_16M_SR_WRITE_ENABLE, FLASH_PROTECT_NONE);
    if (flash_ctrl(CMD_FLASH_WRITE_ENABLE, (void *)0) != FLASH_SUCCESS ||
        flash_write((char *)record + 4, sizeof(record) - 4,
            BK7238_TUYA_JOURNAL_START + 4) != FLASH_SUCCESS ||
        flash_ctrl(CMD_FLASH_WRITE_ENABLE, (void *)0) != FLASH_SUCCESS ||
        flash_write((char *)record, 4, BK7238_TUYA_JOURNAL_START) != FLASH_SUCCESS ||
        flash_read((char *)verify, sizeof(verify), BK7238_TUYA_JOURNAL_START) != FLASH_SUCCESS ||
        memcmp(record, verify, sizeof(record)) != 0)
    {
        result = 0;
        if (!bk7238_clear_tuya_journal_unprotected())
        {
            bk7238_ota_stream.cleanup_failed = 1;
        }
    }
    flash_protection_op(FLASH_XTX_16M_SR_WRITE_ENABLE, FLASH_UNPROTECT_LAST_BLOCK);

    if (!result)
    {
        return bk7238_ota_fail(bk7238_ota_stream.owner, 500,
            "could not commit Tuya OTA journal");
    }
    return 1;
}

static int bk7238_ota_finish(bk7238_ota_owner_t owner)
{
    if (!bk7238_ota_is_owner(owner) || bk7238_ota_stream.owner != owner ||
        !bk7238_ota_stream.active || bk7238_ota_stream.failed)
    {
        return 0;
    }
    if (!bk7238_ota_stream.header_validated ||
        bk7238_ota_stream.received_size != bk7238_ota_stream.expected_size)
    {
        return bk7238_ota_fail(owner, 400, "OTA upload ended before the declared size");
    }
    if (!close_ota_internal(0))
    {
        return bk7238_ota_fail(owner, 500, "could not finish OTA flash write");
    }
    if (!bk7238_verify_staged_ota())
    {
        return 0;
    }
    if (bk7238_ota_stream.profile == BK_FLASH_BOOT_PROFILE_TUYA_T1 &&
        !bk7238_commit_tuya_journal())
    {
        return 0;
    }

    addLogAdv(LOG_INFO, LOG_FEATURE_OTA, "BK7238 OTA verified for %s bootloader",
        bk7238_ota_stream.profile == BK_FLASH_BOOT_PROFILE_TUYA_T1 ? "Tuya T1" : "standard Beken");
    bk7238_ota_activation_pending = 1;
    release_ota_lock();
    bk7238_ota_stream.active = 0;
    bk7238_ota_stream.staging_touched = 0;
    return 1;
}

static void bk7238_ota_abort(bk7238_ota_owner_t owner, const char *message)
{
    if (bk7238_ota_is_owner(owner) && bk7238_ota_stream.owner == owner &&
        !bk7238_ota_stream.failed)
    {
        bk7238_ota_fail(owner, 500, message);
    }
}
#endif


httprequest_t httprequest;
static volatile int ota_request_in_progress;

static int ota_request_begin(void)
{
  int started = 0;

  if (ota_request_in_progress)
  {
    return 0;
  }
  rtos_enter_critical();
  if (!ota_request_in_progress)
  {
    ota_request_in_progress = 1;
    started = 1;
  }
  rtos_exit_critical();
  return started;
}

static void ota_request_end(void)
{
  rtos_enter_critical();
  ota_request_in_progress = 0;
  rtos_exit_critical();
}

#if !PLATFORM_BK7238
static int ota_download_started;
static int ota_download_failed;
static unsigned int ota_download_expected;
static unsigned int ota_download_received;
#endif

static void ota_download_reboot(void)
{
  addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"Rebooting in 1 second...");
  CFG_IncrementOTACount();
  CFG_Save_IfThereArePendingChanges();
#if ENABLE_BL_SHARED
  if (DRV_IsMeasuringPower())
  {
    BL09XX_SaveEmeteringStatistics();
  }
#endif
  DRV_SavePowerMeterDriverStatistics();
  rtos_delay_milliseconds(1000);
  bk_reboot();
}

int myhttpclientcallback(httprequest_t* request){
  //httpclient_t *client = &request->client;
  httpclient_data_t *client_data = &request->client_data;
  int should_reboot = 0;

#if PLATFORM_BK7238
  switch(request->state){
    case 0: // start
      if (!bk7238_ota_is_owner(BK7238_OTA_OWNER_URL))
      {
        bk7238_url_ota_failed = 1;
        addLogAdv(LOG_ERROR, LOG_FEATURE_OTA,"OTA HTTP download lost its flash reservation");
        break;
      }
      bk7238_url_ota_failed = 0;
      memset(&bk7238_ota_stream, 0, sizeof(bk7238_ota_stream));
      OTA_SetTotalBytes(0);
      addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"OTA HTTP download started");
      break;
    case 1: // data
      if (bk7238_url_ota_failed || !bk7238_ota_is_owner(BK7238_OTA_OWNER_URL))
      {
        return 1;
      }
      if (!bk7238_ota_stream.active && !bk7238_ota_stream.failed)
      {
        if (client_data->is_chunked || client_data->response_content_len == 0 ||
            client_data->response_content_len == 0xFFFFFFFFU)
        {
          memset(&bk7238_ota_stream, 0, sizeof(bk7238_ota_stream));
          bk7238_ota_stream.active = 1;
          bk7238_ota_stream.owner = BK7238_OTA_OWNER_URL;
          bk7238_ota_fail(BK7238_OTA_OWNER_URL, 411,
            "OTA HTTP server must provide Content-Length");
          bk7238_url_ota_failed = 1;
          return 1;
        }
        if (!bk7238_ota_begin(BK7238_OTA_OWNER_URL, client_data->response_content_len))
        {
          bk7238_url_ota_failed = 1;
          return 1;
        }
      }
      if (request->client_data.response_buf_filled){
        unsigned char *d = (unsigned char *)request->client_data.response_buf;
        unsigned int l = request->client_data.response_buf_filled;
        if (!bk7238_ota_feed(BK7238_OTA_OWNER_URL, d, l))
        {
          bk7238_url_ota_failed = 1;
          return 1;
        }
      }
      break;
    case -1:
    case -2:
      bk7238_url_ota_failed = 1;
      if (bk7238_ota_is_owner(BK7238_OTA_OWNER_URL) && bk7238_ota_stream.active)
      {
        bk7238_ota_abort(BK7238_OTA_OWNER_URL, "OTA HTTP download failed");
      }
      break;
    case 2: // complete
      if (!bk7238_url_ota_failed && bk7238_ota_finish(BK7238_OTA_OWNER_URL))
      {
        should_reboot = 1;
      }
      else
      {
        addLogAdv(LOG_ERROR, LOG_FEATURE_OTA,"OTA HTTP download was not activated");
      }
      OTA_ResetProgress();
      bk7238_url_ota_failed = 0;
      bk7238_ota_release_owner(BK7238_OTA_OWNER_URL);
      break;
  }
#else
  // NOTE: Called from the client thread, beware
  switch(request->state){
    case 0: // start
      ota_download_started = 0;
      ota_download_failed = (sector || ota_flash_locked) ? 1 : 0;
      ota_download_expected = 0;
      ota_download_received = 0;
      OTA_SetTotalBytes(0);
      addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"\rmyhttpclientcallback state %d total %d/%d", request->state, OTA_GetTotalBytes(), request->client_data.response_content_len);
      break;
    case 1: // data
      if (!ota_download_started && !ota_download_failed)
      {
        bk_logic_partition_t *partition = bk_flash_get_info(BK_PARTITION_OTA);
        unsigned int content_length = client_data->response_content_len;
        if (!partition || client_data->is_chunked || content_length == 0 ||
          content_length == 0xFFFFFFFFU || content_length > partition->partition_length ||
          !init_ota(partition->partition_start_addr,
            partition->partition_start_addr + partition->partition_length))
        {
          ota_download_failed = 1;
          addLogAdv(LOG_ERROR, LOG_FEATURE_OTA,"OTA HTTP download header/range rejected");
          break;
        }
        ota_download_expected = content_length;
        ota_download_started = 1;
      }
      if (request->client_data.response_buf_filled){
        unsigned char *d = (unsigned char *)request->client_data.response_buf;
        unsigned int l = request->client_data.response_buf_filled;
        if (ota_download_failed || l > ota_download_expected - ota_download_received ||
          !add_otadata(d, l))
        {
          ota_download_failed = 1;
          if (ota_download_started && (sector || ota_flash_locked))
          {
            abort_ota();
          }
          break;
        }
        ota_download_received += l;
        OTA_SetTotalBytes(ota_download_received);
      }
      break;
    case -1:
    case -2:
      ota_download_failed = 1;
      if (ota_download_started && (sector || ota_flash_locked))
      {
        abort_ota();
      }
      break;
    case 2: // ended, write any remaining bytes to the sector
    {
      int success = !ota_download_failed && ota_download_started &&
        ota_download_received == ota_download_expected && close_ota();
      if (!success && ota_download_started && (sector || ota_flash_locked))
      {
        abort_ota();
      }
      OTA_ResetProgress();
      addLogAdv(success ? LOG_INFO : LOG_ERROR, LOG_FEATURE_OTA,
        "\rmyhttpclientcallback state %d total %d/%d success %d", request->state,
        ota_download_received, ota_download_expected, success);
      ota_download_started = 0;
      ota_download_failed = 0;
      should_reboot = success;
      break;
    }
  }
#endif

  //rtos_delay_milliseconds(500);

  if (request->state == 2){
    //os_free(client_data->response_buf);
    client_data->response_buf = (void*)0;
    client_data->response_buf_len = 0;
    ota_request_end();
  }
  //rtos_delay_milliseconds(100);

  if (should_reboot)
  {
    ota_download_reboot();
  }

  return 0;
}

  // NOTE: these MUST persist
// note: url must have a '/' after host, else it can;t parse it..
static char url[256] = "http://raspberrypi:1880/firmware";
static const char *header = "";
static char *content_type = "text/csv";
static char *post_data = "";
#define BUF_SIZE 2048

char *http_buf = (void *)0;

void otarequest(const char *urlin){
  httprequest_t *request = &httprequest;
  int send_result;

  if (!ota_request_begin()){
    addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"********************http in progress, not starting another");
    return;
  }
#if PLATFORM_BK7238
  if (!bk7238_ota_acquire_owner(BK7238_OTA_OWNER_URL))
  {
    ota_request_end();
    addLogAdv(LOG_ERROR, LOG_FEATURE_OTA,"OTA HTTP download rejected: flash writer is busy");
    return;
  }
  bk7238_url_ota_failed = 0;
#endif

  strncpy(url, urlin, sizeof(url) - 1);
  url[sizeof(url) - 1] = 0;

  OTA_SetTotalBytes(0);
  memset(request, 0, sizeof(*request));
  httpclient_t *client = &request->client;
  httpclient_data_t *client_data = &request->client_data;

  if (http_buf == (void *)0){
    http_buf = os_malloc(BUF_SIZE+1);
    if (http_buf == (void *)0) {
        addLogAdv(LOG_INFO, LOG_FEATURE_OTA,"startrequest Malloc failed.");
#if PLATFORM_BK7238
        bk7238_ota_release_owner(BK7238_OTA_OWNER_URL);
#endif
        ota_request_end();
        return;
    }
    memset(http_buf, 0, BUF_SIZE);
  }
  client_data->response_buf = http_buf;  //Sets a buffer to store the result.
  client_data->response_buf_len = BUF_SIZE;  //Sets the buffer size.
  HTTPClient_SetCustomHeader(client, header);  //Sets the custom header if needed.
  client_data->post_buf = post_data;  //Sets the user data to be posted.
  client_data->post_buf_len = strlen(post_data);  //Sets the post data length.
  client_data->post_content_type = content_type;  //Sets the content type.
  request->data_callback = &myhttpclientcallback;
  request->port = 80;//HTTP_PORT;
  request->url = url;
  request->method = HTTPCLIENT_GET;
  request->timeout = 10000;
  send_result = HTTPClient_Async_SendGeneric(request);
  if (send_result != 0)
  {
    addLogAdv(LOG_ERROR, LOG_FEATURE_OTA,"OTA HTTP download thread could not be started");
#if PLATFORM_BK7238
    bk7238_ota_release_owner(BK7238_OTA_OWNER_URL);
#endif
    ota_request_end();
    return;
  }
  //+2 Updating ota_status to 0 as before.
  OTA_ResetProgress();
  OTA_IncrementProgress(1);
 }

int http_rest_post_flash(http_request_t* request, int startaddr, int maxaddr)
{
	int total = 0;
	int towrite;
	char* writebuf = request->bodystart;
	int writelen = request->bodylen;
	int receive_len;

	ADDLOG_DEBUG(LOG_FEATURE_OTA, "flash post len %d", request->contentLength);
	if (request->contentLength <= 0)
	{
		return http_rest_error(request, 411, "Content-Length is required");
	}
	if (writelen < 0 || writelen > request->contentLength)
	{
		return http_rest_error(request, 400, "invalid initial request body length");
	}
	if (maxaddr <= startaddr || request->contentLength > maxaddr - startaddr)
	{
		return http_rest_error(request, 413, "flash upload exceeds requested range");
	}
#if PLATFORM_BK7238
	if (!bk7238_ota_acquire_owner(BK7238_OTA_OWNER_FLASH))
	{
		return http_rest_error(request, 409, "flash writer is busy");
	}
#else
	if (sector || ota_flash_locked)
	{
		return http_rest_error(request, 409, "flash writer is busy");
	}
#endif
	if (!init_ota(startaddr, maxaddr))
	{
#if PLATFORM_BK7238
		bk7238_ota_release_owner(BK7238_OTA_OWNER_FLASH);
#endif
		return http_rest_error(request, 500, "could not initialize flash writer");
	}

	towrite = request->contentLength;
	while (towrite > 0)
	{
		if (writelen <= 0)
		{
			writebuf = request->received;
			receive_len = request->receivedLenmax < towrite ? request->receivedLenmax : towrite;
			writelen = recv(request->fd, writebuf, receive_len, 0);
		}
		if (writelen <= 0)
		{
			abort_ota();
#if PLATFORM_BK7238
			bk7238_ota_release_owner(BK7238_OTA_OWNER_FLASH);
#endif
			return http_rest_error(request, 400, "flash upload ended before Content-Length");
		}
		if (writelen > towrite)
		{
			writelen = towrite;
		}
		if (!add_otadata((unsigned char*)writebuf, writelen))
		{
			abort_ota();
#if PLATFORM_BK7238
			bk7238_ota_release_owner(BK7238_OTA_OWNER_FLASH);
#endif
			return http_rest_error(request, 500, "flash write failed");
		}
		total += writelen;
		towrite -= writelen;
		writelen = 0;
	}
	if (!close_ota())
	{
#if PLATFORM_BK7238
		bk7238_ota_release_owner(BK7238_OTA_OWNER_FLASH);
#endif
		return http_rest_error(request, 500, "could not finish flash write");
	}
#if PLATFORM_BK7238
	bk7238_ota_release_owner(BK7238_OTA_OWNER_FLASH);
#endif
	ADDLOG_DEBUG(LOG_FEATURE_OTA, "%d total bytes written", total);
	http_setup(request, httpMimeTypeJson);
	hprintf255(request, "{\"size\":%d}", total);
	poststr(request, NULL);
	CFG_IncrementOTACount();
	return 0;
}

#if PLATFORM_BK7238
static int bk7238_http_ota_error(http_request_t *request)
{
    int code = bk7238_ota_stream.error_code ? bk7238_ota_stream.error_code : 500;
    const char *message = bk7238_ota_stream.error_message ?
        bk7238_ota_stream.error_message : "OTA failed";
    bk7238_ota_release_owner(BK7238_OTA_OWNER_POST);
    return http_rest_error(request, code, (char *)message);
}
#endif

int http_rest_post_ota(http_request_t* request)
{
#if PLATFORM_BK7238
    int total = 0;
    int remaining;
    int received;
    int receive_len;
    char *buffer;

    ADDLOG_DEBUG(LOG_FEATURE_OTA, "BK7238 OTA post len %d", request->contentLength);
    if (request->contentLength <= 0)
    {
        return http_rest_error(request, 411, "Content-Length is required");
    }
    if (request->bodylen < 0 || request->bodylen > request->contentLength)
    {
        return http_rest_error(request, 400, "invalid initial request body length");
    }
    if (!bk7238_ota_acquire_owner(BK7238_OTA_OWNER_POST))
    {
        return http_rest_error(request, 409, "flash writer is busy");
    }
    if (!bk7238_ota_begin(BK7238_OTA_OWNER_POST, request->contentLength))
    {
        return bk7238_http_ota_error(request);
    }

    if (request->bodylen &&
        !bk7238_ota_feed(BK7238_OTA_OWNER_POST,
            (unsigned char *)request->bodystart, request->bodylen))
    {
        return bk7238_http_ota_error(request);
    }
    total = request->bodylen;
    remaining = request->contentLength - total;
    buffer = request->received;

    while (remaining > 0)
    {
        receive_len = request->receivedLenmax < remaining ? request->receivedLenmax : remaining;
        received = recv(request->fd, buffer, receive_len, 0);
        if (received <= 0)
        {
            bk7238_ota_fail(BK7238_OTA_OWNER_POST, 400,
                "OTA upload ended before Content-Length");
            return bk7238_http_ota_error(request);
        }
        if (!bk7238_ota_feed(BK7238_OTA_OWNER_POST,
            (unsigned char *)buffer, received))
        {
            return bk7238_http_ota_error(request);
        }
        total += received;
        remaining -= received;
    }

    if (!bk7238_ota_finish(BK7238_OTA_OWNER_POST))
    {
        return bk7238_http_ota_error(request);
    }

    bk7238_ota_release_owner(BK7238_OTA_OWNER_POST);
    http_setup(request, httpMimeTypeJson);
    hprintf255(request, "{\"size\":%d,\"format\":\"%s\"}", total,
        bk_flash_get_boot_profile() == BK_FLASH_BOOT_PROFILE_TUYA_T1 ? "pczl" : "rbl");
    poststr(request, NULL);
    CFG_IncrementOTACount();
    RESET_ScheduleModuleReset(3);
    return 0;
#else
    bk_logic_partition_t *partition = bk_flash_get_info(BK_PARTITION_OTA);

    if (!partition)
    {
        return http_rest_error(request, 500, "OTA partition is unavailable");
    }
    return http_rest_post_flash(request, partition->partition_start_addr,
        partition->partition_start_addr + partition->partition_length);
#endif
}


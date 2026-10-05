#include "../../new_common.h"
#include "../hal_flashConfig.h"
#include "sv6x66_storage.h"
#include "sv6x66_fs.h"
#include <stddef.h>

extern SSV_FS fs_handle;
static xSemaphoreHandle storage_mutex;
static int config_read_failed;
#define STORAGE_MAGIC 0x53563653u
// The checksum covers metadata as well as data, including the sequence.
typedef struct {
    uint32_t magic;
    uint16_t version, kind;
    uint32_t sequence, length, checksum;
} storage_header_t;
static const char *record_name(uint16_t kind, unsigned slot)
{
    if (kind == SV6X66_STORAGE_CFG) return slot ? "obk_cfg1" : "obk_cfg0";
    if (kind == SV6X66_STORAGE_VARS) return slot ? "obk_vars1" : "obk_vars0";
    return NULL;
}
static uint32_t hash_bytes(uint32_t hash, const void *data, uint32_t len)
{
    const uint8_t *p = data;
    while (len--) hash = (hash ^ *p++) * 16777619u;
    return hash;
}
static uint32_t record_hash(const storage_header_t *h, const void *data)
{
    uint32_t hash = hash_bytes(2166136261u, h, offsetof(storage_header_t, checksum));
    return hash_bytes(hash, data, h->length);
}
// Return 1 for valid, 0 for absent/corrupt, -1 for an I/O failure.
static int read_slot(uint16_t kind, unsigned slot, void *data, uint32_t len, uint32_t *sequence)
{
    storage_header_t h;
    SSV_FILE_STAT st;
    SSV_FILE file = FS_open(fs_handle, record_name(kind, slot), SPIFFS_RDONLY, 0);
    int valid = 0;
    if (file < 0) return FS_errno(fs_handle) == SPIFFS_ERR_NOT_FOUND ? 0 : -1;
    if (FS_fstat(fs_handle, file, &st) < 0) valid = -1;
    else if (st.size == sizeof(h) + len) {
        if (FS_read(fs_handle, file, &h, sizeof(h)) != sizeof(h)) valid = -1;
        else if (h.magic == STORAGE_MAGIC && h.version == 1 && h.kind == kind && h.length == len) {
            if (FS_read(fs_handle, file, data, len) != (int32_t)len) valid = -1;
            else valid = record_hash(&h, data) == h.checksum;
        }
    }
    if (FS_close(fs_handle, file) < 0) valid = -1;
    if (valid == 1 && sequence) *sequence = h.sequence;
    return valid;
}
static int latest_slot(uint16_t kind, void *scratch, uint32_t len, uint32_t *sequence)
{
    uint32_t seq[2] = {0, 0};
    int a = read_slot(kind, 0, scratch, len, &seq[0]);
    int b = read_slot(kind, 1, scratch, len, &seq[1]);
    int slot;
    if (a < 0 || b < 0) return -2;
    if (!a && !b) return -1;
    slot = !a ? 1 : !b ? 0 : ((int32_t)(seq[1] - seq[0]) > 0);
    *sequence = seq[slot];
    return slot;
}
void SV6X66_StorageInit(void)
{
    storage_mutex = xSemaphoreCreateMutex();
    SV6X66_FlashVarsInit();
}
int SV6X66_StorageLock(void)
{
    return fs_handle && storage_mutex && xSemaphoreTake(storage_mutex, portMAX_DELAY) == pdTRUE;
}
void SV6X66_StorageUnlock(void) { xSemaphoreGive(storage_mutex); }
int SV6X66_StorageReadRecord(uint16_t kind, void *target, uint32_t len)
{
    uint32_t sequence;
    int slot, valid = -1;
    void *scratch;
    if (!target || !len || len > INT32_MAX - sizeof(storage_header_t) || !record_name(kind, 0)) return -1;
    scratch = os_malloc(len);
    if (!scratch) return -1;
    if (SV6X66_StorageLock()) {
        slot = latest_slot(kind, scratch, len, &sequence);
        if (slot == -1) valid = 0;
        if (slot >= 0 && read_slot(kind, slot, scratch, len, NULL) == 1) {
            memcpy(target, scratch, len);
            valid = len;
        }
        SV6X66_StorageUnlock();
    }
    os_free(scratch);
    return valid;
}
int SV6X66_StorageWriteRecord(uint16_t kind, const void *source, uint32_t len)
{
    storage_header_t h = {STORAGE_MAGIC, 1, kind, 0, len, 0};
    uint32_t sequence = 0, checked_sequence;
    SSV_FILE file;
    int slot, valid = 0;
    void *scratch;
    if (!source || !len || len > INT32_MAX - sizeof(h) || !record_name(kind, 0)) return 0;
    scratch = os_malloc(len);
    if (!scratch) return 0;
    if (SV6X66_StorageLock()) {
        slot = latest_slot(kind, scratch, len, &sequence);
        if (slot == -2) {
            SV6X66_StorageUnlock();
            os_free(scratch);
            return 0;
        }
        slot = slot < 0 ? 0 : slot ^ 1;
        h.sequence = sequence + 1;
        h.checksum = record_hash(&h, source);
        file = FS_open(fs_handle, record_name(kind, slot), SPIFFS_CREAT | SPIFFS_TRUNC | SPIFFS_WRONLY, 0);
        if (file >= 0) {
            int written = FS_write(fs_handle, file, &h, sizeof(h)) == sizeof(h) &&
                FS_write(fs_handle, file, (void *)source, len) == (int32_t)len;
            if (FS_close(fs_handle, file) < 0) written = 0;
            if (written && read_slot(kind, slot, scratch, len, &checked_sequence) == 1 &&
                checked_sequence == h.sequence && !memcmp(source, scratch, len)) valid = len;
        }
        SV6X66_StorageUnlock();
    }
    os_free(scratch);
    return valid;
}
int HAL_Configuration_ReadConfigMemory(void *target, int len)
{
    int result = len > 0 ? SV6X66_StorageReadRecord(SV6X66_STORAGE_CFG, target, len) : 0;
    // The shared app falls back to defaults on a failed read. Do not let
    // those defaults replace unknown stored configuration in this boot.
    config_read_failed = result < 0;
    return result > 0 ? result : 0;
}
int HAL_Configuration_SaveConfigMemory(void *source, int len)
{
    return !config_read_failed && len > 0 && SV6X66_StorageWriteRecord(SV6X66_STORAGE_CFG, source, len) == len;
}

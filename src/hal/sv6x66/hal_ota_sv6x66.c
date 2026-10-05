#include "../../new_common.h"
#include "../../new_cfg.h"
#include "../../httpserver/new_http.h"
#include "../hal_ota.h"
#include "sv6x66_storage.h"
#include "ota_stage.h"
#include "sv6x66_fs.h"
#include "tools/ota_api/md5.h"

extern SSV_FS fs_handle;
typedef struct { SSV_FILE file; } ota_file_t;
static int marker_absent(void)
{
    SSV_FILE_STAT st;
    return FS_stat(fs_handle, "ota_info.bin", &st) < 0 && FS_errno(fs_handle) == SPIFFS_ERR_NOT_FOUND;
}
static int disarm(void *context)
{
    SSV_FILE_STAT st;
    (void)context;
    if (FS_stat(fs_handle, "ota_info.bin", &st) == 0 && FS_remove(fs_handle, "ota_info.bin") < 0) return -1;
    return marker_absent() ? 0 : -1;
}
int SV6X66_OTAStartup(void)
{
    int result;
#if defined(SV6X66_STOCK_CKW04)
    return 0; // Preserve stock OTA markers; its activation contract differs.
#endif
    if (!SV6X66_StorageLock()) return -1;
    result = disarm(NULL);
    // Marker removal must be confirmed before touching the previous payload.
    if (!result) FS_remove(fs_handle, "ota.bin");
    SV6X66_StorageUnlock();
    return result;
}
static int begin_payload(void *context)
{
    ota_file_t *state = context;
    state->file = FS_open(fs_handle, "ota.bin", SPIFFS_CREAT | SPIFFS_TRUNC | SPIFFS_WRONLY, 0);
    return state->file < 0 ? -1 : 0;
}
static int write_payload(void *context, const uint8_t *data, size_t length)
{
    ota_file_t *state = context;
    return FS_write(fs_handle, state->file, (void *)data, length) == length ? 0 : -1;
}
static int close_payload(void *context)
{
    ota_file_t *state = context;
    int result = FS_close(fs_handle, state->file);
    state->file = -1;
    return result < 0 ? -1 : 0;
}
static int verify_payload(void *context, uint32_t length, const uint8_t expected[16])
{
    SSV_FILE_STAT st;
    MD5_CTX md5;
    uint8_t buffer[512], digest[16];
    uint32_t left = length;
    SSV_FILE file = FS_open(fs_handle, "ota.bin", SPIFFS_RDONLY, 0);
    int valid;
    (void)context;
    if (file < 0) return -1;
    valid = FS_fstat(fs_handle, file, &st) == 0 && st.size == length;
    MD5_Init(&md5);
    while (valid && left) {
        uint32_t chunk = left > sizeof(buffer) ? sizeof(buffer) : left;
        if (FS_read(fs_handle, file, buffer, chunk) != chunk) { valid = 0; break; }
        MD5_Update(&md5, buffer, chunk);
        left -= chunk;
    }
    MD5_Final(digest, &md5);
    if (FS_close(fs_handle, file) < 0) valid = 0;
    return valid && !memcmp(digest, expected, 16) ? 0 : -1;
}
static int publish_marker(void *context, const uint8_t digest[16])
{
    SSV_FILE_STAT st;
    uint8_t readback[16];
    SSV_FILE file = FS_open(fs_handle, "ota_info.bin", SPIFFS_CREAT | SPIFFS_TRUNC | SPIFFS_WRONLY, 0);
    int valid;
    (void)context;
    if (file < 0) return -1;
    // Payload is fully validated before creation: existence itself activates OTA.
    valid = FS_write(fs_handle, file, (void *)digest, 16) == 16;
    if (FS_close(fs_handle, file) < 0) valid = 0;
    if (!valid) return -1;
    file = FS_open(fs_handle, "ota_info.bin", SPIFFS_RDONLY, 0);
    if (file < 0) return -1;
    valid = FS_fstat(fs_handle, file, &st) == 0 && st.size == 16 &&
        FS_read(fs_handle, file, readback, 16) == 16 && !memcmp(digest, readback, 16);
    if (FS_close(fs_handle, file) < 0) valid = 0;
    return valid ? 0 : -1;
}
int http_rest_post_flash(http_request_t *request, int startaddr, int maxaddr)
{
    ota_stage_t stage;
    ota_file_t state = {-1};
    ota_stage_ops_t ops = { &state, disarm, begin_payload, write_payload, close_payload, verify_payload, publish_marker };
    int remaining, result;
    int receive_timeout_ms = 30000;
    (void)startaddr; (void)maxaddr;
    if (!request) return -1;
#if defined(SV6X66_STOCK_CKW04)
    return http_rest_error(request, 501, "Stock CKW04 OTA is not supported; use the UART artifact");
#endif
    if (request->contentLength <= 0 || request->bodylen < 0 ||
        request->bodylen > request->contentLength || request->receivedLenmax <= 0 ||
        ota_stage_init(&stage, &ops, request->contentLength))
        return http_rest_error(request, 400, "Invalid SV6166F OTA envelope length");
    if (setsockopt(request->fd, SOL_SOCKET, SO_RCVTIMEO, &receive_timeout_ms, sizeof(receive_timeout_ms)) < 0)
        return http_rest_error(request, 500, "Cannot set OTA receive timeout");
    if (!SV6X66_StorageLock()) return http_rest_error(request, 500, "SV6166F storage unavailable");
    OTA_ResetProgress();
    OTA_IncrementProgress(1);
    OTA_SetTotalBytes(request->contentLength);
    result = ota_stage_feed(&stage, request->bodystart, request->bodylen);
    remaining = request->contentLength - request->bodylen;
    OTA_IncrementProgress(request->bodylen);
    while (!result && remaining > 0) {
        int limit = remaining < request->receivedLenmax ? remaining : request->receivedLenmax;
        int received = recv(request->fd, request->received, limit, 0);
        if (received <= 0) { result = -1; break; }
        result = ota_stage_feed(&stage, request->received, received);
        remaining -= received;
        OTA_IncrementProgress(received);
    }
    if (!result) result = ota_stage_finish(&stage);
    if (result) ota_stage_abort(&stage);
    SV6X66_StorageUnlock();
    OTA_ResetProgress();
    if (result && stage.disarm_failed)
        return http_rest_error(request, 500, "OTA activation state uncertain; staged image preserved");
    if (result) return http_rest_error(request, 400, "SV6166F OTA validation or storage failed");
    CFG_IncrementOTACount();
    http_setup(request, httpMimeTypeJson);
    hprintf255(request, "{\"size\":%d}", request->contentLength);
    poststr(request, NULL);
    RESET_ScheduleModuleReset(3);
    return 0;
}

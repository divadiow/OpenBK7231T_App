#ifndef SV6X66_OTA_STAGE_H
#define SV6X66_OTA_STAGE_H
#include <stddef.h>
#include <stdint.h>
#define OTA_STAGE_HEADER_SIZE 48u
#define OTA_STAGE_PREFIX_SIZE 40u
// Envelope integers use little-endian byte order. CRC32 covers the first 44 bytes.
#define OTA_STAGE_MAGIC "OBKSV616"
typedef struct {
    void *context;
    int (*disarm_marker)(void *);
    int (*begin_payload)(void *);
    int (*write_payload)(void *, const uint8_t *, size_t);
    int (*close_payload)(void *);
    int (*verify_payload)(void *, uint32_t, const uint8_t[16]);
    int (*publish_marker)(void *, const uint8_t[16]);
} ota_stage_ops_t;
typedef struct {
    ota_stage_ops_t ops;
    uint8_t header[48], prefix[40], digest[16];
    uint32_t request_length, payload_length, received;
    size_t header_used;
    int opened, failed, finished, disarm_failed;
} ota_stage_t;
int ota_stage_init(ota_stage_t *, const ota_stage_ops_t *, uint32_t request_length);
int ota_stage_feed(ota_stage_t *, const void *, size_t);
int ota_stage_finish(ota_stage_t *);
void ota_stage_abort(ota_stage_t *);
int ota_stage_layout_valid(const uint8_t prefix[40]);
#endif

#include "ota_stage.h"
#include "sv6x66_layout.h"
#include <string.h>

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static uint32_t crc32(const uint8_t *p, size_t n)
{
    uint32_t crc = 0xffffffffu;
    while (n--) {
        unsigned bit;
        crc ^= *p++;
        for (bit = 0; bit < 8; bit++) crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return crc ^ 0xffffffffu;
}
int ota_stage_layout_valid(const uint8_t p[40])
{
    return le32(p + 4) == 40 && le32(p + 8) == 80 && le32(p + 12) == 4 &&
        le32(p + 16) == SV6X66_MAIN_SIZE && le32(p + 20) == SV6X66_FLASH_SIZE &&
        le32(p + 24) == 0 && le32(p + 28) == 0 && le32(p + 36) == SV6X66_RAW_SIZE;
}
void ota_stage_abort(ota_stage_t *s)
{
    if (!s || s->finished) return;
    s->failed = 1;
    if (s->opened) { s->ops.close_payload(s->ops.context); s->opened = 0; }
    // Never truncate/remove the payload if disarming fails.
    if (s->ops.disarm_marker(s->ops.context)) s->disarm_failed = 1;
}
int ota_stage_init(ota_stage_t *s, const ota_stage_ops_t *ops, uint32_t request_length)
{
    if (!s || !ops || !ops->disarm_marker || !ops->begin_payload || !ops->write_payload ||
        !ops->close_payload || !ops->verify_payload || !ops->publish_marker) return -1;
    memset(s, 0, sizeof(*s));
    s->ops = *ops;
    s->request_length = request_length;
    if (request_length <= OTA_STAGE_HEADER_SIZE + SV6X66_APP_START ||
        request_length > OTA_STAGE_HEADER_SIZE + SV6X66_FS_START) { s->failed = 1; return -1; }
    return 0;
}
static int header_ready(ota_stage_t *s)
{
    const uint8_t *h = s->header;
    uint32_t length = le32(h + 12);
    if (memcmp(h, OTA_STAGE_MAGIC, 8) || le32(h + 8) != 1 || le32(h + 40) ||
        le32(h + 44) != crc32(h, 44) || le32(h + 16) != SV6X66_APP_START ||
        le32(h + 20) != SV6X66_FS_START || length <= SV6X66_APP_START ||
        length > SV6X66_FS_START || s->request_length != OTA_STAGE_HEADER_SIZE + length) return -1;
    s->payload_length = length;
    memcpy(s->digest, h + 24, 16);
    if (s->ops.disarm_marker(s->ops.context) || s->ops.begin_payload(s->ops.context)) return -1;
    s->opened = 1;
    return 0;
}
int ota_stage_feed(ota_stage_t *s, const void *data, size_t n)
{
    const uint8_t *p = data;
    if (!s || s->failed || s->finished || (!data && n)) return -1;
    if (!n) return 0;
    if (s->header_used < 48) {
        size_t take = 48 - s->header_used;
        if (take > n) take = n;
        memcpy(s->header + s->header_used, p, take);
        s->header_used += take; p += take; n -= take;
        if (s->header_used < 48) return 0;
        if (header_ready(s)) { ota_stage_abort(s); return -1; }
    }
    if (n > s->payload_length - s->received) { ota_stage_abort(s); return -1; }
    if (s->received < 40) {
        size_t take = 40 - s->received;
        if (take > n) take = n;
        memcpy(s->prefix + s->received, p, take);
    }
    if (n && s->ops.write_payload(s->ops.context, p, n)) { ota_stage_abort(s); return -1; }
    s->received += n;
    return 0;
}
int ota_stage_finish(ota_stage_t *s)
{
    int closed;
    if (!s || s->failed || s->finished) return -1;
    if (s->header_used != 48 || !s->opened || s->received != s->payload_length) {
        ota_stage_abort(s); return -1;
    }
    closed = s->ops.close_payload(s->ops.context);
    s->opened = 0;
    if (closed || !ota_stage_layout_valid(s->prefix) ||
        s->ops.verify_payload(s->ops.context, s->payload_length, s->digest) ||
        s->ops.publish_marker(s->ops.context, s->digest)) {
        ota_stage_abort(s); return -1;
    }
    s->finished = 1;
    return 0;
}

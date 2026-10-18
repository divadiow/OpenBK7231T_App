#if PLATFORM_SV6X66

#include "new_common.h"
#include "../../../platforms/SV6X66/obk/ota.h"
#include "../../../platforms/SV6X66/obk/flash.h"
#include "crypto/cryptoApi.h"
#include "httpserver/new_http.h"
#include "httpserver/rest_interface.h"
#include "hal/hal_ota.h"
#include <string.h>

static struct {
	uint32_t size[2], done[2], target, total;
	uint8_t expected[2][16], header[84], prefix[4];
	psDigestContext_t digest[2];
	int locked, carrier, erased, verified;
} update;
static uint32_t sv6x66_read_le32(const uint8_t *p) {
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static int hexvalue(char c) {
	if (c >= '0' && c <= '9')
		return c - '0';
	if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
	return -1;
}
void SV6X66_UpdateAbort(void) {
	if (update.locked)
		SV6X66_FlashUnlock();
	memset(&update, 0, sizeof(update));
}
int SV6X66_UpdateBegin(const uint8_t *header, int carrier) {
	uint32_t current = SV6X66_RunningBank(), i, j;
	/* ProfileValid requires startup JEDEC identification, actual XIP bank,
		* selector agreement and erased selector tail before any flash lock. */
	if (update.locked || !SV6X66_ProfileValid() || sv6x66_read_le32(header) != 0x73737600 ||
		sv6x66_read_le32(header + 12) != 1 || sv6x66_read_le32(header + 16) != 0 || SV6X66_FlashLock())
		return -1;
	memset(&update, 0, sizeof(update));
	update.locked = 1;
	update.carrier = carrier;
	update.target = current == SV6X66_BANK_A ? 1 : 0;
	memcpy(update.header, header, 84);
	for (i = 0; i < 2; i++) {
		update.size[i] = sv6x66_read_le32(header + 4 + 4 * i);
		if (update.size[i] > SV6X66_BANK_SIZE || (!carrier && update.size[i] < 68))
			goto fail;
		if (carrier && ((i == update.target && update.size[i] < 36) || (i != update.target && update.size[i] != 0)))
			goto fail;
		for (j = 0; j < 16; j++) {
			int hi = hexvalue(header[20 + i * 32 + j * 2]), lo = hexvalue(header[21 + i * 32 + j * 2]);
			if (hi < 0 || lo < 0)
				goto fail;
			update.expected[i][j] = (hi << 4) | lo;
		}
		psMd5Init(&update.digest[i]);
		update.total += update.size[i];
	}
	return 0;
fail:
	SV6X66_UpdateAbort();
	return -1;
}
uint32_t SV6X66_UpdateRemaining(void) {
	return update.locked ? update.total - update.done[0] - update.done[1] : 0;
}
int SV6X66_UpdateData(const uint8_t *data, size_t length) {
	if (!update.locked || update.verified || length > SV6X66_UpdateRemaining())
		return -1;
	while (length) {
		uint32_t index = update.done[0] < update.size[0] ? 0 : 1;
		uint32_t n = update.size[index] - update.done[index];
		uint32_t bank = index ? SV6X66_BANK_B : SV6X66_BANK_A;
		if (n > length)
			n = length;
		if (index == update.target) {
			if (!update.erased) {
				uint32_t prefix = 4 - update.done[index], sector;
				if (prefix > n)
					prefix = n;
				memcpy(update.prefix + update.done[index], data, prefix);
				psMd5Update(&update.digest[index], data, prefix);
				update.done[index] += prefix;
				data += prefix;
				length -= prefix;
				if (update.done[index] < 4)
					continue;
				if (memcmp(update.prefix, "\x48\x00\x00\x02", 4))
					goto fail;
				for (sector = 0; sector < SV6X66_BANK_SIZE; sector += 4096) {
					if (SV6X66_FlashErase(bank + sector))
						goto fail;
					taskYIELD();
				}
				update.erased = 1;
				if (SV6X66_FlashWrite(bank, update.prefix, 4))
					goto fail;
				continue;
			}
			if (SV6X66_FlashWrite(bank + update.done[index], data, n))
				goto fail;
		}
		psMd5Update(&update.digest[index], data, n);
		update.done[index] += n;
		data += n;
		length -= n;
	}
	return 0;
fail:
	SV6X66_UpdateAbort();
	return -1;
}
static uint32_t crc32(const uint8_t *p, uint32_t n) {
	uint32_t c = UINT32_MAX, i;
	while (n--) {
		c ^= *p++;
		for (i = 0; i < 8; i++)
			c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1)));
	}
	return c ^ UINT32_MAX;
}
int SV6X66_UpdateVerify(void) {
	uint8_t digest[16];
	psDigestContext_t readback;
	uint32_t i, bank = update.target ? SV6X66_BANK_B : SV6X66_BANK_A;
	uint32_t length = update.size[update.target];
	const uint8_t *raw = (const uint8_t *)(SV6X66_FLASH_BASE + bank), *id;
	if (!update.locked || SV6X66_UpdateRemaining() || !update.erased || update.verified)
		return -1;
	for (i = 0; i < 2; i++) {
		if (psMd5Final(&update.digest[i], digest) < 0 || memcmp(digest, update.expected[i], 16))
			goto fail;
	}
	psMd5Init(&readback);
	psMd5Update(&readback, raw, length);
	if (psMd5Final(&readback, digest) < 0 || memcmp(digest, update.expected[update.target], 16))
		goto fail;
	if (update.carrier) {
		id = raw + length - 32;
		if (memcmp(id, "SV6CARR1", 8) || sv6x66_read_le32(id + 8) != update.target || sv6x66_read_le32(id + 12) != bank ||
			sv6x66_read_le32(id + 16) != length || sv6x66_read_le32(id + 20) != crc32(raw, length - 32) || sv6x66_read_le32(id + 24) || sv6x66_read_le32(id + 28))
			goto fail;
		for (i = length; i < SV6X66_BANK_SIZE; i++)
			if (raw[i] != 0xff)
				goto fail;
	} else if (SV6X66_ValidateBank(bank, length))
		goto fail;
	update.verified = 1;
	return 0;
fail:
	SV6X66_UpdateAbort();
	return -1;
}
int SV6X66_UpdateChecksums(const char *ascii) {
	return update.locked && !memcmp(ascii, update.header + 20, 64) ? 0 : -1;
}
int SV6X66_UpdateCommit(void) {
	int result;
	if (!update.locked || !update.verified)
		return -1;
	result = SV6X66_CommitBank(update.target ? SV6X66_BANK_B : SV6X66_BANK_A);
	SV6X66_UpdateAbort();
	return result;
}

int http_rest_post_flash(http_request_t *request, int startaddr, int maxaddr) {
	uint8_t header[84], buffer[512];
	size_t have = 0;
	int received = request->bodylen, remaining = request->contentLength, owned = 0;
	const uint8_t *data = (const uint8_t *)request->bodystart;
	(void)startaddr;
	(void)maxaddr;
	if (!SV6X66_ProfileValid())
		return http_rest_error(request, -1, "CKW04 physical flash/bank profile validation failed");
	if (remaining < 84 || remaining > 84 + 2 * SV6X66_BANK_SIZE || received < 0 || received > remaining)
		goto fail;
	while (have < 84) {
		size_t n = 84 - have;
		if (!received) {
			received = recv(request->fd, buffer, sizeof(buffer), 0);
			data = buffer;
			if (received <= 0 || received > remaining)
				goto fail;
		}
		if (n > (size_t)received)
			n = received;
		memcpy(header + have, data, n);
		have += n;
		data += n;
		received -= n;
		remaining -= n;
	}
	if (SV6X66_UpdateBegin(header, 0))
		goto fail;
	owned = 1;
	if (SV6X66_UpdateRemaining() != (uint32_t)remaining)
		goto fail;
	OTA_ResetProgress();
	OTA_IncrementProgress(1);
	OTA_SetTotalBytes(remaining);
	while (remaining) {
		if (!received) {
			int n = remaining < (int)sizeof(buffer) ? remaining : sizeof(buffer);
			received = recv(request->fd, buffer, n, 0);
			data = buffer;
			if (received <= 0 || received > remaining)
				goto fail;
		}
		if (SV6X66_UpdateData(data, received))
			goto fail;
		remaining -= received;
		OTA_IncrementProgress(received);
		received = 0;
	}
	if (SV6X66_UpdateVerify() || SV6X66_UpdateCommit())
		goto fail;
	owned = 0;
	http_setup(request, httpMimeTypeJson);
	poststr(request, "{\"status\":\"verified\"}");
	poststr(request, NULL);
	RESET_ScheduleModuleReset(2);
	return 0;
fail:
	if (owned)
		SV6X66_UpdateAbort();
	OTA_ResetProgress();
	return http_rest_error(request, -1, "SV6X66 image, transport or flash verification failed");
}

#endif

#include "sv6x66_loader.h"
#include "hsuart/drv_hsuart.h"
#include "pinmux/drv_pinmux.h"
#include "wdt/drv_wdt.h"
#include "sys/flash.h"
#include "sys/xip.h"
#include "xz.h"
#include "crypto/cryptoApi.h"
#include <stddef.h>
#include <stdint.h>

#if defined(SV6X66_BANK_B_LOADER)

#define FLASH_XIP_BASE 0x30000000u
#define BANK_B_XIP 0x300A6000u
#define BANK_B_END_XIP 0x30141000u
#define BOOT_FLAG_OFFSET 0x00004000u
#define LOADER_BAUD 115200u
#define ACK 0x06u
#define NAK 0x15u
#define STAGE_HEADER_BYTES 80u
#define STAGE_PACKED_MAX 65536u
#define STAGE_LOAD_MIN 0x00004000u
#define STAGE_LOAD_END 0x0000F400u
#define STAGE_DICT_BYTES 8192u
#define STAGE_FLAGS 1u

static uint8_t stage_packed[STAGE_PACKED_MAX];

static uint32_t get_le32(const uint8_t *data)
{
	return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
		((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t length)
{
	while (length--) {
		uint32_t bit;
		crc ^= *data++;
		for (bit = 0u; bit < 8u; bit++)
			crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
	}
	return crc;
}

static uint32_t crc32(const uint8_t *data, size_t length)
{
	return crc32_update(0xFFFFFFFFu, data, length) ^ 0xFFFFFFFFu;
}

static void uart_write_all(const void *data, size_t length)
{
	const uint8_t *position = (const uint8_t *)data;
	while (length != 0u) {
		int32_t sent = drv_hsuart_write_fifo(position, (int32_t)length, HSUART_BLOCKING);
		if (sent > 0) {
			position += sent;
			length -= (size_t)sent;
		}
	}
	while (!drv_hsuart_is_transmitter_idle()) {}
}

static uint8_t uart_read_byte(void)
{
	uint8_t value;
	while (drv_hsuart_read_fifo(&value, 1, HSUART_BLOCKING) != 1) {}
	return value;
}

static void uart_read_all(void *data, size_t length)
{
	uint8_t *position = (uint8_t *)data;
	while (length != 0u) {
		int32_t received = drv_hsuart_read_fifo(position, (int32_t)length, HSUART_BLOCKING);
		if (received > 0) {
			position += received;
			length -= (size_t)received;
		}
	}
}

static void uart_text(const char *text)
{
	const char *end = text;
	while (*end != '\0')
		end++;
	uart_write_all(text, (size_t)(end - text));
}

static int stage_header_valid(const uint8_t *header)
{
	static const uint8_t magic[8] = {'S','V','6','S','T','G','0','1'};
	uint32_t load;
	uint32_t entry;
	uint32_t raw_size;
	uint32_t packed_size;
	uint32_t i;
	for (i = 0u; i < sizeof(magic); i++)
		if (header[i] != magic[i])
			return 0;
	load = get_le32(header + 16);
	entry = get_le32(header + 20);
	raw_size = get_le32(header + 24);
	packed_size = get_le32(header + 28);
	if (get_le32(header + 8) != 1u || get_le32(header + 12) != STAGE_HEADER_BYTES ||
		get_le32(header + 32) != STAGE_DICT_BYTES || get_le32(header + 36) != STAGE_FLAGS)
		return 0;
	if (load < STAGE_LOAD_MIN || load >= STAGE_LOAD_END || raw_size == 0u ||
		raw_size > STAGE_LOAD_END - load)
		return 0;
	if (entry < load || entry >= load + raw_size)
		return 0;
	return packed_size != 0u && packed_size <= STAGE_PACKED_MAX;
}

static int stage_sha256_valid(const uint8_t *data, uint32_t length, const uint8_t *expected)
{
	psDigestContext_t context;
	uint8_t digest[32];
	uint32_t i;
	psSha256Init(&context);
	psSha256Update(&context, data, length);
	if (psSha256Final(&context, digest) < 0)
		return 0;
	for (i = 0u; i < sizeof(digest); i++)
		if (digest[i] != expected[i])
			return 0;
	return 1;
}

static void run_stage_agent(void)
{
	uint8_t header[STAGE_HEADER_BYTES];
	uint32_t load;
	uint32_t raw_size;
	uint32_t packed_size;
	struct xz_dec *decoder;
	struct xz_buf buffers;
	enum xz_ret result;

	uart_text("SV6LDR02 STAGE SEND HEADER=80 PACKED_MAX=65536\r\n");
	uart_read_all(header, sizeof(header));
	if (!stage_header_valid(header)) {
		uart_write_all((const uint8_t[]){NAK}, 1u);
		uart_text("SV6LDR02 STAGE FAIL_HEADER\r\n");
		return;
	}
	uart_write_all((const uint8_t[]){ACK}, 1u);
	load = get_le32(header + 16);
	raw_size = get_le32(header + 24);
	packed_size = get_le32(header + 28);
	uart_read_all(stage_packed, packed_size);
	if (crc32(stage_packed, packed_size) != get_le32(header + 44)) {
		uart_text("SV6LDR02 STAGE FAIL_PACKED_CRC\r\n");
		return;
	}

	xz_crc32_init();
	SV6X66_XzArenaReset();
	decoder = xz_dec_init(XZ_SINGLE, 0u);
	if (decoder == NULL) {
		uart_text("SV6LDR02 STAGE FAIL_DECODER_ALLOC\r\n");
		return;
	}
	buffers.in = stage_packed;
	buffers.in_pos = 0u;
	buffers.in_size = packed_size;
	buffers.out = (uint8_t *)(uintptr_t)load;
	buffers.out_pos = 0u;
	buffers.out_size = raw_size;
	result = xz_dec_run(decoder, &buffers);
	xz_dec_end(decoder);
	if (result != XZ_STREAM_END || buffers.in_pos != packed_size || buffers.out_pos != raw_size) {
		uart_text("SV6LDR02 STAGE FAIL_XZ\r\n");
		return;
	}
	if (crc32((const uint8_t *)(uintptr_t)load, raw_size) != get_le32(header + 40)) {
		uart_text("SV6LDR02 STAGE FAIL_RAW_CRC\r\n");
		return;
	}
	if (!stage_sha256_valid((const uint8_t *)(uintptr_t)load, raw_size, header + 48)) {
		uart_text("SV6LDR02 STAGE FAIL_RAW_SHA256\r\n");
		return;
	}
	uart_text("SV6LDR02 STAGE VALIDATED JUMP\r\n");
	((void (*)(void))(uintptr_t)get_le32(header + 20))();
	uart_text("SV6LDR02 STAGE FAIL_RETURNED\r\n");
}

void SV6X66_Loader(void)
{
	volatile uint32_t *const boot_flag = (volatile uint32_t *)(FLASH_XIP_BASE + BOOT_FLAG_OFFSET);
	uintptr_t self = (uintptr_t)&SV6X66_Loader;

	xip_init();
	xip_enter();
	drv_wdt_init();
	drv_wdt_disable(MCU_WDT);
	drv_wdt_disable(SYS_WDT);

	if (self >= BANK_B_XIP && self < BANK_B_END_XIP && *boot_flag == 0u && flash_init())
		flash_sector_erase(BOOT_FLAG_OFFSET);

	drv_pinmux_manual_function_select_enable(SEL_UART1);
	drv_hsuart_init();
	drv_hsuart_sw_rst();
	drv_hsuart_set_fifo(HSUART_INT_RX_FIFO_TRIG_LV_1);
	if (drv_hsuart_set_format(LOADER_BAUD, HSUART_WLS_8, HSUART_STB_1, HSUART_PARITY_DISABLE) != 0)
		for (;;) {}

	if (!(self >= BANK_B_XIP && self < BANK_B_END_XIP)) {
		uart_text("SV6LDR02 REFUSE_NOT_BANK_B\r\n");
		for (;;) {}
	}
	if (*boot_flag != 0xFFFFFFFFu) {
		uart_text("SV6LDR02 ROLLBACK_ARM_FAILED\r\n");
		for (;;) {}
	}

	for (;;) {
		uint8_t command;
		uart_text("SV6LDR02 READY A_ARMED STAGE BAUD=115200\r\n");
		do {
			command = uart_read_byte();
		} while (command != (uint8_t)'S');
		run_stage_agent();
	}
}

#endif

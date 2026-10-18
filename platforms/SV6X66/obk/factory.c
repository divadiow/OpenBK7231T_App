#include "new_common.h"
#include "hal/hal_generic.h"
#include "ota.h"
#include "hsuart/drv_hsuart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int sendall(const void *source, size_t length) {
	const uint8_t *p = source;
	while (length) {
		int n = drv_hsuart_write_fifo(p, length, HSUART_BLOCKING);
		if (n <= 0)
			return -1;
		p += n;
		length -= n;
	}
	while (!drv_hsuart_is_transmitter_idle()) {
	}
	return 0;
}
static int text(const char *s) {
	return sendall(s, strlen(s));
}
static int readall(void *destination, size_t length, uint32_t timeout) {
	uint8_t *p = destination;
	uint32_t until = OS_GetSysTick() + OS_MS2TICK(timeout);
	while (length) {
		int n = drv_hsuart_read_fifo(p, length, HSUART_NON_BLOCKING);
		if (n > 0) {
			p += n;
			length -= n;
		} else {
			if ((int32_t)(OS_GetSysTick() - until) >= 0)
				return -1;
			OS_MsDelay(1);
		}
	}
	return 0;
}
static int line(char *buffer, size_t capacity, uint32_t timeout) {
	size_t n = 0;
	while (n + 1 < capacity) {
		if (readall(buffer + n, 1, timeout))
			return -1;
		if (buffer[n++] == '\n') {
			buffer[n] = 0;
			return (int)n;
		}
	}
	return -1;
}
static uint8_t crc8(const uint8_t *p, size_t n) {
	uint8_t c = 0;
	while (n--) {
		int i;
		c ^= *p++;
		for (i = 0; i < 8; i++)
			c = (c & 128) ? (uint8_t)((c << 1) ^ 7) : (uint8_t)(c << 1);
	}
	return c;
}
static int receive_carrier(void) {
	uint8_t frame[512], ack[5], header[84];
	uint16_t sequence = 0;
	uint32_t remaining, first;
	/* First frame contains the entire validated header. No erase occurs until
		* a complete inactive-bank payload prefix has been received and checked. */
	if (readall(frame, 88, 10000) || frame[0] != 0x55 || frame[1] != 0xaa || frame[2] || frame[3])
		return -1;
	memcpy(header, frame + 4, 84);
	if (SV6X66_UpdateBegin(header, 1))
		return -1;
	remaining = SV6X66_UpdateRemaining();
	first = remaining < 424 ? remaining : 424;
	if (readall(frame + 88, first, 10000) || SV6X66_UpdateData(frame + 88, first))
		goto fail;
	remaining -= first;
	memcpy(ack, frame, 4);
	ack[4] = crc8(frame + 4, 84 + first);
	if (sendall(ack, 5))
		goto fail;
	sequence++;
	while (remaining) {
		uint32_t n = remaining < 508 ? remaining : 508;
		if (readall(frame, 4 + n, 10000) || frame[0] != 0x55 || frame[1] != 0xaa ||
			frame[2] != (uint8_t)(sequence >> 8) || frame[3] != (uint8_t)sequence)
			goto fail;
		if (SV6X66_UpdateData(frame + 4, n))
			goto fail;
		memcpy(ack, frame, 4);
		ack[4] = crc8(frame + 4, n);
		if (sendall(ack, 5))
			goto fail;
		sequence++;
		remaining -= n;
	}
	return 0;
fail:
	SV6X66_UpdateAbort();
	return -1;
}
int SV6X66_FactoryWindow(void) {
	char command[128], reply[160];
	uint32_t until = OS_GetSysTick() + OS_MS2TICK(3000);
	int entered = 0, staged = 0, checksums = 0;
	SV6X66_LogEnable(0);
	while ((int32_t)(OS_GetSysTick() - until) < 0) {
		text("factory?\r\n");
		if (line(command, sizeof(command), 500) > 0 && !strcmp(command, "factory!\r\n")) {
			entered = 1;
			break;
		}
	}
	if (!entered) {
		SV6X66_LogEnable(1);
		return 0;
	}
	text("factory mode\r\n");
	for (;;) {
		if (line(command, sizeof(command), 30000) <= 0)
			goto fail;
		if (!strcmp(command, "AT+VER\r\n"))
			text("OpenSV6X66/1 CKW04\r\n");
		else if (!strcmp(command, "AT+OBK_BANK?\r\n")) {
			uint32_t bank = SV6X66_RunningBank();
			if (!SV6X66_ProfileValid()) {
				text("FAILED\r\n");
				continue;
			}
			snprintf(reply, sizeof(reply), "OBKSV6/1 CKW04 RUN=%c SEL=%08X BASE=%08X SIZE=0009B000 FLASH=00200000\r\n",
					 bank == SV6X66_BANK_A ? 'A' : 'B', (unsigned int)(bank == SV6X66_BANK_A ? UINT32_MAX : 0), (unsigned int)bank);
			text(reply);
		} else if (!strncmp(command, "AT+BAUD=", 8)) {
			char *end;
			unsigned long baud = strtoul(command + 8, &end, 10);
			if (strcmp(end, "\r\n") || (baud != 9600 && baud != 115200 && baud != 460800 && baud != 921600 && baud != 1500000 && baud != 2000000)) {
				text("FAILED\r\n");
				continue;
			}
			text("OK\r\n");
			OS_MsDelay(20);
			if (drv_hsuart_set_format(baud, HSUART_WLS_8, HSUART_STB_1, HSUART_PARITY_DISABLE))
				goto fail;
		} else if (!strcmp(command, "AT+UPGRADE_START\r\n") && !staged) {
			if (!SV6X66_ProfileValid()) {
				text("FAILED\r\n");
				continue;
			}
			text("OK\r\n");
			if (receive_carrier())
				goto fail;
			staged = 1;
		} else if (!strncmp(command, "AT+VERIFT=", 10) && staged && strlen(command) == 76) {
			if (SV6X66_UpdateChecksums(command + 10) || SV6X66_UpdateVerify())
				goto fail;
			checksums = 1;
			text("OK\r\n");
		} else if (!strcmp(command, "AT+UPGRADE_END\r\n") && staged && checksums) {
			if (SV6X66_UpdateCommit())
				goto fail;
			text("OK\r\n");
			HAL_RebootModule();
		} else
			text("FAILED\r\n");
	}
fail:
	SV6X66_UpdateAbort();
	text("FAILED\r\n");
	/* Do not silently continue a partially completed recovery session into
		* application startup. Reset establishes a fresh UART/selector session. */
	for (;;)
		OS_MsDelay(1000);
}

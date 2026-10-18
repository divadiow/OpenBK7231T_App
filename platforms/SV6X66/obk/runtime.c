#define SV6X66_ALLOC_IMPLEMENTATION
#include "new_common.h"
#include "flash.h"
#include "uart/drv_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "lwip/stats.h"
#include "lwip/memp.h"

/* Original OBK application allocations use one prefix and SDK RTOS heap.
	* Vendor OS_MemAlloc/pvPortMalloc and libc allocations retain matching APIs;
	* there is deliberately no global malloc/free linker wrapping. */
typedef union {
	size_t size;
	uint64_t alignment;
} allocation_header;

/* Vendor flash_page_program calls memcpy after disabling XIP. Link wrapping
	* keeps that dependency in initialized RAM without changing SDK/libc sources.
	* Volatile byte accesses prevent replacement of this loop with memcpy itself. */
void *__attribute__((section(".fast_boot_code"))) __wrap_memcpy(void *destination, const void *source, size_t bytes) {
	volatile uint8_t *out = destination;
	const volatile uint8_t *in = source;
	while (bytes--)
		*out++ = *in++;
	return destination;
}

void *os_malloc(size_t length) {
	allocation_header *p;
	if (!length || length > SIZE_MAX - sizeof(*p))
		return NULL;
	p = pvPortMalloc(length + sizeof(*p));
	if (!p)
		return NULL;
	p->size = length;
	return p + 1;
}
void os_free(void *ptr) {
	if (ptr)
		vPortFree((allocation_header *)ptr - 1);
}
void *os_calloc(size_t count, size_t size) {
	void *p;
	if (size && count > SIZE_MAX / size)
		return NULL;
	p = os_malloc(count * size);
	if (p)
		memset(p, 0, count * size);
	return p;
}
void *os_realloc(void *old, size_t length) {
	void *p;
	size_t previous;
	if (!old)
		return os_malloc(length);
	if (!length) {
		os_free(old);
		return NULL;
	}
	previous = ((allocation_header *)old - 1)->size;
	p = os_malloc(length);
	if (p) {
		memcpy(p, old, previous < length ? previous : length);
		os_free(old);
	}
	return p;
}

static volatile int log_enabled = 1;
void SV6X66_LogEnable(int enable) {
	log_enabled = enable;
}
int SV6X66_LogUartSetBaud(void) {
	while (!drv_uart_is_transmitter_idle()) {
	}
	return drv_uart_set_format(115200, UART_WORD_LEN_8, UART_STOP_BIT_1, UART_PARITY_DISABLE);
}
int SV6X66_Printf(const char *format, ...) {
	char buffer[384];
	va_list args;
	int wanted, written = 0, length;
	if (!log_enabled)
		return 0;
	va_start(args, format);
	wanted = vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	if (wanted < 0)
		return wanted;
	length = wanted < (int)sizeof(buffer) ? wanted : (int)sizeof(buffer) - 1;
	while (written < length) {
		int n = drv_uart_write_fifo((const uint8_t *)buffer + written, length - written, UART_BLOCKING);
		if (n <= 0)
			break;
		written += n;
	}
	return written;
}
int __wrap_printf(const char *format, ...) {
	char buffer[384];
	va_list args;
	int n;
	va_start(args, format);
	n = vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);
	if (n < 0)
		return n;
	return SV6X66_Printf("%s", buffer);
}

static uintptr_t __attribute__((noinline)) executing_pc(void) {
	return (uintptr_t)__builtin_return_address(0);
}
uint32_t SV6X66_RunningBank(void) {
	uintptr_t pc = executing_pc() - SV6X66_FLASH_BASE;
#if SV6X66_LINK_BANK == SV6X66_BANK_A
	if (pc < SV6X66_BANK_A || pc >= SV6X66_BANK_A + SV6X66_BANK_SIZE)
		return 0;
#elif SV6X66_LINK_BANK == SV6X66_BANK_B
	if (pc < SV6X66_BANK_B || pc >= SV6X66_BANK_B + SV6X66_BANK_SIZE)
		return 0;
#else
#error Invalid SV6X66_LINK_BANK
#endif
	if (pc >= SV6X66_BANK_A && pc < SV6X66_BANK_A + SV6X66_BANK_SIZE)
		return SV6X66_BANK_A;
	if (pc >= SV6X66_BANK_B && pc < SV6X66_BANK_B + SV6X66_BANK_SIZE)
		return SV6X66_BANK_B;
	return 0;
}
int SV6X66_ProfileValid(void) {
	const volatile uint32_t *selector = (const volatile uint32_t *)(SV6X66_FLASH_BASE + SV6X66_SELECTOR);
	uint32_t bank = SV6X66_RunningBank(), i;
	if (!SV6X66_FlashProfileValid() || !bank || selector[0] != (bank == SV6X66_BANK_A ? UINT32_MAX : 0))
		return 0;
	for (i = 1; i < 4096 / 4; i++)
		if (selector[i] != UINT32_MAX)
			return 0;
	return 1;
}
err_t SV6X66_DnsLookup(const char *name, ip_addr_t *address) {
	struct hostent *entry;
	if (!name || !address)
		return ERR_ARG;
	entry = gethostbyname(name);
	if (!entry || entry->h_length != 4 || !entry->h_addr_list || !entry->h_addr_list[0])
		return ERR_RTE;
	memcpy(&address->addr, entry->h_addr_list[0], 4);
	return ERR_OK;
}
int LWIP_GetMaxSockets(void) {
	return MEMP_NUM_NETCONN;
}
int LWIP_GetActiveSockets(void) {
#if LWIP_STATS && MEMP_STATS
	return lwip_stats.memp[MEMP_NETCONN].used;
#else
	return -1; /* Statistics unavailable, not an invented zero active sockets. */
#endif
}

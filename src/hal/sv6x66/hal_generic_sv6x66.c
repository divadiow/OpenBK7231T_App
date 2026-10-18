#if PLATFORM_SV6X66

#include "../../new_common.h"
#include "../hal_generic.h"
#include "flash.h"
#include "sys/flash.h"
#include "sys/xip.h"
#include "wdt/drv_wdt.h"

void HAL_RebootModule(void) {
	drv_wdt_init();
	drv_wdt_enable(SYS_WDT, 1);
	for (;;) {
	}
}
void HAL_Delay_us(int delay) {
	uint32_t start = OS_GetUsSysTick();
	if (delay <= 0)
		return;
	while ((uint32_t)(OS_GetUsSysTick() - start) < (uint32_t)delay) {
	}
}
int HAL_FlashRead(char *buffer, int bytes, int address) {
	if (!buffer || bytes < 0 || address < 0 || (uint32_t)bytes > SV6X66_FLASH_SIZE ||
		(uint32_t)address > SV6X66_FLASH_SIZE - (uint32_t)bytes)
		return -1;
	__builtin_memcpy(buffer, (const void *)(SV6X66_FLASH_BASE + (uint32_t)address), bytes);
	return bytes;
}

static SemaphoreHandle_t flash_mutex;
static uint8_t page[256];
static uint32_t identified_jedec;
#define CKW04_JEDEC 0x001540c8u
void SV6X66_FlashInit(void) {
	flash_mutex = xSemaphoreCreateMutex();
}
int SV6X66_FlashLock(void) {
	return flash_mutex && xSemaphoreTake(flash_mutex, 0) == pdTRUE ? 0 : -1;
}
void SV6X66_FlashUnlock(void) {
	if (flash_mutex)
		xSemaphoreGive(flash_mutex);
}

static int mutable_range(uint32_t address, size_t bytes) {
	uint32_t active = SV6X66_RunningBank();
	uint32_t inactive = active == SV6X66_BANK_A ? SV6X66_BANK_B : SV6X66_BANK_A;
	if (!SV6X66_FlashProfileValid() || !active || bytes > SV6X66_FLASH_SIZE || address > SV6X66_FLASH_SIZE - bytes)
		return 0;
	if (address >= inactive && address + bytes <= inactive + SV6X66_BANK_SIZE)
		return 1;
	return address >= 0x141000 && address + bytes <= SV6X66_FLASH_SIZE;
}

/* Entire mutation call chain and data must remain in initialized SRAM while
	* XIP is disabled. build.py audits ELF placement of this and SDK callees. */
/* The public vendor API returns the first RX word of command9F/read3.
	* Ignore the unrequested fourth byte. No caller-owned scratch RAM or private
	* controller implementation is used. The API restores XIP before return. */
/* These RAM boundaries must survive optimization as exact callable symbols. */
static uint32_t __attribute__((section(".fast_boot_code"), noinline, noclone)) identify_ram(void) {
	uint32_t id;
	OS_DeclareCritical();
	OS_EnterCritical();
	id = flash_init() & 0x00ffffffu;
	OS_ExitCritical();
	return id;
}
int SV6X66_FlashIdentify(void) {
	if (SV6X66_FlashLock())
		return -1;
	identified_jedec = identify_ram();
	SV6X66_FlashUnlock();
	return SV6X66_FlashProfileValid() ? 0 : -1;
}
int SV6X66_FlashProfileValid(void) {
	return identified_jedec == CKW04_JEDEC && ((identified_jedec >> 16) & 255) == 21;
}

/* Vendor erase/program each manage their own XIP leave/enter. flash_init
	* first refreshes SDK SRAM pointers and validates the actual chip again;
	* its internal XIP enter is not assumed to leave the controller disabled. */
static int __attribute__((section(".fast_boot_code"), noinline, noclone)) erase_ram(uint32_t address) {
	int result = 0;
	OS_DeclareCritical();
	OS_EnterCritical();
	if ((flash_init() & 0x00ffffffu) != CKW04_JEDEC) {
		identified_jedec = 0;
		result = -1;
	} else
		flash_sector_erase(address);
	OS_ExitCritical();
	return result;
}
static int __attribute__((section(".fast_boot_code"), noinline, noclone)) program_ram(uint32_t address, uint32_t bytes) {
	int result = 0;
	OS_DeclareCritical();
	OS_EnterCritical();
	if ((flash_init() & 0x00ffffffu) != CKW04_JEDEC) {
		identified_jedec = 0;
		result = -1;
	} else
		flash_page_program(address, bytes, page);
	OS_ExitCritical();
	return result;
}
int SV6X66_FlashErase(uint32_t address) {
	const volatile uint32_t *mapped;
	size_t i;
	if ((address & 4095) || !mutable_range(address, 4096))
		return -1;
	if (erase_ram(address))
		return -1;
	mapped = (const volatile uint32_t *)(SV6X66_FLASH_BASE + address);
	for (i = 0; i < 1024; i++)
		if (mapped[i] != UINT32_MAX)
			return -1;
	return 0;
}
int SV6X66_FlashWrite(uint32_t address, const void *source, size_t length) {
	const uint8_t *bytes = source;
	if (!source || !mutable_range(address, length))
		return -1;
	while (length) {
		size_t n = 256 - (address & 255);
		if (n > length)
			n = length;
		memcpy(page, bytes, n);
		if (program_ram(address, n))
			return -1;
		if (memcmp((const void *)(SV6X66_FLASH_BASE + address), bytes, n))
			return -1;
		address += n;
		bytes += n;
		length -= n;
	}
	return 0;
}
static uint32_t sv6x66_read_le32(const uint8_t *p) {
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
int SV6X66_ValidateBank(uint32_t bank, uint32_t length) {
	const uint8_t *raw = (const uint8_t *)(SV6X66_FLASH_BASE + bank), *id;
	uint32_t i;
	if ((bank != SV6X66_BANK_A && bank != SV6X66_BANK_B) || length < 68 || length > SV6X66_BANK_SIZE)
		return -1;
	if (memcmp(raw, "\x48\x00\x00\x02", 4))
		return -1;
	id = raw + length - 64;
	if (memcmp(id, "OBKSV6X1", 8) || sv6x66_read_le32(id + 8) != bank || sv6x66_read_le32(id + 12) != SV6X66_BANK_SIZE ||
		sv6x66_read_le32(id + 16) != length || sv6x66_read_le32(id + 20) != 1)
		return -1;
	for (i = 24; i < 64; i++)
		if (id[i])
			return -1;
	for (i = length; i < SV6X66_BANK_SIZE; i++)
		if (raw[i] != 0xff)
			return -1;
	return 0;
}
int SV6X66_CommitBank(uint32_t destination) {
	const volatile uint32_t *selector = (const volatile uint32_t *)(SV6X66_FLASH_BASE + SV6X66_SELECTOR);
	uint32_t old = SV6X66_RunningBank(), i, expected;
	if (!SV6X66_ProfileValid() || destination == (uint32_t)old ||
		(destination != SV6X66_BANK_A && destination != SV6X66_BANK_B))
		return -1;
	expected = destination == SV6X66_BANK_A ? UINT32_MAX : 0;
	/* For this profile only, tail words are all FF. Unknown tail fails before
		* erase. B->A commits at erase; A->B commits at zero word programming. */
	if (erase_ram(SV6X66_SELECTOR))
		return -1;
	for (i = 0; i < 1024; i++)
		if (selector[i] != UINT32_MAX)
			return -1;
	if (expected == 0) {
		memset(page, 0, 4);
		if (program_ram(SV6X66_SELECTOR, 4))
			return -1;
	}
	if (selector[0] != expected)
		return -1;
	for (i = 1; i < 1024; i++)
		if (selector[i] != UINT32_MAX)
			return -1;
	return 0;
}

int HAL_FlashWrite(char *buffer, unsigned int bytes, unsigned int address) {
	int result;
	if (SV6X66_FlashLock())
		return -1;
	result = SV6X66_FlashWrite(address, buffer, bytes);
	SV6X66_FlashUnlock();
	return result;
}
int HAL_FlashEraseSector(int address) {
	int result;
	if (address < 0 || SV6X66_FlashLock())
		return -1;
	result = SV6X66_FlashErase((uint32_t)address);
	SV6X66_FlashUnlock();
	return result;
}

#endif

#if PLATFORM_SV6X66

#include "../../new_common.h"
#include "../hal_generic.h"

void HAL_RebootModule(void) {
	SV6X66_Reboot();
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

#endif

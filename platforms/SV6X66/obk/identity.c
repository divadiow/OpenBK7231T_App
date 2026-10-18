#include <stdint.h>
#ifndef SV6X66_LINK_BANK
#error SV6X66_LINK_BANK must name the physical linked bank
#endif
extern char __obk_image_bytes;
/* Linker positions this final64byte object after SDK config data. */
const struct {
	char magic[8];
	uint32_t bank, capacity;
	const void *length;
	uint32_t format;
	uint8_t reserved[40];
} sv6x66_identity __attribute__((section(".obk_identity"), used)) = {
	{'O', 'B', 'K', 'S', 'V', '6', 'X', '1'}, SV6X66_LINK_BANK, 0x9b000, &__obk_image_bytes, 1, {0}};

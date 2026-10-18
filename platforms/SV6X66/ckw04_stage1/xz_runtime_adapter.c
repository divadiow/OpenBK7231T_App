#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define XZ_ARENA_BYTES 18432u

static uint8_t xz_arena[XZ_ARENA_BYTES] __attribute__((aligned(8)));
static size_t xz_arena_used;

/* XZ Embedded in the vendor SDK is shared with the recovery image and names
	* its tiny libc surface rec_*.  The Bank-B loader runs in the normal SDK
	* runtime, so provide only the decoder dependencies as direct adapters. */
void *rec_memcpy(void *dest, const void *src, size_t n) { return memcpy(dest, src, n); }
void *rec_memset(void *dest, int value, size_t n) { return memset(dest, value, n); }
int rec_memcmp(const void *left, const void *right, size_t n) { return memcmp(left, right, n); }
void *rec_memmove(void *dest, const void *src, size_t n) { return memmove(dest, src, n); }
void SV6X66_XzArenaReset(void) { xz_arena_used = 0u; }

void *rec_malloc(size_t n)
{
	size_t aligned = (n + 7u) & ~(size_t)7u;
	void *result;

	if (aligned < n || aligned > XZ_ARENA_BYTES - xz_arena_used)
		return NULL;
	result = xz_arena + xz_arena_used;
	xz_arena_used += aligned;
	return result;
}

void rec_free(void *p) { (void)p; }

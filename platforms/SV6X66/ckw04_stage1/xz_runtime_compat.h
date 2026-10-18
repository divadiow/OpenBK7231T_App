#ifndef SV6X66_XZ_RUNTIME_COMPAT_H
#define SV6X66_XZ_RUNTIME_COMPAT_H

#include <stddef.h>
#include <stdint.h>

/* The pinned recovery XZ decoder names its libc dependencies rec_*.
 * Keep those exact, typed declarations when compiling it in the normal SDK
 * runtime; implicit allocator declarations corrupt pointers on target. */
void *rec_memset(void *dest, int value, size_t count);
void *rec_memcpy(void *dest, const void *src, size_t count);
int rec_memcmp(const void *left, const void *right, size_t count);
void *rec_memmove(void *dest, const void *src, size_t count);
void *rec_malloc(size_t count);
void rec_free(void *pointer);
void SV6X66_XzArenaReset(void);

#define memset rec_memset
#define memcpy rec_memcpy
#define memcmp rec_memcmp
#define memmove rec_memmove
#define malloc rec_malloc
#define free rec_free

#endif

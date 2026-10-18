#ifndef OBK_SV6X66_ALLOC_H
#define OBK_SV6X66_ALLOC_H
#include <stddef.h>
void *os_malloc(size_t);
void *os_calloc(size_t, size_t);
void *os_realloc(void *, size_t);
void os_free(void *);
/* This header is force-included only for original OBK application objects.
 * SDK archives and SDK source retain their documented allocation/free APIs. */
#ifndef SV6X66_ALLOC_IMPLEMENTATION
#define malloc os_malloc
#define calloc os_calloc
#define realloc os_realloc
#define free os_free
#endif
#endif

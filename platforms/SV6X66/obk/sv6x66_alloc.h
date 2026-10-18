#ifndef OBK_SV6X66_ALLOC_H
#define OBK_SV6X66_ALLOC_H
#include <stddef.h>
void *SV6X66_Malloc(size_t);
void *SV6X66_Calloc(size_t, size_t);
void *SV6X66_Realloc(void *, size_t);
void SV6X66_Free(void *);
/* This header is force-included only for original OBK application objects.
 * SDK archives and SDK source retain their documented allocation/free APIs. */
#ifndef SV6X66_ALLOC_IMPLEMENTATION
#define malloc SV6X66_Malloc
#define calloc SV6X66_Calloc
#define realloc SV6X66_Realloc
#define free SV6X66_Free
#endif
#endif

#ifndef SV6X66_STORAGE_H
#define SV6X66_STORAGE_H
#include <stdint.h>
#define SV6X66_STORAGE_CFG 1u
#define SV6X66_STORAGE_VARS 2u
void SV6X66_StorageInit(void);
int SV6X66_StorageLock(void);
void SV6X66_StorageUnlock(void);
// Returns len on success, 0 if no valid record, -1 on an indeterminate read.
int SV6X66_StorageReadRecord(uint16_t kind, void *target, uint32_t len);
int SV6X66_StorageWriteRecord(uint16_t kind, const void *source, uint32_t len);
void SV6X66_FlashVarsInit(void);
#endif

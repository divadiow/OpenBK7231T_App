#ifndef OBK_SV6X66_OTA_H
#define OBK_SV6X66_OTA_H
#include <stdint.h>
#include <stddef.h>
#define SV6X66_OTA_HEADER 84
int SV6X66_UpdateBegin(const uint8_t *, int);
int SV6X66_UpdateData(const uint8_t *, size_t);
int SV6X66_UpdateVerify(void);
int SV6X66_UpdateCommit(void);
void SV6X66_UpdateAbort(void);
uint32_t SV6X66_UpdateRemaining(void);
int SV6X66_UpdateChecksums(const char *);
#endif

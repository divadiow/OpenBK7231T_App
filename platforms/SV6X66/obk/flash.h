#ifndef OBK_SV6X66_FLASH_H
#define OBK_SV6X66_FLASH_H
#include <stdint.h>
#include <stddef.h>
void SV6X66_FlashInit(void);
int SV6X66_FlashIdentify(void);
int SV6X66_FlashProfileValid(void);
int SV6X66_FlashLock(void);
void SV6X66_FlashUnlock(void);
int SV6X66_FlashErase(uint32_t);
int SV6X66_FlashWrite(uint32_t,const void *,size_t);
int SV6X66_CommitBank(uint32_t);
int SV6X66_ValidateBank(uint32_t,uint32_t);
#endif

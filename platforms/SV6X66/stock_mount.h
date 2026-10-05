#ifndef SV6X66_STOCK_MOUNT_H
#define SV6X66_STOCK_MOUNT_H
#include <stdint.h>
#include "fsal.h"
SSV_FS SV6X66_StockMount(void);
int SV6X66_StockHeaderValid(const uint8_t header[40]);
#endif

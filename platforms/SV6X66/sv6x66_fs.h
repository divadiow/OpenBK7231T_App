#ifndef SV6X66_FS_H
#define SV6X66_FS_H
#include "fsal.h"
// Vendor FSAL wrappers require a private flag set by autoformatting FS_init.
// The stock profile mounts explicitly and uses the public SPIFFS operations.
#if defined(SV6X66_STOCK_CKW04)
#define FS_open prvSPIFFS_open
#define FS_read prvSPIFFS_read
#define FS_write prvSPIFFS_write
#define FS_lseek prvSPIFFS_lseek
#define FS_remove prvSPIFFS_remove
#define FS_fremove prvSPIFFS_fremove
#define FS_stat prvSPIFFS_stat
#define FS_fstat prvSPIFFS_fstat
#define FS_flush prvSPIFFS_fflush
#define FS_close prvSPIFFS_close
#define FS_rename prvSPIFFS_rename
#define FS_opendir prvSPIFFS_opendir
#define FS_closedir prvSPIFFS_closedir
#define FS_readdir prvSPIFFS_readdir
#endif
#endif

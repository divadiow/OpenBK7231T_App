#ifndef TEST_FSAL_H
#define TEST_FSAL_H
#include <stdint.h>
typedef void *SSV_FS;
typedef int16_t SSV_FILE;
typedef struct { uint32_t size; } SSV_FILE_STAT;
#define SPIFFS_RDONLY 1
#define SPIFFS_WRONLY 2
#define SPIFFS_CREAT 4
#define SPIFFS_TRUNC 8
#define SPIFFS_ERR_NOT_FOUND -10002
int test_fs_errno(SSV_FS);
#define FS_errno test_fs_errno
SSV_FILE FS_open(SSV_FS, const char *, uint32_t, uint32_t);
int32_t FS_read(SSV_FS, SSV_FILE, void *, uint32_t);
int32_t FS_write(SSV_FS, SSV_FILE, void *, uint32_t);
int32_t FS_close(SSV_FS, SSV_FILE);
int32_t FS_fstat(SSV_FS, SSV_FILE, SSV_FILE_STAT *);
#endif

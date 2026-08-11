OBK_DIR = ../../..

CFLAGS +=  -DPLATFORM_TXW81X

# Keep sdk/OpenTXW81X/libs/libFLASH.a from TXW81x FPV v2.5.3.7-40416.
# The HAL uses the vendor OTA API in sector-erase mode because the 1 MiB
# OpenBeken EasyFlash area begins at 0xEF000, inside the final 64 KiB block.

# The SDK generates APP_compress.bin with AesEnable=0. Route the customer-ID
# helper through a guarded wrapper that validates and accepts that normal
# unencrypted format while preserving the vendor check for encrypted images.
LDFLAGS += -Wl,--wrap=fwinfo_check_customer_id

# libcommon.a contains its own weak ota_fwinfo_get() fallback returning -1.
# Bind every reference, including references originating inside that archive
# member, directly to the known-good TXW81X implementation in the HAL.
LDFLAGS += -Wl,--defsym=ota_fwinfo_get=txw81x_ota_fwinfo_get

# Non-invasive trace wrappers for the otherwise opaque first-call setup path.
# They record state only while an OTA SDK call is active and log afterwards.
LDFLAGS += -Wl,--wrap=_os_malloc
LDFLAGS += -Wl,--wrap=get_loader_mark
LDFLAGS += -Wl,--wrap=spi_nor_open
LDFLAGS += -Wl,--wrap=spi_nor_read
LDFLAGS += -Wl,--wrap=spi_nor_close
LDFLAGS += -Wl,--wrap=spi_nor_sector_erase
LDFLAGS += -Wl,--wrap=spi_nor_block_erase
LDFLAGS += -Wl,--wrap=spi_nor_write
LDFLAGS += -Wl,--wrap=get_current_loader_addr

INCLUDES += -I$(OBK_DIR)/libraries/easyflash/inc

SRC_C  += $(OBK_DIR)/src/hal/txw81x/hal_flashVars_txw81x.c
SRC_C  += $(OBK_DIR)/src/hal/txw81x/hal_flashConfig_txw81x.c
SRC_C  += $(OBK_DIR)/src/hal/txw81x/hal_generic_txw81x.c
SRC_C  += $(OBK_DIR)/src/hal/txw81x/hal_main_txw81x.c
SRC_C  += $(OBK_DIR)/src/hal/txw81x/hal_ota_txw81x.c
SRC_C  += $(OBK_DIR)/src/hal/txw81x/hal_pins_txw81x.c
SRC_C  += $(OBK_DIR)/src/hal/txw81x/hal_wifi_txw81x.c
SRC_C  += $(OBK_DIR)/src/driver/drv_txw81x_camera.c

OBK_SRCS = $(OBK_DIR)/src/
include $(OBK_DIR)/platforms/obk_main.mk
SRC_C += $(OBKM_SRC)
CFLAGS += $(OBK_CFLAGS)

SRC_C += $(OBK_DIR)/libraries/easyflash/ports/ef_port.c
SRC_C += $(OBK_DIR)/libraries/easyflash/src/easyflash.c
#SRC_C += $(OBK_DIR)/libraries/easyflash/src/ef_cmd.c
SRC_C += $(OBK_DIR)/libraries/easyflash/src/ef_env.c
SRC_C += $(OBK_DIR)/libraries/easyflash/src/ef_env_legacy.c
SRC_C += $(OBK_DIR)/libraries/easyflash/src/ef_env_legacy_wl.c
SRC_C += $(OBK_DIR)/libraries/easyflash/src/ef_iap.c
SRC_C += $(OBK_DIR)/libraries/easyflash/src/ef_log.c
SRC_C += $(OBK_DIR)/libraries/easyflash/src/ef_utils.c

INCLUDES += -I$(OBK_DIR)/include
BERRY_MODULEPATH = $(OBK_DIR)/src/berry/modules
BERRY_SRCPATH = $(OBK_DIR)/libraries/berry/src

include $(OBK_DIR)/libraries/berry.mk

#SRC_C += $(BERRY_SRC_C)
SRC_C += $(OBK_DIR)/libraries/mqtt_patched.c

OBK_DIR = ../../..

CFLAGS +=  -DPLATFORM_TXW81X

# The SDK packages APP_compress.bin with AesEnable=0, but its OTA helper
# rejects unencrypted images. The HAL wrapper keeps the vendor check for
# encrypted images while accepting a structurally valid unencrypted header.
LDFLAGS += -Wl,--wrap=fwinfo_check_customer_id

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

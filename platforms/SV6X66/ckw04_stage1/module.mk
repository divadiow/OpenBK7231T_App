LIB_SRC := platforms/SV6X66/main.c
LIB_SRC += platforms/SV6X66/stock_entry.c
LIB_SRC += platforms/SV6X66/stock_runtime.c
LIB_SRC += platforms/SV6X66/sv6x66_loader.c
LIB_SRC += platforms/SV6X66/xz_runtime_adapter.c
LIB_SRC += framework/uOTA/src/2nd_boot/xz/linux/lib/xz/xz_crc32.c
LIB_SRC += framework/uOTA/src/2nd_boot/xz/linux/lib/xz/xz_dec_lzma2.c
LIB_SRC += framework/uOTA/src/2nd_boot/xz/linux/lib/xz/xz_dec_stream.c
LOCAL_INC += -Iobk_app/framework/uOTA/src/2nd_boot
LOCAL_INC += -Iobk_app/framework/uOTA/src/2nd_boot/xz/linux/include/linux
LOCAL_INC += -Iobk_app/framework/uOTA/src/2nd_boot/xz/linux/lib/xz
LIBRARY_NAME := ckw04_stage1
LOCAL_INC += -Iobk_app/include -Iobk_app/platforms/SV6X66
$(eval $(call build-lib,$(LIBRARY_NAME),$(LIB_SRC),,$(OBK_CFLAGS),$(LOCAL_INC),,$(MYDIR)))

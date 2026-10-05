OBK_SRCS := src/
include obk_app/platforms/obk_main.mk
# These complete HAL implementations replace the weak generic versions.
# A normal static archive link can otherwise select the weak member first.
LIB_SRC := $(filter-out src/hal/generic/hal_pins_generic.c src/hal/generic/hal_wifi_generic.c,$(OBKM_SRC))
LIB_SRC += platforms/SV6X66/main.c
LIB_SRC += platforms/SV6X66/stock_mount.c
LIB_SRC += platforms/SV6X66/stock_entry.c
LIB_SRC += platforms/SV6X66/stock_runtime.c
LIB_SRC += platforms/SV6X66/ota_stage.c
LIB_SRC += platforms/SV6X66/my_lwip2_mqtt_replacement.c
LIB_SRC += platforms/SV6X66/mqtt_dispatch.c
LIB_SRC += src/hal/sv6x66/hal_generic_sv6x66.c
LIB_SRC += src/hal/sv6x66/hal_wifi_sv6x66.c
LIB_SRC += src/hal/sv6x66/hal_pins_sv6x66.c
LIB_SRC += src/hal/sv6x66/hal_flashConfig_sv6x66.c
LIB_SRC += src/hal/sv6x66/hal_flashVars_sv6x66.c
LIB_SRC += src/hal/sv6x66/hal_ota_sv6x66.c
LIBRARY_NAME := openbeken
LOCAL_INC += -Iobk_app/include -Iobk_app/platforms/SV6X66
$(eval $(call build-lib,$(LIBRARY_NAME),$(LIB_SRC),,$(OBK_CFLAGS),$(LOCAL_INC),,$(MYDIR)))

OBK_SRCS := src/
include obk_app/platforms/obk_main.mk
# Match this profile's ENABLE_LITTLEFS=0. Persistence uses bounded SPIFFS;
# common sources list LittleFS unconditionally and its backend is unsupported.
LIB_SRC := $(filter-out $(OBK_SRCS)littlefs/%,$(OBKM_SRC))
LIB_SRC += platforms/SV6X66/ckw04_stage1/stock_entry.c
LIB_SRC += platforms/SV6X66/ckw04_stage1/stock_runtime.c
LIB_SRC += platforms/SV6X66/obk/main.c platforms/SV6X66/obk/runtime.c
LIB_SRC += platforms/SV6X66/obk/identity.c platforms/SV6X66/obk/flash.c
LIB_SRC += platforms/SV6X66/obk/ota.c platforms/SV6X66/obk/factory.c
LIB_SRC += platforms/SV6X66/obk/wifi.c platforms/SV6X66/obk/pins.c
LIB_SRC += platforms/SV6X66/obk/storage.c
LIB_SRC += platforms/SV6X66/obk/mqtt_socket.c
LIB_SRC += platforms/SV6X66/obk/factory_identity.c
LIB_SRC += platforms/SV6X66/obk/phy_adapter.c
LIB_SRC += ../components/obk/lwip_mdns_sv6/obk_adapter.c
LIB_SRC += ../components/obk/lwip_mdns_sv6/mdns.c ../components/obk/lwip_mdns_sv6/mdns_domain.c
LIB_SRC += ../components/obk/lwip_mdns_sv6/mdns_out.c ../components/obk/lwip_mdns_sv6/compat.c
LIBRARY_NAME := openbeken
LOCAL_INC += -Iobk_app/include -Iobk_app/src -Iobk_app/src/cJSON -Iobk_app/platforms/SV6X66/obk
LOCAL_INC += -Iobk_app/platforms/SV6X66/obk/include
LOCAL_INC += -Icomponents/obk/lwip_mdns_sv6/include
LOCAL_CFLAGS += -include obk_app/platforms/SV6X66/obk/sv6x66_alloc.h $(OBK_CFLAGS)
$(eval $(call build-lib,$(LIBRARY_NAME),$(LIB_SRC),,$(LOCAL_CFLAGS),$(LOCAL_INC),,$(MYDIR)))

# Vendor project defaults and binary driver ABI are kept intact.
PROJ_DIR := projects/lite_mac
export PROJ_DIR
PROJECT := OpenSV6166F
include $(PROJ_DIR)/mk/project.mk
IMPORT_DIR := $(filter-out $(PROJ_DIR)/src/app $(PROJ_DIR)/src/cli components/net/tcpip/iperf-2.0.5 components/third_party/cJSON,$(IMPORT_DIR))

# cJSON 1.5 and OBK 1.7 share the public node ABI; link the app implementation once.
IMPORT_DIR += obk_app
OBK_APP_VERSION ?= SV6166F_dev
CFLAGS += -DPLATFORM_SV6X66=1 -DUSER_SW_VER='"$(OBK_APP_VERSION)"'
CFLAGS += -I$(TOPDIR)/obk_app/platforms/SV6X66 -I$(TOPDIR)/obk_app/src

# Pull only the standalone MD5 archive member; vendor URL OTA remains disabled.
STATIC_LIB += components/tools/ota_api/libota_api.a

LDFLAGS += -lm
LDFLAGS += -Wl,--wrap=wifi_cfg_get_addr1,--wrap=wifi_cfg_get_addr2
ifeq ($(OBK_LAYOUT),stock-ckw04)
LDSCRIPT_S := obk_app/platforms/SV6X66/stock_flash.lds.S
SETTING_PARTITION_MAIN_SIZE := 0xAF000
CFLAGS := $(filter-out -DSETTING_PARTITION_MAIN_SIZE=% -DXTAL=%,$(CFLAGS))
LDFLAGS += -Wl,--wrap=_soc_clk_init,--wrap=_soc_io_init,--wrap=xip_init,--wrap=xip_leave,--wrap=xip_enter
CFLAGS += -DXTAL=25 -DSV6X66_STOCK_CKW04=1 -DSETTING_PARTITION_MAIN_SIZE=0xAF000
endif

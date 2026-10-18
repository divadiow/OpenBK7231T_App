PROJ_DIR := projects/lite_mac
export PROJ_DIR
PROJECT := OpenSV6166F
include $(PROJ_DIR)/mk/project.mk
IMPORT_DIR := $(filter-out $(PROJ_DIR)/src/app $(PROJ_DIR)/src/cli components/net/tcpip/iperf-2.0.5 components/third_party/cJSON,$(IMPORT_DIR))
IMPORT_DIR += obk_app
CFLAGS += -DPLATFORM_SV6X66=1 -DSV6X66_STOCK_CKW04=1 -DSV6X66_BANK_B_LOADER=1
CFLAGS += -I$(TOPDIR)/obk_app/platforms/SV6X66 -I$(TOPDIR)/obk_app/src
LDSCRIPT_S := obk_app/platforms/SV6X66/stock_flash.lds.S
SETTING_PARTITION_MAIN_SIZE := 0x9B000
CFLAGS := $(filter-out -DSETTING_PARTITION_MAIN_SIZE=% -DXTAL=%,$(CFLAGS))
LDFLAGS += -Wl,--wrap=_soc_clk_init,--wrap=_soc_io_init,--wrap=xip_init,--wrap=xip_leave,--wrap=xip_enter
CFLAGS += -DXTAL=25 -DSETTING_PARTITION_MAIN_SIZE=0x9B000

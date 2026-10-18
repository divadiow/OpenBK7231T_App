PROJ_DIR := projects/lite_mac
export PROJ_DIR
PROJECT := OpenSV6X66
include $(PROJ_DIR)/mk/project.mk
IMPORT_DIR := $(filter-out $(PROJ_DIR)/src/app $(PROJ_DIR)/src/cli components/net/tcpip/iperf-2.0.5 components/third_party/cJSON,$(IMPORT_DIR))
IMPORT_DIR += obk_app
CFLAGS += -DPLATFORM_SV6X66=1 -DSV6X66_STOCK_CKW04=1 -DSV6X66_LINK_BANK=$(OBK_BANK)
# OBK headers belong to obk_app/module.mk only. Global paths would shadow
# SDK flash.h and cJSON.h in vendor modules with different include contracts.
CFLAGS += -DUSER_SW_VER='"$(APP_VERSION)"'
LDSCRIPT_S := obk_app/platforms/SV6X66/obk/application.lds.S
SETTING_PARTITION_MAIN_SIZE := 0x9B000
CFLAGS := $(filter-out -DSETTING_PARTITION_MAIN_SIZE=% -DXTAL=%,$(CFLAGS))
CFLAGS += -DXTAL=26 -DSETTING_PARTITION_MAIN_SIZE=0x9B000
# SPIFFS must read existing flags before programming a cleared-bit update;
# the shared flash writer intentionally retains exact readback verification.
CFLAGS += -DSPIFFS_NO_BLIND_WRITES=1
LDFLAGS += -Wl,--wrap=_soc_clk_init,--wrap=_soc_io_init,--wrap=xip_init,--wrap=xip_leave,--wrap=xip_enter,--wrap=printf
LDFLAGS += -Wl,--wrap=wifi_cfg_init,--wrap=wifi_cfg_get_addr1,--wrap=wifi_cfg_get_addr2,--wrap=wifi_cfg_deinit,--wrap=wifi_cfg_write_cfg,--wrap=wifi_cfg_replace_mem_addr1,--wrap=wifi_cfg_replace_mem_addr2
LDFLAGS += -Wl,--wrap=memcpy
# Preserve the real AP-start return that DUT_wifi_start otherwise discards.
LDFLAGS += -Wl,--wrap=softap_start
# Use SDK calibration macros with the board's explicit 26 MHz clock profile.
LDFLAGS += -Wl,--wrap=drv_phy_cali
# Select metadata directly before the SDK archive group. The application
# library dependency builds this object; an archive root alone was insufficient.
PROJECT_OBJ += out/obk_app/platforms/SV6X66/obk/identity.o
# Select real GPIO definitions before lazy generic weak archive fallbacks.
# module.mk already builds this object as an application-library dependency.
PROJECT_OBJ += out/obk_app/platforms/SV6X66/obk/pins.o
# SDK links LDFLAGS after the application archive group; resolve its math calls.
LDFLAGS += -lm

# Two HTTP workers reserve heap for lwIP and the streaming OTA handler.
CFLAGS += -DMAX_SOCKETS_TCP=3
# Match BL602_ALT/BL616 to bound heap retained by closed polling connections.
CFLAGS += -DTCP_MSL=1000

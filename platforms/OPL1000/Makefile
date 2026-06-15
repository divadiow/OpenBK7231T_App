# OpenOPL1000 first-stage bring-up firmware for the Opulinks OPL1000 A2 SDK.
#
# This intentionally builds a small SDK-native station image first, rather than
# pretending the full OpenBeken HAL exists for OPL1000 yet.  It proves that the
# GitHub workflow, SDK submodule and RF/STA path are usable.

APP_NAME ?= OpenOPL1000
APP_VERSION ?= dev
SDK_ROOT ?= ../../sdk/OpenOPL1000
BUILD_DIR ?= build
OUTPUT_DIR ?= ../../output/$(APP_VERSION)
OUTPUT_BASENAME ?= $(APP_NAME)_$(APP_VERSION)

ARM_GCC_PATH ?= $(ARM_NONE_EABI_GCC_PATH)
ifneq (,$(wildcard $(ARM_GCC_PATH)/bin/arm-none-eabi-gcc))
CROSS_COMPILE ?= $(ARM_GCC_PATH)/bin/arm-none-eabi-
else ifneq (,$(wildcard $(ARM_GCC_PATH)/arm-none-eabi-gcc))
CROSS_COMPILE ?= $(ARM_GCC_PATH)/arm-none-eabi-
else
CROSS_COMPILE ?= arm-none-eabi-
endif

CC      := $(CROSS_COMPILE)gcc
OBJCOPY := $(CROSS_COMPILE)objcopy
SIZE    := $(CROSS_COMPILE)size

APS_ROOT       := $(SDK_ROOT)/SDK/APS
APS_PATCH_ROOT := $(SDK_ROOT)/SDK/APS_PATCH
LINK_SCRIPT    := $(APS_ROOT)/project/opl1000/link.ld
SYMDEFS        := $(APS_ROOT)/targets/opl1000/object/symdefs_gcc.o
PATCH_LIB      := $(APS_ROOT)/targets/opl1000/Output-GCC/opl1000_app_m3.a
M0_IMAGE       := $(SDK_ROOT)/FW_Pack/opl1000_m0.bin
OTA_LOADER     := $(SDK_ROOT)/FW_Pack/opl1000_ota_loader.bin

SOURCES := main_patch.c openopl1000_wifi_sta.c openopl1000_http_server.c
OBJECTS := $(addprefix $(BUILD_DIR)/,$(SOURCES:.c=.o))

DEFS  := -DGCC
DEFS  += -DARMCM3
DEFS  += -DOPL1000_APS
DEFS  += -D__noSIMULATOR__
DEFS  += -D__BLE__
DEFS  += -D__LE_HOST_USE_CMD__
DEFS  += -D__LWIP_TASK__
DEFS  += -D__WPA_SUPPLICANT__
DEFS  += -D__noHW_CRYPTO_ENGINE__
DEFS  += -D__WIFI_MAC_TASK__
DEFS  += -D__NL1000_An__
DEFS  += -D__PMP_ENABLE__
DEFS  += -D__PMP_REGION__
DEFS  += -D__HEAP_REGION__
DEFS  += -DnoLWIP_DYNAMIC_DEBUG_ENABLE
DEFS  += -D__AT_CMD_TASK__
DEFS  += -D__noRTL_SIMULATION__
DEFS  += -D_ARMABI_PURE=
DEFS  += -DOPENOPL1000_WIFI_SSID=\"test\"
DEFS  += -DOPENOPL1000_WIFI_PASSWORD=\"1234abcd\"

# Include set taken from the vendor GCC station example and made SDK_ROOT-relative.
INCLUDES := -I.
INCLUDES += -I $(SDK_ROOT)/SDK/APS/apps/le_app/cmd_app
INCLUDES += -I $(SDK_ROOT)/SDK/APS/apps/le_app/mtc_app
INCLUDES += -I $(SDK_ROOT)/SDK/APS/apps/le_app/pts_app
INCLUDES += -I $(SDK_ROOT)/SDK/APS/apps/wifi_app
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_auxadc
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_dbg_uart
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_dma
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_i2c
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_patch
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_pwm
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_spi
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_system
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_tick
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_tmr
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_uart
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_vic
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/hal_wdt
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/chip/opl1000/securityipdriver
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/CMSIS/Device/opl1000/Include
INCLUDES += -I $(SDK_ROOT)/SDK/APS/driver/CMSIS/Include
INCLUDES += -I $(SDK_ROOT)/SDK/APS/FreeRtos/Source/include
INCLUDES += -I $(SDK_ROOT)/SDK/APS/FreeRtos/Source/portable/GCC/ARM_CM3
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/at
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/ble_controller_layer/inc
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/cli
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/common/sys_api
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/common/sys_api/rom_if
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/common/sys_ctrl
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/controller_task
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/data_flow
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/diag_task
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/le_stack
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/msg
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/mw_fim
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/mw_ota
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/ps_task
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/wifi_controller_layer
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/wifi_controller_layer/rom_if
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/wifi_mac
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/wifi_mac/common
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/wifi_mac/crypto
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/netlink/wifi_mac/utils
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/fatfs/src
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/httpclient
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/lwip-2.0.3
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/lwip-2.0.3/lwip/src/include
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/lwip-2.0.3/ports/freertos/include
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/lwip-2.0.3/ports/freertos/netif
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/lwip-2.0.3/ports/rom_if
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/mbedtls/configs
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/mbedtls/include/mbedtls
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/mbedtls/port/include
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/tinycrypt/include
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/tinycrypt/include/tinycrypt
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/ap
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/common
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/crypto
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/drivers
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/eapol_auth
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/eapol_supp
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/eap_common
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/eap_peer
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/eap_server
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/l2_packet
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/radius
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/rsn_supp
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/tls
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/utils
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/src/wps
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/wpa_supplicant
INCLUDES += -I $(SDK_ROOT)/SDK/APS/middleware/third_party/wpa_supplicant-0.7.3/wpa_supplicant/dbus
INCLUDES += -I $(SDK_ROOT)/SDK/APS/project/common
INCLUDES += -I $(SDK_ROOT)/SDK/APS/project/opl1000/boot_sequence
INCLUDES += -I $(SDK_ROOT)/SDK/APS/project/opl1000/include
INCLUDES += -I $(SDK_ROOT)/SDK/APS/project/opl1000/startup
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/apps/le_app/cmd_app
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/apps/le_app/mtc_app
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/apps/le_app/pts_app
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/apps/wifi_app
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_auxadc
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_dbg_uart
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_dma
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_i2c
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_patch
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_pwm
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_spi
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_system
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_tick
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_tmr
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_uart
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_vic
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/hal_wdt
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/chip/opl1000/securityipdriver
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/CMSIS/Device/opl1000/Include
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/driver/CMSIS/Include
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/FreeRtos/Source/include
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/FreeRtos/Source/portable/Keil/ARM_CM3
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/at
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/ble_controller_layer/inc
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/cli
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/common/sys_api
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/common/sys_api/rom_if
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/common/sys_ctrl
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/controller_task
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/data_flow
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/diag_task
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/le_stack
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/le_stack/patch
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/msg
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/mw_fim
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/mw_ota
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/ps_task
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/wifi_controller_layer
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/wifi_controller_layer/rom_if
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/wifi_mac
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/wifi_mac/common
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/wifi_mac/crypto
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/netlink/wifi_mac/utils
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/fatfs/src
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/httpclient
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/lwip-2.0.3
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/lwip-2.0.3/lwip/src/include
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/lwip-2.0.3/ports/freertos/include
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/lwip-2.0.3/ports/freertos/netif
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/lwip-2.0.3/ports/rom_if
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/mbedtls/configs
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/mbedtls/include/mbedtls
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/mbedtls/port/include
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/tinycrypt/include/tinycrypt
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/ap
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/common
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/crypto
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/drivers
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/eapol_auth
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/eapol_supp
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/eap_common
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/eap_peer
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/eap_server
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/l2_packet
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/radius
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/rsn_supp
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/tls
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/utils
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/src/wps
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/wpa_supplicant
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/middleware/third_party/wpa_supplicant-0.7.3/wpa_supplicant/dbus
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/project/common
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/project/opl1000/boot_sequence
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/project/opl1000/include
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/project/opl1000/startup
INCLUDES += -I $(SDK_ROOT)/SDK/APS_PATCH/project/opl1000/lib

CFLAGS  := $(DEFS)
CFLAGS  += -mthumb -mcpu=cortex-m3 -mlittle-endian
CFLAGS  += -std=c99 -Os -x none
CFLAGS  += -include sys_common.h
CFLAGS  += -nostartfiles -no-pie
CFLAGS  += -fdata-sections -ffunction-sections
CFLAGS  += -specs=nosys.specs
CFLAGS  += -ffreestanding
CFLAGS  += -Wall -Wextra
CFLAGS  += -Wno-implicit-function-declaration -Wno-int-conversion
CFLAGS  += -Wno-unused-parameter -Wno-missing-field-initializers
CFLAGS  += $(INCLUDES)

LDFLAGS := -T $(LINK_SCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--entry=__Patch_EntryPoint
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(OUTPUT_BASENAME).map
LDFLAGS += -Xlinker --just-symbols=$(SYMDEFS)

.PHONY: all clean check-sdk print-config
all: check-sdk $(BUILD_DIR)/$(OUTPUT_BASENAME).bin package

check-sdk:
	@test -d "$(SDK_ROOT)" || (echo "Missing SDK_ROOT=$(SDK_ROOT). Put the OPL1000 A2 SDK submodule at sdk/OpenOPL1000."; exit 1)
	@test -f "$(LINK_SCRIPT)" || (echo "Missing $(LINK_SCRIPT)"; exit 1)
	@test -f "$(SYMDEFS)" || (echo "Missing $(SYMDEFS)"; exit 1)
	@test -f "$(PATCH_LIB)" || (echo "Missing $(PATCH_LIB)"; exit 1)

print-config:
	@echo "APP_NAME=$(APP_NAME)"
	@echo "APP_VERSION=$(APP_VERSION)"
	@echo "SDK_ROOT=$(SDK_ROOT)"
	@echo "CC=$(CC)"

$(BUILD_DIR):
	@mkdir -p $@

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(OUTPUT_BASENAME).elf: $(OBJECTS)
	$(CC) -o $@ $(LDFLAGS) $(CFLAGS) $(OBJECTS) $(PATCH_LIB)
	-$(SIZE) $@

$(BUILD_DIR)/$(OUTPUT_BASENAME).bin: $(BUILD_DIR)/$(OUTPUT_BASENAME).elf
	$(OBJCOPY) -O binary $< $@

package: $(BUILD_DIR)/$(OUTPUT_BASENAME).bin
	@mkdir -p "$(OUTPUT_DIR)"
	cp "$(BUILD_DIR)/$(OUTPUT_BASENAME).bin" "$(OUTPUT_DIR)/$(OUTPUT_BASENAME).bin"
	@if [ -f "$(M0_IMAGE)" ]; then cp "$(M0_IMAGE)" "$(OUTPUT_DIR)/$(APP_NAME)_M0_$(APP_VERSION).bin"; fi
	@if [ -f "$(OTA_LOADER)" ]; then cp "$(OTA_LOADER)" "$(OUTPUT_DIR)/$(APP_NAME)_OTA_LOADER_$(APP_VERSION).bin"; fi
	@printf '%s\n' \
		'4,HW,COLD,M3' \
		'0x00008000,0xB24EF00E' \
		'4,HW,COLD,M0' \
		'0x00004000,0x47384F00' \
		'0x00004004,0x00608001' \
		'4,CODE,COLD,M3' \
		'0x004164a0,.\\$(OUTPUT_BASENAME).bin' \
		> "$(OUTPUT_DIR)/$(APP_NAME)_PatchData_$(APP_VERSION).txt"

clean:
	rm -rf "$(BUILD_DIR)"

# OpenOPL1000 real-port target for the Opulinks OPL1000 A2 SDK.
#
# This builds the real OpenBeken core/web/command runtime against the Opulinks
# patch SDK. OPL1000 support is intentionally STA-only because the vendor SDK
# does not provide SoftAP.

APP_NAME ?= OpenOPL1000
APP_VERSION ?= dev
SDK_ROOT ?= ../../sdk/OpenOPL1000
OBK_ROOT ?= ../..
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
NO_PIE_FLAG := $(shell printf 'int main(void){return 0;}\n' | $(CC) -x c -no-pie -c -o /dev/null - >/dev/null 2>&1 && echo -no-pie)

APS_ROOT       := $(SDK_ROOT)/SDK/APS
APS_PATCH_ROOT := $(SDK_ROOT)/SDK/APS_PATCH
LINK_SCRIPT    := $(OBK_ROOT)/platforms/OPL1000/link_split_m3.ld
SYMDEFS        := $(APS_ROOT)/targets/opl1000/object/symdefs_gcc.o
PATCH_LIB      := $(APS_ROOT)/targets/opl1000/Output-GCC/opl1000_app_m3_lite.a
M0_IMAGE       := $(SDK_ROOT)/FW_Pack/opl1000_m0.bin
OTA_LOADER     := $(SDK_ROOT)/FW_Pack/opl1000_ota_loader.bin
PATCHDATA_SCRIPT := $(OBK_ROOT)/platforms/OPL1000/pack/PatchData.txt

OBK_SRCS := $(OBK_ROOT)/src/

SOURCES := main_patch.c
SOURCES += $(OBK_SRCS)user_main.c
SOURCES += $(OBK_SRCS)base64/base64.c
SOURCES += $(OBK_SRCS)cJSON/cJSON.c
SOURCES += $(OBK_SRCS)cmnds/cmd_channels.c
SOURCES += $(OBK_SRCS)cmnds/cmd_enums.c
SOURCES += $(OBK_SRCS)cmnds/cmd_eventHandlers.c
SOURCES += $(OBK_SRCS)cmnds/cmd_if.c
SOURCES += $(OBK_SRCS)cmnds/cmd_main.c
SOURCES += $(OBK_SRCS)cmnds/cmd_repeatingEvents.c
SOURCES += $(OBK_SRCS)cmnds/cmd_tasmota.c
SOURCES += $(OBK_SRCS)cmnds/cmd_tcp.c
SOURCES += $(OBK_SRCS)cmnds/cmd_tokenizer.c
SOURCES += $(OBK_SRCS)driver/drv_deviceclock.c
SOURCES += $(OBK_SRCS)driver/drv_main.c
SOURCES += $(OBK_SRCS)driver/drv_timed_events.c
SOURCES += $(OBK_SRCS)driver/drv_uart.c
SOURCES += $(OBK_SRCS)hal/opl1000/hal_adc_opl1000.c
SOURCES += $(OBK_SRCS)hal/opl1000/hal_flashConfig_opl1000.c
SOURCES += $(OBK_SRCS)hal/opl1000/hal_fim_config_opl1000.c
SOURCES += $(OBK_SRCS)hal/opl1000/hal_flashVars_opl1000.c
SOURCES += $(OBK_SRCS)hal/opl1000/hal_generic_opl1000.c
SOURCES += $(OBK_SRCS)hal/opl1000/hal_hwtimer_opl1000.c
SOURCES += $(OBK_SRCS)hal/opl1000/hal_ota_opl1000.c
SOURCES += $(OBK_SRCS)hal/opl1000/hal_pins_opl1000.c
SOURCES += $(OBK_SRCS)hal/opl1000/hal_uart_opl1000.c
SOURCES += $(OBK_SRCS)hal/opl1000/hal_wifi_opl1000.c
SOURCES += $(OBK_SRCS)httpserver/http_basic_auth.c
SOURCES += $(OBK_SRCS)httpserver/http_fns.c
SOURCES += $(OBK_SRCS)httpserver/http_tcp_server.c
SOURCES += $(OBK_SRCS)httpserver/json_interface.c
SOURCES += $(OBK_SRCS)httpserver/new_http.c
SOURCES += $(OBK_SRCS)httpserver/new_tcp_server.c
SOURCES += $(OBK_SRCS)httpserver/rest_interface.c
SOURCES += $(OBK_SRCS)jsmn/jsmn.c
SOURCES += $(OBK_SRCS)libraries/obktime/obktime.c
SOURCES += $(OBK_SRCS)logging/logging.c
SOURCES += $(OBK_SRCS)new_cfg.c
SOURCES += $(OBK_SRCS)new_common.c
SOURCES += $(OBK_SRCS)new_pins.c
SOURCES += $(OBK_SRCS)rgb2hsv.c
SOURCES += $(OBK_SRCS)tiny_crc8.c

OBJECTS := $(addprefix $(BUILD_DIR)/,$(notdir $(SOURCES:.c=.o)))
VPATH := $(sort $(dir $(SOURCES)))

DEFS  := -DGCC
DEFS  += -DARMCM3
DEFS  += -DOPL1000_APS
DEFS  += -DPLATFORM_OPL1000=1
DEFS  += -DOPENOPL1000_NO_BLE=1
DEFS  += -D__noSIMULATOR__
DEFS  += -D__LWIP_TASK__
DEFS  += -D__WPA_SUPPLICANT__
DEFS  += -D__noHW_CRYPTO_ENGINE__
DEFS  += -D__WIFI_MAC_TASK__
DEFS  += -D__NL1000_An__
DEFS  += -D__PMP_ENABLE__
DEFS  += -D__PMP_REGION__
DEFS  += -D__HEAP_REGION__
DEFS  += -DnoLWIP_DYNAMIC_DEBUG_ENABLE
DEFS  += -D__noRTL_SIMULATION__
DEFS  += -D_ARMABI_PURE=
DEFS  += -DLWIP_TIMEVAL_PRIVATE=0
DEFS  += -DOPENOPL1000_WIFI_SSID=\"OPL1000_AP\"
DEFS  += -DOPENOPL1000_WIFI_PASSWORD=\"1234abcd\"

# Key include paths first; broad SDK paths follow for the vendor patch headers.
INCLUDES := -I.
INCLUDES += -I $(OBK_ROOT)/src
INCLUDES += -I $(OBK_ROOT)/include
INCLUDES += -I $(OBK_ROOT)/src/hal
INCLUDES += -I $(OBK_ROOT)/src/hal/opl1000
INCLUDES += -I $(APS_ROOT)/FreeRtos/Source/include
INCLUDES += -I $(APS_ROOT)/FreeRtos/Source/portable/GCC/ARM_CM3
INCLUDES += -I $(APS_PATCH_ROOT)/FreeRtos/Source/include
INCLUDES += -I $(APS_PATCH_ROOT)/FreeRtos/Source/portable/GCC/ARM_CM3
INCLUDES += -I $(APS_ROOT)/driver/CMSIS/Include
INCLUDES += -I $(APS_ROOT)/driver/CMSIS/Device/opl1000/Include
INCLUDES += -I $(APS_ROOT)/middleware/third_party/lwip-2.0.3
INCLUDES += -I $(APS_ROOT)/middleware/third_party/lwip-2.0.3/lwip/src/include
INCLUDES += -I $(APS_ROOT)/middleware/third_party/lwip-2.0.3/ports/freertos/include
INCLUDES += -I $(APS_ROOT)/middleware/third_party/lwip-2.0.3/ports/freertos/netif
INCLUDES += -I $(APS_PATCH_ROOT)/middleware/third_party/lwip-2.0.3
INCLUDES += -I $(APS_PATCH_ROOT)/middleware/third_party/lwip-2.0.3/lwip/src/include
INCLUDES += -I $(APS_PATCH_ROOT)/middleware/third_party/lwip-2.0.3/ports/freertos/include
INCLUDES += -I $(APS_PATCH_ROOT)/middleware/third_party/lwip-2.0.3/ports/freertos/netif
INCLUDES += -I $(APS_ROOT)/middleware/third_party/httpclient
INCLUDES += -I $(APS_PATCH_ROOT)/middleware/third_party/httpclient
INCLUDES += -I $(APS_ROOT)/middleware/third_party/wpa_supplicant-0.7.3
INCLUDES += -I $(APS_ROOT)/middleware/third_party/wpa_supplicant-0.7.3/src
INCLUDES += -I $(APS_ROOT)/middleware/third_party/wpa_supplicant-0.7.3/src/rsn_supp
INCLUDES += -I $(APS_ROOT)/middleware/third_party/wpa_supplicant-0.7.3/src/drivers
INCLUDES += -I $(APS_ROOT)/middleware/third_party/wpa_supplicant-0.7.3/src/utils
INCLUDES += -I $(APS_ROOT)/middleware/third_party/wpa_supplicant-0.7.3/wpa_supplicant
INCLUDES += -I $(APS_PATCH_ROOT)/middleware/third_party/wpa_supplicant-0.7.3
INCLUDES += -I $(APS_PATCH_ROOT)/middleware/third_party/wpa_supplicant-0.7.3/src
INCLUDES += -I $(APS_PATCH_ROOT)/middleware/third_party/wpa_supplicant-0.7.3/src/rsn_supp
INCLUDES += -I $(APS_PATCH_ROOT)/middleware/third_party/wpa_supplicant-0.7.3/src/drivers
INCLUDES += -I $(APS_PATCH_ROOT)/middleware/third_party/wpa_supplicant-0.7.3/src/utils
INCLUDES += -I $(APS_PATCH_ROOT)/middleware/third_party/wpa_supplicant-0.7.3/wpa_supplicant
INCLUDES += $(shell find \
    "$(APS_PATCH_ROOT)/apps" "$(APS_PATCH_ROOT)/driver" "$(APS_PATCH_ROOT)/middleware/netlink" "$(APS_PATCH_ROOT)/project" \
    "$(APS_ROOT)/apps" "$(APS_ROOT)/driver" "$(APS_ROOT)/middleware/netlink" "$(APS_ROOT)/project" \
    -type d \
    ! -path "*/.git*" \
    ! -path "*/FreeRtos/Source/portable/Keil*" \
    ! -path "*/FreeRtos/Source/portable/RVDS*" \
    ! -path "*/FreeRtos/Source/portable/IAR*" \
    | sed 's#^#-I #')

CFLAGS  := -mthumb -mcpu=cortex-m3 -mlittle-endian
CFLAGS  += -std=gnu99 -Os
CFLAGS  += -ffunction-sections -fdata-sections
CFLAGS  += -flto -fuse-linker-plugin -fno-fat-lto-objects
CFLAGS  += -fno-unwind-tables -fno-asynchronous-unwind-tables
CFLAGS  += -fno-builtin -fno-common -fmessage-length=0
CFLAGS  += -nostartfiles $(NO_PIE_FLAG) -specs=nosys.specs -ffreestanding
CFLAGS  += -include sys_common.h
CFLAGS  += -Wall -Wextra
CFLAGS  += -Wno-implicit-function-declaration -Wno-int-conversion -Wno-unused-parameter
CFLAGS  += -Wno-missing-field-initializers -Wno-unused-function -Wno-unused-variable
CFLAGS  += -Wno-pointer-sign -Wno-format -Wno-sign-compare
CFLAGS  += $(DEFS) $(INCLUDES)

LDFLAGS := -mthumb -mcpu=cortex-m3 -mlittle-endian
LDFLAGS += -T $(LINK_SCRIPT)
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -flto -fuse-linker-plugin
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -Wl,--entry=__Patch_EntryPoint
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(OUTPUT_BASENAME).map
LDFLAGS += -Wl,--wrap=at_func_init_patch
LDFLAGS += -Wl,--wrap=Diag_PatchInit
LDFLAGS += -Wl,--wrap=le_ctrl_pre_patch_init
LDFLAGS += -Wl,--wrap=wpa_cli_func_init_patch
LDFLAGS += -Xlinker --just-symbols=$(SYMDEFS)

.PHONY: all package clean
all: package

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/$(OUTPUT_BASENAME).elf: $(OBJECTS) $(PATCH_LIB) $(LINK_SCRIPT) $(SYMDEFS)
	$(CC) -o $@ $(LDFLAGS) $(CFLAGS) $(OBJECTS) $(PATCH_LIB)
	$(SIZE) $@

$(BUILD_DIR)/$(OUTPUT_BASENAME)_main.bin: $(BUILD_DIR)/$(OUTPUT_BASENAME).elf
	$(OBJCOPY) -O binary --remove-section=.shm_region $< $@

$(BUILD_DIR)/$(OUTPUT_BASENAME)_shm.bin: $(BUILD_DIR)/$(OUTPUT_BASENAME).elf
	$(OBJCOPY) -O binary --only-section=.shm_region $< $@

package: $(BUILD_DIR)/$(OUTPUT_BASENAME)_main.bin $(BUILD_DIR)/$(OUTPUT_BASENAME)_shm.bin
	mkdir -p $(OUTPUT_DIR)
	mkdir -p $(OUTPUT_DIR)/pack_ready
	cp $(BUILD_DIR)/$(OUTPUT_BASENAME)_main.bin $(OUTPUT_DIR)/$(OUTPUT_BASENAME)_main.bin
	cp $(BUILD_DIR)/$(OUTPUT_BASENAME)_shm.bin $(OUTPUT_DIR)/$(OUTPUT_BASENAME)_shm.bin
	cp $(BUILD_DIR)/$(OUTPUT_BASENAME).map $(OUTPUT_DIR)/$(OUTPUT_BASENAME).map
	cp $(M0_IMAGE) $(OUTPUT_DIR)/$(APP_NAME)_M0_$(APP_VERSION).bin
	cp $(OTA_LOADER) $(OUTPUT_DIR)/$(APP_NAME)_OTA_LOADER_$(APP_VERSION).bin
	cp $(PATCHDATA_SCRIPT) $(OUTPUT_DIR)/$(APP_NAME)_PatchData_$(APP_VERSION).txt
	cp $(PATCHDATA_SCRIPT) $(OUTPUT_DIR)/pack_ready/PatchData.txt
	cp $(M0_IMAGE) $(OUTPUT_DIR)/pack_ready/opl1000_m0.bin
	cp $(BUILD_DIR)/$(OUTPUT_BASENAME)_main.bin $(OUTPUT_DIR)/pack_ready/opl1000_app_m3_main.bin
	cp $(BUILD_DIR)/$(OUTPUT_BASENAME)_shm.bin $(OUTPUT_DIR)/pack_ready/opl1000_app_m3_shm.bin
	cd $(OUTPUT_DIR)/pack_ready && if command -v zip >/dev/null 2>&1; then zip -q -r ../$(APP_NAME)_pack_ready_$(APP_VERSION).zip PatchData.txt opl1000_m0.bin opl1000_app_m3_main.bin opl1000_app_m3_shm.bin; else python3 -m zipfile -c ../$(APP_NAME)_pack_ready_$(APP_VERSION).zip PatchData.txt opl1000_m0.bin opl1000_app_m3_main.bin opl1000_app_m3_shm.bin; fi
	@echo "OpenOPL1000 output written to $(OUTPUT_DIR)"
	@echo "Opulinks split-M3 pack-ready files written to $(OUTPUT_DIR)/pack_ready"
	@echo "Pack with: PatchData.txt + opl1000_m0.bin + opl1000_app_m3_main.bin + opl1000_app_m3_shm.bin"

clean:
	rm -rf $(BUILD_DIR)

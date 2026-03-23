set(OBK_SRCS "../../../src/")
include($ENV{IDF_PATH}/../../platforms/obk_main.cmake)

set(BERRY_SRCPATH "../../../libraries/berry/src")
set(BERRY_MODULEPATH "${OBK_SRCS}berry/modules")
include($ENV{IDF_PATH}/../../libraries/berry.cmake)

if(DEFINED ENV{OBK_ENABLE_BERRY_BUILD})
	set(OBK_ENABLE_BERRY_BUILD "$ENV{OBK_ENABLE_BERRY_BUILD}")
else()
	set(OBK_ENABLE_BERRY_BUILD "0")
endif()

set(OBK_BERRY_SRC)
if(OBK_ENABLE_BERRY_BUILD)
	list(APPEND OBK_BERRY_SRC ${BERRY_SRC_C})
endif()

set(PROJ_ALL_SRC
	${OBK_SRCS}hal/espidf/hal_adc_espidf.c
	${OBK_SRCS}hal/espidf/hal_flashConfig_espidf.c
	${OBK_SRCS}hal/espidf/hal_flashVars_espidf.c
	${OBK_SRCS}hal/espidf/hal_generic_espidf.c
	${OBK_SRCS}hal/espidf/hal_hwtimer_espidf.c
	${OBK_SRCS}hal/espidf/hal_main_espidf.c
	${OBK_SRCS}hal/espidf/hal_pins_espidf.c
	${OBK_SRCS}hal/espidf/hal_wifi_espidf.c
	${OBK_SRCS}hal/espidf/hal_uart_espidf.c
	${OBK_SRCS}hal/espidf/hal_ota_espidf.c
	${OBKM_SRC}
	${OBK_BERRY_SRC}
	../../../sdk/OpenESP8266/components/lwip/lwip/src/apps/mdns/mdns.c
	../../../libraries/mqtt_patched.c
)
idf_component_register(SRCS ${PROJ_ALL_SRC}
			PRIV_REQUIRES lwip nvs_flash app_update)

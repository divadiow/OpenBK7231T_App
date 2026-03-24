# Component Makefile
#
## These include paths would be exported to project level
COMPONENT_ADD_INCLUDEDIRS += src/ src/httpserver/ src/httpclient/ src/cmnds/ src/logging/ src/hal/bl602/ src/mqtt/ src/cJSON src/base64 src/driver src/devicegroups src/bitmessage src/littlefs libraries/berry/src src/libraries/obktime/ include/

## not be exported to project level
COMPONENT_PRIV_INCLUDEDIRS :=

CPPFLAGS += -DOBK_VARIANT=${OBK_VARIANT} -Wno-undef -Os -ffunction-sections -fdata-sections -fmerge-constants
CXXFLAGS += -Wno-delete-non-virtual-dtor -Wno-error=format -Os -ffunction-sections -fdata-sections -fmerge-constants

## This component's src 
COMPONENT_SRCS := 

COMPONENT_OBJS := $(patsubst %.c,%.o, $(COMPONENT_SRCS))
COMPONENT_OBJS := $(patsubst %.S,%.o, $(COMPONENT_OBJS))

COMPONENT_SRCDIRS := src/ src/jsmn src/httpserver src/httpclient src/cmnds src/logging src/hal/bl602 src/mqtt src/i2c src/cJSON src/base64 src/driver src/devicegroups src/bitmessage src/littlefs src/hal/generic libraries/berry/src src/berry src/berry/modules src/libraries/obktime src/libraries/IRremoteESP8266/src

COMPONENT_ADD_LDFLAGS = -Wl,--gc-sections -Wl,--whole-archive -l$(COMPONENT_NAME) -Wl,--no-whole-archive

# Minimal TXW81X diagnostic build.
# Intentionally does NOT include OpenBeken application sources.
# The OpenTXW81X project keeps its proven vendor startup/system/device/link/package path.

OBK_DIR = ../../..
CFLAGS += -DTXW81X_DIAG_MINIMAL
SRC_C += $(OBK_DIR)/platforms/TXW81X/txw81x_diag_main.c

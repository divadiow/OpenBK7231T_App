#!/bin/bash
# Shared inner build script for all beken_freertos_sdk targets (bk7231u, bk7238, bk7251, bk7252n, etc.)
# Arguments: APP_NAME APP_VERSION SDK_DIR OBK_VARIANT ACTION SOC_TARGET
set -e

APP_NAME="$1"
APP_VERSION="$2"
SDK_DIR="$3"
export OBK_VARIANT="$4"
ACTION="$5"
SOC_TARGET="$6"

if [ -z "$SOC_TARGET" ]; then
    echo "[ERROR] SOC_TARGET (arg 6) is required! e.g. bk7231u, bk7238, bk7251, bk7252n"
    exit 1
fi

echo "[INFO] Inner build script started for $SOC_TARGET"

SCRIPT_DIR="$(pwd)"

# Keep native Windows Python ahead of any MSYS tool directory added below.
# The SDK packager wrapper uses os.name to select beken_packager.exe versus
# the Linux ELF, so running it under MSYS Python selects the wrong executable.
WINDOWS_PYTHON="$(command -v python 2>/dev/null || true)"

# Export the borrowed ARM GCC toolchain to the environment for Beken FreeRTOS SDK Makefile to pick up
TOOLCHAIN_RAW="$(pwd)/sdk/OpenBK7231N/platforms/BK7231N/toolchain/windows/gcc-arm-none-eabi-4_9-2015q1/bin/"
export ARM_GCC_TOOLCHAIN="$(cygpath -u "$TOOLCHAIN_RAW")/"
export PATH="$ARM_GCC_TOOLCHAIN:$PATH"
echo "[INFO] Toolchain: $ARM_GCC_TOOLCHAIN"

# Set a localized TEMP dir to avoid PyInstaller extraction permission errors (AntiVirus/Defender locking)
# beken_packager.exe is a PyInstaller bundle that needs to extract VCRUNTIME140.dll
# Use native Windows Temp directory where DLL drops are historically trusted by Defender
# We must double-escape backslashes so MSYS Make doesn't swallow them when passing to subshells!
LOCAL_TMP_POSIX="$(cygpath -u "$USERPROFILE/AppData/Local/Temp/obk_p_$$")"
LOCAL_TMP="$(cygpath -w "$LOCAL_TMP_POSIX" | sed 's/\\/\\\\/g')"
echo "[INFO] Setting local TEMP dir: $LOCAL_TMP"
mkdir -p "$LOCAL_TMP_POSIX"
export TEMP="$LOCAL_TMP"
export TMP="$LOCAL_TMP"
export TMPDIR="$LOCAL_TMP"

# Navigate to the build system directory
cd "$SDK_DIR"

# First, check if make is available
if type make > /dev/null 2>&1; then
    echo "[INFO] GNU make is available."
else
    echo "[INFO] GNU make not found in Git Bash. Attempting to get it..."
    MAKE_FOUND=0
    for MAKE_PATH in /c/ProgramData/chocolatey/bin/make.exe /w/TOOLS/msys64/usr/bin/make.exe /c/msys64/usr/bin/make.exe; do
        if [ -f "$MAKE_PATH" ]; then
            export PATH="$(dirname "$MAKE_PATH"):$PATH"
            MAKE_FOUND=1
            echo "[INFO] Found make at $MAKE_PATH"
            break
        fi
    done
    if [ $MAKE_FOUND -eq 0 ]; then
        echo "[ERROR] GNU make not found!"
        exit 1
    fi
fi

if [ "$ACTION" = "clean" ]; then
    echo "[INFO] Running $SOC_TARGET clean from $(pwd)..."
    make clean -C ./ OBK_DIR="../../src" OBK_VARIANT="$OBK_VARIANT"
    exit 0
fi

BUILD_ID="v1|$SOC_TARGET|$APP_VERSION|$OBK_VARIANT"
BUILD_STAMP="./out/.obk_build_identity"
LAST_BUILD_ID=""
if [ -f "$BUILD_STAMP" ]; then
    IFS= read -r LAST_BUILD_ID < "$BUILD_STAMP" || true
fi
if [ "$LAST_BUILD_ID" != "$BUILD_ID" ]; then
    echo "[INFO] Build identity changed ($LAST_BUILD_ID -> $BUILD_ID); cleaning stale objects..."
    make clean -C ./ OBK_DIR="../../src" OBK_VARIANT="$OBK_VARIANT"
fi

if [ -z "$WINDOWS_PYTHON" ] || ! "$WINDOWS_PYTHON" -c 'import os, sys; raise SystemExit(os.name != "nt" or sys.version_info.major != 3)'; then
    echo "[ERROR] Native Windows Python 3 is required for Beken packaging."
    exit 1
fi
WINDOWS_PYTHON_DIR="$(dirname "$WINDOWS_PYTHON")"

# Some MSYS installations include a POSIX Python beside make. Keep the native
# interpreter first so application.mk's hard-coded 'python' command invokes
# beken_packager.exe rather than the bundled Linux beken_packager ELF.
export PATH="$WINDOWS_PYTHON_DIR:$PATH"
echo "[INFO] Windows Python: $WINDOWS_PYTHON"

echo "[INFO] Running $SOC_TARGET build from $(pwd)..."

# Strip GCC 4.9 unsupported Wno-error flags from application.mk
sed -i 's/-Wno-error=implicit-function-declaration -Wno-error=incompatible-pointer-types -Wno-error=return-mismatch -Wno-error=int-conversion -Wno-error=changes-meaning//g' application.mk
# Force encrypt.exe on Windows MSYS (which identifies as MINGW64, unlike MINGW32 tracked in application.mk)
sed -i 's/"\.\/tools\/crc_binary\/encrypt_n"/"\.\/tools\/crc_binary\/encrypt\.exe"/g' application.mk

# Replace encrypt.exe with the compatible one from OpenBK7231T SDK that supports multi-key arguments
cp "$(cygpath -u "$SCRIPT_DIR/sdk/OpenBK7231T/platforms/bk7231t/bk7231t_os/tools/generate/package_tool/windows/encrypt.exe")" ./tools/crc_binary/encrypt.exe

make TEMP="$LOCAL_TMP" TMP="$LOCAL_TMP" TMPDIR="$LOCAL_TMP" "$SOC_TARGET" -j8 USER_SW_VER="$APP_VERSION" OBK_VARIANT="$OBK_VARIANT" OBK_DIR="../../src"

printf '%s\n' "$BUILD_ID" > "$BUILD_STAMP"

# Generate OTA RBL if bsp.bin exists
if [ -f ./out/bsp.bin ]; then
    ./tools/rtt_ota/rt_ota_packaging_tool_cli.exe -f ./out/bsp.bin -o ./out/app.rbl -p app -c gzip -s aes -k 0123456789ABCDEF0123456789ABCDEF -i 0123456789ABCDEF -v "$APP_VERSION"
fi

echo "[INFO] Build complete!"

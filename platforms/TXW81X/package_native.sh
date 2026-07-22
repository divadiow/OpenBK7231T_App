#!/bin/sh
# Native TXW81x APP.bin and APP_compress.bin packaging for OpenBeken.
set -eu

ROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
PROJECT_DIR="$ROOT_DIR/sdk/OpenTXW81X/project"
PACKER="$ROOT_DIR/tools/txw81x_pack/txw81x_pack.py"
LOADER="$ROOT_DIR/sdk/OpenTXW81X/sdk/chip/txw81x/loader_compress.bin"

: "${TXW81X_SDK_VERSION:=02050307}"
: "${TXW81X_SVN_VERSION:=8EB5}"
: "${TXW81X_IMAGE_VERSION:=0}"
: "${TXW81X_BUILD_DATE:=$(date -u +%F)}"

for required in \
    "$PROJECT_DIR/project.hex" \
    "$PROJECT_DIR/parameter.bincfg" \
    "$PROJECT_DIR/makecode.ini" \
    "$LOADER" \
    "$PACKER"; do
    if [ ! -f "$required" ]; then
        echo "TXW81x packaging input not found: $required" >&2
        exit 1
    fi
done

echo "Packaging TXW81x images natively with Python"
echo "  SDK version : 0x$TXW81X_SDK_VERSION"
echo "  SVN version : 0x$TXW81X_SVN_VERSION"
echo "  Image version: 0x$TXW81X_IMAGE_VERSION"
echo "  Build date   : $TXW81X_BUILD_DATE"

python3 "$PACKER" pack-app \
    --input "$PROJECT_DIR/project.hex" \
    --parameter-cfg "$PROJECT_DIR/parameter.bincfg" \
    --makecode-ini "$PROJECT_DIR/makecode.ini" \
    --sdk-version "$TXW81X_SDK_VERSION" \
    --svn-version "$TXW81X_SVN_VERSION" \
    --app-version "$TXW81X_IMAGE_VERSION" \
    --date "$TXW81X_BUILD_DATE" \
    --output "$PROJECT_DIR/APP.bin"

python3 "$PACKER" pack-ota \
    --app "$PROJECT_DIR/APP.bin" \
    --loader "$LOADER" \
    --output "$PROJECT_DIR/APP_compress.bin"

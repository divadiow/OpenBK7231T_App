#!/usr/bin/env python3
"""Build the isolated CKW04 Bank-B stage-one loader."""

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

from stock_layout import BANK_B_XIP, CKW04_FS_XIP, linker_overlay, uart_image


SDK_COMMIT = "5bb261d980fd51e67b94b41c7f5c9868cc43301d"
LOADER_COMMIT = "7aa8a641836d6fcdeb0b3af5714a4cc634f306b4"
APP_COMMIT = "32269ac9e9e249c337cee136d8514b2ac9b71eb3"
SOURCE_FILES = (
	"main.c",
	"stock_entry.c",
	"stock_runtime.c",
	"sv6x66_loader.c",
	"sv6x66_loader.h",
	"xz_runtime_adapter.c",
	"xz_runtime_compat.h",
)
XZ_FILES = (
	"linux/include/linux/xz.h",
	"linux/lib/xz/xz_config.h",
	"linux/lib/xz/xz_crc32.c",
	"linux/lib/xz/xz_dec_lzma2.c",
	"linux/lib/xz/xz_dec_stream.c",
	"linux/lib/xz/xz_lzma2.h",
	"linux/lib/xz/xz_private.h",
	"linux/lib/xz/xz_stream.h",
)
REQUIRED_SYMBOLS = (
	"APP_Init",
	"SV6X66_Loader",
	"SV6X66_StockEntryHeader",
	"SV6X66_StockEntry",
	"__wrap__soc_clk_init",
	"__wrap__soc_io_init",
	"__wrap_xip_init",
	"__wrap_xip_leave",
	"__wrap_xip_enter",
	"flash_init",
	"flash_sector_erase",
	"xz_crc32_init",
	"xz_dec_init",
	"xz_dec_run",
	"psSha256Init",
	"drv_hsuart_init",
	"drv_hsuart_set_format",
)
RAM_SYMBOLS = (
	"__wrap__soc_clk_init",
	"__wrap__soc_io_init",
	"__wrap_xip_init",
	"__wrap_xip_leave",
	"__wrap_xip_enter",
	"flash_init",
	"flash_sector_erase",
)


def sha256(path: Path) -> str:
	return hashlib.sha256(path.read_bytes()).hexdigest()


def run(command, **kwargs):
	print("+ " + " ".join(map(str, command)), flush=True)
	return subprocess.run(command, check=True, **kwargs)


def main() -> None:
	parser = argparse.ArgumentParser()
	parser.add_argument("--toolchain", required=True, type=Path)
	parser.add_argument("--build-dir", type=Path)
	parser.add_argument("--output", type=Path)
	parser.add_argument("--jobs", default=4, type=int)
	parser.add_argument("--sdk-commit", required=True)
	args = parser.parse_args()
	if args.jobs < 1:
		parser.error("--jobs must be positive")
	if args.sdk_commit.lower() != SDK_COMMIT:
		parser.error("--sdk-commit does not match the pinned OpenSV6X66 source")

	source = Path(__file__).resolve().parent
	repository = source.parents[2]
	sdk = repository / "sdk/OpenSV6X66/sdk"
	xz = repository / "sdk/OpenSV6X66/framework/uOTA/src/2nd_boot/xz"
	build_root = (args.build_dir or repository / "output/SV6X66_CKW04_STAGE1/build").resolve()
	output = (args.output or repository / "output/SV6X66_CKW04_STAGE1/artifacts").resolve()
	marker = build_root / ".ckw04-stage1-build"
	if build_root.exists():
		if not marker.exists():
			raise SystemExit("refusing to replace an unowned build directory")
		shutil.rmtree(build_root)
	build_root.mkdir(parents=True)
	marker.write_text("generated CKW04 stage-one build\n", encoding="ascii")
	output.mkdir(parents=True, exist_ok=True)
	if not (sdk / "Makefile").is_file():
		raise SystemExit("active bounded OpenSV6X66 SDK is missing")
	for relative in XZ_FILES:
		if not (xz / relative).is_file():
			raise SystemExit("missing pinned XZ source: " + relative)

	stage = build_root / "sdk"
	shutil.copytree(sdk, stage)
	overlay = stage / "obk_app"
	platform = overlay / "platforms/SV6X66"
	platform.mkdir(parents=True)
	for name in SOURCE_FILES:
		shutil.copy2(source / name, platform / name)
	shutil.copy2(source / "module.mk", overlay / "module.mk")
	shutil.copy2(source / "project.mk", platform / "project.mk")
	for relative in XZ_FILES:
		destination = overlay / "framework/uOTA/src/2nd_boot/xz" / relative
		destination.parent.mkdir(parents=True, exist_ok=True)
		shutil.copy2(xz / relative, destination)
	xz_header = overlay / "framework/uOTA/src/2nd_boot/xz/linux/include/linux/xz.h"
	text = xz_header.read_text(encoding="utf-8")
	needle = '#include "rec_pub.h"'
	if text.count(needle) != 1:
		raise SystemExit("unexpected pinned XZ header")
	xz_header.write_text(text.replace(needle, '#include "xz_runtime_compat.h"'), encoding="utf-8")

	template = sdk / "projects/lite_mac/ld/lite_ilm_flash.lds.S"
	linker = linker_overlay(template.read_text(encoding="utf-8"), BANK_B_XIP, CKW04_FS_XIP)
	(platform / "stock_flash.lds.S").write_text(linker, encoding="utf-8")
	(stage / "build/project_cfg.mk").write_text(
		"include obk_app/platforms/SV6X66/project.mk\n", encoding="ascii")

	toolchain = args.toolchain.resolve()
	compiler = toolchain / "nds32le-elf-gcc"
	objcopy = toolchain / "nds32le-elf-objcopy"
	objdump = toolchain / "nds32le-elf-objdump"
	nm = toolchain / "nds32le-elf-nm"
	compiler_version = subprocess.check_output([str(compiler), "--version"], text=True).splitlines()[0]
	print(compiler_version)
	environment = dict(os.environ, PATH=str(toolchain) + os.pathsep + os.environ["PATH"])
	target = stage / "image/OpenSV6166F.elf"
	run([
		"make", "-j" + str(args.jobs), str(target),
		"MQTT_EN=0", "HTTPD_EN=0", "HTTPC_EN=0", "SSL_EN=0", "MBED_EN=0",
		"IPERF3_EN=0", "SMARTCONFIG_EN=0", "PING_EN=0", "TFTP_EN=0",
		"BUILD_OPTION=RELEASE", "BUILD_SHOW_ILM_INFO=0", "BUILD_SHOW_DLM_INFO=0",
	], cwd=stage, env=environment)

	elf = stage / "image/OpenSV6166F.elf"
	map_file = stage / "image/OpenSV6166F.map"
	symbol_text = subprocess.check_output([str(nm), "--defined-only", str(elf)], text=True)
	symbols = {}
	for line in symbol_text.splitlines():
		fields = line.split()
		if len(fields) >= 3 and re.fullmatch(r"[0-9a-fA-F]+", fields[0]):
			symbols[fields[-1]] = (int(fields[0], 16), fields[-2])
	missing = [name for name in REQUIRED_SYMBOLS if name not in symbols]
	if missing:
		raise RuntimeError("missing required symbols: " + ", ".join(missing))

	raw, sections = uart_image(elf.read_bytes(), BANK_B_XIP, CKW04_FS_XIP)
	ram_sections = [sections[name] for name in (".fast_boot_code", ".prog_in_sram")]
	if any(not any(section["vma"] <= symbols[name][0] < section["vma"] + section["bytes"]
			for section in ram_sections) for name in RAM_SYMBOLS):
		raise RuntimeError("clock/XIP/flash routines are not all linked in initialized SRAM")
	check = build_root / "objcopy.bin"
	run([str(objcopy), "-O", "binary", str(elf), str(check)])
	if check.read_bytes() != raw:
		raise RuntimeError("objcopy and audited UART serialization differ")

	stem = "CKW04_SV6X66_stage1_115200"
	raw_path = output / (stem + ".raw.bin")
	gzip_path = output / (stem + ".bin.gz")
	elf_path = output / (stem + ".elf")
	map_path = output / (stem + ".map")
	nm_path = output / (stem + ".nm")
	dis_path = output / (stem + ".dis")
	raw_path.write_bytes(raw)
	with gzip_path.open("wb") as destination:
		with gzip.GzipFile(filename="", mode="wb", fileobj=destination, mtime=0) as stream:
			stream.write(raw)
	shutil.copy2(elf, elf_path)
	shutil.copy2(map_file, map_path)
	nm_path.write_text(symbol_text, encoding="ascii")
	with dis_path.open("w", encoding="ascii") as stream:
		run([str(objdump), "-dr", str(elf)], stdout=stream)

	if b"BAUD=115200" not in raw or b"BAUD=2000000" in raw:
		raise RuntimeError("artifact does not contain only the fixed 115200 transport identity")
	manifest = {
		"target": "CKW04/SV6x66 Bank-B stage one",
		"baud": 115200,
		"sdk_commit": SDK_COMMIT,
		"loader_commit": LOADER_COMMIT,
		"pinned_app_commit": APP_COMMIT,
		"toolchain": compiler_version,
		"hardware_tested": False,
		"load_range": [BANK_B_XIP, CKW04_FS_XIP],
		"required_symbols": list(REQUIRED_SYMBOLS),
		"artifacts": {
			path.name: {"bytes": path.stat().st_size, "sha256": sha256(path)}
			for path in (gzip_path, raw_path, elf_path, map_path, nm_path, dis_path)
		},
	}
	(output / (stem + ".json")).write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
	print(json.dumps(manifest["artifacts"], indent=2))


if __name__ == "__main__":
	main()

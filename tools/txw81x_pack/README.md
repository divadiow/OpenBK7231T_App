# Native TXW81x firmware packer

This directory contains a standard-library-only Python replacement for the TXW81x post-build operations previously performed by Taixin's Windows tools:

- `BinScript.exe` for Intel HEX remapping and parameter extraction;
- `makecode.exe` for the flashable SPI application image (`APP.bin`);
- `xz.exe` plus `compress.exe` for the compressed OTA image (`APP_compress.bin`).

The OpenBeken TXW81x target invokes this packer through `platforms/TXW81X/package_native.sh`. The implementation removes Wine, Windows and WSL from the TXW81x build path, but generated images still require real-hardware flash and OTA validation before the change is considered production-proven.

## Files

- `txw81x_pack.py` — pack, inspect and validate APP/OTA images.
- `tests/test_txw81x_pack.py` — synthetic tests plus opt-in known-good Taixin integration tests.
- `tests/reference_manifest.json` — hashes and parsed values for public known-good OpenBeken Actions outputs.
- `ATTRIBUTION.md` — retained MIT attribution for RNode-derived portions.

No Taixin executable, SDK source, loader or reference firmware image is included. The existing OpenTXW81X checkout supplies `makecode.ini`, `parameter.bincfg` and `loader_compress.bin` at build time.

## Normal OpenBeken build

From the OpenBeken repository root:

```bash
make APP_VERSION=dev APP_NAME=OpenTXW81X VARIANT=default OpenTXW81X
```

The target compiles with the Linux C-SKY toolchain and then calls:

```bash
sh platforms/TXW81X/package_native.sh
```

The wrapper derives SDK, SVN, project-type and application-version metadata from the checked-out OpenTXW81X headers. Its build date is UTC today, or `SOURCE_DATE_EPOCH` when that reproducibility variable is set.

For controlled comparisons, these optional environment variables override the derived fields:

- `TXW81X_SDK_VERSION`
- `TXW81X_SVN_VERSION`
- `TXW81X_IMAGE_VERSION`
- `TXW81X_BUILD_DATE` (`YYYY-MM-DD`)
- `PYTHON` (defaults to `python3`)

## Direct commands

### Build `APP.bin`

```bash
python3 tools/txw81x_pack/txw81x_pack.py pack-app \
  --input sdk/OpenTXW81X/project/project.hex \
  --parameter-cfg sdk/OpenTXW81X/project/parameter.bincfg \
  --makecode-ini sdk/OpenTXW81X/project/makecode.ini \
  --sdk-version 02050307 \
  --svn-version 8EB5 \
  --app-version 0 \
  --output sdk/OpenTXW81X/project/APP.bin
```

The input may also be a raw binary. Direct packer use requires explicit `--sdk-version`, `--svn-version` and `--app-version` values; `--date` and `--template-app` are available for controlled forensic comparisons. Normal builds should use `package_native.sh`, which derives these fields from the SDK checkout.

### Build `APP_compress.bin`

```bash
python3 tools/txw81x_pack/txw81x_pack.py pack-ota \
  --app sdk/OpenTXW81X/project/APP.bin \
  --loader sdk/OpenTXW81X/sdk/chip/txw81x/loader_compress.bin \
  --output sdk/OpenTXW81X/project/APP_compress.bin
```

### Inspect either format

```bash
python3 tools/txw81x_pack/txw81x_pack.py inspect APP.bin
python3 tools/txw81x_pack/txw81x_pack.py inspect APP_compress.bin
```

Output is JSON containing parsed headers, addresses, lengths, version fields and checksum results.

## Tests

Always-available tests:

```bash
python3 -m unittest discover -s tools/txw81x_pack/tests -v
```

Known-good integration tests require two public OpenBeken Actions outputs and the loader already present in the OpenTXW81X checkout:

```bash
TXW81X_REFERENCE_DIR=/path/to/reference/files \
TXW81X_LOADER=sdk/OpenTXW81X/sdk/chip/txw81x/loader_compress.bin \
python3 -m unittest discover -s tools/txw81x_pack/tests -v
```

The integration suite checks hashes, parsed fields, all CRCs, deterministic output, and byte-identical APP and OTA reconstruction.

## Validation status

Software validation completed:

- the recorded July 12 APP and OTA references rebuild byte-for-byte;
- the latest July 22 WSL/Taixin outputs rebuild byte-for-byte when the exact same raw application and metadata are supplied;
- the production wrapper reproduces those latest outputs byte-for-byte;
- the complete compile-and-package path runs natively on Ubuntu 22.04;
- Python's XZ stream matches Taixin's legacy `xz.exe -0` using preset 0 and a 32 KiB dictionary.

Still required:

- wired flashing and cold-boot testing of a Python-generated APP on recoverable TXW81x hardware;
- real OTA testing with serial logging and a recovery path;
- validation of encrypted/AES and non-SPI image modes, which are intentionally unsupported here.

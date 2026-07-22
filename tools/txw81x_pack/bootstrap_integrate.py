#!/usr/bin/env python3
"""Apply the reviewed TXW81x native-packaging integration to this branch."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

OLD_MAKE = """.PHONY: OpenTXW81X
OpenTXW81X: prebuild_OpenTXW81X
	cd sdk/OpenTXW81X/project && make APP_VERSION=$(APP_VERSION) OBK_VARIANT=$(OBK_VARIANT) -j $(shell nproc) && \
	./BinScript.exe BinScript.BinScript > /dev/null && ./makecode.exe > /dev/null
	mkdir -p output/$(APP_VERSION)
	cp sdk/OpenTXW81X/project/APP.bin output/$(APP_VERSION)/OpenTXW81X_$(APP_VERSION).bin
	cp sdk/OpenTXW81X/project/APP_compress.bin output/$(APP_VERSION)/OpenTXW81X_$(APP_VERSION)_ota.img
"""

NEW_MAKE = """.PHONY: OpenTXW81X
OpenTXW81X: prebuild_OpenTXW81X
	$(MAKE) -C sdk/OpenTXW81X/project APP_VERSION=$(APP_VERSION) OBK_VARIANT=$(OBK_VARIANT) -j $(shell nproc)
	sh platforms/TXW81X/package_native.sh
	mkdir -p output/$(APP_VERSION)
	cp sdk/OpenTXW81X/project/APP.bin output/$(APP_VERSION)/OpenTXW81X_$(APP_VERSION).bin
	cp sdk/OpenTXW81X/project/APP_compress.bin output/$(APP_VERSION)/OpenTXW81X_$(APP_VERSION)_ota.img
"""

OLD_WORKFLOW = """  build_txw81x:
    name: Build TXW81X
    needs: refs
    runs-on: windows-latest
    defaults:
      run:
        shell: wsl-bash {0}
    strategy:
      fail-fast: false
      matrix:
        include:
          - platform: OpenTXW81X
            variant: default
    steps:
      - name: Source checkout
        uses: actions/checkout@v4
      - uses: Vampire/setup-wsl@v6
        with:
          wsl-version: 1
          distribution: Ubuntu-22.04
          additional-packages: make
      - name: Run make
        run: |
          make APP_VERSION=${{ needs.refs.outputs.version }}${{ matrix.variant != 'default' && '_' || '' }}${{ matrix.variant != 'default' && matrix.variant || '' }} APP_NAME=${{ matrix.platform }} VARIANT=\"${{ matrix.variant }}\" ${{ matrix.platform }}
      - name: Save build assets
"""

NEW_WORKFLOW = """  build_txw81x:
    name: Build TXW81X
    needs: refs
    runs-on: ubuntu-22.04
    strategy:
      fail-fast: false
      matrix:
        include:
          - platform: OpenTXW81X
            variant: default
    steps:
      - name: Source checkout
        uses: actions/checkout@v4
      - name: Test native TXW81x packer
        run: python3 -m unittest discover -s tools/txw81x_pack/tests -v
      - name: Run make
        run: |
          make APP_VERSION=${{ needs.refs.outputs.version }}${{ matrix.variant != 'default' && '_' || '' }}${{ matrix.variant != 'default' && matrix.variant || '' }} APP_NAME=${{ matrix.platform }} VARIANT=\"${{ matrix.variant }}\" ${{ matrix.platform }}
      - name: Save build assets
"""

WRAPPER = '#!/bin/sh\n# Native TXW81x APP.bin and APP_compress.bin packaging for OpenBeken.\nset -eu\n\nROOT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)\nSDK_ROOT="$ROOT_DIR/sdk/OpenTXW81X"\nPROJECT_DIR="$SDK_ROOT/project"\nPACKER="$ROOT_DIR/tools/txw81x_pack/txw81x_pack.py"\nLOADER="$SDK_ROOT/sdk/chip/txw81x/loader_compress.bin"\nPYTHON=${PYTHON:-python3}\n\nfor required in \\\n    "$PROJECT_DIR/project.hex" \\\n    "$PROJECT_DIR/parameter.bincfg" \\\n    "$PROJECT_DIR/makecode.ini" \\\n    "$SDK_ROOT/sdk/include/version.h" \\\n    "$SDK_ROOT/sdk/include/svn_version.h" \\\n    "$SDK_ROOT/sdk/include/app_version.h" \\\n    "$SDK_ROOT/project/sys_config.h" \\\n    "$SDK_ROOT/sdk/include/chip/tx_platform.h" \\\n    "$LOADER" \\\n    "$PACKER"; do\n    if [ ! -f "$required" ]; then\n        echo "TXW81x packaging input not found: $required" >&2\n        exit 1\n    fi\ndone\n\n# Derive the same metadata fields used by makecode.exe from the checked-out SDK.\neval "$("$PYTHON" - "$SDK_ROOT" <<\'PY\'\nimport re\nimport shlex\nimport sys\nfrom pathlib import Path\n\nroot = Path(sys.argv[1])\n\ndef define(path: Path, name: str) -> str:\n    pattern = re.compile(rf\'^\\s*#\\s*define\\s+{re.escape(name)}\\s+(.+?)\\s*$\')\n    for line in path.read_text(encoding=\'utf-8\', errors=\'replace\').splitlines():\n        match = pattern.match(line)\n        if match:\n            return re.split(r\'/\\*|//\', match.group(1), maxsplit=1)[0].strip().split()[0]\n    raise SystemExit(f\'missing #define {name} in {path}\')\n\ndef number(text: str) -> int:\n    text = text.strip().strip(\'()\')\n    text = re.sub(r\'[uUlL]+$\', \'\', text)\n    return int(text, 0)\n\nversion_h = root / \'sdk/include/version.h\'\nsvn_h = root / \'sdk/include/svn_version.h\'\napp_h = root / \'sdk/include/app_version.h\'\nsys_config_h = root / \'project/sys_config.h\'\nplatform_h = root / \'sdk/include/chip/tx_platform.h\'\n\nmajor = number(define(version_h, \'SDK_MVER\'))\nbranch = number(define(version_h, \'SDK_BVER\'))\npatch = number(define(version_h, \'SDK_PVER\'))\nproject_name = define(sys_config_h, \'PROJECT_TYPE\')\nproject_type = number(define(platform_h, project_name))\nsvn = number(define(svn_h, \'SVN_VERSION\'))\napp = number(define(app_h, \'APP_VERSION\'))\n\nvalues = {\n    \'TXW81X_DERIVED_SDK_VERSION\': f\'{major:02X}{branch:02X}{patch:02X}{project_type:02X}\',\n    \'TXW81X_DERIVED_SVN_VERSION\': f\'{svn:X}\',\n    \'TXW81X_DERIVED_IMAGE_VERSION\': f\'{app:X}\',\n}\nfor key, value in values.items():\n    print(f\'{key}={shlex.quote(value)}\')\nPY\n)"\n\n: "${TXW81X_SDK_VERSION:=$TXW81X_DERIVED_SDK_VERSION}"\n: "${TXW81X_SVN_VERSION:=$TXW81X_DERIVED_SVN_VERSION}"\n: "${TXW81X_IMAGE_VERSION:=$TXW81X_DERIVED_IMAGE_VERSION}"\n\nset -- "$PYTHON" "$PACKER" pack-app \\\n    --input "$PROJECT_DIR/project.hex" \\\n    --parameter-cfg "$PROJECT_DIR/parameter.bincfg" \\\n    --makecode-ini "$PROJECT_DIR/makecode.ini" \\\n    --sdk-version "$TXW81X_SDK_VERSION" \\\n    --svn-version "$TXW81X_SVN_VERSION" \\\n    --app-version "$TXW81X_IMAGE_VERSION" \\\n    --output "$PROJECT_DIR/APP.bin"\n\nif [ "${TXW81X_BUILD_DATE+x}" = x ]; then\n    set -- "$@" --date "$TXW81X_BUILD_DATE"\nfi\n\nprintf \'%s\\n\' \'Packaging TXW81x APP.bin natively with Python\'\n"$@"\n\nprintf \'%s\\n\' \'Packaging TXW81x APP_compress.bin natively with Python\'\n"$PYTHON" "$PACKER" pack-ota \\\n    --app "$PROJECT_DIR/APP.bin" \\\n    --loader "$LOADER" \\\n    --output "$PROJECT_DIR/APP_compress.bin"\n'
README = "# Native TXW81x firmware packer\n\nThis directory contains a standard-library-only Python replacement for the TXW81x post-build operations previously performed by Taixin's Windows tools:\n\n- `BinScript.exe` for Intel HEX remapping and parameter extraction;\n- `makecode.exe` for the flashable SPI application image (`APP.bin`);\n- `xz.exe` plus `compress.exe` for the compressed OTA image (`APP_compress.bin`).\n\nThe OpenBeken TXW81x target invokes this packer through `platforms/TXW81X/package_native.sh`. The implementation removes Wine, Windows and WSL from the TXW81x build path, but generated images still require real-hardware flash and OTA validation before the change is considered production-proven.\n\n## Files\n\n- `txw81x_pack.py` — pack, inspect and validate APP/OTA images.\n- `tests/test_txw81x_pack.py` — synthetic tests plus opt-in known-good Taixin integration tests.\n- `tests/reference_manifest.json` — hashes and parsed values for public known-good OpenBeken Actions outputs.\n- `ATTRIBUTION.md` — retained MIT attribution for RNode-derived portions.\n\nNo Taixin executable, SDK source, loader or reference firmware image is included. The existing OpenTXW81X checkout supplies `makecode.ini`, `parameter.bincfg` and `loader_compress.bin` at build time.\n\n## Normal OpenBeken build\n\nFrom the OpenBeken repository root:\n\n```bash\nmake APP_VERSION=dev APP_NAME=OpenTXW81X VARIANT=default OpenTXW81X\n```\n\nThe target compiles with the Linux C-SKY toolchain and then calls:\n\n```bash\nsh platforms/TXW81X/package_native.sh\n```\n\nThe wrapper derives SDK, SVN, project-type and application-version metadata from the checked-out OpenTXW81X headers. Its build date is UTC today, or `SOURCE_DATE_EPOCH` when that reproducibility variable is set.\n\nFor controlled comparisons, these optional environment variables override the derived fields:\n\n- `TXW81X_SDK_VERSION`\n- `TXW81X_SVN_VERSION`\n- `TXW81X_IMAGE_VERSION`\n- `TXW81X_BUILD_DATE` (`YYYY-MM-DD`)\n- `PYTHON` (defaults to `python3`)\n\n## Direct commands\n\n### Build `APP.bin`\n\n```bash\npython3 tools/txw81x_pack/txw81x_pack.py pack-app \\\n  --input sdk/OpenTXW81X/project/project.hex \\\n  --parameter-cfg sdk/OpenTXW81X/project/parameter.bincfg \\\n  --makecode-ini sdk/OpenTXW81X/project/makecode.ini \\\n  --sdk-version 02050307 \\\n  --svn-version 8EB5 \\\n  --app-version 0 \\\n  --output sdk/OpenTXW81X/project/APP.bin\n```\n\nThe input may also be a raw binary. Direct packer use requires explicit `--sdk-version`, `--svn-version` and `--app-version` values; `--date` and `--template-app` are available for controlled forensic comparisons. Normal builds should use `package_native.sh`, which derives these fields from the SDK checkout.\n\n### Build `APP_compress.bin`\n\n```bash\npython3 tools/txw81x_pack/txw81x_pack.py pack-ota \\\n  --app sdk/OpenTXW81X/project/APP.bin \\\n  --loader sdk/OpenTXW81X/sdk/chip/txw81x/loader_compress.bin \\\n  --output sdk/OpenTXW81X/project/APP_compress.bin\n```\n\n### Inspect either format\n\n```bash\npython3 tools/txw81x_pack/txw81x_pack.py inspect APP.bin\npython3 tools/txw81x_pack/txw81x_pack.py inspect APP_compress.bin\n```\n\nOutput is JSON containing parsed headers, addresses, lengths, version fields and checksum results.\n\n## Tests\n\nAlways-available tests:\n\n```bash\npython3 -m unittest discover -s tools/txw81x_pack/tests -v\n```\n\nKnown-good integration tests require two public OpenBeken Actions outputs and the loader already present in the OpenTXW81X checkout:\n\n```bash\nTXW81X_REFERENCE_DIR=/path/to/reference/files \\\nTXW81X_LOADER=sdk/OpenTXW81X/sdk/chip/txw81x/loader_compress.bin \\\npython3 -m unittest discover -s tools/txw81x_pack/tests -v\n```\n\nThe integration suite checks hashes, parsed fields, all CRCs, deterministic output, and byte-identical APP and OTA reconstruction.\n\n## Validation status\n\nSoftware validation completed:\n\n- the recorded July 12 APP and OTA references rebuild byte-for-byte;\n- the latest July 22 WSL/Taixin outputs rebuild byte-for-byte when the exact same raw application and metadata are supplied;\n- the production wrapper reproduces those latest outputs byte-for-byte;\n- the complete compile-and-package path runs natively on Ubuntu 22.04;\n- Python's XZ stream matches Taixin's legacy `xz.exe -0` using preset 0 and a 32 KiB dictionary.\n\nStill required:\n\n- wired flashing and cold-boot testing of a Python-generated APP on recoverable TXW81x hardware;\n- real OTA testing with serial logging and a recovery path;\n- validation of encrypted/AES and non-SPI image modes, which are intentionally unsupported here.\n"

REPORT_REPLACEMENTS = (
    (
        "This work is isolated on `txw81x-python-packaging-research`. It does not alter the production TXW81x target or the production GitHub Actions workflow. The implementation is a research replacement for the post-link packaging stage only. Successful compilation and byte-identical software reconstruction do **not** prove that a generated image is bootable or OTA-safe on TXW81x hardware.",
        "The implementation was developed in isolation and is now wired into the TXW81x target on the integration branch. The production change replaces only the post-link packaging stage and moves the TXW81x CI job from Windows/WSL to Ubuntu. Successful compilation and byte-identical software reconstruction do **not** prove that a generated image is bootable or OTA-safe on TXW81x hardware.",
    ),
    (
        "A branch-only Ubuntu research workflow is included solely to repeat these tests on `ubuntu-22.04`; it does not alter the production workflow or release target.",
        "The complete compiler and packaging path has also run on `ubuntu-22.04`. A later same-input comparison used the raw code and metadata from the July 22 WSL/Taixin build: both the wrapper-generated `APP.bin` and `APP_compress.bin` were byte-identical to that build. This avoids the invalid comparison of separately compiled applications containing different `__DATE__`/`__TIME__` strings.",
    ),
    (
        "- Version metadata is explicit. Production integration must derive SDK, SVN, application version and build date reproducibly from the build, rather than hard-code reference values.",
        "- Version metadata is derived by `package_native.sh` from the checked-out OpenTXW81X headers. The build date follows UTC or `SOURCE_DATE_EPOCH`; explicit environment overrides remain available for forensic reproducibility.",
    ),
    (
        "8. Only after those tests, replace the production Makefile commands and move the TXW81x CI job from Windows/WSL to Ubuntu.",
        "8. Keep the change in draft/review status until those hardware tests pass; software equivalence alone is not a bootability or OTA-safety guarantee.",
    ),
)


def replace_exact(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"expected exactly one integration block in {path}, found {count}")
    path.write_text(text.replace(old, new), encoding="utf-8")


def main() -> None:
    replace_exact(ROOT / "Makefile", OLD_MAKE, NEW_MAKE)
    replace_exact(ROOT / ".github/workflows/workflow.yaml", OLD_WORKFLOW, NEW_WORKFLOW)

    wrapper_path = ROOT / "platforms/TXW81X/package_native.sh"
    wrapper_path.write_text(WRAPPER, encoding="utf-8")
    (ROOT / "tools/txw81x_pack/README.md").write_text(README, encoding="utf-8")

    report_path = ROOT / "docs/TXW81X_NATIVE_PACKAGING_RESEARCH.md"
    report = report_path.read_text(encoding="utf-8")
    for old, new in REPORT_REPLACEMENTS:
        count = report.count(old)
        if count != 1:
            raise SystemExit(f"expected one report text match, found {count}: {old[:60]}")
        report = report.replace(old, new)
    report_path.write_text(report, encoding="utf-8")

    makefile = (ROOT / "Makefile").read_text(encoding="utf-8")
    target_start = makefile.index(".PHONY: OpenTXW81X")
    target_end = makefile.index("\n.PHONY: OpenRDA5981", target_start)
    target = makefile[target_start:target_end]
    forbidden_make = ("BinScript.exe", "makecode.exe", "compress.exe")
    found_make = [item for item in forbidden_make if item in target]
    if found_make:
        raise SystemExit(f"proprietary tools remain in TXW81x target: {found_make}")

    workflow = (ROOT / ".github/workflows/workflow.yaml").read_text(encoding="utf-8")
    start = workflow.index("  build_txw81x:")
    end = workflow.index("\n  merge:", start)
    block = workflow[start:end]
    forbidden_ci = ("windows-latest", "wsl-bash", "Vampire/setup-wsl")
    found_ci = [item for item in forbidden_ci if item in block]
    if found_ci:
        raise SystemExit(f"WSL/Windows references remain in TXW81x job: {found_ci}")

    print("Applied native TXW81x Makefile and GitHub Actions integration")


if __name__ == "__main__":
    main()

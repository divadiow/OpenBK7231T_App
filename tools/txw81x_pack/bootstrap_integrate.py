#!/usr/bin/env python3
"""Apply the reviewed TXW81x native-packaging integration to this branch."""
from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

OLD_MAKE = """.PHONY: OpenTXW81X
OpenTXW81X: prebuild_OpenTXW81X
\tcd sdk/OpenTXW81X/project && make APP_VERSION=$(APP_VERSION) OBK_VARIANT=$(OBK_VARIANT) -j $(shell nproc) && \\
\t./BinScript.exe BinScript.BinScript > /dev/null && ./makecode.exe > /dev/null
\tmkdir -p output/$(APP_VERSION)
\tcp sdk/OpenTXW81X/project/APP.bin output/$(APP_VERSION)/OpenTXW81X_$(APP_VERSION).bin
\tcp sdk/OpenTXW81X/project/APP_compress.bin output/$(APP_VERSION)/OpenTXW81X_$(APP_VERSION)_ota.img
"""

NEW_MAKE = """.PHONY: OpenTXW81X
OpenTXW81X: prebuild_OpenTXW81X
\t$(MAKE) -C sdk/OpenTXW81X/project APP_VERSION=$(APP_VERSION) OBK_VARIANT=$(OBK_VARIANT) -j $(shell nproc)
\tsh platforms/TXW81X/package_native.sh
\tmkdir -p output/$(APP_VERSION)
\tcp sdk/OpenTXW81X/project/APP.bin output/$(APP_VERSION)/OpenTXW81X_$(APP_VERSION).bin
\tcp sdk/OpenTXW81X/project/APP_compress.bin output/$(APP_VERSION)/OpenTXW81X_$(APP_VERSION)_ota.img
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
      - name: Set up Python
        uses: actions/setup-python@v5
        with:
          python-version: '3.11'
      - name: Test native TXW81x packer
        run: python3 -m unittest discover -s tools/txw81x_pack/tests -v
      - name: Run make
        run: |
          make APP_VERSION=${{ needs.refs.outputs.version }}${{ matrix.variant != 'default' && '_' || '' }}${{ matrix.variant != 'default' && matrix.variant || '' }} APP_NAME=${{ matrix.platform }} VARIANT=\"${{ matrix.variant }}\" ${{ matrix.platform }}
      - name: Save build assets
"""


def replace_exact(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"expected exactly one integration block in {path}, found {count}")
    path.write_text(text.replace(old, new), encoding="utf-8")


def main() -> None:
    replace_exact(ROOT / "Makefile", OLD_MAKE, NEW_MAKE)
    replace_exact(ROOT / ".github/workflows/workflow.yaml", OLD_WORKFLOW, NEW_WORKFLOW)

    workflow = (ROOT / ".github/workflows/workflow.yaml").read_text(encoding="utf-8")
    start = workflow.index("  build_txw81x:")
    end = workflow.index("\n  merge:", start)
    block = workflow[start:end]
    forbidden = ("windows-latest", "wsl-bash", "Vampire/setup-wsl")
    found = [item for item in forbidden if item in block]
    if found:
        raise SystemExit(f"WSL/Windows references remain in TXW81x job: {found}")

    print("Applied native TXW81x Makefile and GitHub Actions integration")


if __name__ == "__main__":
    main()

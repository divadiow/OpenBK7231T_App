#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

WARNING_RE = re.compile(r"\bwarning:", re.IGNORECASE)
ERROR_RE = re.compile(
    r"\berror:|undefined reference|region [`'\"]?.+[`'\"]? overflowed|collect2: error|make(?:\[\d+\])?: \*\*\*",
    re.IGNORECASE,
)

FINGERPRINTS = {
    "cjson_unicode_escape": (
        "cJSON.c",
        "snprintf",
        "directive output truncated",
    ),
    "nonvoid_return": (
        "return' with no value",
        "return’ with no value",
        "return with no value",
        "control reaches end of non-void function",
    ),
    "implicit_allocator_declaration": (
        "implicit declaration of function ‘os_realloc’",
        "implicit declaration of function 'os_realloc'",
        "implicit declaration of function ‘pvPortMalloc’",
        "implicit declaration of function 'pvPortMalloc'",
        "implicit declaration of function ‘pvPortRealloc’",
        "implicit declaration of function 'pvPortRealloc'",
        "implicit declaration of function ‘os_malloc’",
        "implicit declaration of function 'os_malloc'",
        "implicit declaration of function ‘os_free’",
        "implicit declaration of function 'os_free'",
    ),
    "maybe_uninitialized": (
        "may be used uninitialized",
        "is used uninitialized",
    ),
    "shift_count_width": (
        "shift count >= width of type",
        "left shift count >= width of type",
    ),
    "linker_symbol_size_change": (
        "size of symbol",
        "changed from",
    ),
    "format_truncation": (
        "output may be truncated",
        "directive output may be truncated",
    ),
    "incompatible_pointer": (
        "incompatible pointer type",
        "incompatible-pointer-types",
    ),
}

FIRMWARE_SUFFIXES = {
    ".bin", ".img", ".rbl", ".fls", ".ota", ".xz", ".uf2", ".hex",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--artifacts-root", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--source-sha", required=True)
    parser.add_argument("--set-name", required=True)
    return parser.parse_args()


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def collect_target(manifest_path: Path) -> dict:
    root = manifest_path.parent
    manifest = json.loads(read_text(manifest_path))
    build_log = root / "metadata" / "build.log"
    if not build_log.exists():
        build_log = root / "files" / ".obk-ci" / "build.log"
    log_text = read_text(build_log) if build_log.exists() else ""
    lines = log_text.splitlines()

    warning_lines = [line.strip() for line in lines if WARNING_RE.search(line)]
    error_lines = [line.strip() for line in lines if ERROR_RE.search(line)]

    fingerprints = {}
    lowered = log_text.lower()
    for name, patterns in FINGERPRINTS.items():
        if name == "cjson_unicode_escape":
            matched = (
                "cjson.c" in lowered
                and "snprintf" in lowered
                and (
                    "directive output truncated" in lowered
                    or "output may be truncated" in lowered
                )
            )
            fingerprints[name] = int(matched)
        elif name == "linker_symbol_size_change":
            fingerprints[name] = sum(
                1
                for line in lines
                if "size of symbol" in line.lower() and "changed from" in line.lower()
            )
        else:
            fingerprints[name] = sum(lowered.count(pattern.lower()) for pattern in patterns)

    outputs = []
    for item in manifest.get("files", []):
        path = item.get("path", "")
        suffix = Path(path).suffix.lower()
        if suffix in FIRMWARE_SUFFIXES and path.startswith("output/"):
            outputs.append(
                {
                    "path": path,
                    "bytes": int(item.get("bytes", 0)),
                    "sha256": item.get("sha256"),
                }
            )
    outputs.sort(key=lambda item: item["bytes"], reverse=True)

    success_markers = (
        "build complete" in lowered
        or "project build complete" in lowered
        or "build finished" in lowered
        or "application binary" in lowered
        or bool(outputs)
    )

    return {
        "platform": manifest.get("platform", "unknown"),
        "variant": manifest.get("variant", "unknown"),
        "artifact_directory": root.parent.name,
        "collected_files": int(manifest.get("collected_file_count", 0)),
        "collected_bytes": int(manifest.get("collected_bytes", 0)),
        "build_log_present": build_log.exists(),
        "success_or_output_present": bool(success_markers),
        "warning_count": len(warning_lines),
        "error_count": len(error_lines),
        "warnings": warning_lines,
        "errors": error_lines,
        "fingerprints": fingerprints,
        "outputs": outputs,
    }


def main() -> int:
    args = parse_args()
    artifacts_root = Path(args.artifacts_root).resolve()
    output_root = Path(args.output_dir).resolve() / args.source_sha
    output_root.mkdir(parents=True, exist_ok=True)

    manifests = sorted(artifacts_root.rglob("manifest.json"))
    targets = [collect_target(path) for path in manifests]
    targets.sort(key=lambda item: (item["platform"], item["variant"]))

    aggregate_fingerprints = {
        name: sum(target["fingerprints"].get(name, 0) for target in targets)
        for name in FINGERPRINTS
    }
    result = {
        "source_sha": args.source_sha,
        "set_name": args.set_name,
        "target_count": len(targets),
        "successful_or_output_target_count": sum(
            1 for target in targets if target["success_or_output_present"]
        ),
        "warning_count": sum(target["warning_count"] for target in targets),
        "error_count": sum(target["error_count"] for target in targets),
        "fingerprints": aggregate_fingerprints,
        "targets": targets,
    }

    (output_root / "summary.json").write_text(
        json.dumps(result, indent=2), encoding="utf-8"
    )

    md = [
        f"# Remediation diagnostics: `{args.source_sha}`",
        "",
        f"- Target set: **{args.set_name}**",
        f"- Targets found: **{result['target_count']}**",
        f"- Targets with build output: **{result['successful_or_output_target_count']}**",
        f"- Warning lines: **{result['warning_count']}**",
        f"- Error lines: **{result['error_count']}**",
        "",
        "## Known-warning fingerprints",
        "",
        "| Fingerprint | Count |",
        "| --- | ---: |",
    ]
    for name, count in aggregate_fingerprints.items():
        md.append(f"| `{name}` | {count} |")

    md.extend(
        [
            "",
            "## Targets",
            "",
            "| Platform | Variant | Output | Warnings | Errors | Largest firmware output |",
            "| --- | --- | ---: | ---: | ---: | --- |",
        ]
    )
    for target in targets:
        largest = target["outputs"][0] if target["outputs"] else None
        largest_text = (
            f"`{largest['path']}` ({largest['bytes']:,} bytes)" if largest else "none"
        )
        md.append(
            f"| {target['platform']} | {target['variant']} | "
            f"{'yes' if target['success_or_output_present'] else 'no'} | "
            f"{target['warning_count']} | {target['error_count']} | {largest_text} |"
        )

    (output_root / "summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")

    with (output_root / "warnings.txt").open("w", encoding="utf-8") as handle:
        for target in targets:
            if not target["warnings"]:
                continue
            handle.write(f"## {target['platform']} / {target['variant']}\n")
            for line in target["warnings"]:
                handle.write(line + "\n")
            handle.write("\n")

    with (output_root / "errors.txt").open("w", encoding="utf-8") as handle:
        for target in targets:
            if not target["errors"]:
                continue
            handle.write(f"## {target['platform']} / {target['variant']}\n")
            for line in target["errors"]:
                handle.write(line + "\n")
            handle.write("\n")

    print(json.dumps(result, indent=2))
    return 0 if result["error_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())

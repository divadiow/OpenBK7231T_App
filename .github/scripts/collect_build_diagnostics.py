#!/usr/bin/env python3
"""Collect linker outputs and generate compact binary-size diagnostics."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import platform as host_platform
import shutil
import subprocess
import sys
from typing import Iterable, Sequence

DIAGNOSTIC_SUFFIXES = {
    ".map", ".elf", ".axf", ".out", ".lst", ".lss", ".sym", ".nm",
    ".size", ".siz", ".mem", ".dis", ".dump", ".dmp", ".log",
}
FIRMWARE_SUFFIXES = {
    ".bin", ".img", ".rbl", ".fls", ".ota", ".xz", ".uf2", ".hex",
}
SPECIAL_NAMES = {
    "compile_commands.json", "project_description.json", "flasher_args.json",
    "CMakeCache.txt", "link.txt", "memory.ld", "sections.ld", "sdkconfig",
    "sdkconfig.old", "partitions.csv", "partition-table.csv",
}
BUILDISH_PARTS = {
    "build", "out", "output", "image", "images", "release", "debug",
    "obj", "objects", "gcc", "bin", "dist",
}
SKIP_PARTS = {".git", ".cache", "node_modules", "__pycache__"}
EXECUTABLE_SUFFIXES = {".elf", ".axf", ".out"}
MAX_FILE_BYTES = 256 * 1024 * 1024


def run_capture(command: Sequence[str], cwd: Path, timeout: int = 60) -> tuple[int, str]:
    try:
        completed = subprocess.run(
            list(command),
            cwd=str(cwd),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            errors="replace",
            timeout=timeout,
            check=False,
        )
        return completed.returncode, completed.stdout
    except (OSError, subprocess.SubprocessError) as exc:
        return 127, f"{type(exc).__name__}: {exc}\n"


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def is_buildish(path: Path) -> bool:
    return any(part.lower() in BUILDISH_PARTS for part in path.parts)


def should_collect(relative: Path) -> bool:
    if any(part in SKIP_PARTS for part in relative.parts):
        return False

    name_lower = relative.name.lower()
    suffix_lower = relative.suffix.lower()

    if suffix_lower in DIAGNOSTIC_SUFFIXES:
        return True
    if relative.name in SPECIAL_NAMES:
        return is_buildish(relative) or len(relative.parts) <= 3
    if name_lower.endswith((".factory.bin", ".ota.bin", ".bin.xz", ".bin.xz.ota")):
        return is_buildish(relative)
    if suffix_lower in FIRMWARE_SUFFIXES:
        return is_buildish(relative)
    if suffix_lower == ".ld":
        return is_buildish(relative)
    return False


def iter_candidates(root: Path, destination: Path) -> Iterable[Path]:
    destination_resolved = destination.resolve()
    for current_root, directory_names, file_names in os.walk(root):
        current = Path(current_root)
        directory_names[:] = [
            name for name in directory_names
            if name not in SKIP_PARTS
            and (current / name).resolve() != destination_resolved
        ]
        for file_name in file_names:
            source = current / file_name
            try:
                if not source.is_file() or source.is_symlink():
                    continue
                relative = source.relative_to(root)
                if should_collect(relative):
                    yield source
            except (OSError, ValueError):
                continue


def unique_tools(names: Sequence[str]) -> list[str]:
    result: list[str] = []
    seen: set[str] = set()
    for name in names:
        resolved = shutil.which(name)
        if resolved and resolved not in seen:
            seen.add(resolved)
            result.append(resolved)
    return result


def tool_candidates(kind: str) -> list[str]:
    prefixes = [
        "arm-none-eabi-", "riscv64-unknown-elf-", "riscv32-unknown-elf-",
        "riscv32-esp-elf-", "xtensa-esp32-elf-", "xtensa-esp32s2-elf-",
        "xtensa-esp32s3-elf-", "xtensa-lx106-elf-", "csky-abiv2-elf-",
    ]
    return unique_tools([f"{prefix}{kind}" for prefix in prefixes] + [kind])


def first_success(commands: Iterable[Sequence[str]], cwd: Path, timeout: int = 60) -> tuple[str, str]:
    attempts: list[str] = []
    for command in commands:
        code, output = run_capture(command, cwd, timeout)
        attempts.append(f"$ {' '.join(command)}\n{output}")
        if code == 0:
            return "\n".join(attempts), command[0]
    return "\n".join(attempts), ""


def safe_report_name(relative: Path) -> str:
    text = "__".join(relative.parts)
    return "".join(character if character.isalnum() or character in "._-" else "_" for character in text)


def analyse_executable(source: Path, relative: Path, reports_dir: Path, root: Path) -> list[str]:
    base_name = safe_report_name(relative)
    generated: list[str] = []

    file_tool = shutil.which("file")
    if file_tool:
        _, output = run_capture([file_tool, str(source)], root)
        report = reports_dir / f"{base_name}.file.txt"
        report.write_text(output, encoding="utf-8", errors="replace")
        generated.append(str(report.relative_to(reports_dir.parent)))

    size_tools = tool_candidates("size")
    if size_tools:
        commands = []
        for tool in size_tools:
            commands.extend(([tool, "-A", str(source)], [tool, "-B", str(source)]))
        output, _ = first_success(commands, root)
        report = reports_dir / f"{base_name}.size.txt"
        report.write_text(output, encoding="utf-8", errors="replace")
        generated.append(str(report.relative_to(reports_dir.parent)))

    nm_tools = tool_candidates("nm")
    if nm_tools:
        output, _ = first_success(
            ([tool, "-S", "--size-sort", "--print-size", str(source)] for tool in nm_tools),
            root,
            timeout=120,
        )
        report = reports_dir / f"{base_name}.nm-size-sort.txt"
        report.write_text(output, encoding="utf-8", errors="replace")
        generated.append(str(report.relative_to(reports_dir.parent)))

    readelf_tools = tool_candidates("readelf")
    if readelf_tools:
        output, _ = first_success(
            ([tool, "-h", "-S", "-l", str(source)] for tool in readelf_tools),
            root,
        )
        report = reports_dir / f"{base_name}.readelf.txt"
        report.write_text(output, encoding="utf-8", errors="replace")
        generated.append(str(report.relative_to(reports_dir.parent)))

    objdump_tools = tool_candidates("objdump")
    if objdump_tools:
        output, _ = first_success(
            ([tool, "-h", "-t", str(source)] for tool in objdump_tools),
            root,
            timeout=120,
        )
        report = reports_dir / f"{base_name}.objdump-sections-symbols.txt"
        report.write_text(output, encoding="utf-8", errors="replace")
        generated.append(str(report.relative_to(reports_dir.parent)))

    return generated


def write_command_report(root: Path, destination: Path, build_log: Path | None) -> None:
    metadata_dir = destination / "metadata"
    metadata_dir.mkdir(parents=True, exist_ok=True)

    commands = {
        "git-status.txt": ["git", "status", "--short", "--branch"],
        "git-head.txt": ["git", "show", "-s", "--format=fuller", "HEAD"],
        "submodules.txt": ["git", "submodule", "status", "--recursive"],
        "disk-usage.txt": ["du", "-ah", "output", "build", "sdk"],
    }
    for file_name, command in commands.items():
        _, output = run_capture(command, root, timeout=120)
        (metadata_dir / file_name).write_text(output, encoding="utf-8", errors="replace")

    tool_names = [
        "python3", "cmake", "make", "gcc", "clang", "arm-none-eabi-gcc",
        "riscv32-esp-elf-gcc", "xtensa-esp32-elf-gcc", "xtensa-lx106-elf-gcc",
    ]
    versions: list[str] = []
    for tool_name in tool_names:
        resolved = shutil.which(tool_name)
        if not resolved:
            continue
        _, output = run_capture([resolved, "--version"], root)
        versions.append(f"## {tool_name}\npath: {resolved}\n{output}\n")
    (metadata_dir / "tool-versions.txt").write_text("\n".join(versions), encoding="utf-8")

    environment = {
        key: value
        for key, value in sorted(os.environ.items())
        if key.startswith(("GITHUB_", "RUNNER_", "ARM_", "IDF_", "PATH"))
        and "TOKEN" not in key and "SECRET" not in key
    }
    environment["python"] = sys.version
    environment["host_platform"] = host_platform.platform()
    (metadata_dir / "environment.json").write_text(
        json.dumps(environment, indent=2, sort_keys=True), encoding="utf-8"
    )

    if build_log and build_log.is_file():
        shutil.copy2(build_log, metadata_dir / "build.log")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--platform", required=True)
    parser.add_argument("--variant", required=True)
    parser.add_argument("--build-log", type=Path)
    args = parser.parse_args()

    root = args.root.resolve()
    destination = args.output.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    files_dir = destination / "files"
    reports_dir = destination / "reports"
    files_dir.mkdir(parents=True, exist_ok=True)
    reports_dir.mkdir(parents=True, exist_ok=True)

    write_command_report(root, destination, args.build_log)

    manifest: list[dict[str, object]] = []
    executable_sources: list[tuple[Path, Path]] = []
    skipped: list[str] = []

    for source in sorted(iter_candidates(root, destination), key=lambda item: str(item)):
        relative = source.relative_to(root)
        try:
            stat = source.stat()
            if stat.st_size > MAX_FILE_BYTES:
                skipped.append(f"{relative}: {stat.st_size} bytes exceeds limit")
                continue
            target = files_dir / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            entry = {
                "path": str(relative),
                "bytes": stat.st_size,
                "sha256": sha256_file(source),
            }
            manifest.append(entry)
            if source.suffix.lower() in EXECUTABLE_SUFFIXES:
                executable_sources.append((source, relative))
        except OSError as exc:
            skipped.append(f"{relative}: {exc}")

    generated_reports: dict[str, list[str]] = {}
    for source, relative in executable_sources:
        generated_reports[str(relative)] = analyse_executable(source, relative, reports_dir, root)

    manifest_payload = {
        "platform": args.platform,
        "variant": args.variant,
        "root": str(root),
        "collected_file_count": len(manifest),
        "collected_bytes": sum(int(entry["bytes"]) for entry in manifest),
        "files": manifest,
        "generated_reports": generated_reports,
        "skipped": skipped,
    }
    (destination / "manifest.json").write_text(
        json.dumps(manifest_payload, indent=2, sort_keys=True), encoding="utf-8"
    )
    with (destination / "manifest.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["path", "bytes", "sha256"])
        writer.writeheader()
        writer.writerows(manifest)

    summary = [
        f"# Build diagnostics: {args.platform} / {args.variant}",
        "",
        f"Collected files: **{len(manifest)}**",
        f"Collected bytes: **{manifest_payload['collected_bytes']}**",
        f"ELF/AXF/OUT files analysed: **{len(executable_sources)}**",
        f"Skipped files: **{len(skipped)}**",
        "",
        "The manifest preserves each file's original repository-relative path, size and SHA-256.",
    ]
    (destination / "SUMMARY.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
    print("\n".join(summary))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

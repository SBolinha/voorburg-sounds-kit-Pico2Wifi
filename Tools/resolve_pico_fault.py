#!/usr/bin/env python3
"""Resolve a captured Pico fault against the exact ELF from that firmware build.

Only function names and source locations are requested from addr2line. ELF
contents, strings, and firmware settings are never printed. Addresses may move
between builds, so a newly compiled ELF cannot reliably decode an older fault.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import subprocess
import sys
from pathlib import Path


TOOL_NAME = "arm-none-eabi-addr2line"


def hex_address(value: str) -> int:
    """Accept a 32-bit register value in hexadecimal, with an optional 0x."""
    try:
        address = int(value, 16)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"expected a hexadecimal register value, got {value!r}"
        ) from exc
    if not 0 <= address <= 0xFFFFFFFF:
        raise argparse.ArgumentTypeError("register value must fit in 32 bits")
    return address


def executable(path: Path) -> bool:
    return path.is_file() and os.access(path, os.X_OK)


def find_addr2line(explicit: str | None) -> Path:
    if explicit:
        expanded = os.path.expanduser(explicit)
        path = Path(expanded)
        if executable(path):
            return path.resolve()
        found = shutil.which(expanded)
        if found:
            return Path(found).resolve()
        raise ValueError(f"addr2line executable not found or not executable: {explicit}")

    found = shutil.which(TOOL_NAME)
    if found:
        return Path(found).resolve()

    tools_root = Path.home() / "Library/Arduino15/packages/rp2040/tools"
    candidates = [path for path in tools_root.glob(f"**/{TOOL_NAME}") if executable(path)]
    if candidates:
        # Multiple Arduino core versions can coexist; report the selected tool
        # below so a lookup can be repeated with the same executable.
        return max(candidates, key=lambda path: (path.stat().st_mtime, str(path))).resolve()
    raise ValueError(
        f"{TOOL_NAME} is not installed in PATH or {tools_root}. "
        "Install the RP2040 Arduino toolchain or supply --addr2line /path/to/"
        f"{TOOL_NAME}."
    )


def elf_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        if source.read(4) != b"\x7fELF":
            raise ValueError("the supplied file is not an ELF artifact")
        source.seek(0)
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve(tool: Path, elf: Path, address: int) -> list[tuple[str, str]]:
    result = subprocess.run(
        [str(tool), "-f", "-C", "-i", "-e", str(elf), f"0x{address:08x}"],
        capture_output=True,
        text=True,
        check=False,
        timeout=30,
    )
    if result.returncode:
        detail = result.stderr.strip() or f"exit status {result.returncode}"
        raise ValueError(f"addr2line failed for 0x{address:08x}: {detail}")
    lines = result.stdout.splitlines()
    if not lines or len(lines) % 2:
        raise ValueError(f"addr2line returned incomplete results for 0x{address:08x}")
    return list(zip(lines[::2], lines[1::2]))


def print_resolution(label: str, frames: list[tuple[str, str]]) -> bool:
    print(f"\n{label}")
    resolved = False
    for index, (function, location) in enumerate(frames):
        prefix = "  " if index == 0 else "  inlined into "
        print(f"{prefix}{function}")
        print(f"    {location}")
        if function != "??" or not location.startswith("??"):
            resolved = True
    if not resolved:
        print(
            "  No symbol or source location found (??). Check that this is the "
            "exact ELF used to flash the device and that the address is valid.",
            file=sys.stderr,
        )
    elif any(location.startswith("??") for _, location in frames):
        print(
            "  Source line unavailable for part of this result; the ELF may "
            "lack debug information.",
            file=sys.stderr,
        )
    return resolved


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Decode a Pico fault PC and optional LR using an exact firmware ELF.",
        epilog=(
            "Keep the ELF from the same build that was flashed: addresses are unstable "
            "between builds. This utility reads only the specified artifact, prints "
            "its SHA256 and address resolutions, and runs locally without a shell."
        ),
    )
    parser.add_argument("elf", type=Path, help="exact compiled ELF used to flash the device")
    parser.add_argument("--pc", required=True, type=hex_address, metavar="HEX", help="captured PC in hexadecimal")
    parser.add_argument("--lr", type=hex_address, metavar="HEX", help="optional captured LR in hexadecimal")
    parser.add_argument("--addr2line", metavar="PATH", help=f"explicit {TOOL_NAME} executable; otherwise discover it locally")
    args = parser.parse_args()

    elf = args.elf.expanduser().resolve()
    if not elf.is_file():
        parser.error(f"ELF file does not exist or is not a regular file: {elf}")
    if args.lr is not None and (args.lr & ~1) < 2:
        parser.error("LR is too small to compute the call site (LR & ~1) - 2")

    try:
        tool = find_addr2line(args.addr2line)
        checksum = elf_sha256(elf)
        print(f"ELF: {elf}")
        print(f"ELF SHA256: {checksum}")
        print(f"addr2line: {tool}")
        print("Build requirement: use the exact compiled ELF from the flashed firmware.")

        pc = args.pc & ~1
        success = print_resolution(
            f"PC captured: 0x{args.pc:08x}; lookup after clearing Thumb bit: 0x{pc:08x}",
            resolve(tool, elf, pc),
        )
        if args.lr is not None:
            call_site = (args.lr & ~1) - 2
            if args.lr >= 0xFFFFFFE0:
                print(
                    "LR resembles an exception-return value; it may not identify a caller.",
                    file=sys.stderr,
                )
            lr_success = print_resolution(
                f"LR call site candidate: 0x{call_site:08x} "
                f"(captured LR 0x{args.lr:08x}; (LR & ~1) - 2)",
                resolve(tool, elf, call_site),
            )
            success = success and lr_success
            print("LR call site is an approximation; LR must contain a saved return address.")
        return 0 if success else 1
    except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

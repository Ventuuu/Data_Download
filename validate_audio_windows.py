#!/usr/bin/env python3
"""
Validate periodic AUD0 windows in a Smart Eyewear NAND dump.

Expected firmware behavior:
- NAND page size: 4096 bytes
- Page header: <IBBHII (16 bytes)
- Audio magic: b"AUD0"
- PCM: signed int16 mono, 48 kHz by default
- One useful audio window: 24,000 samples = 48,000 bytes
- Expected page pattern per window:
    23 pages with 2,048-byte payload
    1 page with 896-byte payload

The input file must be the raw page dump written by the existing USB receiver,
without LOGSTART/LOGEND markers.
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

PAGE_SIZE = 4096
PAGE_HEADER_FORMAT = "<IBBHII"
PAGE_HEADER_SIZE = struct.calcsize(PAGE_HEADER_FORMAT)
MAGIC_AUDIO = b"AUD0"

DEFAULT_SAMPLE_RATE_HZ = 48_000
DEFAULT_TARGET_SAMPLES = 24_000
BYTES_PER_SAMPLE = 2

EXPECTED_FULL_AUDIO_PAYLOAD_BYTES = 2048
EXPECTED_FINAL_AUDIO_PAYLOAD_BYTES = 896
EXPECTED_FULL_AUDIO_PAGES = 23


@dataclass
class AudioPage:
    physical_page_index: int
    version: int
    header_size: int
    payload_bytes: int
    page_sequence: int
    page_timestamp_ms: int


@dataclass
class AudioWindow:
    window_index: int
    pages: list[AudioPage] = field(default_factory=list)
    total_bytes: int = 0
    complete: bool = False
    errors: list[str] = field(default_factory=list)

    @property
    def total_samples(self) -> int:
        return self.total_bytes // BYTES_PER_SAMPLE

    @property
    def first_page(self) -> AudioPage | None:
        return self.pages[0] if self.pages else None

    @property
    def last_page(self) -> AudioPage | None:
        return self.pages[-1] if self.pages else None

    @property
    def payload_pattern(self) -> list[int]:
        return [page.payload_bytes for page in self.pages]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate 500 ms AUD0 windows in a Smart Eyewear NAND dump."
    )
    parser.add_argument(
        "dump",
        nargs="?",
        type=Path,
        help="Path to the NAND dump .bin file. A file picker opens if omitted.",
    )
    parser.add_argument(
        "--sample-rate",
        type=int,
        default=DEFAULT_SAMPLE_RATE_HZ,
        help=f"PCM sample rate in Hz (default: {DEFAULT_SAMPLE_RATE_HZ}).",
    )
    parser.add_argument(
        "--target-samples",
        type=int,
        default=DEFAULT_TARGET_SAMPLES,
        help=f"Useful samples per window (default: {DEFAULT_TARGET_SAMPLES}).",
    )
    parser.add_argument(
        "--expected-windows",
        type=int,
        default=None,
        help="Optional expected number of complete windows.",
    )
    parser.add_argument(
        "--no-files",
        action="store_true",
        help="Print the report only; do not write TXT and CSV output files.",
    )
    return parser.parse_args()


def choose_file() -> Path | None:
    try:
        from tkinter import Tk, filedialog
    except ImportError:
        return None

    root = Tk()
    root.withdraw()
    try:
        selected = filedialog.askopenfilename(
            title="Select Smart Eyewear NAND dump",
            filetypes=[
                ("Binary dump", "*.bin"),
                ("All files", "*.*"),
            ],
        )
    finally:
        root.destroy()

    return Path(selected) if selected else None


def iter_pages(dump_path: Path) -> Iterable[tuple[int, bytes]]:
    file_size = dump_path.stat().st_size
    if file_size % PAGE_SIZE != 0:
        raise ValueError(
            f"Dump size {file_size} is not a multiple of {PAGE_SIZE} bytes. "
            "The file may be incomplete or may still contain protocol markers."
        )

    with dump_path.open("rb") as handle:
        physical_index = 0
        while True:
            page = handle.read(PAGE_SIZE)
            if not page:
                break
            if len(page) != PAGE_SIZE:
                raise ValueError(
                    f"Incomplete page at physical index {physical_index}: "
                    f"{len(page)} bytes."
                )
            yield physical_index, page
            physical_index += 1


def parse_audio_pages(dump_path: Path) -> tuple[list[AudioPage], list[str], int]:
    audio_pages: list[AudioPage] = []
    warnings: list[str] = []
    total_pages = 0

    for physical_index, page in iter_pages(dump_path):
        total_pages += 1
        try:
            (
                magic_word,
                version,
                header_size,
                payload_bytes,
                page_sequence,
                page_timestamp_ms,
            ) = struct.unpack(PAGE_HEADER_FORMAT, page[:PAGE_HEADER_SIZE])
        except struct.error as exc:
            warnings.append(
                f"Page {physical_index}: invalid header ({exc})."
            )
            continue

        magic = struct.pack("<I", magic_word)
        if magic != MAGIC_AUDIO:
            continue

        if header_size < PAGE_HEADER_SIZE:
            warnings.append(
                f"AUD0 page {physical_index}: header_size={header_size}, "
                f"smaller than {PAGE_HEADER_SIZE}; skipped."
            )
            continue

        if header_size > PAGE_SIZE:
            warnings.append(
                f"AUD0 page {physical_index}: header_size={header_size}, "
                f"larger than page size; skipped."
            )
            continue

        available = PAGE_SIZE - header_size
        if payload_bytes > available:
            warnings.append(
                f"AUD0 page {physical_index}: payload_bytes={payload_bytes}, "
                f"available={available}; skipped."
            )
            continue

        if payload_bytes % BYTES_PER_SAMPLE != 0:
            warnings.append(
                f"AUD0 page {physical_index}: odd payload size "
                f"{payload_bytes} bytes."
            )

        audio_pages.append(
            AudioPage(
                physical_page_index=physical_index,
                version=version,
                header_size=header_size,
                payload_bytes=payload_bytes,
                page_sequence=page_sequence,
                page_timestamp_ms=page_timestamp_ms,
            )
        )

    return audio_pages, warnings, total_pages


def group_windows(
    audio_pages: list[AudioPage],
    target_bytes: int,
) -> list[AudioWindow]:
    windows: list[AudioWindow] = []
    current = AudioWindow(window_index=0)

    for page in audio_pages:
        if current.total_bytes + page.payload_bytes > target_bytes:
            current.errors.append(
                "Adding the next AUD0 page would exceed the expected "
                f"{target_bytes}-byte window."
            )
            windows.append(current)
            current = AudioWindow(window_index=len(windows))

        current.pages.append(page)
        current.total_bytes += page.payload_bytes

        if current.total_bytes == target_bytes:
            current.complete = True
            windows.append(current)
            current = AudioWindow(window_index=len(windows))

    if current.pages:
        current.errors.append(
            f"Partial final window: {current.total_bytes}/{target_bytes} bytes."
        )
        windows.append(current)

    return windows


def validate_window(
    window: AudioWindow,
    target_bytes: int,
    target_samples: int,
) -> None:
    if window.total_bytes % BYTES_PER_SAMPLE != 0:
        window.errors.append("Window byte count is not divisible by 2.")

    if window.complete:
        if window.total_bytes != target_bytes:
            window.errors.append(
                f"Complete window has {window.total_bytes} bytes, "
                f"expected {target_bytes}."
            )
        if window.total_samples != target_samples:
            window.errors.append(
                f"Complete window has {window.total_samples} samples, "
                f"expected {target_samples}."
            )

        expected_pattern = (
            [EXPECTED_FULL_AUDIO_PAYLOAD_BYTES] * EXPECTED_FULL_AUDIO_PAGES
            + [EXPECTED_FINAL_AUDIO_PAYLOAD_BYTES]
        )
        if window.payload_pattern != expected_pattern:
            window.errors.append(
                "Unexpected AUD0 payload pattern. Expected "
                "23 × 2048 bytes followed by 896 bytes; got "
                + ", ".join(str(value) for value in window.payload_pattern)
                + "."
            )

    sequences = [page.page_sequence for page in window.pages]
    for previous, current in zip(sequences, sequences[1:]):
        if current != previous + 1:
            # Other record types may legitimately occupy page sequences between
            # AUD0 pages. Report this as information rather than corruption.
            window.errors.append(
                f"Non-consecutive AUD0 page_sequence: {previous} -> {current}. "
                "This may be valid if SENS/LRAW pages are interleaved."
            )


def u32_delta(current: int, previous: int) -> int:
    return (current - previous) & 0xFFFFFFFF


def build_report(
    dump_path: Path,
    total_pages: int,
    audio_pages: list[AudioPage],
    windows: list[AudioWindow],
    warnings: list[str],
    sample_rate_hz: int,
    target_samples: int,
    expected_windows: int | None,
) -> str:
    total_audio_bytes = sum(page.payload_bytes for page in audio_pages)
    total_audio_samples = total_audio_bytes // BYTES_PER_SAMPLE
    total_duration_s = (
        total_audio_samples / float(sample_rate_hz)
        if sample_rate_hz > 0
        else 0.0
    )
    complete_windows = [window for window in windows if window.complete]
    partial_windows = [window for window in windows if not window.complete]

    lines = [
        "Smart Eyewear AUD0 periodic-window validation",
        f"Dump: {dump_path}",
        "",
        "Global summary",
        f"Total physical pages: {total_pages}",
        f"AUD0 pages: {len(audio_pages)}",
        f"Audio bytes: {total_audio_bytes}",
        f"Audio samples: {total_audio_samples}",
        f"Concatenated audio duration [s]: {total_duration_s:.6f}",
        f"Complete 500 ms windows: {len(complete_windows)}",
        f"Partial/invalid windows: {len(partial_windows)}",
    ]

    if expected_windows is not None:
        status = "PASS" if len(complete_windows) == expected_windows else "FAIL"
        lines.append(
            f"Expected complete windows: {expected_windows} -> {status}"
        )

    lines.extend(["", "Per-window details"])

    previous_start_timestamp: int | None = None
    for window in windows:
        first = window.first_page
        last = window.last_page
        status = "COMPLETE" if window.complete else "PARTIAL"

        lines.extend(
            [
                "",
                f"Window {window.window_index}: {status}",
                f"  AUD0 pages: {len(window.pages)}",
                f"  Bytes: {window.total_bytes}",
                f"  Samples: {window.total_samples}",
                (
                    f"  Duration [s]: "
                    f"{window.total_samples / float(sample_rate_hz):.6f}"
                    if sample_rate_hz > 0
                    else "  Duration [s]: unavailable"
                ),
            ]
        )

        if first is not None and last is not None:
            lines.extend(
                [
                    (
                        "  Physical pages: "
                        f"{first.physical_page_index} .. "
                        f"{last.physical_page_index}"
                    ),
                    (
                        "  Page sequence: "
                        f"{first.page_sequence} .. {last.page_sequence}"
                    ),
                    f"  First page timestamp [ms]: {first.page_timestamp_ms}",
                    f"  Last page timestamp [ms]: {last.page_timestamp_ms}",
                ]
            )

            if previous_start_timestamp is not None:
                interval = u32_delta(
                    first.page_timestamp_ms,
                    previous_start_timestamp,
                )
                lines.append(
                    f"  First-page timestamp interval from previous window [ms]: "
                    f"{interval}"
                )
            previous_start_timestamp = first.page_timestamp_ms

        lines.append(
            "  Payload pattern: "
            + ", ".join(str(value) for value in window.payload_pattern)
        )

        if window.errors:
            lines.append("  Notes:")
            lines.extend(f"    - {error}" for error in window.errors)
        else:
            lines.append("  Validation: PASS")

    lines.extend(["", "Parser warnings"])
    lines.extend(f"- {warning}" for warning in warnings)
    if not warnings:
        lines.append("- None")

    strict_failures = [
        window
        for window in windows
        if (not window.complete)
        or any(
            not error.startswith("Non-consecutive AUD0 page_sequence")
            for error in window.errors
        )
    ]

    expected_failure = (
        expected_windows is not None
        and len(complete_windows) != expected_windows
    )

    lines.extend(
        [
            "",
            "Final result",
            (
                "PASS"
                if not strict_failures and not expected_failure
                else "FAIL"
            ),
        ]
    )

    return "\n".join(lines) + "\n"


def write_csv(path: Path, windows: list[AudioWindow], sample_rate_hz: int) -> None:
    fieldnames = [
        "window_index",
        "complete",
        "audio_pages",
        "audio_bytes",
        "audio_samples",
        "duration_s",
        "first_physical_page",
        "last_physical_page",
        "first_page_sequence",
        "last_page_sequence",
        "first_page_timestamp_ms",
        "last_page_timestamp_ms",
        "payload_pattern",
        "notes",
    ]

    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()

        for window in windows:
            first = window.first_page
            last = window.last_page
            writer.writerow(
                {
                    "window_index": window.window_index,
                    "complete": window.complete,
                    "audio_pages": len(window.pages),
                    "audio_bytes": window.total_bytes,
                    "audio_samples": window.total_samples,
                    "duration_s": (
                        f"{window.total_samples / float(sample_rate_hz):.6f}"
                        if sample_rate_hz > 0
                        else ""
                    ),
                    "first_physical_page": (
                        first.physical_page_index if first else ""
                    ),
                    "last_physical_page": (
                        last.physical_page_index if last else ""
                    ),
                    "first_page_sequence": (
                        first.page_sequence if first else ""
                    ),
                    "last_page_sequence": (
                        last.page_sequence if last else ""
                    ),
                    "first_page_timestamp_ms": (
                        first.page_timestamp_ms if first else ""
                    ),
                    "last_page_timestamp_ms": (
                        last.page_timestamp_ms if last else ""
                    ),
                    "payload_pattern": " ".join(
                        str(value) for value in window.payload_pattern
                    ),
                    "notes": " | ".join(window.errors),
                }
            )


def main() -> int:
    args = parse_args()

    dump_path = args.dump
    if dump_path is None:
        dump_path = choose_file()
        if dump_path is None:
            print(
                "No dump selected. Pass the .bin path as a command-line argument.",
                file=sys.stderr,
            )
            return 2

    dump_path = dump_path.expanduser().resolve()
    if not dump_path.is_file():
        print(f"File not found: {dump_path}", file=sys.stderr)
        return 2

    if args.sample_rate <= 0:
        print("--sample-rate must be positive.", file=sys.stderr)
        return 2

    if args.target_samples <= 0:
        print("--target-samples must be positive.", file=sys.stderr)
        return 2

    target_bytes = args.target_samples * BYTES_PER_SAMPLE

    try:
        audio_pages, warnings, total_pages = parse_audio_pages(dump_path)
    except (OSError, ValueError) as exc:
        print(f"Unable to parse dump: {exc}", file=sys.stderr)
        return 1

    windows = group_windows(audio_pages, target_bytes)
    for window in windows:
        validate_window(window, target_bytes, args.target_samples)

    report = build_report(
        dump_path=dump_path,
        total_pages=total_pages,
        audio_pages=audio_pages,
        windows=windows,
        warnings=warnings,
        sample_rate_hz=args.sample_rate,
        target_samples=args.target_samples,
        expected_windows=args.expected_windows,
    )
    print(report)

    if not args.no_files:
        report_path = dump_path.with_name(
            dump_path.stem + "_audio_window_validation.txt"
        )
        csv_path = dump_path.with_name(
            dump_path.stem + "_audio_windows.csv"
        )
        report_path.write_text(report, encoding="utf-8")
        write_csv(csv_path, windows, args.sample_rate)
        print(f"Report written to: {report_path}")
        print(f"CSV written to: {csv_path}")

    complete_count = sum(window.complete for window in windows)
    has_strict_failure = any(
        (not window.complete)
        or any(
            not error.startswith("Non-consecutive AUD0 page_sequence")
            for error in window.errors
        )
        for window in windows
    )
    expected_failure = (
        args.expected_windows is not None
        and complete_count != args.expected_windows
    )

    return 1 if has_strict_failure or expected_failure else 0


if __name__ == "__main__":
    raise SystemExit(main())

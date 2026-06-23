"""
Smart Eyewear NAND Logger - USB receiver, parser and visualizer.

USB protocol:
    "LOGSTART" + uint32 total_pages + total_pages*4096 bytes + "LOGEND!!"

NAND page header:
    <IBBHII, 16 bytes minimum
    uint32 magic, uint8 version, uint8 header_size, uint16 payload_bytes,
    uint32 page_sequence, uint32 page_timestamp_ms

Page magic:
    b"SENS" -> IMU records, 40 bytes each; bytes 17..38 reserved
    b"AUD0" -> PCM int16 audio
    b"LRAW" -> AS7341 raw samples
    b"LITE" -> optional legacy AS7341 session result, 40-byte payload

LRAW record layout:
    <II10H, 28 bytes
    uint32 sample_elapsed_ms, uint32 sample_index,
    uint16 F1, F2, F3, F4, F5, F6, F7, F8, Clear, NIR raw counts

This script preserves raw LRAW values exactly in the CSV and adds byte-level
diagnostics so firmware serialization, page offset, field layout, padding and
saturation hypotheses can be evaluated from the original bytes. Suspicious data
is detected through monotonicity, contiguity, timestamp, 65535 saturation,
constant-channel, artificial-ramp and nearly-identical-channel checks. If LRAW
data is severely suspicious, raw light plots are skipped by default while the
binary dump, CSV, diagnostics report and summary are still written. Python can
detect inconsistencies, but it cannot always prove the firmware-side cause.
"""

import os
import struct
import tempfile
import wave
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from tkinter import Tk, filedialog, ttk, messagebox, StringVar, Label, Button, Entry

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import serial
from serial.tools import list_ports


PAGE_SIZE = 4096
PAGE_HEADER_FORMAT = "<IBBHII"
PAGE_HEADER_SIZE = 16
HEADER_SIZE = PAGE_HEADER_SIZE
SENSOR_RECORD_SIZE = 40

START_MARKER = b"LOGSTART"
END_MARKER = b"LOGEND!!"

MAGIC_SENSOR = b"SENS"
MAGIC_AUDIO = b"AUD0"
MAGIC_LIGHT_RAW = b"LRAW"
MAGIC_LIGHT = b"LITE"

LIGHT_RAW_RECORD_FORMAT = "<II10H"
LIGHT_RAW_RECORD_SIZE = 28
MAX_REASONABLE_LRAW_ELAPSED_MS = 24 * 60 * 60 * 1000
PLOT_SUSPICIOUS_LIGHT_DATA = False
RUN_INTERNAL_TESTS = False

LIGHT_RESULT_PAYLOAD_SIZE = 40
LIGHT_RESULT_STRUCT_FORMAT = "<10H4IB3s"
NORMALIZATION_SCALE = 10000.0

if struct.calcsize(LIGHT_RAW_RECORD_FORMAT) != LIGHT_RAW_RECORD_SIZE:
    raise RuntimeError("Unexpected LRAW record size")

if struct.calcsize(PAGE_HEADER_FORMAT) != PAGE_HEADER_SIZE:
    raise RuntimeError("Unexpected NAND page header size")

if struct.calcsize(LIGHT_RESULT_STRUCT_FORMAT) != LIGHT_RESULT_PAYLOAD_SIZE:
    raise RuntimeError("Internal error: LITE payload struct size does not match expected size")

LIGHT_LEVEL_LABELS = {
    0: "DARK",
    1: "LOW",
    2: "NORMAL_INDOOR",
    3: "BRIGHT",
    4: "OUTDOOR",
    5: "DIRECT_SUN",
}


LIGHT_RAW_COLUMNS = [
    "sample_elapsed_ms",
    "sample_elapsed_s",
    "sample_index",
    "f1_counts",
    "f2_counts",
    "f3_counts",
    "f4_counts",
    "f5_counts",
    "f6_counts",
    "f7_counts",
    "f8_counts",
    "clear_counts",
    "nir_counts",
    "physical_page_index",
    "page_sequence",
    "page_timestamp_ms",
    "record_index_in_page",
    "record_payload_offset",
    "record_absolute_page_offset",
    "page_version",
    "page_header_size",
    "page_payload_bytes",
]

LIGHT_RAW_CHANNEL_COLUMNS = [
    "f1_counts",
    "f2_counts",
    "f3_counts",
    "f4_counts",
    "f5_counts",
    "f6_counts",
    "f7_counts",
    "f8_counts",
    "clear_counts",
    "nir_counts",
]

LIGHT_RAW_CHANNEL_LABELS = {
    "f1_counts": "F1",
    "f2_counts": "F2",
    "f3_counts": "F3",
    "f4_counts": "F4",
    "f5_counts": "F5",
    "f6_counts": "F6",
    "f7_counts": "F7",
    "f8_counts": "F8",
    "clear_counts": "Clear",
    "nir_counts": "NIR",
}

LIGHT_RESULT_COLUMNS = [
    "format_version",
    "normalized_f1_raw",
    "normalized_f2_raw",
    "normalized_f3_raw",
    "normalized_f4_raw",
    "normalized_f5_raw",
    "normalized_f6_raw",
    "normalized_f7_raw",
    "normalized_f8_raw",
    "normalized_nir_raw",
    "normalized_f1",
    "normalized_f2",
    "normalized_f3",
    "normalized_f4",
    "normalized_f5",
    "normalized_f6",
    "normalized_f7",
    "normalized_f8",
    "normalized_nir",
    "clear_mean_counts",
    "ambient_light_index",
    "sample_count",
    "acquisition_duration_ms",
    "acquisition_duration_s",
    "session_start_ms",
    "light_level_class",
    "light_level_label",
    "page_sequence",
    "page_timestamp_ms",
    "physical_page_index",
    "page_version",
    "page_header_size",
    "page_payload_bytes",
]

DEFAULT_BAUD_RATE = 250000
DEFAULT_AUDIO_SAMPLE_RATE = 48000

def u16_le(data: bytes) -> int:
    return struct.unpack("<H", data)[0]


def i16_le(data: bytes) -> int:
    return struct.unpack("<h", data)[0]


def read_exact(ser: serial.Serial, n_bytes: int) -> bytes:
    data = bytearray()
    while len(data) < n_bytes:
        chunk = ser.read(n_bytes - len(data))
        if not chunk:
            raise TimeoutError(
                f"Serial timeout: expected {n_bytes} bytes, received {len(data)} bytes"
            )
        data.extend(chunk)
    return bytes(data)


def wait_for_marker(ser: serial.Serial, marker: bytes) -> None:
    window = bytearray()
    print(f"Waiting for marker {marker!r}...")
    while True:
        byte = ser.read(1)
        if not byte:
            continue
        window.extend(byte)
        if len(window) > len(marker):
            del window[0]
        if bytes(window) == marker:
            print("Marker found.")
            return


def write_wav_int16_mono(filename: Path, pcm_bytes: bytes, sample_rate_hz: int) -> None:
    with wave.open(str(filename), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate_hz)
        wav.writeframes(pcm_bytes)


def receive_and_save_data(com_port: str, baud_rate: int, bin_filename: Path, timeout_s: float = 10.0) -> int:
    with serial.Serial(com_port, baud_rate, timeout=timeout_s) as ser:
        print(f"Connected to {com_port} at {baud_rate} baud.")
        wait_for_marker(ser, START_MARKER)

        total_pages_raw = read_exact(ser, 4)
        total_pages = struct.unpack("<I", total_pages_raw)[0]
        print(f"Total NAND pages to receive: {total_pages}")

        if total_pages == 0:
            raise RuntimeError("Device reported zero pages. Nothing to download.")

        with open(bin_filename, "wb") as f:
            for page_idx in range(total_pages):
                page = read_exact(ser, PAGE_SIZE)
                f.write(page)
                print(f"Received page {page_idx + 1}/{total_pages}")

        end_marker = read_exact(ser, len(END_MARKER))
        if end_marker != END_MARKER:
            raise RuntimeError(f"Unexpected end marker {end_marker!r}; expected {END_MARKER!r}")

    print(f"Binary dump saved to: {bin_filename}")
    return total_pages


@dataclass
class ParseStats:
    total_pages: int = 0
    sensor_pages: int = 0
    audio_pages: int = 0
    light_raw_pages: int = 0
    light_pages: int = 0
    unknown_pages: int = 0
    sensor_records: int = 0
    light_raw_records: int = 0
    light_records: int = 0
    audio_bytes: int = 0
    invalid_light_raw_pages: int = 0
    empty_light_raw_pages: int = 0
    light_raw_payload_remainder_bytes: int = 0
    invalid_light_raw_records: int = 0
    duplicate_sample_indices: int = 0
    duplicate_timestamps: int = 0
    non_monotonic_sample_indices: int = 0
    non_monotonic_timestamps: int = 0
    suspicious_timestamp_records: int = 0
    values_equal_65535: int = 0
    plots_skipped: int = 0


def parse_sensor_record(record: bytes) -> dict:
    if len(record) != SENSOR_RECORD_SIZE:
        raise ValueError(f"Sensor record must be {SENSOR_RECORD_SIZE} bytes")

    hh = record[0]
    mm = record[1]
    ss = record[2]
    sss = u16_le(record[3:5])

    acc_x_raw = i16_le(record[5:7])
    acc_y_raw = i16_le(record[7:9])
    acc_z_raw = i16_le(record[9:11])

    gyro_x_raw = i16_le(record[11:13])
    gyro_y_raw = i16_le(record[13:15])
    gyro_z_raw = i16_le(record[15:17])

    legacy_light_reserved = record[17:39]
    legacy_light_reserved_all_zero = all(byte == 0 for byte in legacy_light_reserved)
    if not legacy_light_reserved_all_zero:
        print("Warning: SENS legacy light reserved bytes 17..38 are not all zero.")

    time_ms_record = hh * 3600000 + mm * 60000 + ss * 1000 + sss

    # Adjust if your IMU full-scale is different.
    acc_sensitivity_g_per_lsb = 2.0 / 32767.0
    gyro_sensitivity_dps_per_lsb = 1.0 / 175.0

    return {
        "hh": hh, "mm": mm, "ss": ss, "sss": sss,
        "time_ms_record": time_ms_record,
        "acc_x_raw": acc_x_raw, "acc_y_raw": acc_y_raw, "acc_z_raw": acc_z_raw,
        "gyro_x_raw": gyro_x_raw, "gyro_y_raw": gyro_y_raw, "gyro_z_raw": gyro_z_raw,
        "acc_x_g": acc_x_raw * acc_sensitivity_g_per_lsb,
        "acc_y_g": acc_y_raw * acc_sensitivity_g_per_lsb,
        "acc_z_g": acc_z_raw * acc_sensitivity_g_per_lsb,
        "gyro_x_dps": gyro_x_raw * gyro_sensitivity_dps_per_lsb,
        "gyro_y_dps": gyro_y_raw * gyro_sensitivity_dps_per_lsb,
        "gyro_z_dps": gyro_z_raw * gyro_sensitivity_dps_per_lsb,
        "legacy_light_reserved_all_zero": legacy_light_reserved_all_zero,
    }


def parse_light_raw_record(record: bytes) -> dict:
    if len(record) != LIGHT_RAW_RECORD_SIZE:
        raise ValueError(
            f"LRAW record must be {LIGHT_RAW_RECORD_SIZE} bytes; got {len(record)}"
        )

    (
        sample_elapsed_ms,
        sample_index,
        f1_counts,
        f2_counts,
        f3_counts,
        f4_counts,
        f5_counts,
        f6_counts,
        f7_counts,
        f8_counts,
        clear_counts,
        nir_counts,
    ) = struct.unpack(LIGHT_RAW_RECORD_FORMAT, record)

    return {
        "sample_elapsed_ms": sample_elapsed_ms,
        "sample_elapsed_s": sample_elapsed_ms / 1000.0,
        "sample_index": sample_index,
        "f1_counts": f1_counts,
        "f2_counts": f2_counts,
        "f3_counts": f3_counts,
        "f4_counts": f4_counts,
        "f5_counts": f5_counts,
        "f6_counts": f6_counts,
        "f7_counts": f7_counts,
        "f8_counts": f8_counts,
        "clear_counts": clear_counts,
        "nir_counts": nir_counts,
    }


def format_light_raw_record_debug(
    record: bytes,
    physical_page_index: int,
    page_sequence: int,
    record_index_in_page: int,
) -> str:
    hex_bytes = " ".join(f"{byte:02X}" for byte in record)
    lines = [
        "LRAW record byte-level debug",
        f"physical_page_index: {physical_page_index}",
        f"page_sequence: {page_sequence}",
        f"record_index_in_page: {record_index_in_page}",
        f"record_length: {len(record)}",
        f"raw_bytes_hex: {hex_bytes}",
    ]

    if len(record) == LIGHT_RAW_RECORD_SIZE:
        parsed = parse_light_raw_record(record)
        decoded_fields = [
            ("sample_elapsed_ms", parsed["sample_elapsed_ms"]),
            ("sample_index", parsed["sample_index"]),
            ("F1", parsed["f1_counts"]),
            ("F2", parsed["f2_counts"]),
            ("F3", parsed["f3_counts"]),
            ("F4", parsed["f4_counts"]),
            ("F5", parsed["f5_counts"]),
            ("F6", parsed["f6_counts"]),
            ("F7", parsed["f7_counts"]),
            ("F8", parsed["f8_counts"]),
            ("Clear", parsed["clear_counts"]),
            ("NIR", parsed["nir_counts"]),
        ]
        lines.extend(f"{name}: {value}" for name, value in decoded_fields)
    else:
        lines.append("decoded: unavailable; invalid record length")

    return "\n".join(lines)


def format_first_light_record_field_map(record: bytes) -> str:
    field_slices = [
        ("sample_elapsed_ms", 0, 4, "<I"),
        ("sample_index", 4, 8, "<I"),
        ("f1_counts", 8, 10, "<H"),
        ("f2_counts", 10, 12, "<H"),
        ("f3_counts", 12, 14, "<H"),
        ("f4_counts", 14, 16, "<H"),
        ("f5_counts", 16, 18, "<H"),
        ("f6_counts", 18, 20, "<H"),
        ("f7_counts", 20, 22, "<H"),
        ("f8_counts", 22, 24, "<H"),
        ("clear_counts", 24, 26, "<H"),
        ("nir_counts", 26, 28, "<H"),
    ]

    lines = [
        "First LRAW record field map",
        f"{'Field':<20} {'Bytes':<9} {'Hex':<15} Decoded value",
    ]
    for name, start, end, fmt in field_slices:
        field_bytes = record[start:end]
        hex_bytes = " ".join(f"{byte:02X}" for byte in field_bytes)
        if len(field_bytes) == struct.calcsize(fmt):
            decoded = struct.unpack(fmt, field_bytes)[0]
        else:
            decoded = "invalid length"
        lines.append(f"{name:<20} {start}..{end - 1:<5} {hex_bytes:<15} {decoded}")

    return "\n".join(lines)


def _safe_corrcoef(left: np.ndarray, right: np.ndarray):
    if len(left) < 2 or len(right) < 2:
        return None
    if not np.all(np.isfinite(left)) or not np.all(np.isfinite(right)):
        return None
    if float(np.std(left)) == 0.0 or float(np.std(right)) == 0.0:
        return None
    corr = float(np.corrcoef(left, right)[0, 1])
    if not np.isfinite(corr):
        return None
    return corr


def validate_light_raw_dataframe(light_raw_df: pd.DataFrame) -> dict:
    diagnostics = {
        "is_empty": True,
        "is_severely_suspicious": False,
        "sample_index_monotonic": True,
        "timestamp_monotonic": True,
        "sample_index_sequence_valid": True,
        "duplicate_sample_indices": 0,
        "duplicate_timestamps": 0,
        "non_monotonic_sample_index_steps": 0,
        "non_monotonic_timestamp_steps": 0,
        "maximum_elapsed_ms": None,
        "suspicious_timestamp_count": 0,
        "channel_65535_counts": {},
        "channel_65535_percentages": {},
        "constant_channels": [],
        "ramp_channels": [],
        "nearly_identical_channel_pairs": [],
        "out_of_range_counts": {},
        "warnings": [],
        "conclusion": "insufficient evidence",
    }

    missing_columns = [column for column in LIGHT_RAW_COLUMNS if column not in light_raw_df.columns]
    if missing_columns:
        diagnostics["warnings"].append(
            "Missing LRAW columns: " + ", ".join(missing_columns)
        )
        diagnostics["is_severely_suspicious"] = True
        diagnostics["conclusion"] = "multiple anomalies detected"
        return diagnostics

    if light_raw_df.empty:
        return diagnostics

    diagnostics["is_empty"] = False
    n_records = len(light_raw_df)

    indices = light_raw_df["sample_index"].to_numpy(dtype=np.uint64)
    elapsed_ms = light_raw_df["sample_elapsed_ms"].to_numpy(dtype=np.uint64)

    if n_records > 1:
        index_diffs = np.diff(indices.astype(np.int64))
        time_diffs = np.diff(elapsed_ms.astype(np.int64))
        negative_index_steps = int(np.sum(index_diffs < 0))
        negative_time_steps = int(np.sum(time_diffs < 0))
        diagnostics["non_monotonic_sample_index_steps"] = negative_index_steps
        diagnostics["non_monotonic_timestamp_steps"] = negative_time_steps
        diagnostics["sample_index_monotonic"] = negative_index_steps == 0
        diagnostics["timestamp_monotonic"] = negative_time_steps == 0
        if negative_index_steps:
            diagnostics["warnings"].append(
                f"Warning: LRAW sample_index decreases {negative_index_steps} time(s)."
            )
        if negative_time_steps:
            diagnostics["warnings"].append(
                f"Warning: LRAW sample_elapsed_ms decreases {negative_time_steps} time(s)."
            )

    duplicate_indices = int(light_raw_df["sample_index"].duplicated().sum())
    duplicate_timestamps = int(light_raw_df["sample_elapsed_ms"].duplicated().sum())
    diagnostics["duplicate_sample_indices"] = duplicate_indices
    diagnostics["duplicate_timestamps"] = duplicate_timestamps
    if duplicate_indices:
        diagnostics["warnings"].append(
            f"Warning: {duplicate_indices} duplicate LRAW sample index value(s)."
        )
    if duplicate_timestamps:
        diagnostics["warnings"].append(
            f"Warning: {duplicate_timestamps} duplicate LRAW timestamp value(s)."
        )

    if n_records:
        expected = np.arange(n_records, dtype=np.uint64)
        expected_from_first = np.arange(indices[0], indices[0] + n_records, dtype=np.uint64)
        sequence_valid = bool(
            np.array_equal(indices, expected) or np.array_equal(indices, expected_from_first)
        )
        diagnostics["sample_index_sequence_valid"] = sequence_valid
        if not sequence_valid:
            diagnostics["warnings"].append(
                "Warning: LRAW sample_index is not a contiguous sequence."
            )

    maximum_elapsed_ms = int(elapsed_ms.max()) if n_records else None
    diagnostics["maximum_elapsed_ms"] = maximum_elapsed_ms
    suspicious_timestamp_count = int(np.sum(elapsed_ms > MAX_REASONABLE_LRAW_ELAPSED_MS))
    diagnostics["suspicious_timestamp_count"] = suspicious_timestamp_count
    if maximum_elapsed_ms is not None and maximum_elapsed_ms > MAX_REASONABLE_LRAW_ELAPSED_MS:
        diagnostics["warnings"].append(
            "Warning: LRAW duration exceeds 24 hours. "
            "Possible serialization, offset or binary layout mismatch."
        )

    mostly_65535_channels = []
    for channel in LIGHT_RAW_CHANNEL_COLUMNS:
        series = pd.to_numeric(light_raw_df[channel], errors="coerce")
        count_65535 = int((series == 65535).sum())
        percentage_65535 = (100.0 * count_65535 / n_records) if n_records else 0.0
        diagnostics["channel_65535_counts"][channel] = count_65535
        diagnostics["channel_65535_percentages"][channel] = percentage_65535
        if percentage_65535 > 50.0:
            mostly_65535_channels.append(channel)
            diagnostics["warnings"].append(
                f"Warning: channel {LIGHT_RAW_CHANNEL_LABELS[channel]} is mostly 65535. "
                "Possible saturation or binary parsing error."
            )

        out_of_range = int(((series < 0) | (series > 65535) | series.isna()).sum())
        diagnostics["out_of_range_counts"][channel] = out_of_range
        if out_of_range:
            diagnostics["warnings"].append(
                f"Warning: channel {LIGHT_RAW_CHANNEL_LABELS[channel]} has "
                f"{out_of_range} out-of-range value(s)."
            )

        if series.nunique(dropna=False) <= 1:
            value = series.iloc[0] if n_records else None
            label = LIGHT_RAW_CHANNEL_LABELS[channel]
            diagnostics["constant_channels"].append(label)
            if value == 65535:
                diagnostics["warnings"].append(
                    f"Warning: channel {label} is constant at 65535."
                )
            else:
                diagnostics["warnings"].append(f"Warning: channel {label} is constant.")

    x = indices.astype(float)
    for channel in LIGHT_RAW_CHANNEL_COLUMNS:
        y = pd.to_numeric(light_raw_df[channel], errors="coerce").to_numpy(dtype=float)
        if len(y) < 3 or float(np.nanstd(y)) == 0.0:
            continue
        corr = _safe_corrcoef(x, y)
        if corr is None:
            continue
        diffs = np.diff(y)
        if len(diffs) < 2 or not np.all(np.isfinite(diffs)):
            continue
        mean_abs_diff = max(abs(float(np.mean(diffs))), 1.0)
        relative_diff_variation = float(np.std(diffs)) / mean_abs_diff
        if abs(corr) > 0.995 and relative_diff_variation < 0.01:
            label = LIGHT_RAW_CHANNEL_LABELS[channel]
            diagnostics["ramp_channels"].append(label)
            diagnostics["warnings"].append(
                f"Warning: channel {label} is almost perfectly correlated with sample_index. "
                "Possible serialization or field-offset error."
            )

    for left_index, left_channel in enumerate(LIGHT_RAW_CHANNEL_COLUMNS):
        left = pd.to_numeric(light_raw_df[left_channel], errors="coerce").to_numpy(dtype=float)
        if len(left) < 3 or float(np.nanstd(left)) == 0.0:
            continue
        for right_channel in LIGHT_RAW_CHANNEL_COLUMNS[left_index + 1:]:
            right = pd.to_numeric(light_raw_df[right_channel], errors="coerce").to_numpy(dtype=float)
            if len(right) < 3 or float(np.nanstd(right)) == 0.0:
                continue
            corr = _safe_corrcoef(left, right)
            if corr is None or corr <= 0.9999:
                continue
            mean_abs_diff = float(np.mean(np.abs(left - right)))
            scale = max(float(np.mean(np.abs(left))), float(np.mean(np.abs(right))), 1.0)
            if mean_abs_diff / scale <= 0.001:
                left_label = LIGHT_RAW_CHANNEL_LABELS[left_channel]
                right_label = LIGHT_RAW_CHANNEL_LABELS[right_channel]
                diagnostics["nearly_identical_channel_pairs"].append(
                    f"{left_label}-{right_label}"
                )
                diagnostics["warnings"].append(
                    f"Warning: channels {left_label} and {right_label} are nearly identical. "
                    "Possible repeated field, wrong mapping or wrong buffer offset."
                )

    severe_reasons = 0
    if maximum_elapsed_ms is not None and maximum_elapsed_ms > MAX_REASONABLE_LRAW_ELAPSED_MS:
        severe_reasons += 1
    if not diagnostics["sample_index_sequence_valid"]:
        severe_reasons += 1
    if len(mostly_65535_channels) >= 3:
        severe_reasons += 1
    if len(diagnostics["ramp_channels"]) >= 3:
        severe_reasons += 1
    if len(diagnostics["nearly_identical_channel_pairs"]) >= 2:
        severe_reasons += 1
    if sum(diagnostics["out_of_range_counts"].values()) > 0:
        severe_reasons += 1

    diagnostics["is_severely_suspicious"] = severe_reasons > 0
    if severe_reasons >= 2:
        diagnostics["conclusion"] = "multiple anomalies detected"
    elif len(mostly_65535_channels) >= 3:
        diagnostics["conclusion"] = "possible saturation"
    elif diagnostics["ramp_channels"] or diagnostics["nearly_identical_channel_pairs"]:
        diagnostics["conclusion"] = "possible field layout mismatch"
    elif maximum_elapsed_ms is not None and maximum_elapsed_ms > MAX_REASONABLE_LRAW_ELAPSED_MS:
        diagnostics["conclusion"] = "possible firmware serialization mismatch"
    elif not diagnostics["sample_index_sequence_valid"]:
        diagnostics["conclusion"] = "possible page offset mismatch"
    else:
        diagnostics["conclusion"] = "data plausible"

    return diagnostics


def get_light_plot_x_axis(light_raw_df: pd.DataFrame, diagnostics: dict):
    if light_raw_df.empty:
        return np.array([], dtype=float), "Sample index"

    use_time = (
        diagnostics.get("timestamp_monotonic", False)
        and diagnostics.get("maximum_elapsed_ms") is not None
        and diagnostics.get("maximum_elapsed_ms") <= MAX_REASONABLE_LRAW_ELAPSED_MS
        and diagnostics.get("suspicious_timestamp_count", 0) == 0
    )
    if use_time:
        return light_raw_df["sample_elapsed_s"].to_numpy(dtype=float), "Time [s]"

    print("Warning: LRAW timestamps look invalid; using sample_index for plots.")
    return light_raw_df["sample_index"].to_numpy(dtype=float), "Sample index"


def _format_channel_65535_table(diagnostics: dict) -> str:
    lines = [f"{'Channel':<8} {'Count_65535':>12} {'Percentage':>12}"]
    counts = diagnostics.get("channel_65535_counts", {})
    percentages = diagnostics.get("channel_65535_percentages", {})
    for channel in LIGHT_RAW_CHANNEL_COLUMNS:
        label = LIGHT_RAW_CHANNEL_LABELS[channel]
        count = int(counts.get(channel, 0))
        percentage = float(percentages.get(channel, 0.0))
        lines.append(f"{label:<8} {count:>12} {percentage:>11.3f}%")
    return "\n".join(lines)


def _format_first_decoded_records(light_raw_df: pd.DataFrame, limit: int = 20) -> str:
    debug_columns = [
        "sample_elapsed_ms",
        "sample_index",
        "f1_counts",
        "f2_counts",
        "f3_counts",
        "f4_counts",
        "f5_counts",
        "f6_counts",
        "f7_counts",
        "f8_counts",
        "clear_counts",
        "nir_counts",
        "physical_page_index",
        "page_sequence",
        "record_index_in_page",
        "record_payload_offset",
    ]
    if light_raw_df.empty:
        return "No LRAW records decoded."
    available_columns = [column for column in debug_columns if column in light_raw_df.columns]
    return light_raw_df[available_columns].head(limit).to_string(index=False)

def light_level_label(light_level_class: int) -> str:
    return LIGHT_LEVEL_LABELS.get(light_level_class, f"UNKNOWN_{light_level_class}")


def parse_light_result_payload(payload: bytes) -> dict:
    if len(payload) < LIGHT_RESULT_PAYLOAD_SIZE:
        raise ValueError(
            f"LITE payload must be at least {LIGHT_RESULT_PAYLOAD_SIZE} bytes; got {len(payload)}"
        )

    # Decode only the fixed 40-byte LITE result. Any remaining bytes are page padding.
    payload = payload[:LIGHT_RESULT_PAYLOAD_SIZE]
    unpacked = struct.unpack(LIGHT_RESULT_STRUCT_FORMAT, payload)
    (
        format_version,
        normalized_f1_raw,
        normalized_f2_raw,
        normalized_f3_raw,
        normalized_f4_raw,
        normalized_f5_raw,
        normalized_f6_raw,
        normalized_f7_raw,
        normalized_f8_raw,
        normalized_nir_raw,
        clear_mean_counts,
        sample_count,
        acquisition_duration_ms,
        session_start_ms,
        light_level_class_value,
        reserved,
    ) = unpacked

    if reserved != b"\x00\x00\x00":
        print(f"Warning: LITE reserved bytes are not zero: {reserved!r}")

    raw_values = {
        "normalized_f1_raw": normalized_f1_raw,
        "normalized_f2_raw": normalized_f2_raw,
        "normalized_f3_raw": normalized_f3_raw,
        "normalized_f4_raw": normalized_f4_raw,
        "normalized_f5_raw": normalized_f5_raw,
        "normalized_f6_raw": normalized_f6_raw,
        "normalized_f7_raw": normalized_f7_raw,
        "normalized_f8_raw": normalized_f8_raw,
        "normalized_nir_raw": normalized_nir_raw,
    }

    parsed = {
        "format_version": format_version,
        **raw_values,
    }

    for key, value in raw_values.items():
        parsed[key[:-4]] = value / NORMALIZATION_SCALE

    parsed.update(
        {
            "clear_mean_counts": clear_mean_counts,
            "ambient_light_index": clear_mean_counts,
            "sample_count": sample_count,
            "acquisition_duration_ms": acquisition_duration_ms,
            "acquisition_duration_s": acquisition_duration_ms / 1000.0,
            "session_start_ms": session_start_ms,
            "light_level_class": light_level_class_value,
            "light_level_label": light_level_label(light_level_class_value),
        }
    )

    return parsed


def apply_parse_stats_to_light_diagnostics(diagnostics: dict, stats: ParseStats) -> None:
    diagnostics["parse_invalid_light_raw_pages"] = stats.invalid_light_raw_pages
    diagnostics["parse_payload_remainder_bytes"] = stats.light_raw_payload_remainder_bytes
    diagnostics["parse_invalid_light_raw_records"] = stats.invalid_light_raw_records

    if stats.invalid_light_raw_pages:
        diagnostics["warnings"].append(
            f"Warning: {stats.invalid_light_raw_pages} invalid LRAW page(s) skipped."
        )
    if stats.light_raw_payload_remainder_bytes:
        diagnostics["warnings"].append(
            "Warning: LRAW payload remainder bytes detected; possible page offset mismatch."
        )
    if stats.invalid_light_raw_records:
        diagnostics["warnings"].append(
            f"Warning: {stats.invalid_light_raw_records} invalid LRAW record(s) skipped."
        )

    if (
        stats.invalid_light_raw_pages
        or stats.invalid_light_raw_records
        or stats.light_raw_payload_remainder_bytes
    ):
        diagnostics["is_severely_suspicious"] = True
        if diagnostics.get("conclusion") == "data plausible":
            diagnostics["conclusion"] = "possible page offset mismatch"
        elif diagnostics.get("conclusion") == "insufficient evidence":
            diagnostics["conclusion"] = "possible page offset mismatch"
        else:
            diagnostics["conclusion"] = "multiple anomalies detected"


def write_light_raw_diagnostics_report(
    report_filename: Path,
    bin_filename: Path,
    stats: ParseStats,
    diagnostics: dict,
    debug_records: list,
    first_record_field_map: str,
    first_decoded_records: str,
    plots_state: str,
) -> bool:
    lines = [
        "AS7341 LRAW diagnostic report",
        f"Analysis datetime: {datetime.now().isoformat(timespec='seconds')}",
        f"Dump filename: {bin_filename.name}",
        "",
        "Parser format",
        f"LRAW record format: {LIGHT_RAW_RECORD_FORMAT}",
        f"LRAW record size: {LIGHT_RAW_RECORD_SIZE} bytes",
        f"NAND page header format: {PAGE_HEADER_FORMAT}",
        f"NAND page header size: {PAGE_HEADER_SIZE} bytes",
        "",
        "Page and record counts",
        f"Total pages: {stats.total_pages}",
        f"LRAW pages: {stats.light_raw_pages}",
        f"LRAW records: {stats.light_raw_records}",
        f"Empty LRAW pages: {stats.empty_light_raw_pages}",
        f"Invalid LRAW pages: {stats.invalid_light_raw_pages}",
        f"LRAW payload remainder bytes: {stats.light_raw_payload_remainder_bytes}",
        f"Invalid LRAW records: {stats.invalid_light_raw_records}",
        "",
        "First 5 LRAW records in hex",
    ]

    if debug_records:
        lines.extend(debug_records)
    else:
        lines.append("No LRAW records decoded.")

    lines.extend(
        [
            "",
            "First record field map",
            first_record_field_map or "No LRAW record available.",
            "",
            "First 20 decoded LRAW records",
            first_decoded_records,
            "",
            "Sequence checks",
            f"sample_index monotonic: {diagnostics.get('sample_index_monotonic')}",
            f"timestamp monotonic: {diagnostics.get('timestamp_monotonic')}",
            f"Non-monotonic sample index steps: {diagnostics.get('non_monotonic_sample_index_steps')}",
            f"Non-monotonic timestamp steps: {diagnostics.get('non_monotonic_timestamp_steps')}",
            f"sample_index contiguous sequence: {diagnostics.get('sample_index_sequence_valid')}",
            f"Duplicate sample indices: {diagnostics.get('duplicate_sample_indices')}",
            f"Duplicate timestamps: {diagnostics.get('duplicate_timestamps')}",
            f"Maximum elapsed ms: {diagnostics.get('maximum_elapsed_ms')}",
            f"Suspicious timestamp records: {diagnostics.get('suspicious_timestamp_count')}",
            "",
            "Values equal to 65535",
            _format_channel_65535_table(diagnostics),
            "",
            "Constant channels",
            ", ".join(diagnostics.get("constant_channels", [])) or "None",
            "",
            "Suspicious ramp channels",
            ", ".join(diagnostics.get("ramp_channels", [])) or "None",
            "",
            "Nearly identical channel pairs",
            ", ".join(diagnostics.get("nearly_identical_channel_pairs", [])) or "None",
            "",
            "Counts out of range",
        ]
    )

    out_of_range_counts = diagnostics.get("out_of_range_counts", {})
    for channel in LIGHT_RAW_CHANNEL_COLUMNS:
        label = LIGHT_RAW_CHANNEL_LABELS[channel]
        lines.append(f"{label}: {int(out_of_range_counts.get(channel, 0))}")

    lines.extend(
        [
            "",
            "Warnings",
        ]
    )
    warnings = diagnostics.get("warnings", [])
    lines.extend(warnings if warnings else ["None"])

    lines.extend(
        [
            "",
            f"Automatic diagnostic conclusion: {diagnostics.get('conclusion')}",
            f"Plots generated or skipped: {plots_state}",
            "",
        ]
    )

    try:
        report_filename.write_text("\n".join(lines), encoding="utf-8")
    except OSError as exc:
        print(f"Warning: could not write LRAW diagnostics report: {exc}")
        return False

    print(f"Light raw diagnostics report saved to: {report_filename}")
    return True


def parse_nand_dump(
    bin_filename: Path,
    imu_csv_filename: Path,
    light_raw_csv_filename: Path,
    light_csv_filename: Path,
    wav_filename: Path,
    summary_filename: Path,
    light_raw_diagnostics_filename: Path,
    audio_sample_rate_hz: int,
):
    sensor_rows = []
    light_raw_rows = []
    light_rows = []
    audio_bytes = bytearray()
    light_raw_debug_records = []
    first_light_record_field_map = ""
    stats = ParseStats()

    with open(bin_filename, "rb") as f:
        physical_page_index = 0
        while True:
            page = f.read(PAGE_SIZE)
            if not page:
                break
            if len(page) != PAGE_SIZE:
                print(
                    f"Warning: incomplete final page at index {physical_page_index}; "
                    f"length={len(page)}"
                )
                break

            stats.total_pages += 1
            header = page[:PAGE_HEADER_SIZE]

            try:
                (
                    magic_word,
                    version,
                    header_size,
                    payload_bytes,
                    page_sequence,
                    page_timestamp_ms,
                ) = struct.unpack(PAGE_HEADER_FORMAT, header)
            except struct.error:
                stats.unknown_pages += 1
                physical_page_index += 1
                continue

            magic = struct.pack("<I", magic_word)
            is_light_raw_page = magic == MAGIC_LIGHT_RAW
            if is_light_raw_page:
                stats.light_raw_pages += 1

            if header_size != PAGE_HEADER_SIZE:
                print(
                    f"Warning: page_sequence={page_sequence} has header_size={header_size}; "
                    f"expected at least {PAGE_HEADER_SIZE}"
                )

            if header_size < PAGE_HEADER_SIZE:
                print(
                    f"Warning: page_sequence={page_sequence} has invalid "
                    f"header_size={header_size}; page skipped"
                )
                if is_light_raw_page:
                    stats.invalid_light_raw_pages += 1
                else:
                    stats.unknown_pages += 1
                physical_page_index += 1
                continue

            if header_size > PAGE_SIZE:
                print(
                    f"Warning: page_sequence={page_sequence} has invalid "
                    f"header_size={header_size}; page skipped"
                )
                if is_light_raw_page:
                    stats.invalid_light_raw_pages += 1
                else:
                    stats.unknown_pages += 1
                physical_page_index += 1
                continue

            if payload_bytes > PAGE_SIZE - header_size:
                print(
                    f"Warning: page_sequence={page_sequence} has payload_bytes={payload_bytes}, "
                    f"larger than available {PAGE_SIZE - header_size}; page skipped"
                )
                if is_light_raw_page:
                    stats.invalid_light_raw_pages += 1
                else:
                    stats.unknown_pages += 1
                physical_page_index += 1
                continue

            payload_start = header_size
            payload_end = header_size + payload_bytes
            if payload_end > len(page):
                print(
                    f"Warning: page_sequence={page_sequence} payload_end={payload_end} "
                    f"exceeds page length={len(page)}; page skipped"
                )
                if is_light_raw_page:
                    stats.invalid_light_raw_pages += 1
                else:
                    stats.unknown_pages += 1
                physical_page_index += 1
                continue

            payload = page[payload_start:payload_end]

            if magic == MAGIC_SENSOR:
                stats.sensor_pages += 1
                n_records = payload_bytes // SENSOR_RECORD_SIZE
                if payload_bytes % SENSOR_RECORD_SIZE != 0:
                    print(
                        f"Warning: SENS page_sequence={page_sequence} has "
                        f"payload_bytes={payload_bytes}, not a multiple of "
                        f"{SENSOR_RECORD_SIZE}"
                    )
                for record_index in range(n_records):
                    start = record_index * SENSOR_RECORD_SIZE
                    end = start + SENSOR_RECORD_SIZE
                    parsed = parse_sensor_record(payload[start:end])
                    parsed["physical_page_index"] = physical_page_index
                    parsed["page_sequence"] = page_sequence
                    parsed["page_timestamp_ms"] = page_timestamp_ms
                    parsed["record_index_in_page"] = record_index
                    sensor_rows.append(parsed)

            elif magic == MAGIC_AUDIO:
                stats.audio_pages += 1
                audio_bytes.extend(payload)

            elif magic == MAGIC_LIGHT_RAW:
                if payload_bytes == 0:
                    print(
                        f"Warning: empty LRAW page at page_sequence={page_sequence}"
                    )
                    stats.empty_light_raw_pages += 1
                    physical_page_index += 1
                    continue

                n_records = payload_bytes // LIGHT_RAW_RECORD_SIZE
                remainder = payload_bytes % LIGHT_RAW_RECORD_SIZE
                if remainder:
                    print(
                        f"Warning: LRAW page_sequence={page_sequence} has "
                        f"payload_bytes={payload_bytes}, not a multiple of "
                        f"{LIGHT_RAW_RECORD_SIZE}; ignoring {remainder} trailing byte(s)"
                    )
                    stats.light_raw_payload_remainder_bytes += int(remainder)

                for record_index_in_page in range(n_records):
                    record_start = record_index_in_page * LIGHT_RAW_RECORD_SIZE
                    record_end = record_start + LIGHT_RAW_RECORD_SIZE
                    record = payload[record_start:record_end]
                    if len(record) != LIGHT_RAW_RECORD_SIZE:
                        print(
                            f"Warning: invalid LRAW record length at page_sequence="
                            f"{page_sequence}, record_index={record_index_in_page}: "
                            f"{len(record)}"
                        )
                        stats.invalid_light_raw_records += 1
                        continue

                    try:
                        parsed = parse_light_raw_record(record)
                    except ValueError as exc:
                        print(f"Warning: invalid LRAW record skipped: {exc}")
                        stats.invalid_light_raw_records += 1
                        continue

                    if len(light_raw_debug_records) < 5:
                        debug_text = format_light_raw_record_debug(
                            record=record,
                            physical_page_index=physical_page_index,
                            page_sequence=page_sequence,
                            record_index_in_page=record_index_in_page,
                        )
                        light_raw_debug_records.append(debug_text)
                        print(debug_text)

                    if not first_light_record_field_map:
                        first_light_record_field_map = format_first_light_record_field_map(record)
                        print(first_light_record_field_map)

                    record_payload_offset = record_index_in_page * LIGHT_RAW_RECORD_SIZE
                    parsed["physical_page_index"] = physical_page_index
                    parsed["page_sequence"] = page_sequence
                    parsed["page_timestamp_ms"] = page_timestamp_ms
                    parsed["record_index_in_page"] = record_index_in_page
                    parsed["record_payload_offset"] = record_payload_offset
                    parsed["record_absolute_page_offset"] = header_size + record_payload_offset
                    parsed["page_version"] = version
                    parsed["page_header_size"] = header_size
                    parsed["page_payload_bytes"] = payload_bytes
                    light_raw_rows.append(parsed)

            elif magic == MAGIC_LIGHT:
                # Legacy compatibility: old firmware may still emit one LITE page.
                stats.light_pages += 1
                if payload_bytes < LIGHT_RESULT_PAYLOAD_SIZE:
                    print(
                        f"Warning: LITE page_sequence={page_sequence} has "
                        f"payload_bytes={payload_bytes}; expected at least "
                        f"{LIGHT_RESULT_PAYLOAD_SIZE}"
                    )
                    physical_page_index += 1
                    continue

                if payload_bytes != LIGHT_RESULT_PAYLOAD_SIZE:
                    print(
                        f"Warning: LITE page_sequence={page_sequence} has "
                        f"payload_bytes={payload_bytes}; using first "
                        f"{LIGHT_RESULT_PAYLOAD_SIZE} bytes"
                    )

                parsed = parse_light_result_payload(payload)
                parsed["physical_page_index"] = physical_page_index
                parsed["page_sequence"] = page_sequence
                parsed["page_timestamp_ms"] = page_timestamp_ms
                parsed["page_version"] = version
                parsed["page_header_size"] = header_size
                parsed["page_payload_bytes"] = payload_bytes
                light_rows.append(parsed)

            else:
                stats.unknown_pages += 1
                print(
                    f"Unknown page at physical_page={physical_page_index}: "
                    f"magic={magic!r}, sequence={page_sequence}, "
                    f"payload={payload_bytes}"
                )

            physical_page_index += 1

    stats.sensor_records = len(sensor_rows)
    stats.light_raw_records = len(light_raw_rows)
    stats.light_records = len(light_rows)
    stats.audio_bytes = len(audio_bytes)

    imu_df = pd.DataFrame(sensor_rows)
    light_raw_df = pd.DataFrame(light_raw_rows, columns=LIGHT_RAW_COLUMNS)
    light_df = pd.DataFrame(light_rows, columns=LIGHT_RESULT_COLUMNS)

    light_raw_diagnostics = validate_light_raw_dataframe(light_raw_df)
    apply_parse_stats_to_light_diagnostics(light_raw_diagnostics, stats)
    stats.duplicate_sample_indices = int(light_raw_diagnostics.get("duplicate_sample_indices", 0))
    stats.duplicate_timestamps = int(light_raw_diagnostics.get("duplicate_timestamps", 0))
    stats.non_monotonic_sample_indices = int(
        light_raw_diagnostics.get("non_monotonic_sample_index_steps", 0)
    )
    stats.non_monotonic_timestamps = int(
        light_raw_diagnostics.get("non_monotonic_timestamp_steps", 0)
    )
    stats.suspicious_timestamp_records = int(light_raw_diagnostics.get("suspicious_timestamp_count", 0))
    stats.values_equal_65535 = int(sum(light_raw_diagnostics.get("channel_65535_counts", {}).values()))

    print("First 20 decoded LRAW records:")
    first_decoded_records = _format_first_decoded_records(light_raw_df)
    print(first_decoded_records)

    print("LRAW 65535 table:")
    print(_format_channel_65535_table(light_raw_diagnostics))

    for warning in light_raw_diagnostics.get("warnings", []):
        print(warning)

    plots_state = "generated"
    if light_raw_df.empty:
        plots_state = "skipped (no LRAW records)"
    elif light_raw_diagnostics.get("is_severely_suspicious") and not PLOT_SUSPICIOUS_LIGHT_DATA:
        plots_state = "skipped (suspicious LRAW data)"
        stats.plots_skipped += 1
    elif light_raw_diagnostics.get("is_severely_suspicious"):
        plots_state = "generated with WARNING: suspicious LRAW data"

    imu_df.to_csv(imu_csv_filename, index=False)
    light_raw_df.to_csv(light_raw_csv_filename, index=False)
    light_df.to_csv(light_csv_filename, index=False)

    write_wav_int16_mono(wav_filename, bytes(audio_bytes), audio_sample_rate_hz)

    report_written = write_light_raw_diagnostics_report(
        report_filename=light_raw_diagnostics_filename,
        bin_filename=bin_filename,
        stats=stats,
        diagnostics=light_raw_diagnostics,
        debug_records=light_raw_debug_records,
        first_record_field_map=first_light_record_field_map,
        first_decoded_records=first_decoded_records,
        plots_state=plots_state,
    )

    summary = (
        f"Total pages: {stats.total_pages}\n"
        f"Sensor pages: {stats.sensor_pages}\n"
        f"Audio pages: {stats.audio_pages}\n"
        f"LRAW pages: {stats.light_raw_pages}\n"
        f"Legacy light result pages: {stats.light_pages}\n"
        f"Unknown pages: {stats.unknown_pages}\n"
        f"Sensor records: {stats.sensor_records}\n"
        f"LRAW records: {stats.light_raw_records}\n"
        f"Invalid LRAW pages: {stats.invalid_light_raw_pages}\n"
        f"Empty LRAW pages: {stats.empty_light_raw_pages}\n"
        f"Payload remainder bytes: {stats.light_raw_payload_remainder_bytes}\n"
        f"Invalid LRAW records: {stats.invalid_light_raw_records}\n"
        f"Duplicate sample indices: {stats.duplicate_sample_indices}\n"
        f"Duplicate timestamps: {stats.duplicate_timestamps}\n"
        f"Non-monotonic sample indices: {stats.non_monotonic_sample_indices}\n"
        f"Non-monotonic timestamps: {stats.non_monotonic_timestamps}\n"
        f"Maximum elapsed time: {light_raw_diagnostics.get('maximum_elapsed_ms')} ms\n"
        f"Constant channels: {', '.join(light_raw_diagnostics.get('constant_channels', [])) or 'None'}\n"
        f"Suspicious ramp channels: {', '.join(light_raw_diagnostics.get('ramp_channels', [])) or 'None'}\n"
        f"Nearly identical channel pairs: {', '.join(light_raw_diagnostics.get('nearly_identical_channel_pairs', [])) or 'None'}\n"
        f"Diagnostic conclusion: {light_raw_diagnostics.get('conclusion')}\n"
        f"Diagnostic report filename: {light_raw_diagnostics_filename.name if report_written else 'not written'}\n"
        f"Plots generated or skipped: {plots_state}\n"
        f"Legacy light result records: {stats.light_records}\n"
        f"Audio bytes: {stats.audio_bytes}\n"
        f"Audio samples: {stats.audio_bytes // 2}\n"
        f"Audio sample rate: {audio_sample_rate_hz} Hz\n"
        f"IMU CSV file: {imu_csv_filename.name}\n"
        f"Light raw CSV file: {light_raw_csv_filename.name}\n"
        f"Legacy light CSV file: {light_csv_filename.name}\n"
        f"WAV file: {wav_filename.name}\n"
    )

    summary += "Number of 65535 values per channel:\n"
    for channel in LIGHT_RAW_CHANNEL_COLUMNS:
        label = LIGHT_RAW_CHANNEL_LABELS[channel]
        count = int(light_raw_diagnostics.get("channel_65535_counts", {}).get(channel, 0))
        summary += f"{label}: {count}\n"

    if not light_raw_df.empty:
        summary += (
            f"First light sample index: {int(light_raw_df['sample_index'].iloc[0])}\n"
            f"Last light sample index: {int(light_raw_df['sample_index'].iloc[-1])}\n"
            f"Light acquisition duration: "
            f"{int(light_raw_df['sample_elapsed_ms'].max())} ms\n"
        )
        for channel in LIGHT_RAW_CHANNEL_COLUMNS:
            summary += (
                f"{channel} min: {int(light_raw_df[channel].min())}\n"
                f"{channel} max: {int(light_raw_df[channel].max())}\n"
                f"{channel} mean: {light_raw_df[channel].mean():.3f}\n"
            )

        nir_max_row = light_raw_df.loc[light_raw_df["nir_counts"].idxmax()]
        summary += (
            f"Maximum NIR count: {int(nir_max_row['nir_counts'])}\n"
            f"NIR maximum sample index: {int(nir_max_row['sample_index'])}\n"
            f"NIR maximum time: {float(nir_max_row['sample_elapsed_s']):.3f} s\n"
        )

    if not light_df.empty:
        last_light = light_df.iloc[-1]
        summary += (
            f"Legacy last light class: {last_light['light_level_label']}\n"
            f"Legacy last Clear mean counts: {last_light['clear_mean_counts']}\n"
            f"Legacy last sample count: {last_light['sample_count']}\n"
        )

    summary_filename.write_text(summary, encoding="utf-8")

    print(summary)
    print(f"IMU CSV saved to: {imu_csv_filename}")
    print(f"Light raw CSV saved to: {light_raw_csv_filename}")
    print(f"Legacy light CSV saved to: {light_csv_filename}")
    print(f"Audio WAV saved to: {wav_filename}")
    print(f"Summary saved to: {summary_filename}")

    return imu_df, light_raw_df, light_df, bytes(audio_bytes), stats, light_raw_diagnostics


def plot_imu_data(df: pd.DataFrame, output_prefix: Path) -> None:
    if df.empty:
        print("No IMU data available; skipping IMU plots.")
        return

    t = df["time_ms_record"].to_numpy(dtype=float) / 1000.0
    if len(t) > 1 and np.any(np.diff(t) < 0):
        t = np.arange(len(df), dtype=float)
        x_label = "Sample index"
    else:
        x_label = "Time [s]"

    plt.figure(figsize=(12, 5))
    plt.plot(t, df["acc_x_g"], label="Acc X")
    plt.plot(t, df["acc_y_g"], label="Acc Y")
    plt.plot(t, df["acc_z_g"], label="Acc Z")
    plt.xlabel(x_label)
    plt.ylabel("Acceleration [g]")
    plt.title("Accelerometer")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    acc_png = output_prefix.with_name(output_prefix.name + "_accelerometer.png")
    plt.savefig(acc_png, dpi=200)

    plt.figure(figsize=(12, 5))
    plt.plot(t, df["gyro_x_dps"], label="Gyro X")
    plt.plot(t, df["gyro_y_dps"], label="Gyro Y")
    plt.plot(t, df["gyro_z_dps"], label="Gyro Z")
    plt.xlabel(x_label)
    plt.ylabel("Angular rate [deg/s]")
    plt.title("Gyroscope")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    gyro_png = output_prefix.with_name(output_prefix.name + "_gyroscope.png")
    plt.savefig(gyro_png, dpi=200)


def plot_light_results(light_df: pd.DataFrame, output_prefix: Path) -> None:
    if light_df.empty:
        print("No LITE pages found; skipping light plots.")
        return

    channels = ["F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "NIR"]
    wavelengths_nm = [415, 445, 480, 515, 555, 590, 630, 680, 910]
    value_columns = [
        "normalized_f1",
        "normalized_f2",
        "normalized_f3",
        "normalized_f4",
        "normalized_f5",
        "normalized_f6",
        "normalized_f7",
        "normalized_f8",
        "normalized_nir",
    ]
    x_labels = [f"{channel}\n{wavelength} nm" for channel, wavelength in zip(channels, wavelengths_nm)]

    for row_index, (_, row) in enumerate(light_df.iterrows(), start=1):
        values = [row[column] for column in value_columns]
        suffix = "_light_signature.png"
        if len(light_df) > 1:
            suffix = f"_light_signature_{row_index:02d}.png"

        title = (
            "AS7341 normalized multispectral signature\n"
            f"Light level: {row['light_level_label']} | "
            f"Ambient light index: {row['clear_mean_counts']} counts | "
            f"Samples: {row['sample_count']} | "
            f"Duration: {row['acquisition_duration_ms']} ms"
        )

        plt.figure(figsize=(10, 5))
        plt.bar(x_labels, values)
        plt.ylim(0, 1.05)
        plt.xlabel("Channel and central wavelength")
        plt.ylabel("Normalized response")
        plt.title(title)
        plt.grid(axis="y", alpha=0.35)
        plt.tight_layout()
        light_png = output_prefix.with_name(output_prefix.name + suffix)
        plt.savefig(light_png, dpi=200)

        print(f"Ambient light index: {row['clear_mean_counts']} counts")
        print(f"Light level: {row['light_level_label']}")


def plot_light_raw_channels(
    light_raw_df: pd.DataFrame,
    output_prefix: Path,
    diagnostics: dict,
) -> None:
    if light_raw_df.empty:
        print("No LRAW pages found; skipping raw light channel plots.")
        return

    if diagnostics.get("is_severely_suspicious") and not PLOT_SUSPICIOUS_LIGHT_DATA:
        print("Light raw data appears corrupted or misparsed.")
        print("Plots skipped. Inspect the byte-level diagnostic report.")
        return

    channel_plots = [
        ("F1 - 415 nm", "f1_counts"),
        ("F2 - 445 nm", "f2_counts"),
        ("F3 - 480 nm", "f3_counts"),
        ("F4 - 515 nm", "f4_counts"),
        ("F5 - 555 nm", "f5_counts"),
        ("F6 - 590 nm", "f6_counts"),
        ("F7 - 630 nm", "f7_counts"),
        ("F8 - 680 nm", "f8_counts"),
        ("Clear", "clear_counts"),
        ("NIR - 910 nm", "nir_counts"),
    ]

    x, x_label = get_light_plot_x_axis(light_raw_df, diagnostics)
    title_suffix = ""
    if diagnostics.get("is_severely_suspicious"):
        title_suffix = "\nWARNING: suspicious LRAW data"

    fig, axes = plt.subplots(
        len(channel_plots),
        1,
        figsize=(15, 24),
        sharex=True,
    )
    fig.suptitle("AS7341 raw channel counts over time" + title_suffix, fontsize=14)

    for axis, (label, column) in zip(axes, channel_plots):
        axis.plot(x, light_raw_df[column].to_numpy(dtype=float), linewidth=1.2)
        axis.set_title(label)
        axis.set_ylabel("Raw ADC counts")
        axis.grid(True)

    axes[-1].set_xlabel(x_label)
    plt.tight_layout(rect=(0, 0, 1, 0.985))
    output_file = output_prefix.with_name(
        output_prefix.name + "_light_raw_channels.png"
    )
    plt.savefig(output_file, dpi=200)
    print(f"Raw light channel plot saved to: {output_file}")

    plt.figure(figsize=(14, 7))
    for label, column in channel_plots:
        plt.plot(x, light_raw_df[column].to_numpy(dtype=float), label=label, linewidth=1.0)
    plt.xlabel(x_label)
    plt.ylabel("Raw ADC counts")
    plt.title("AS7341 raw channel counts over time" + title_suffix)
    plt.grid(True)
    plt.legend(ncol=2)
    plt.tight_layout()
    all_channels_file = output_prefix.with_name(
        output_prefix.name + "_light_raw_all_channels.png"
    )
    plt.savefig(all_channels_file, dpi=200)
    print(f"Combined raw light plot saved to: {all_channels_file}")

    nir_max_idx = light_raw_df["nir_counts"].idxmax()
    nir_max_row = light_raw_df.loc[nir_max_idx]
    nir_max_value = int(nir_max_row["nir_counts"])
    nir_max_time = float(nir_max_row["sample_elapsed_s"])
    nir_max_sample = int(nir_max_row["sample_index"])

    plt.figure(figsize=(12, 5))
    plt.plot(x, light_raw_df["nir_counts"].to_numpy(dtype=float), label="NIR - 910 nm")
    max_x = nir_max_time if x_label == "Time [s]" else nir_max_sample
    plt.scatter([max_x], [nir_max_value], zorder=3)
    plt.annotate(
        f"max={nir_max_value}\nsample={nir_max_sample}\ntime={nir_max_time:.3f} s",
        xy=(max_x, nir_max_value),
        xytext=(10, 10),
        textcoords="offset points",
    )
    plt.xlabel(x_label)
    plt.ylabel("Raw ADC counts")
    plt.title("AS7341 NIR raw counts" + title_suffix)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    nir_file = output_prefix.with_name(output_prefix.name + "_light_raw_nir.png")
    plt.savefig(nir_file, dpi=200)
    print(f"NIR plot saved to: {nir_file}")

    print(f"Maximum NIR count: {nir_max_value}")
    print(f"NIR maximum sample index: {nir_max_sample}")
    print(f"NIR maximum time: {nir_max_time:.3f} s")


def plot_audio_waveform(audio_bytes: bytes, sample_rate_hz: int, output_prefix: Path) -> None:
    if not audio_bytes:
        print("No audio data available; skipping audio plot.")
        return

    if len(audio_bytes) % 2 != 0:
        print("Warning: audio byte count is odd; last byte ignored.")
        audio_bytes = audio_bytes[:-1]

    audio = np.frombuffer(audio_bytes, dtype="<i2")
    t = np.arange(len(audio), dtype=float) / float(sample_rate_hz)

    plt.figure(figsize=(12, 4))
    plt.plot(t, audio)
    plt.xlabel("Time [s]")
    plt.ylabel("PCM amplitude [int16]")
    plt.title("Audio waveform")
    plt.grid(True)
    plt.tight_layout()
    audio_png = output_prefix.with_name(output_prefix.name + "_audio_waveform.png")
    plt.savefig(audio_png, dpi=200)


def gui_select_com_and_folder():
    root = Tk()
    root.title("Smart Eyewear NAND Logger")
    root.geometry("460x360")
    root.resizable(False, False)

    Label(root, text="Select the COM Port:", font=("Segoe UI", 10)).pack(pady=5)

    com_var = StringVar()
    ports = [p.device for p in list_ports.comports()]
    if not ports:
        ports = ["No COM Port found"]

    com_box = ttk.Combobox(root, textvariable=com_var, values=ports, state="readonly", width=36)
    com_box.pack(pady=5)
    com_box.current(0)

    folder_var = StringVar()

    def browse_folder():
        folder = filedialog.askdirectory(title="Select output folder")
        if folder:
            folder_var.set(folder)

    Label(root, text="Output folder:", font=("Segoe UI", 10)).pack(pady=5)
    Button(root, text="Select folder...", command=browse_folder).pack(pady=2)
    Label(root, textvariable=folder_var, fg="blue", wraplength=420).pack(pady=5)

    baud_var = StringVar(value=str(DEFAULT_BAUD_RATE))
    audio_fs_var = StringVar(value=str(DEFAULT_AUDIO_SAMPLE_RATE))

    Label(root, text="Baud rate:", font=("Segoe UI", 10)).pack(pady=3)
    Entry(root, textvariable=baud_var, width=16).pack()

    Label(root, text="Audio sample rate after MDF [Hz]:", font=("Segoe UI", 10)).pack(pady=3)
    Entry(root, textvariable=audio_fs_var, width=16).pack()

    result = {"confirmed": False}

    def confirm():
        if "No COM Port found" in com_var.get():
            messagebox.showerror("Error", "No valid COM port selected.")
            return
        if not folder_var.get():
            messagebox.showerror("Error", "Select a valid output folder.")
            return
        try:
            int(baud_var.get())
            int(audio_fs_var.get())
        except ValueError:
            messagebox.showerror("Error", "Baud rate and audio sample rate must be integers.")
            return

        result["confirmed"] = True
        root.destroy()

    Button(root, text="Confirm", command=confirm, bg="#4CAF50", fg="white").pack(pady=16)
    root.mainloop()

    if not result["confirmed"]:
        return None, None, None, None

    return com_var.get(), Path(folder_var.get()), int(baud_var.get()), int(audio_fs_var.get())


def _lraw_magic_word() -> int:
    return struct.unpack("<I", MAGIC_LIGHT_RAW)[0]


def _make_test_page(
    magic: bytes = MAGIC_LIGHT_RAW,
    version: int = 1,
    header_size: int = PAGE_HEADER_SIZE,
    payload: bytes = b"",
    page_sequence: int = 1,
    page_timestamp_ms: int = 0,
) -> bytes:
    magic_word = struct.unpack("<I", magic)[0]
    header = struct.pack(
        PAGE_HEADER_FORMAT,
        magic_word,
        version,
        header_size,
        len(payload),
        page_sequence,
        page_timestamp_ms,
    )
    if header_size < len(header):
        body = header
    else:
        body = header + (b"\xAA" * (header_size - len(header)))
    body += payload
    return body.ljust(PAGE_SIZE, b"\x00")[:PAGE_SIZE]


def _make_light_raw_test_df(records: list) -> pd.DataFrame:
    rows = []
    for record_index, values in enumerate(records):
        row = {
            "sample_elapsed_ms": values.get("sample_elapsed_ms", record_index * 100),
            "sample_elapsed_s": values.get("sample_elapsed_ms", record_index * 100) / 1000.0,
            "sample_index": values.get("sample_index", record_index),
            "f1_counts": values.get("f1_counts", record_index + 1),
            "f2_counts": values.get("f2_counts", record_index + 2),
            "f3_counts": values.get("f3_counts", record_index + 3),
            "f4_counts": values.get("f4_counts", record_index + 4),
            "f5_counts": values.get("f5_counts", record_index + 5),
            "f6_counts": values.get("f6_counts", record_index + 6),
            "f7_counts": values.get("f7_counts", record_index + 7),
            "f8_counts": values.get("f8_counts", record_index + 8),
            "clear_counts": values.get("clear_counts", record_index + 9),
            "nir_counts": values.get("nir_counts", record_index + 10),
            "physical_page_index": 0,
            "page_sequence": 1,
            "page_timestamp_ms": 0,
            "record_index_in_page": record_index,
            "record_payload_offset": record_index * LIGHT_RAW_RECORD_SIZE,
            "record_absolute_page_offset": PAGE_HEADER_SIZE + record_index * LIGHT_RAW_RECORD_SIZE,
            "page_version": 1,
            "page_header_size": PAGE_HEADER_SIZE,
            "page_payload_bytes": len(records) * LIGHT_RAW_RECORD_SIZE,
        }
        rows.append(row)
    return pd.DataFrame(rows, columns=LIGHT_RAW_COLUMNS)


def _run_parse_test_dump(dump_bytes: bytes):
    with tempfile.TemporaryDirectory() as temp_dir:
        temp_path = Path(temp_dir)
        bin_filename = temp_path / "test_nand_dump.bin"
        bin_filename.write_bytes(dump_bytes)
        return parse_nand_dump(
            bin_filename=bin_filename,
            imu_csv_filename=temp_path / "test_imu.csv",
            light_raw_csv_filename=temp_path / "test_light_raw_counts.csv",
            light_csv_filename=temp_path / "test_light_results.csv",
            wav_filename=temp_path / "test_audio.wav",
            summary_filename=temp_path / "test_summary.txt",
            light_raw_diagnostics_filename=temp_path / "test_light_raw_diagnostics.txt",
            audio_sample_rate_hz=DEFAULT_AUDIO_SAMPLE_RATE,
        )


def run_internal_tests() -> None:
    record = struct.pack(
        "<II10H",
        1234,
        7,
        100,
        200,
        300,
        400,
        500,
        600,
        700,
        800,
        900,
        1000,
    )
    assert len(record) == LIGHT_RAW_RECORD_SIZE
    parsed = parse_light_raw_record(record)
    assert parsed["sample_elapsed_ms"] == 1234
    assert parsed["sample_index"] == 7
    assert parsed["f1_counts"] == 100
    assert parsed["f8_counts"] == 800
    assert parsed["clear_counts"] == 900
    assert parsed["nir_counts"] == 1000

    page = _make_test_page(header_size=PAGE_HEADER_SIZE, payload=record)
    _, light_raw_df, _, _, stats, _ = _run_parse_test_dump(page)
    assert stats.light_raw_records == 1
    assert int(light_raw_df["sample_elapsed_ms"].iloc[0]) == 1234

    page = _make_test_page(header_size=PAGE_HEADER_SIZE + 4, payload=record)
    _, light_raw_df, _, _, stats, _ = _run_parse_test_dump(page)
    assert stats.light_raw_records == 1
    assert int(light_raw_df["record_absolute_page_offset"].iloc[0]) == PAGE_HEADER_SIZE + 4

    page = _make_test_page(header_size=PAGE_HEADER_SIZE - 4, payload=record)
    _, light_raw_df, _, _, stats, _ = _run_parse_test_dump(page)
    assert stats.invalid_light_raw_pages == 1
    assert light_raw_df.empty

    page = _make_test_page(payload=record + b"XYZ")
    _, light_raw_df, _, _, stats, _ = _run_parse_test_dump(page)
    assert stats.light_raw_payload_remainder_bytes == 3
    assert stats.light_raw_records == 1
    assert len(light_raw_df) == 1

    absurd_df = _make_light_raw_test_df(
        [{"sample_elapsed_ms": 4_000_000_000, "sample_index": 0}]
    )
    absurd_diagnostics = validate_light_raw_dataframe(absurd_df)
    _, x_label = get_light_plot_x_axis(absurd_df, absurd_diagnostics)
    assert absurd_diagnostics["suspicious_timestamp_count"] == 1
    assert absurd_diagnostics["is_severely_suspicious"]
    assert x_label == "Sample index"

    invalid_index_df = _make_light_raw_test_df(
        [{"sample_index": index, "sample_elapsed_ms": n * 100} for n, index in enumerate([0, 1, 4, 4, 3])]
    )
    invalid_index_diagnostics = validate_light_raw_dataframe(invalid_index_df)
    assert invalid_index_diagnostics["duplicate_sample_indices"] == 1
    assert not invalid_index_diagnostics["sample_index_monotonic"]
    assert not invalid_index_diagnostics["sample_index_sequence_valid"]

    ramp_df = _make_light_raw_test_df(
        [{"sample_index": i, "f1_counts": i * 1000, "sample_elapsed_ms": i * 100} for i in range(10)]
    )
    ramp_diagnostics = validate_light_raw_dataframe(ramp_df)
    assert "F1" in ramp_diagnostics["ramp_channels"]

    saturated_df = _make_light_raw_test_df(
        [{"sample_index": i, "nir_counts": 65535, "sample_elapsed_ms": i * 100} for i in range(6)]
    )
    saturated_diagnostics = validate_light_raw_dataframe(saturated_df)
    assert saturated_diagnostics["channel_65535_counts"]["nir_counts"] == 6
    assert saturated_diagnostics["channel_65535_percentages"]["nir_counts"] == 100.0

    identical_df = _make_light_raw_test_df(
        [
            {
                "sample_index": i,
                "f5_counts": i * 10 + 1,
                "f6_counts": i * 10 + 1,
                "f7_counts": i * 10 + 1,
                "sample_elapsed_ms": i * 100,
            }
            for i in range(8)
        ]
    )
    identical_diagnostics = validate_light_raw_dataframe(identical_df)
    assert "F5-F6" in identical_diagnostics["nearly_identical_channel_pairs"]
    assert "F5-F7" in identical_diagnostics["nearly_identical_channel_pairs"]

    no_lraw_page = _make_test_page(magic=MAGIC_SENSOR, payload=b"")
    _, light_raw_df, _, _, stats, diagnostics = _run_parse_test_dump(no_lraw_page)
    assert stats.light_raw_pages == 0
    assert light_raw_df.empty
    assert list(light_raw_df.columns) == LIGHT_RAW_COLUMNS
    assert diagnostics["is_empty"]

    print("Internal synthetic tests passed.")


def main():
    com_port, save_folder, baud_rate, audio_sample_rate = gui_select_com_and_folder()

    if not com_port or save_folder is None:
        print("Application stopped.")
        return

    timestamp = datetime.now().strftime("SmartEyewear_%Y%m%d_%H%M%S")
    output_prefix = save_folder / timestamp

    bin_filename = output_prefix.with_name(output_prefix.name + "_nand_dump.bin")
    imu_csv_filename = output_prefix.with_name(output_prefix.name + "_imu.csv")
    light_raw_csv_filename = output_prefix.with_name(
        output_prefix.name + "_light_raw_counts.csv"
    )
    light_csv_filename = output_prefix.with_name(output_prefix.name + "_light_results.csv")
    wav_filename = output_prefix.with_name(output_prefix.name + "_audio.wav")
    summary_filename = output_prefix.with_name(output_prefix.name + "_summary.txt")
    light_raw_diagnostics_filename = output_prefix.with_name(
        output_prefix.name + "_light_raw_diagnostics.txt"
    )

    try:
        receive_and_save_data(
            com_port=com_port,
            baud_rate=baud_rate,
            bin_filename=bin_filename,
        )
    except (serial.SerialException, TimeoutError, RuntimeError) as exc:
        messagebox.showerror("Download error", str(exc))
        print(f"Download error: {exc}")
        return

    try:
        imu_df, light_raw_df, light_df, audio_bytes, _, light_raw_diagnostics = parse_nand_dump(
            bin_filename=bin_filename,
            imu_csv_filename=imu_csv_filename,
            light_raw_csv_filename=light_raw_csv_filename,
            light_csv_filename=light_csv_filename,
            wav_filename=wav_filename,
            summary_filename=summary_filename,
            light_raw_diagnostics_filename=light_raw_diagnostics_filename,
            audio_sample_rate_hz=audio_sample_rate,
        )

        plot_imu_data(imu_df, output_prefix)
        plot_light_raw_channels(light_raw_df, output_prefix, light_raw_diagnostics)
        plot_light_results(light_df, output_prefix)
        plot_audio_waveform(audio_bytes, audio_sample_rate, output_prefix)
        plt.show()

    except Exception as exc:
        messagebox.showerror("Processing error", str(exc))
        print(f"Processing error: {exc}")
        return

    messagebox.showinfo(
        "Completed",
        "Download and processing completed.\n\n"
        f"Files saved in:\n{save_folder}"
    )


if __name__ == "__main__":
    if RUN_INTERNAL_TESTS:
        run_internal_tests()
    else:
        main()

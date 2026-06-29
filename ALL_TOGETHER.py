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
    b"AFEA" -> packed embedded audio feature records
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

import argparse
import binascii
import os
import csv
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
MAGIC_AUDIO_FEATURE = b"AFEA"
MAGIC_LIGHT_FEATURE = b"LFEA"
MAGIC_LIGHT_RAW = b"LRAW"
MAGIC_LIGHT = b"LITE"

AUDIO_FEATURE_RECORD_VERSION = 1
AUDIO_FEATURE_RECORD_SIZE = 24
AUDIO_FEATURE_RECORD_FORMAT = "<IIHhhhhhHBB"
AUDIO_DB_CENTI_INVALID = -32768
AUDIO_WINDOW_SAMPLES = 24000
AUDIO_WINDOW_BYTES = AUDIO_WINDOW_SAMPLES * 2
AUDIO_WINDOW_EXPECTED_PAYLOAD_PATTERN = (2048,) * 23 + (896,)
AUDIO_FEATURE_DB_TOLERANCE_DB = 0.02
AUDIO_FEATURE_MEAN_TOLERANCE_COUNTS = 0

AUDIO_FLAG_COMPLETE = 1 << 0
AUDIO_FLAG_ACQUISITION_VALID = 1 << 1
AUDIO_FLAG_A_WEIGHTED_VALID = 1 << 2
AUDIO_FLAG_CLIPPED = 1 << 3
AUDIO_FLAG_HIGH_LEVEL = 1 << 4
AUDIO_FLAG_SILENT_OR_UNAVAILABLE = 1 << 5
AUDIO_FLAG_IMPULSIVE_EVENT = 1 << 6
AUDIO_FLAG_RESERVED = 1 << 7
AUDIO_RECONSTRUCTIBLE_FLAG_MASK = 0x7F

LIGHT_FEATURE_RECORD_VERSION = 1
LIGHT_FEATURE_RECORD_FORMAT = "<II10HBB"
LIGHT_FEATURE_RECORD_SIZE = 30
LIGHT_FEATURE_RECORDS_PER_PAGE = (PAGE_SIZE - PAGE_HEADER_SIZE) // LIGHT_FEATURE_RECORD_SIZE

LIGHT_FLAG_COMPLETE = 1 << 0
LIGHT_FLAG_ACQUISITION_VALID = 1 << 1
LIGHT_FLAG_CLASSIFICATION_VALID = 1 << 2
LIGHT_FLAG_SATURATED = 1 << 3
LIGHT_FLAG_I2C_ERROR = 1 << 4
LIGHT_FLAG_SMUX_ERROR = 1 << 5
LIGHT_FLAG_RESERVED_6 = 1 << 6
LIGHT_FLAG_RESERVED_7 = 1 << 7
LIGHT_RECONSTRUCTIBLE_FLAG_MASK = 0x0F

LIGHT_EXPOSURE_UNAVAILABLE = 255
LIGHT_EXPOSURE_LABELS = {
    0: "DARK",
    1: "LOW_EXPOSURE",
    2: "MODERATE_EXPOSURE",
    3: "HIGH_EXPOSURE",
    4: "VERY_HIGH_EXPOSURE",
    LIGHT_EXPOSURE_UNAVAILABLE: "UNAVAILABLE",
}

LIGHT_THRESHOLD_DARK_TO_LOW_COUNTS = 3
LIGHT_THRESHOLD_LOW_TO_MODERATE_COUNTS = 50
LIGHT_THRESHOLD_MODERATE_TO_HIGH_COUNTS = 6500
LIGHT_THRESHOLD_HIGH_TO_VERY_HIGH_COUNTS = 9800
LIGHT_SATURATION_CLEAR_COUNTS = 10000
LIGHT_FEATURE_BOUNDARY_THRESHOLDS = (
    LIGHT_THRESHOLD_DARK_TO_LOW_COUNTS,
    LIGHT_THRESHOLD_LOW_TO_MODERATE_COUNTS,
    LIGHT_THRESHOLD_MODERATE_TO_HIGH_COUNTS,
    LIGHT_THRESHOLD_HIGH_TO_VERY_HIGH_COUNTS,
)

AUDIO_ENVIRONMENT_LABELS = {
    0: "VERY_QUIET",
    1: "QUIET",
    2: "MODERATE",
    3: "LIVELY",
    4: "NOISY",
    5: "VERY_NOISY",
    6: "HIGH_EXPOSURE",
    255: "UNAVAILABLE",
}

AUDIO_ENVIRONMENT_CLASS_NAMES = {
    0: "SOUND_VERY_QUIET",
    1: "SOUND_QUIET",
    2: "SOUND_MODERATE",
    3: "SOUND_LIVELY",
    4: "SOUND_NOISY",
    5: "SOUND_VERY_NOISY",
    6: "SOUND_HIGH_EXPOSURE",
}

AUDIO_ENVIRONMENT_CLASS_RANGES = {
    0: "< 35 dBA",
    1: "35-45 dBA",
    2: "45-55 dBA",
    3: "55-65 dBA",
    4: "65-75 dBA",
    5: "75-85 dBA",
    6: ">= 85 dBA",
}

AUDIO_ENVIRONMENT_CLASS_INTERVALS = {
    0: "Estimated LAeq < 35 dBA",
    1: "35 <= Estimated LAeq < 45 dBA",
    2: "45 <= Estimated LAeq < 55 dBA",
    3: "55 <= Estimated LAeq < 65 dBA",
    4: "65 <= Estimated LAeq < 75 dBA",
    5: "75 <= Estimated LAeq < 85 dBA",
    6: "Estimated LAeq >= 85 dBA",
}

AUDIO_ENVIRONMENT_CLASS_TICK_LABELS = (
    "0 — Very quiet",
    "1 — Quiet",
    "2 — Moderate",
    "3 — Lively",
    "4 — Noisy",
    "5 — Very noisy",
    "6 — High exposure",
)

AUDIO_FEATURE_COLUMNS = [
    "window_sequence", "window_start_ms", "sample_count", "mean_counts_rounded",
    "rms_z_centi_dbfs", "rms_z_dbfs", "rms_a_centi_dbfs", "rms_a_dbfs",
    "estimated_laeq_centi_dba", "estimated_laeq_dba",
    "peak_centi_dbfs", "peak_dbfs", "clipped_sample_count",
    "environment_class", "environment_label", "flags",
    "flag_complete", "flag_acquisition_valid", "flag_a_weighted_valid",
    "flag_clipped", "flag_high_level", "flag_silent_or_unavailable",
    "flag_impulsive_event", "flag_reserved", "physical_page_index",
    "page_sequence", "page_timestamp_ms", "record_index_in_page",
    "record_payload_offset", "page_version", "page_header_size",
    "page_payload_bytes", "boot_session", "session_elapsed_s",
    "observed_elapsed_s", "record_valid", "environment_class_name",
    "environment_class_range", "python_environment_class",
    "environment_class_match",
]

AUDIO_FEATURE_METRIC_FIELDS = (
    ("mean_counts_rounded", "Mean microphone value", "counts"),
    ("rms_z_dbfs", "RMS Z", "dBFS"),
    ("rms_a_dbfs", "RMS A-weighted", "dBFS"),
    ("estimated_laeq_dba", "Estimated LAeq", "dBA"),
    ("peak_dbfs", "Peak", "dBFS"),
    ("clipped_sample_count", "Clipped samples per window", "samples"),
)

AUDIO_FEATURE_FLAG_FIELDS = (
    ("flag_complete", "Complete"),
    ("flag_acquisition_valid", "Acquisition valid"),
    ("flag_a_weighted_valid", "A-weighting valid"),
    ("flag_clipped", "Clipped"),
    ("flag_high_level", "High level"),
    ("flag_silent_or_unavailable", "Silent/unavailable"),
    ("flag_impulsive_event", "Impulsive event"),
    ("flag_reserved", "Reserved"),
)

AUDIO_WINDOW_COLUMNS = [
    "audio_window_index", "audio_bytes", "audio_samples", "complete",
    "structurally_valid", "first_physical_page", "last_physical_page",
    "first_page_sequence", "last_page_sequence", "first_page_timestamp_ms",
    "last_page_timestamp_ms", "payload_pattern",
]

AUDIO_FEATURE_COMPARISON_COLUMNS = [
    "audio_window_index", "firmware_window_sequence", "firmware_window_start_ms",
    "sample_count_firmware", "sample_count_python", "sample_count_match",
    "mean_firmware_rounded", "mean_python", "mean_python_rounded", "mean_match",
    "rms_z_firmware_dbfs", "rms_z_python_dbfs", "rms_z_error_db", "rms_z_match",
    "rms_a_firmware_dbfs", "rms_a_python_iir_dbfs", "rms_a_error_db", "rms_a_match",
    "laeq_firmware_dba", "laeq_python_dba", "laeq_error_db", "laeq_match",
    "peak_firmware_dbfs", "peak_python_dbfs", "peak_error_db", "peak_match",
    "clipped_firmware", "clipped_python", "clipped_match",
    "environment_firmware", "environment_python", "environment_match",
    "flags_firmware", "flags_python", "flags_compared_mask", "flags_match",
    "overall_match",
]

LIGHT_FEATURE_COLUMNS = [
    "window_sequence", "sample_timestamp_ms", "boot_session",
    "f1", "f2", "f3", "f4", "f5", "f6", "f7", "f8", "clear", "nir",
    "exposure_class", "exposure_label", "flags", "flags_hex",
    "flag_complete", "flag_acquisition_valid", "flag_classification_valid",
    "flag_saturated", "flag_i2c_error", "flag_smux_error",
    "flag_reserved_6", "flag_reserved_7",
    "expected_saturated", "saturation_match",
    "python_previous_exposure_class", "python_exposure_class",
    "classification_evaluated", "classification_match", "classification_reason",
    "python_flags_reconstructible",
    "flags_reconstructible_match", "reserved_flags_valid", "record_valid",
    "overall_match", "physical_page_index", "page_sequence",
    "page_timestamp_ms", "record_index_in_page", "record_payload_offset",
    "page_version", "page_header_size", "page_payload_bytes",
]

COMPACT_FEATURE_ALIGNMENT_COLUMNS = [
    "window_sequence", "audio_feature_present", "light_feature_present",
    "audio_window_start_ms", "light_sample_timestamp_ms",
    "light_minus_audio_timestamp_ms", "audio_environment_class",
    "light_exposure_class", "audio_flags", "light_flags", "pair_status",
]

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

if struct.calcsize(AUDIO_FEATURE_RECORD_FORMAT) != AUDIO_FEATURE_RECORD_SIZE:
    raise RuntimeError("Unexpected AFEA record size")

assert struct.calcsize(LIGHT_FEATURE_RECORD_FORMAT) == LIGHT_FEATURE_RECORD_SIZE

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


# ---------------------------------------------------------------------------
# Personal light exposure classification from LRAW Clear counts
# ---------------------------------------------------------------------------
# This is an empirical, wearer-relative classification. It is NOT a lux
# measurement and must not be presented as one.
#
# Update LIGHT_SENSOR_GAIN and LIGHT_SENSOR_INTEGRATION_TIME_MS whenever the
# firmware exposure settings change. The classifier rescales measured Clear
# counts to the reference exposure below; with the current 8x/27.8 ms settings
# the reference-normalized Clear value is equal to the raw Clear value except
# for the optional dark offset.
LIGHT_SENSOR_GAIN = 8.0
LIGHT_SENSOR_INTEGRATION_TIME_MS = 27.8

LIGHT_CLASSIFICATION_REFERENCE_GAIN = 8.0
LIGHT_CLASSIFICATION_REFERENCE_INTEGRATION_TIME_MS = 27.8

LIGHT_DARK_OFFSET_COUNTS = 0.0
LIGHT_ADC_FULL_SCALE_COUNTS = 10000.0
LIGHT_CLASSIFICATION_SMOOTHING_SECONDS = 1.0

LIGHT_CLASSIFICATION_THRESHOLDS_REFERENCE_COUNTS = (
    2.0,       # DARK -> LOW_EXPOSURE
    50.0,      # LOW_EXPOSURE -> MODERATE_EXPOSURE
    6500.0,    # MODERATE_EXPOSURE -> HIGH_EXPOSURE
    9800.0,    # HIGH_EXPOSURE -> VERY_HIGH_EXPOSURE
)

LIGHT_CLASSIFICATION_CATEGORIES = (
    (
        0,
        "DARK",
        "The wearer is exposed to practically no detectable light",
    ),
    (
        1,
        "LOW_EXPOSURE",
        "The wearer is exposed to a low light level",
    ),
    (
        2,
        "MODERATE_EXPOSURE",
        "The wearer is exposed to an ordinary light level",
    ),
    (
        3,
        "HIGH_EXPOSURE",
        "The wearer is exposed to a high light level",
    ),
    (
        4,
        "VERY_HIGH_EXPOSURE",
        "The wearer is exposed to a very high light level",
    ),
)

if len(LIGHT_CLASSIFICATION_THRESHOLDS_REFERENCE_COUNTS) != (
    len(LIGHT_CLASSIFICATION_CATEGORIES) - 1
):
    raise RuntimeError("Light classification thresholds must be one fewer than categories")

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
INT16_FULL_SCALE = 32768.0
AUDIO_AMPLIFIED_TARGET_DBFS = -1.0
AUDIO_LEVEL_WINDOW_MS = 50.0
AUDIO_LEVEL_HOP_MS = 25.0
AUDIO_DBFS_FLOOR = -160.0
AUDIO_PSD_SEGMENT_SAMPLES = 4096
AUDIO_PSD_OVERLAP = 0.5
AUDIO_SPECTROGRAM_NFFT = 2048
AUDIO_SPECTROGRAM_OVERLAP = 0.75
AUDIO_SPECTROGRAM_MAX_FREQUENCY_HZ = 12000.0
AUDIO_SPECTROGRAM_DYNAMIC_RANGE_DB = 80.0
AUDIO_HISTOGRAM_BINS = 200
AUDIO_MAGNITUDE_SEGMENT_SAMPLES = 4096
AUDIO_MAGNITUDE_OVERLAP = 0.5
AUDIO_MAGNITUDE_MAX_FREQUENCY_HZ = 12000.0
AUDIO_MAGNITUDE_DBFS_FLOOR = -160.0
SPL_CALIBRATION_OFFSET_DB = 122.40
SPL_CALIBRATION_REPEATABILITY_STD_DB = 0.53
SPL_CALIBRATION_FREQUENCY_HZ = 1000.0
SPL_CALIBRATION_WEIGHTING = "Z"
SPL_CALIBRATION_REFERENCE = "Smartphone sound level meter application"
SPL_CALIBRATION_APPROXIMATE = True
SPL_CALIBRATION_PHONE_LEVELS_DBZ = (59.0, 68.0, 76.0)
SPL_CALIBRATION_SENSOR_LEVELS_DBFS = (-56.763, -46.350, -40.499)
AUDIO_EMBEDDED_IIR_SAMPLE_RATE_HZ = 48000
AUDIO_EMBEDDED_IIR_BIQUADS = (
    (0.96525096525, -1.34730163086, 0.38205066561, -1.34730722798, 0.34905752979),
    (0.94696969696, -1.89393939393, 0.94696969696, -1.89387049481, 0.89515976917),
    (0.64666542810, -0.38362237137, -0.26304305672, -1.34730722798, 0.34905752979),
)
AUDIO_SOUND_LEVEL_WINDOW_S = 1.0
AUDIO_SOUND_LEVEL_HOP_S = 1.0
NIOSH_REFERENCE_LEVEL_DBA = 85.0
NIOSH_REFERENCE_DURATION_HOURS = 8.0
NIOSH_EXCHANGE_RATE_DB = 3.0
MIN_SOUND_CLASSIFICATION_DURATION_S = 0.5
SHORT_ACQUISITION_THRESHOLD_S = 10.0
SOUND_ENVIRONMENT_THRESHOLDS_DBA = (
    35.0,
    45.0,
    55.0,
    65.0,
    75.0,
    85.0,
)
SOUND_ENVIRONMENT_CATEGORIES = (
    ("very_quiet", "Very quiet", "Minimal background noise"),
    ("quiet", "Quiet", "Low background noise"),
    (
        "moderate",
        "Moderate",
        "Clearly audible but generally comfortable environment",
    ),
    (
        "lively",
        "Lively",
        "Conversations or sustained activity are clearly present",
    ),
    (
        "noisy",
        "Noisy",
        "Sustained and potentially distracting sound environment",
    ),
    ("very_noisy", "Very noisy", "Prolonged exposure should be monitored"),
    (
        "high_exposure",
        "High exposure level",
        "Potential hearing risk depending on exposure duration",
    ),
)
INVALID_SOUND_ENVIRONMENT_CATEGORY = (
    "invalid",
    "Unavailable",
    "Sound level estimate unavailable",
)

def u16_le(data: bytes) -> int:
    return struct.unpack("<H", data)[0]


def i16_le(data: bytes) -> int:
    return struct.unpack("<h", data)[0]


def _decode_db_centi(raw_value: int) -> float:
    if raw_value == AUDIO_DB_CENTI_INVALID:
        return float("nan")
    return raw_value / 100.0


def parse_audio_feature_record(record: bytes) -> dict:
    if len(record) != AUDIO_FEATURE_RECORD_SIZE:
        raise ValueError(
            f"AFEA record must be {AUDIO_FEATURE_RECORD_SIZE} bytes; got {len(record)}"
        )

    (
        window_sequence,
        window_start_ms,
        sample_count,
        mean_counts_rounded,
        rms_z_centi_dbfs,
        rms_a_centi_dbfs,
        estimated_laeq_centi_dba,
        peak_centi_dbfs,
        clipped_sample_count,
        environment_class,
        flags,
    ) = struct.unpack(AUDIO_FEATURE_RECORD_FORMAT, record)

    return {
        "window_sequence": window_sequence,
        "window_start_ms": window_start_ms,
        "sample_count": sample_count,
        "mean_counts_rounded": mean_counts_rounded,
        "rms_z_centi_dbfs": rms_z_centi_dbfs,
        "rms_z_dbfs": _decode_db_centi(rms_z_centi_dbfs),
        "rms_a_centi_dbfs": rms_a_centi_dbfs,
        "rms_a_dbfs": _decode_db_centi(rms_a_centi_dbfs),
        "estimated_laeq_centi_dba": estimated_laeq_centi_dba,
        "estimated_laeq_dba": _decode_db_centi(estimated_laeq_centi_dba),
        "peak_centi_dbfs": peak_centi_dbfs,
        "peak_dbfs": _decode_db_centi(peak_centi_dbfs),
        "clipped_sample_count": clipped_sample_count,
        "environment_class": environment_class,
        "environment_label": AUDIO_ENVIRONMENT_LABELS.get(environment_class, "UNKNOWN"),
        "flags": flags,
        "flag_complete": bool(flags & AUDIO_FLAG_COMPLETE),
        "flag_acquisition_valid": bool(flags & AUDIO_FLAG_ACQUISITION_VALID),
        "flag_a_weighted_valid": bool(flags & AUDIO_FLAG_A_WEIGHTED_VALID),
        "flag_clipped": bool(flags & AUDIO_FLAG_CLIPPED),
        "flag_high_level": bool(flags & AUDIO_FLAG_HIGH_LEVEL),
        "flag_silent_or_unavailable": bool(flags & AUDIO_FLAG_SILENT_OR_UNAVAILABLE),
        "flag_impulsive_event": bool(flags & AUDIO_FLAG_IMPULSIVE_EVENT),
        "flag_reserved": bool(flags & AUDIO_FLAG_RESERVED),
    }


def parse_light_feature_record(record: bytes) -> dict:
    if len(record) != LIGHT_FEATURE_RECORD_SIZE:
        raise ValueError(
            f"LFEA record must be {LIGHT_FEATURE_RECORD_SIZE} bytes; got {len(record)}"
        )

    (
        window_sequence,
        sample_timestamp_ms,
        f1,
        f2,
        f3,
        f4,
        f5,
        f6,
        f7,
        f8,
        clear,
        nir,
        exposure_class,
        flags,
    ) = struct.unpack(LIGHT_FEATURE_RECORD_FORMAT, record)

    return {
        "window_sequence": window_sequence,
        "sample_timestamp_ms": sample_timestamp_ms,
        "f1": f1,
        "f2": f2,
        "f3": f3,
        "f4": f4,
        "f5": f5,
        "f6": f6,
        "f7": f7,
        "f8": f8,
        "clear": clear,
        "nir": nir,
        "exposure_class": exposure_class,
        "exposure_label": LIGHT_EXPOSURE_LABELS.get(exposure_class, "UNKNOWN"),
        "flags": flags,
        "flags_hex": f"0x{flags:02X}",
        "flag_complete": bool(flags & LIGHT_FLAG_COMPLETE),
        "flag_acquisition_valid": bool(flags & LIGHT_FLAG_ACQUISITION_VALID),
        "flag_classification_valid": bool(flags & LIGHT_FLAG_CLASSIFICATION_VALID),
        "flag_saturated": bool(flags & LIGHT_FLAG_SATURATED),
        "flag_i2c_error": bool(flags & LIGHT_FLAG_I2C_ERROR),
        "flag_smux_error": bool(flags & LIGHT_FLAG_SMUX_ERROR),
        "flag_reserved_6": bool(flags & LIGHT_FLAG_RESERVED_6),
        "flag_reserved_7": bool(flags & LIGHT_FLAG_RESERVED_7),
    }


def light_hysteresis_return_threshold(threshold: int) -> int:
    return_margin = max(1, (threshold * 5 + 99) // 100)
    return threshold - return_margin


def classify_light_with_hysteresis(
    clear_counts: int,
    previous_class: int | None,
) -> int:
    if previous_class is None:
        classification = 0
        while (
            classification < 4
            and clear_counts >= LIGHT_FEATURE_BOUNDARY_THRESHOLDS[classification]
        ):
            classification += 1
    else:
        if previous_class not in range(5):
            raise ValueError(f"Invalid previous light exposure class: {previous_class}")
        classification = previous_class

        while (
            classification < 4
            and clear_counts >= LIGHT_FEATURE_BOUNDARY_THRESHOLDS[classification]
        ):
            classification += 1

        while classification > 0:
            threshold = LIGHT_FEATURE_BOUNDARY_THRESHOLDS[classification - 1]
            return_threshold = light_hysteresis_return_threshold(threshold)
            if clear_counts >= return_threshold:
                break
            classification -= 1

    if clear_counts >= LIGHT_SATURATION_CLEAR_COUNTS:
        classification = 4

    return classification


def classify_light_without_history(clear_counts: int) -> int | None:
    """Return a class only when Clear determines it without prior state."""
    classification = classify_light_with_hysteresis(clear_counts, None)
    if classification >= 4:
        return classification

    next_threshold = LIGHT_FEATURE_BOUNDARY_THRESHOLDS[classification]
    if clear_counts >= light_hysteresis_return_threshold(next_threshold):
        return None
    return classification


def round_half_away_from_zero(value: float) -> int:
    if not np.isfinite(value):
        raise ValueError("Cannot round a non-finite value")
    if value >= 0.0:
        return int(np.floor(value + 0.5))
    return int(np.ceil(value - 0.5))


def encode_db_centi_like_firmware(value: float) -> int:
    if not np.isfinite(value):
        return AUDIO_DB_CENTI_INVALID
    rounded = round_half_away_from_zero(float(value) * 100.0)
    return min(32767, max(-32767, rounded))


def encode_mean_counts_like_firmware(value: float) -> int:
    if not np.isfinite(value):
        return 0
    rounded = round_half_away_from_zero(float(value))
    return min(32767, max(-32768, rounded))


def compute_audio_window_basic_metrics(audio: np.ndarray) -> dict:
    samples = np.asarray(audio, dtype=np.int16)
    if samples.size == 0:
        raise ValueError("Cannot compute audio metrics from an empty window")

    x = samples.astype(np.float64)
    mean_counts = float(np.mean(x))
    zero_mean = x - mean_counts
    rms_counts = float(np.sqrt(np.mean(zero_mean ** 2)))
    rms_dbfs = (
        float(20.0 * np.log10(rms_counts / INT16_FULL_SCALE))
        if rms_counts > 0.0
        else float("-inf")
    )
    absolute_peak_counts = float(np.max(np.abs(x)))
    peak_dbfs = (
        float(20.0 * np.log10(absolute_peak_counts / INT16_FULL_SCALE))
        if absolute_peak_counts > 0.0
        else float("-inf")
    )
    clipped = (samples == -32768) | (samples == 32767)
    clipped_sample_count = int(np.count_nonzero(clipped))

    return {
        "mean_counts": mean_counts,
        "mean_counts_rounded": encode_mean_counts_like_firmware(mean_counts),
        "rms_zero_mean_counts": rms_counts,
        "rms_zero_mean_dbfs": rms_dbfs,
        "absolute_peak_counts": int(absolute_peak_counts),
        "peak_dbfs": peak_dbfs,
        "clipped_sample_count": clipped_sample_count,
        "clipped_sample_percentage": 100.0 * clipped_sample_count / samples.size,
    }


def compute_embedded_iir_a_weighted_metrics(
    audio: np.ndarray,
    sample_rate_hz: int,
) -> dict:
    if sample_rate_hz != AUDIO_EMBEDDED_IIR_SAMPLE_RATE_HZ:
        raise ValueError(
            "Embedded A-weighting comparison requires exactly "
            f"{AUDIO_EMBEDDED_IIR_SAMPLE_RATE_HZ} Hz; got {sample_rate_hz} Hz"
        )

    samples = np.asarray(audio, dtype=np.int16)
    if samples.size == 0:
        raise ValueError("Cannot apply A-weighting to an empty audio window")

    x = samples.astype(np.float64)
    mean_counts = float(np.mean(x))
    states = [[0.0, 0.0] for _ in AUDIO_EMBEDDED_IIR_BIQUADS]
    weighted_energy = 0.0

    # DF-II transposed, denominator 1 + a1*z^-1 + a2*z^-2.
    for sample in x:
        section_input = float(sample - mean_counts)
        for section_index, (b0, b1, b2, a1, a2) in enumerate(
            AUDIO_EMBEDDED_IIR_BIQUADS
        ):
            s1, s2 = states[section_index]
            output = b0 * section_input + s1
            next_s1 = b1 * section_input - a1 * output + s2
            next_s2 = b2 * section_input - a2 * output
            if not all(np.isfinite(value) for value in (output, next_s1, next_s2)):
                raise ValueError("Non-finite state in embedded A-weighting cascade")
            states[section_index] = [next_s1, next_s2]
            section_input = output
        weighted_energy += section_input * section_input
        if not np.isfinite(weighted_energy) or weighted_energy < 0.0:
            raise ValueError("Invalid accumulated A-weighted energy")

    rms_counts = float(np.sqrt(weighted_energy / samples.size))
    rms_dbfs = (
        float(20.0 * np.log10(rms_counts / INT16_FULL_SCALE))
        if rms_counts > 0.0
        else float("-inf")
    )
    estimated_laeq_dba = (
        rms_dbfs + SPL_CALIBRATION_OFFSET_DB
        if np.isfinite(rms_dbfs)
        else float("-inf")
    )
    return {
        "a_weighted_rms_counts": rms_counts,
        "a_weighted_rms_dbfs": rms_dbfs,
        "estimated_laeq_dba": estimated_laeq_dba,
    }


def classify_embedded_audio_environment(estimated_laeq_dba: float) -> int:
    if not np.isfinite(estimated_laeq_dba):
        return 255
    for class_index, threshold in enumerate(SOUND_ENVIRONMENT_THRESHOLDS_DBA):
        if estimated_laeq_dba < threshold:
            return class_index
    return 6


def build_expected_audio_flags(
    *,
    complete: bool,
    acquisition_valid: bool,
    a_weighting_valid: bool,
    clipped_sample_count: int,
    estimated_laeq_dba: float,
) -> int:
    flags = 0
    if complete:
        flags |= AUDIO_FLAG_COMPLETE
    if acquisition_valid:
        flags |= AUDIO_FLAG_ACQUISITION_VALID
    if a_weighting_valid:
        flags |= AUDIO_FLAG_A_WEIGHTED_VALID
    if clipped_sample_count > 0:
        flags |= AUDIO_FLAG_CLIPPED
    if np.isfinite(estimated_laeq_dba) and estimated_laeq_dba >= 85.0:
        flags |= AUDIO_FLAG_HIGH_LEVEL
    if not a_weighting_valid:
        flags |= AUDIO_FLAG_SILENT_OR_UNAVAILABLE
    return flags


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


@dataclass
class AudioMetrics:
    sample_rate_hz: int
    sample_count: int
    duration_s: float
    minimum_sample: int | None
    maximum_sample: int | None
    mean_sample: float
    dc_offset_counts: float
    rms_counts: float
    rms_dbfs: float
    absolute_peak_counts: float
    peak_dbfs: float
    crest_factor: float
    crest_factor_db: float
    clipped_sample_count: int
    clipped_sample_percentage: float
    amplification_gain_linear: float
    amplification_gain_db: float
    amplified_target_peak_dbfs: float


@dataclass
class EstimatedSoundLevelMetrics:
    calibration_offset_db: float
    calibration_repeatability_std_db: float
    calibration_frequency_hz: float
    calibration_weighting: str
    calibration_reference: str
    calibration_approximate: bool
    unweighted_rms_dbfs: float
    a_weighted_rms_dbfs: float
    estimated_lzeq_dbz: float
    estimated_laeq_dba: float


@dataclass
class SoundEnvironmentAssessment:
    estimated_laeq_dba: float
    acquisition_duration_s: float
    category_code: str
    category_label: str
    category_description: str
    niosh_max_exposure_hours: float
    niosh_max_exposure_minutes: float
    niosh_recorded_dose_percent: float
    niosh_reference_level_dba: float
    niosh_exchange_rate_db: float
    warning_level: str
    warning_message: str
    short_acquisition: bool
    estimate_valid: bool


@dataclass
class LightEnvironmentAssessment:
    sample_count: int
    acquisition_duration_s: float
    smoothing_window_samples: int
    median_clear_counts: float
    median_reference_clear_counts: float
    median_ambient_light_score: float
    dominant_class_code: int
    dominant_class_label: str
    dominant_class_description: str
    saturated_sample_count: int
    saturated_sample_percentage: float
    class_counts: dict[str, int]
    class_percentages: dict[str, float]


def write_wav_int16_mono(filename: Path, pcm_bytes: bytes, sample_rate_hz: int) -> None:
    with wave.open(str(filename), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate_hz)
        wav.writeframes(pcm_bytes)


def decode_audio_int16(audio_bytes: bytes) -> np.ndarray:
    """Decode raw PCM bytes as signed little-endian int16 mono samples."""
    if len(audio_bytes) % 2 != 0:
        print("Warning: audio byte count is odd; last byte ignored.")
        audio_bytes = audio_bytes[:-1]
    return np.frombuffer(audio_bytes, dtype="<i2")


def _db20_from_ratio(ratio: float) -> float:
    if ratio <= 0.0 or not np.isfinite(ratio):
        return float("-inf")
    return float(20.0 * np.log10(ratio))


def _db10_from_power(power: np.ndarray) -> np.ndarray:
    result = np.full(power.shape, float("-inf"), dtype=np.float64)
    positive = power > 0.0
    result[positive] = 10.0 * np.log10(power[positive])
    return result


def _format_db(value: float, unit: str = "dBFS") -> str:
    if np.isneginf(value):
        return f"-inf {unit}"
    if np.isposinf(value):
        return f"inf {unit}"
    if np.isnan(value):
        return f"nan {unit}"
    return f"{value:.3f} {unit}"


def _format_number(value: float | int | None, digits: int = 3) -> str:
    if value is None:
        return "None"
    if isinstance(value, (int, np.integer)):
        return str(int(value))
    if np.isneginf(value):
        return "-inf"
    if np.isposinf(value):
        return "inf"
    if np.isnan(value):
        return "nan"
    return f"{float(value):.{digits}f}"


def compute_audio_metrics(audio: np.ndarray, sample_rate_hz: int) -> AudioMetrics:
    """Compute global metrics from raw int16 audio samples."""
    sample_count = int(len(audio))
    duration_s = sample_count / float(sample_rate_hz) if sample_rate_hz > 0 else 0.0

    if sample_count == 0:
        target_peak_counts = 32767.0 * (10.0 ** (AUDIO_AMPLIFIED_TARGET_DBFS / 20.0))
        return AudioMetrics(
            sample_rate_hz=sample_rate_hz,
            sample_count=0,
            duration_s=duration_s,
            minimum_sample=None,
            maximum_sample=None,
            mean_sample=0.0,
            dc_offset_counts=0.0,
            rms_counts=0.0,
            rms_dbfs=float("-inf"),
            absolute_peak_counts=0.0,
            peak_dbfs=float("-inf"),
            crest_factor=0.0,
            crest_factor_db=float("-inf"),
            clipped_sample_count=0,
            clipped_sample_percentage=0.0,
            amplification_gain_linear=1.0,
            amplification_gain_db=0.0,
            amplified_target_peak_dbfs=AUDIO_AMPLIFIED_TARGET_DBFS,
        )

    audio_float = audio.astype(np.float64)
    minimum_sample = int(audio.min())
    maximum_sample = int(audio.max())
    mean_sample = float(np.mean(audio_float))
    rms_counts = float(np.sqrt(np.mean(audio_float ** 2)))
    absolute_peak_counts = float(np.max(np.abs(audio_float)))

    rms_dbfs = _db20_from_ratio(rms_counts / INT16_FULL_SCALE)
    peak_dbfs = _db20_from_ratio(absolute_peak_counts / INT16_FULL_SCALE)
    if rms_counts > 0.0 and absolute_peak_counts > 0.0:
        crest_factor = float(absolute_peak_counts / rms_counts)
        crest_factor_db = _db20_from_ratio(crest_factor)
    else:
        crest_factor = 0.0
        crest_factor_db = float("-inf")

    clipped_mask = (audio <= -32768) | (audio >= 32767)
    clipped_sample_count = int(np.count_nonzero(clipped_mask))
    clipped_sample_percentage = 100.0 * clipped_sample_count / sample_count

    target_peak_counts = 32767.0 * (10.0 ** (AUDIO_AMPLIFIED_TARGET_DBFS / 20.0))
    if absolute_peak_counts > 0.0:
        gain = float(target_peak_counts / absolute_peak_counts)
    else:
        gain = 1.0
    gain_db = _db20_from_ratio(gain)

    return AudioMetrics(
        sample_rate_hz=sample_rate_hz,
        sample_count=sample_count,
        duration_s=duration_s,
        minimum_sample=minimum_sample,
        maximum_sample=maximum_sample,
        mean_sample=mean_sample,
        dc_offset_counts=mean_sample,
        rms_counts=rms_counts,
        rms_dbfs=rms_dbfs,
        absolute_peak_counts=absolute_peak_counts,
        peak_dbfs=peak_dbfs,
        crest_factor=crest_factor,
        crest_factor_db=crest_factor_db,
        clipped_sample_count=clipped_sample_count,
        clipped_sample_percentage=clipped_sample_percentage,
        amplification_gain_linear=gain,
        amplification_gain_db=gain_db,
        amplified_target_peak_dbfs=AUDIO_AMPLIFIED_TARGET_DBFS,
    )


def _metrics_report_lines(metrics: AudioMetrics) -> list[str]:
    return [
        "Audio metrics",
        f"Audio sample rate [Hz]: {metrics.sample_rate_hz}",
        f"Audio sample count: {metrics.sample_count}",
        f"Audio duration [s]: {metrics.duration_s:.6f}",
        "",
        f"Minimum sample [counts]: {_format_number(metrics.minimum_sample)}",
        f"Maximum sample [counts]: {_format_number(metrics.maximum_sample)}",
        f"Mean sample / DC offset [counts]: {metrics.mean_sample:.6f}",
        "",
        f"RMS amplitude [counts]: {metrics.rms_counts:.6f}",
        f"RMS level [dBFS]: {_format_db(metrics.rms_dbfs)}",
        "",
        f"Absolute peak [counts]: {metrics.absolute_peak_counts:.6f}",
        f"Peak level [dBFS]: {_format_db(metrics.peak_dbfs)}",
        "",
        f"Crest factor [linear]: {metrics.crest_factor:.6f}",
        f"Crest factor [dB]: {_format_db(metrics.crest_factor_db, 'dB')}",
        "",
        f"Clipped samples: {metrics.clipped_sample_count}",
        f"Clipped samples [%]: {metrics.clipped_sample_percentage:.6f}",
        "",
        f"Amplified WAV target peak [dBFS]: {_format_db(metrics.amplified_target_peak_dbfs)}",
        f"Applied gain [linear]: {metrics.amplification_gain_linear:.9f}",
        f"Applied gain [dB]: {_format_db(metrics.amplification_gain_db, 'dB')}",
    ]


def write_audio_metrics_report(
    report_filename: Path,
    metrics: AudioMetrics,
    sound_metrics: EstimatedSoundLevelMetrics | None = None,
    sound_assessment: SoundEnvironmentAssessment | None = None,
) -> None:
    """Write a standalone text report with raw-signal audio metrics."""
    lines = _metrics_report_lines(metrics)
    if sound_metrics is not None:
        lines.extend(_sound_level_report_lines(sound_metrics))
    if sound_assessment is not None:
        lines.extend(_sound_environment_assessment_report_lines(sound_assessment))
    report_filename.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Audio metrics report saved to: {report_filename}")


def append_audio_metrics_to_summary(
    summary_filename: Path,
    metrics: AudioMetrics,
    sound_metrics: EstimatedSoundLevelMetrics | None = None,
    sound_assessment: SoundEnvironmentAssessment | None = None,
) -> None:
    """Append the audio metrics section to the existing parser summary."""
    lines = ["", "Audio metrics summary"]
    lines.extend(_metrics_report_lines(metrics)[1:])
    if sound_metrics is not None:
        lines.extend(
            [
                "",
                "Estimated acoustic sound levels summary",
                f"Calibration offset: {sound_metrics.calibration_offset_db:.2f} dB",
                (
                    "Calibration uncertainty from repeatability: "
                    f"{sound_metrics.calibration_repeatability_std_db:.2f} dB"
                ),
                (
                    "Estimated global LZeq [dBZ]: "
                    f"{_format_db(sound_metrics.estimated_lzeq_dbz, 'dBZ')}"
                ),
                (
                    "Estimated global LAeq [dBA]: "
                    f"{_format_db(sound_metrics.estimated_laeq_dba, 'dBA')}"
                ),
                "Calibration status: approximate"
                if sound_metrics.calibration_approximate
                else "Calibration status: certified",
            ]
        )
    if sound_assessment is not None:
        lines.extend(
            [
                "",
                "Sound environment classification",
                (
                    "Estimated LAeq [dBA]: "
                    f"{_format_db(sound_assessment.estimated_laeq_dba, 'dBA')}"
                ),
                f"Environment category: {sound_assessment.category_label}",
                (
                    "Indicative NIOSH maximum exposure time: "
                    f"{format_exposure_duration(sound_assessment.niosh_max_exposure_hours)}"
                ),
                (
                    "Recorded dose [%]: "
                    f"{_format_number(sound_assessment.niosh_recorded_dose_percent, 3)}"
                ),
                f"Warning level: {sound_assessment.warning_level}",
                f"Warning message: {sound_assessment.warning_message}",
                (
                    "Short acquisition status: "
                    f"{sound_assessment.short_acquisition}"
                ),
            ]
        )
    with summary_filename.open("a", encoding="utf-8") as summary_file:
        summary_file.write("\n".join(lines) + "\n")


def print_audio_metrics(metrics: AudioMetrics) -> None:
    """Print the main audio metrics to the terminal."""
    print("Audio metrics:")
    print(f"  Duration: {metrics.duration_s:.6f} s")
    print(f"  Samples: {metrics.sample_count}")
    print(f"  RMS: {metrics.rms_counts:.3f} counts ({_format_db(metrics.rms_dbfs)})")
    print(f"  Peak: {metrics.absolute_peak_counts:.3f} counts ({_format_db(metrics.peak_dbfs)})")
    print(
        "  Amplified WAV gain: "
        f"{metrics.amplification_gain_linear:.9f} ({_format_db(metrics.amplification_gain_db, 'dB')})"
    )
    print(
        "  Clipped samples: "
        f"{metrics.clipped_sample_count} ({metrics.clipped_sample_percentage:.6f}%)"
    )


def a_weighting_db(frequencies_hz: np.ndarray) -> np.ndarray:
    """Return the standard A-weighting response in dB for each frequency."""
    frequencies = np.asarray(frequencies_hz, dtype=np.float64)
    f_squared = frequencies ** 2
    f1 = 20.598997
    f2 = 107.65265
    f3 = 737.86223
    f4 = 12194.217

    with np.errstate(divide="ignore", invalid="ignore"):
        numerator = (f4 ** 2) * (f_squared ** 2)
        denominator = (
            (f_squared + f1 ** 2)
            * np.sqrt((f_squared + f2 ** 2) * (f_squared + f3 ** 2))
            * (f_squared + f4 ** 2)
        )
        response = numerator / denominator
        weighting = 20.0 * np.log10(response) + 2.0

    weighting = np.asarray(weighting, dtype=np.float64)
    weighting[frequencies <= 0.0] = float("-inf")
    return weighting


def _normalized_zero_mean_audio(audio: np.ndarray) -> np.ndarray:
    if len(audio) == 0:
        return np.array([], dtype=np.float64)
    x = audio.astype(np.float64) / INT16_FULL_SCALE
    return x - float(np.mean(x))


def compute_unweighted_rms_dbfs(audio: np.ndarray) -> tuple[float, float]:
    """Compute zero-mean RMS on full-scale normalized audio."""
    x = _normalized_zero_mean_audio(audio)
    if len(x) == 0:
        return 0.0, float("-inf")
    rms = float(np.sqrt(np.mean(x ** 2)))
    return rms, _db20_from_ratio(rms)


def compute_a_weighted_rms_dbfs(
    audio: np.ndarray,
    sample_rate_hz: int,
) -> tuple[float, float]:
    """Compute A-weighted RMS using an FFT, A-power weights and Parseval."""
    sample_count = len(audio)
    if sample_count == 0 or sample_rate_hz <= 0:
        return 0.0, float("-inf")

    x = _normalized_zero_mean_audio(audio)
    if len(x) == 0:
        return 0.0, float("-inf")

    spectrum = np.fft.rfft(x)
    frequencies = np.fft.rfftfreq(
        x.size,
        d=1.0 / float(sample_rate_hz),
    )
    weighting_db = a_weighting_db(frequencies)
    weighting_power = np.zeros_like(weighting_db, dtype=np.float64)
    finite_weighting = np.isfinite(weighting_db)
    weighting_power[finite_weighting] = 10.0 ** (weighting_db[finite_weighting] / 10.0)

    one_sided_factor = np.ones_like(weighting_power, dtype=np.float64)
    if x.size > 1:
        if x.size % 2 == 0:
            one_sided_factor[1:-1] = 2.0
        else:
            one_sided_factor[1:] = 2.0

    power_bins = np.abs(spectrum) ** 2
    mean_square_a = float(
        np.sum(one_sided_factor * power_bins * weighting_power) / float(x.size ** 2)
    )
    if mean_square_a <= 0.0 or not np.isfinite(mean_square_a):
        return 0.0, float("-inf")

    rms_a = float(np.sqrt(mean_square_a))
    return rms_a, _db20_from_ratio(rms_a)


def compute_estimated_sound_level_metrics(
    audio: np.ndarray,
    sample_rate_hz: int,
) -> EstimatedSoundLevelMetrics:
    """Compute global approximate Z and A sound-level estimates."""
    _, unweighted_rms_dbfs = compute_unweighted_rms_dbfs(audio)
    _, a_weighted_rms_dbfs = compute_a_weighted_rms_dbfs(audio, sample_rate_hz)

    estimated_lzeq_dbz = (
        unweighted_rms_dbfs + SPL_CALIBRATION_OFFSET_DB
        if np.isfinite(unweighted_rms_dbfs)
        else float("-inf")
    )
    estimated_laeq_dba = (
        a_weighted_rms_dbfs + SPL_CALIBRATION_OFFSET_DB
        if np.isfinite(a_weighted_rms_dbfs)
        else float("-inf")
    )

    return EstimatedSoundLevelMetrics(
        calibration_offset_db=SPL_CALIBRATION_OFFSET_DB,
        calibration_repeatability_std_db=SPL_CALIBRATION_REPEATABILITY_STD_DB,
        calibration_frequency_hz=SPL_CALIBRATION_FREQUENCY_HZ,
        calibration_weighting=SPL_CALIBRATION_WEIGHTING,
        calibration_reference=SPL_CALIBRATION_REFERENCE,
        calibration_approximate=SPL_CALIBRATION_APPROXIMATE,
        unweighted_rms_dbfs=unweighted_rms_dbfs,
        a_weighted_rms_dbfs=a_weighted_rms_dbfs,
        estimated_lzeq_dbz=estimated_lzeq_dbz,
        estimated_laeq_dba=estimated_laeq_dba,
    )


def compute_estimated_sound_level_timeseries(
    audio: np.ndarray,
    sample_rate_hz: int,
    window_s: float = AUDIO_SOUND_LEVEL_WINDOW_S,
    hop_s: float = AUDIO_SOUND_LEVEL_HOP_S,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute 1-second estimated sound-level features; final partial windows are ignored."""
    sample_count = len(audio)
    if sample_count == 0 or sample_rate_hz <= 0:
        empty = np.array([], dtype=np.float64)
        return empty, empty, empty, empty, empty

    window_samples = int(round(float(window_s) * float(sample_rate_hz)))
    hop_samples = int(round(float(hop_s) * float(sample_rate_hz)))
    if window_samples <= 0 or hop_samples <= 0:
        empty = np.array([], dtype=np.float64)
        return empty, empty, empty, empty, empty

    window_samples = min(window_samples, sample_count)
    starts = list(range(0, sample_count - window_samples + 1, hop_samples))
    if not starts:
        starts = [0]

    times = np.empty(len(starts), dtype=np.float64)
    unweighted_levels = np.empty(len(starts), dtype=np.float64)
    a_weighted_levels = np.empty(len(starts), dtype=np.float64)
    estimated_lzeq = np.empty(len(starts), dtype=np.float64)
    estimated_laeq = np.empty(len(starts), dtype=np.float64)

    for index, start in enumerate(starts):
        frame = audio[start:start + window_samples]
        times[index] = (start + (len(frame) / 2.0)) / float(sample_rate_hz)
        _, z_dbfs = compute_unweighted_rms_dbfs(frame)
        _, a_dbfs = compute_a_weighted_rms_dbfs(frame, sample_rate_hz)
        unweighted_levels[index] = z_dbfs
        a_weighted_levels[index] = a_dbfs
        estimated_lzeq[index] = (
            z_dbfs + SPL_CALIBRATION_OFFSET_DB if np.isfinite(z_dbfs) else float("-inf")
        )
        estimated_laeq[index] = (
            a_dbfs + SPL_CALIBRATION_OFFSET_DB if np.isfinite(a_dbfs) else float("-inf")
        )

    return times, unweighted_levels, a_weighted_levels, estimated_lzeq, estimated_laeq


def classify_sound_environment(
    estimated_laeq_dba: float,
) -> tuple[str, str, str]:
    """Classify the acoustic environment from an estimated global LAeq value."""
    if not np.isfinite(estimated_laeq_dba) or estimated_laeq_dba < 0.0:
        return INVALID_SOUND_ENVIRONMENT_CATEGORY

    for threshold, category in zip(
        SOUND_ENVIRONMENT_THRESHOLDS_DBA,
        SOUND_ENVIRONMENT_CATEGORIES,
    ):
        if estimated_laeq_dba < threshold:
            return category
    return SOUND_ENVIRONMENT_CATEGORIES[-1]


def compute_niosh_max_exposure_hours(laeq_dba: float) -> float:
    """Compute the indicative NIOSH REL maximum exposure duration in hours."""
    if not np.isfinite(laeq_dba) or laeq_dba < 0.0:
        return float("nan")

    try:
        return float(
            NIOSH_REFERENCE_DURATION_HOURS
            * 2.0 ** (
                (NIOSH_REFERENCE_LEVEL_DBA - laeq_dba)
                / NIOSH_EXCHANGE_RATE_DB
            )
        )
    except OverflowError:
        return float("inf")


def compute_niosh_dose_percent(
    acquisition_duration_s: float,
    max_exposure_hours: float,
) -> float:
    """Compute the indicative NIOSH dose accumulated during the recorded duration."""
    if not np.isfinite(acquisition_duration_s) or acquisition_duration_s <= 0.0:
        return 0.0
    if not np.isfinite(max_exposure_hours):
        return 0.0 if np.isposinf(max_exposure_hours) else float("nan")
    if max_exposure_hours <= 0.0:
        return float("inf")

    acquisition_duration_hours = acquisition_duration_s / 3600.0
    return float(acquisition_duration_hours / max_exposure_hours * 100.0)


def format_exposure_duration(duration_hours: float) -> str:
    """Format an exposure duration with compact human-readable units."""
    if not np.isfinite(duration_hours) or duration_hours <= 0.0:
        return "unavailable"
    if duration_hours > 24.0:
        return "more than 24 h"

    duration_minutes = duration_hours * 60.0
    if duration_minutes < 1.0:
        return "less than 1 min"
    if duration_hours < 1.0:
        rounded_minutes = max(1, int(round(duration_minutes)))
        return f"{rounded_minutes} min"

    rounded_hours = round(duration_hours)
    if abs(duration_hours - rounded_hours) < 0.05:
        return f"{int(rounded_hours)} h"
    if duration_hours < 10.0:
        return f"{duration_hours:.1f} h"
    return f"{duration_hours:.0f} h"


def build_niosh_warning(
    laeq_dba: float,
    max_exposure_hours: float,
    acquisition_duration_s: float,
) -> tuple[str, str]:
    """Build a compact NIOSH-style warning for the estimated LAeq."""
    if not np.isfinite(laeq_dba) or laeq_dba < 0.0:
        return "unavailable", "Sound exposure warning unavailable."

    formatted_time = format_exposure_duration(max_exposure_hours)
    if laeq_dba < 70.0:
        return "low", "Low sound exposure level under ordinary conditions."
    if laeq_dba < 82.0:
        return (
            "notice",
            (
                "Sustained exposure should be monitored, although this level is "
                "below the main NIOSH 85 dBA reference."
            ),
        )
    if laeq_dba < 85.0:
        return (
            "caution",
            (
                "If this level remains constant, avoid exposure longer than "
                f"approximately {formatted_time}."
            ),
        )
    if laeq_dba < 94.0:
        return (
            "warning",
            (
                "Potential hearing risk with prolonged exposure. If this level "
                "remains constant, limit exposure to approximately "
                f"{formatted_time}."
            ),
        )
    if laeq_dba <= 100.0:
        return (
            "high",
            (
                "High sound exposure. If this level remains constant, limit "
                f"exposure to approximately {formatted_time}."
            ),
        )
    return (
        "critical",
        (
            "Very high sound exposure. Reduce exposure and move away from the "
            "source when possible. Indicative maximum duration: "
            f"{formatted_time}."
        ),
    )


def assess_sound_environment(
    estimated_laeq_dba: float,
    acquisition_duration_s: float,
) -> SoundEnvironmentAssessment:
    """Assess acoustic environment class, NIOSH exposure time and recorded dose."""
    level_valid = np.isfinite(estimated_laeq_dba) and estimated_laeq_dba >= 0.0
    duration_valid = (
        np.isfinite(acquisition_duration_s)
        and acquisition_duration_s >= MIN_SOUND_CLASSIFICATION_DURATION_S
    )
    estimate_valid = bool(level_valid and duration_valid)
    short_acquisition = bool(acquisition_duration_s < SHORT_ACQUISITION_THRESHOLD_S)

    if level_valid:
        category_code, category_label, category_description = classify_sound_environment(
            estimated_laeq_dba
        )
        max_exposure_hours = compute_niosh_max_exposure_hours(estimated_laeq_dba)
    else:
        category_code, category_label, category_description = (
            INVALID_SOUND_ENVIRONMENT_CATEGORY
        )
        max_exposure_hours = float("nan")

    recorded_dose_percent = compute_niosh_dose_percent(
        acquisition_duration_s,
        max_exposure_hours,
    )
    warning_level, warning_message = build_niosh_warning(
        estimated_laeq_dba,
        max_exposure_hours,
        acquisition_duration_s,
    )

    if level_valid and not duration_valid:
        warning_level = "unavailable"
        warning_message = (
            "Acquisition shorter than 0.5 s: sound level estimate is not "
            "sufficiently stable."
        )
    elif estimate_valid and short_acquisition:
        warning_message = (
            warning_message
            + " Short acquisition: result represents a brief acoustic snapshot."
        )

    return SoundEnvironmentAssessment(
        estimated_laeq_dba=estimated_laeq_dba,
        acquisition_duration_s=acquisition_duration_s,
        category_code=category_code,
        category_label=category_label,
        category_description=category_description,
        niosh_max_exposure_hours=max_exposure_hours,
        niosh_max_exposure_minutes=max_exposure_hours * 60.0,
        niosh_recorded_dose_percent=recorded_dose_percent,
        niosh_reference_level_dba=NIOSH_REFERENCE_LEVEL_DBA,
        niosh_exchange_rate_db=NIOSH_EXCHANGE_RATE_DB,
        warning_level=warning_level,
        warning_message=warning_message,
        short_acquisition=short_acquisition,
        estimate_valid=estimate_valid,
    )


def write_estimated_sound_level_csv(
    filename: Path,
    time_center_s: np.ndarray,
    unweighted_level_dbfs: np.ndarray,
    a_weighted_level_dbfs: np.ndarray,
    estimated_lzeq_dbz: np.ndarray,
    estimated_laeq_dba: np.ndarray,
) -> bool:
    """Write 1-second estimated sound-level features without raw PCM samples."""
    if len(time_center_s) == 0:
        print("No sound-level time windows available; skipping sound-level CSV.")
        return False

    fieldnames = [
        "time_center_s",
        "unweighted_rms_dbfs",
        "a_weighted_rms_dbfs",
        "estimated_lzeq_dbz",
        "estimated_laeq_dba",
        "environment_category",
        "environment_warning_level",
        "niosh_max_exposure_minutes",
        "calibration_offset_db",
        "calibration_approximate",
    ]
    with filename.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row_index in range(len(time_center_s)):
            row_laeq_dba = float(estimated_laeq_dba[row_index])
            environment_category, _, _ = classify_sound_environment(row_laeq_dba)
            max_exposure_hours = compute_niosh_max_exposure_hours(row_laeq_dba)
            warning_level, _ = build_niosh_warning(
                row_laeq_dba,
                max_exposure_hours,
                AUDIO_SOUND_LEVEL_WINDOW_S,
            )
            max_exposure_minutes = max_exposure_hours * 60.0
            writer.writerow(
                {
                    "time_center_s": f"{time_center_s[row_index]:.6f}",
                    "unweighted_rms_dbfs": _format_number(unweighted_level_dbfs[row_index], 6),
                    "a_weighted_rms_dbfs": _format_number(a_weighted_level_dbfs[row_index], 6),
                    "estimated_lzeq_dbz": _format_number(estimated_lzeq_dbz[row_index], 6),
                    "estimated_laeq_dba": _format_number(estimated_laeq_dba[row_index], 6),
                    "environment_category": environment_category,
                    "environment_warning_level": warning_level,
                    "niosh_max_exposure_minutes": _format_number(max_exposure_minutes, 6),
                    "calibration_offset_db": f"{SPL_CALIBRATION_OFFSET_DB:.2f}",
                    "calibration_approximate": str(SPL_CALIBRATION_APPROXIMATE),
                }
            )

    print(f"Audio sound-level CSV saved to: {filename}")
    return True


def plot_estimated_sound_level_timeseries(
    filename: Path,
    time_center_s: np.ndarray,
    estimated_lzeq_dbz: np.ndarray,
    estimated_laeq_dba: np.ndarray,
) -> Path | None:
    """Plot approximate estimated Z and A sound pressure levels over time."""
    if len(time_center_s) == 0:
        print("No sound-level time windows available; skipping estimated sound-level plot.")
        return None

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(time_center_s, estimated_lzeq_dbz, label="Estimated LZeq [dBZ]", linewidth=1.2)
    ax.plot(time_center_s, estimated_laeq_dba, label="Estimated LAeq [dBA]", linewidth=1.2)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Estimated sound pressure level [dB]")
    ax.set_title("Estimated sound pressure level over time")
    ax.grid(True)
    ax.legend()
    info_text = (
        "Approximate smartphone calibration\n"
        f"Offset: {SPL_CALIBRATION_OFFSET_DB:.2f} dB\n"
        f"Reference weighting: {SPL_CALIBRATION_WEIGHTING}\n"
        f"Calibration frequency: {SPL_CALIBRATION_FREQUENCY_HZ:.0f} Hz"
    )
    ax.text(
        0.01,
        0.02,
        info_text,
        transform=ax.transAxes,
        fontsize=8,
        va="bottom",
        bbox={"facecolor": "white", "alpha": 0.75, "edgecolor": "0.7"},
    )
    fig.savefig(
        filename,
        dpi=200,
        bbox_inches="tight",
    )
    plt.close(fig)
    print(f"Estimated sound-level plot saved to: {filename}")
    return filename


def _sound_level_report_lines(sound_metrics: EstimatedSoundLevelMetrics) -> list[str]:
    return [
        "",
        "Estimated acoustic sound levels",
        f"Calibration offset [dB]: {sound_metrics.calibration_offset_db:.2f}",
        (
            "Calibration repeatability std [dB]: "
            f"{sound_metrics.calibration_repeatability_std_db:.2f}"
        ),
        f"Calibration reference: {sound_metrics.calibration_reference}",
        f"Calibration weighting: {sound_metrics.calibration_weighting}",
        f"Calibration frequency [Hz]: {sound_metrics.calibration_frequency_hz:.1f}",
        f"Calibration approximate: {sound_metrics.calibration_approximate}",
        "",
        f"Unweighted RMS level [dBFS]: {_format_db(sound_metrics.unweighted_rms_dbfs)}",
        f"A-weighted RMS level [dBFS]: {_format_db(sound_metrics.a_weighted_rms_dbfs)}",
        "",
        f"Estimated LZeq [dBZ]: {_format_db(sound_metrics.estimated_lzeq_dbz, 'dBZ')}",
        f"Estimated LAeq [dBA]: {_format_db(sound_metrics.estimated_laeq_dba, 'dBA')}",
        "",
        (
            "These sound pressure levels are approximate estimates based on\n"
            "a smartphone Z-weighted calibration at 1000 Hz and are not\n"
            "certified sound-level-meter measurements."
        ),
    ]


def _sound_environment_assessment_report_lines(
    sound_assessment: SoundEnvironmentAssessment,
) -> list[str]:
    return [
        "",
        "Sound environment assessment",
        (
            "Estimated global LAeq [dBA]: "
            f"{_format_db(sound_assessment.estimated_laeq_dba, 'dBA')}"
        ),
        f"Environment category: {sound_assessment.category_label}",
        f"Environment description: {sound_assessment.category_description}",
        f"Acquisition duration [s]: {sound_assessment.acquisition_duration_s:.6f}",
        f"Short acquisition: {sound_assessment.short_acquisition}",
        (
            "NIOSH reference level [dBA]: "
            f"{sound_assessment.niosh_reference_level_dba:.1f}"
        ),
        f"NIOSH exchange rate [dB]: {sound_assessment.niosh_exchange_rate_db:.1f}",
        (
            "Indicative maximum exposure time: "
            f"{format_exposure_duration(sound_assessment.niosh_max_exposure_hours)}"
        ),
        (
            "Recorded NIOSH dose [%]: "
            f"{_format_number(sound_assessment.niosh_recorded_dose_percent, 6)}"
        ),
        f"Warning level: {sound_assessment.warning_level}",
        f"Warning message: {sound_assessment.warning_message}",
        "",
        (
            "The NIOSH exposure time is an indicative projection assuming\n"
            "the measured LAeq remains constant. The sound level itself is\n"
            "an approximate smartphone-calibrated estimate and is not a\n"
            "certified sound-level-meter measurement."
        ),
    ]


def write_sound_environment_assessment_report(
    filename: Path,
    sound_assessment: SoundEnvironmentAssessment,
) -> None:
    """Write a standalone acoustic environment assessment report."""
    lines = [
        (
            "Estimated LAeq: "
            f"{_format_db(sound_assessment.estimated_laeq_dba, 'dBA')}"
        ),
        f"Environment: {sound_assessment.category_label}",
        f"Description: {sound_assessment.category_description}",
        "",
        (
            "Indicative NIOSH maximum exposure time: "
            f"{format_exposure_duration(sound_assessment.niosh_max_exposure_hours)}"
        ),
        f"Recorded duration: {sound_assessment.acquisition_duration_s:.3f} s",
        (
            "Recorded dose: "
            f"{_format_number(sound_assessment.niosh_recorded_dose_percent, 3)} %"
        ),
        "",
        f"Warning level: {sound_assessment.warning_level}",
        "Warning:",
        sound_assessment.warning_message,
        "",
        "Calibration status: approximate",
        "",
        (
            "The NIOSH exposure time is an indicative projection assuming "
            "the measured LAeq remains constant. The sound level itself is "
            "an approximate smartphone-calibrated estimate and is not a "
            "certified sound-level-meter measurement."
        ),
    ]
    filename.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Audio environment assessment saved to: {filename}")


def print_estimated_sound_level_metrics(sound_metrics: EstimatedSoundLevelMetrics) -> None:
    """Print approximate acoustic estimates to the terminal."""
    print("Estimated acoustic sound levels:")
    print(f"  Unweighted RMS level [dBFS]: {_format_db(sound_metrics.unweighted_rms_dbfs)}")
    print(f"  A-weighted RMS level [dBFS]: {_format_db(sound_metrics.a_weighted_rms_dbfs)}")
    print(f"  Estimated LZeq [dBZ]: {_format_db(sound_metrics.estimated_lzeq_dbz, 'dBZ')}")
    print(f"  Estimated LAeq [dBA]: {_format_db(sound_metrics.estimated_laeq_dba, 'dBA')}")
    print(f"  Calibration offset [dB]: {sound_metrics.calibration_offset_db:.2f}")
    status = "approximate" if sound_metrics.calibration_approximate else "certified"
    print(f"  Calibration status: {status}")


def print_sound_environment_assessment(
    sound_assessment: SoundEnvironmentAssessment,
) -> None:
    """Print the acoustic environment assessment to the terminal."""
    print("Sound environment assessment:")
    print(
        "  Estimated global LAeq [dBA]: "
        f"{_format_db(sound_assessment.estimated_laeq_dba, 'dBA')}"
    )
    print(f"  Environment category: {sound_assessment.category_label}")
    print(f"  Environment description: {sound_assessment.category_description}")
    print(
        "  Indicative NIOSH maximum exposure time: "
        f"{format_exposure_duration(sound_assessment.niosh_max_exposure_hours)}"
    )
    print(
        "  Recorded dose [%]: "
        f"{_format_number(sound_assessment.niosh_recorded_dose_percent, 3)}"
    )
    print(f"  Warning level: {sound_assessment.warning_level}")
    print(f"  Warning message: {sound_assessment.warning_message}")
    print(f"  Short acquisition: {sound_assessment.short_acquisition}")


def write_amplified_wav_int16_mono(
    filename: Path,
    audio: np.ndarray,
    sample_rate_hz: int,
    gain: float,
) -> None:
    """Write a peak-normalized int16 mono WAV without changing raw samples."""
    audio_float = audio.astype(np.float64)
    amplified_float = audio_float * gain
    amplified_int16 = np.clip(np.rint(amplified_float), -32768, 32767).astype("<i2")
    write_wav_int16_mono(filename, amplified_int16.tobytes(), sample_rate_hz)


def _frame_starts(sample_count: int, window_samples: int, hop_samples: int) -> list[int]:
    if sample_count <= 0:
        return []
    window_samples = max(1, min(window_samples, sample_count))
    hop_samples = max(1, hop_samples)
    return list(range(0, sample_count - window_samples + 1, hop_samples)) or [0]


def plot_audio_waveform(audio: np.ndarray, sample_rate_hz: int, output_prefix: Path) -> Path | None:
    """Plot the raw PCM int16 waveform in the time domain."""
    if len(audio) == 0:
        print("No audio data available; skipping audio waveform plot.")
        return None

    t = np.arange(len(audio), dtype=np.float64) / float(sample_rate_hz)
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, audio.astype(np.float64), linewidth=0.8)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("PCM amplitude [int16]")
    ax.set_title("Audio waveform")
    ax.grid(True)
    audio_png = output_prefix.with_name(output_prefix.name + "_audio_waveform.png")
    fig.savefig(audio_png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Audio waveform plot saved to: {audio_png}")
    return audio_png


def plot_audio_level_dbfs(audio: np.ndarray, sample_rate_hz: int, output_prefix: Path) -> Path | None:
    """Plot short-time RMS and peak levels in dBFS from raw samples."""
    sample_count = len(audio)
    if sample_count == 0:
        print("No audio data available; skipping audio level plot.")
        return None

    window_samples = max(1, int(round(sample_rate_hz * AUDIO_LEVEL_WINDOW_MS / 1000.0)))
    hop_samples = max(1, int(round(sample_rate_hz * AUDIO_LEVEL_HOP_MS / 1000.0)))
    window_samples = min(window_samples, sample_count)
    starts = _frame_starts(sample_count, window_samples, hop_samples)
    audio_float = audio.astype(np.float64)

    times = np.empty(len(starts), dtype=np.float64)
    rms_dbfs = np.empty(len(starts), dtype=np.float64)
    peak_dbfs = np.empty(len(starts), dtype=np.float64)

    for index, start in enumerate(starts):
        frame = audio_float[start:start + window_samples]
        center = start + (len(frame) / 2.0)
        times[index] = center / float(sample_rate_hz)
        rms = float(np.sqrt(np.mean(frame ** 2))) if len(frame) else 0.0
        peak = float(np.max(np.abs(frame))) if len(frame) else 0.0
        rms_dbfs[index] = _db20_from_ratio(rms / INT16_FULL_SCALE)
        peak_dbfs[index] = _db20_from_ratio(peak / INT16_FULL_SCALE)

    rms_plot = np.maximum(rms_dbfs, AUDIO_DBFS_FLOOR)
    peak_plot = np.maximum(peak_dbfs, AUDIO_DBFS_FLOOR)

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(times, rms_plot, label="RMS level", linewidth=1.2)
    ax.plot(times, peak_plot, label="Peak level", linewidth=1.2)
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Level [dBFS]")
    ax.set_ylim(AUDIO_DBFS_FLOOR, 0.0)
    ax.set_title("Audio level over time")
    ax.grid(True)
    ax.legend()
    output_file = output_prefix.with_name(output_prefix.name + "_audio_level_dbfs.png")
    fig.savefig(output_file, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Audio level plot saved to: {output_file}")
    return output_file


def compute_welch_psd(
    audio: np.ndarray,
    sample_rate_hz: int,
    segment_samples: int = AUDIO_PSD_SEGMENT_SAMPLES,
    overlap: float = AUDIO_PSD_OVERLAP,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute one-sided Welch PSD in full-scale squared per Hz."""
    sample_count = len(audio)
    if sample_count == 0:
        return np.array([], dtype=np.float64), np.array([], dtype=np.float64)

    nperseg = max(1, min(int(segment_samples), sample_count))
    overlap = min(max(float(overlap), 0.0), 0.95)
    hop = max(1, int(round(nperseg * (1.0 - overlap))))
    starts = _frame_starts(sample_count, nperseg, hop)

    x = audio.astype(np.float64) / INT16_FULL_SCALE
    x = x - float(np.mean(x))
    window = np.hanning(nperseg) if nperseg >= 3 else np.ones(nperseg, dtype=np.float64)
    window_power = float(np.sum(window ** 2))
    if window_power == 0.0:
        window = np.ones(nperseg, dtype=np.float64)
        window_power = float(np.sum(window ** 2))

    psd_accumulator = None
    for start in starts:
        segment = x[start:start + nperseg]
        if len(segment) < nperseg:
            segment = np.pad(segment, (0, nperseg - len(segment)))
        spectrum = np.fft.rfft(segment * window)
        psd = (np.abs(spectrum) ** 2) / (float(sample_rate_hz) * window_power)
        if nperseg > 1:
            if nperseg % 2 == 0:
                psd[1:-1] *= 2.0
            else:
                psd[1:] *= 2.0
        if psd_accumulator is None:
            psd_accumulator = psd
        else:
            psd_accumulator += psd

    mean_psd = psd_accumulator / float(len(starts))
    frequencies = np.fft.rfftfreq(nperseg, d=1.0 / float(sample_rate_hz))
    return frequencies, mean_psd


def plot_audio_psd(audio: np.ndarray, sample_rate_hz: int, output_prefix: Path) -> Path | None:
    """Plot Welch PSD in dBFS/Hz."""
    if len(audio) == 0:
        print("No audio data available; skipping audio PSD plot.")
        return None

    frequencies, psd = compute_welch_psd(audio, sample_rate_hz)
    if len(frequencies) == 0:
        return None

    psd_db = _db10_from_power(psd)
    finite = np.isfinite(psd_db)
    if finite.any():
        plot_psd_db = psd_db.copy()
        plot_psd_db[~finite] = float(np.min(psd_db[finite]) - 20.0)
    else:
        plot_psd_db = np.full_like(psd_db, AUDIO_DBFS_FLOOR)

    max_frequency = min(12000.0, sample_rate_hz / 2.0)
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(frequencies, plot_psd_db, linewidth=1.1)
    ax.set_xlim(0.0, max_frequency)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("PSD [dBFS/Hz]")
    ax.set_title("Audio PSD (Welch)")
    ax.grid(True)
    output_file = output_prefix.with_name(output_prefix.name + "_audio_psd.png")
    fig.savefig(output_file, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Audio PSD plot saved to: {output_file}")
    return output_file


def _spectral_segment_length(sample_count: int, requested_samples: int) -> int:
    if sample_count < 2 or requested_samples <= 0:
        return 0
    segment_length = min(int(requested_samples), sample_count)
    if segment_length < requested_samples:
        segment_length = 2 ** int(np.floor(np.log2(segment_length)))
    return max(2, segment_length)


def compute_average_magnitude_spectrum_dbfs(
    audio: np.ndarray,
    sample_rate_hz: int,
    segment_samples: int = AUDIO_MAGNITUDE_SEGMENT_SAMPLES,
    overlap: float = AUDIO_MAGNITUDE_OVERLAP,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute the average one-sided magnitude spectrum in dBFS."""
    sample_count = len(audio)
    if sample_count < 2 or sample_rate_hz <= 0:
        return np.array([], dtype=np.float64), np.array([], dtype=np.float64)

    segment_length = _spectral_segment_length(sample_count, int(segment_samples))
    if segment_length < 2:
        return np.array([], dtype=np.float64), np.array([], dtype=np.float64)

    overlap = min(max(float(overlap), 0.0), 0.95)
    hop = max(1, int(round(segment_length * (1.0 - overlap))))
    starts = _frame_starts(sample_count, segment_length, hop)
    if not starts:
        return np.array([], dtype=np.float64), np.array([], dtype=np.float64)

    x = audio.astype(np.float64) / INT16_FULL_SCALE
    x = x - float(np.mean(x))
    window = (
        np.hanning(segment_length)
        if segment_length >= 3
        else np.ones(segment_length, dtype=np.float64)
    )
    coherent_gain = float(np.sum(window))
    if coherent_gain <= 0.0:
        window = np.ones(segment_length, dtype=np.float64)
        coherent_gain = float(np.sum(window))

    magnitude_accumulator = None
    for start in starts:
        segment = x[start:start + segment_length]
        if len(segment) < segment_length:
            segment = np.pad(segment, (0, segment_length - len(segment)))
        spectrum = np.fft.rfft(segment * window)
        magnitude = np.abs(spectrum) / coherent_gain
        if segment_length > 1:
            if segment_length % 2 == 0:
                magnitude[1:-1] *= 2.0
            else:
                magnitude[1:] *= 2.0
        if magnitude_accumulator is None:
            magnitude_accumulator = magnitude
        else:
            magnitude_accumulator += magnitude

    average_magnitude = magnitude_accumulator / float(len(starts))
    linear_floor = 10.0 ** (AUDIO_MAGNITUDE_DBFS_FLOOR / 20.0)
    magnitude_dbfs = 20.0 * np.log10(np.maximum(average_magnitude, linear_floor))
    frequencies = np.fft.rfftfreq(segment_length, d=1.0 / float(sample_rate_hz))
    return frequencies, magnitude_dbfs


def plot_audio_magnitude_spectrum_dbfs(
    audio: np.ndarray,
    sample_rate_hz: int,
    filename: Path,
) -> Path | None:
    """Plot the average magnitude spectrum of the recorded audio."""
    frequencies, magnitude_dbfs = compute_average_magnitude_spectrum_dbfs(
        audio,
        sample_rate_hz,
    )
    if len(frequencies) == 0 or len(magnitude_dbfs) == 0:
        print("Warning: unable to generate audio magnitude spectrum; insufficient valid audio data.")
        return None

    if np.isnan(magnitude_dbfs).any():
        print("Warning: audio magnitude spectrum contains NaN values; plot skipped.")
        return None

    max_frequency = min(AUDIO_MAGNITUDE_MAX_FREQUENCY_HZ, sample_rate_hz / 2.0)
    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(frequencies, magnitude_dbfs, linewidth=1.1)
    ax.set_xlim(0.0, max_frequency)
    ax.set_ylim(AUDIO_MAGNITUDE_DBFS_FLOOR, 0.0)
    ax.set_xlabel("Frequency [Hz]")
    ax.set_ylabel("Magnitude [dBFS]")
    ax.set_title("Average magnitude spectrum of recorded audio")
    ax.grid(True)
    fig.savefig(
        filename,
        dpi=200,
        bbox_inches="tight",
    )
    plt.close(fig)
    print(f"Audio magnitude spectrum plot saved to: {filename}")
    return filename


def compute_spectrogram_psd(
    audio: np.ndarray,
    sample_rate_hz: int,
    nfft: int = AUDIO_SPECTROGRAM_NFFT,
    overlap: float = AUDIO_SPECTROGRAM_OVERLAP,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute a one-sided PSD spectrogram in dBFS/Hz."""
    sample_count = len(audio)
    if sample_count == 0:
        empty = np.array([], dtype=np.float64)
        return empty, empty, np.empty((0, 0), dtype=np.float64)

    nfft = max(1, min(int(nfft), sample_count))
    overlap = min(max(float(overlap), 0.0), 0.95)
    hop = max(1, int(round(nfft * (1.0 - overlap))))
    starts = _frame_starts(sample_count, nfft, hop)

    x = audio.astype(np.float64) / INT16_FULL_SCALE
    x = x - float(np.mean(x))
    window = np.hanning(nfft) if nfft >= 3 else np.ones(nfft, dtype=np.float64)
    window_power = float(np.sum(window ** 2))
    if window_power == 0.0:
        window = np.ones(nfft, dtype=np.float64)
        window_power = float(np.sum(window ** 2))

    columns = []
    times = np.empty(len(starts), dtype=np.float64)
    for index, start in enumerate(starts):
        segment = x[start:start + nfft]
        if len(segment) < nfft:
            segment = np.pad(segment, (0, nfft - len(segment)))
        spectrum = np.fft.rfft(segment * window)
        psd = (np.abs(spectrum) ** 2) / (float(sample_rate_hz) * window_power)
        if nfft > 1:
            if nfft % 2 == 0:
                psd[1:-1] *= 2.0
            else:
                psd[1:] *= 2.0
        columns.append(_db10_from_power(psd))
        times[index] = (start + (nfft / 2.0)) / float(sample_rate_hz)

    frequencies = np.fft.rfftfreq(nfft, d=1.0 / float(sample_rate_hz))
    spectrogram_db = np.column_stack(columns)
    return times, frequencies, spectrogram_db


def plot_audio_spectrogram(audio: np.ndarray, sample_rate_hz: int, output_prefix: Path) -> Path | None:
    """Plot a PSD spectrogram in dBFS/Hz."""
    if len(audio) == 0:
        print("No audio data available; skipping audio spectrogram.")
        return None

    times, frequencies, spectrogram_db = compute_spectrogram_psd(audio, sample_rate_hz)
    if spectrogram_db.size == 0:
        return None

    max_frequency = min(AUDIO_SPECTROGRAM_MAX_FREQUENCY_HZ, sample_rate_hz / 2.0)
    frequency_mask = frequencies <= max_frequency
    frequencies = frequencies[frequency_mask]
    spectrogram_db = spectrogram_db[frequency_mask, :]

    finite = np.isfinite(spectrogram_db)
    if finite.any():
        vmax = float(np.max(spectrogram_db[finite]))
        vmin = vmax - AUDIO_SPECTROGRAM_DYNAMIC_RANGE_DB
    else:
        vmax = 0.0
        vmin = -AUDIO_SPECTROGRAM_DYNAMIC_RANGE_DB
    plot_data = np.maximum(spectrogram_db, vmin)

    fig, ax = plt.subplots(figsize=(12, 5))
    extent = [
        float(times[0]) if len(times) else 0.0,
        float(times[-1]) if len(times) else 0.0,
        float(frequencies[0]) if len(frequencies) else 0.0,
        float(frequencies[-1]) if len(frequencies) else 0.0,
    ]
    image = ax.imshow(
        plot_data,
        origin="lower",
        aspect="auto",
        extent=extent,
        vmin=vmin,
        vmax=vmax,
        interpolation="nearest",
    )
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Frequency [Hz]")
    ax.set_title("Audio spectrogram")
    colorbar = fig.colorbar(image, ax=ax)
    colorbar.set_label("Power spectral density [dBFS/Hz]")
    output_file = output_prefix.with_name(output_prefix.name + "_audio_spectrogram.png")
    fig.savefig(output_file, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Audio spectrogram saved to: {output_file}")
    return output_file


def plot_audio_histogram(
    audio: np.ndarray,
    metrics: AudioMetrics,
    output_prefix: Path,
) -> Path | None:
    """Plot the raw int16 sample distribution."""
    if len(audio) == 0:
        print("No audio data available; skipping audio histogram.")
        return None

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.hist(audio.astype(np.float64), bins=AUDIO_HISTOGRAM_BINS)
    ax.axvline(
        metrics.mean_sample,
        color="red",
        linestyle="--",
        linewidth=1.2,
        label=f"Mean/DC offset = {metrics.mean_sample:.3f}",
    )
    ax.set_xlabel("PCM amplitude [int16 counts]")
    ax.set_ylabel("Sample count")
    ax.set_title("Audio sample histogram")
    ax.grid(True, axis="y", alpha=0.35)
    ax.legend()
    output_file = output_prefix.with_name(output_prefix.name + "_audio_histogram.png")
    fig.savefig(output_file, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Audio histogram saved to: {output_file}")
    return output_file


def _audio_output_prefix_from_wav(original_wav_filename: Path) -> Path:
    if original_wav_filename.name.endswith("_audio.wav"):
        prefix_name = original_wav_filename.name[:-len("_audio.wav")]
        return original_wav_filename.with_name(prefix_name)
    return original_wav_filename.with_suffix("")


def analyze_and_export_audio(
    audio_bytes: bytes,
    sample_rate_hz: int,
    original_wav_filename: Path,
    summary_filename: Path,
) -> AudioMetrics | None:
    """Decode raw audio once, export WAVs, plots, metrics and summary section."""
    if not audio_bytes:
        message = (
            "No AUD0 raw audio available; skipping WAV and raw audio waveform/"
            "spectral plots."
        )
        print(message)
        with summary_filename.open("a", encoding="utf-8") as summary_file:
            summary_file.write("\nAudio metrics summary\n")
            summary_file.write("AUD0 raw audio analysis skipped: no AUD0 data available.\n")
        return None

    output_prefix = _audio_output_prefix_from_wav(original_wav_filename)
    audio = decode_audio_int16(audio_bytes)
    if len(audio) == 0:
        message = "No complete int16 audio samples available; skipping audio exports."
        print(message)
        with summary_filename.open("a", encoding="utf-8") as summary_file:
            summary_file.write("\nAudio metrics summary\n")
            summary_file.write("Audio analysis skipped: no complete int16 samples available.\n")
        return None

    metrics = compute_audio_metrics(audio, sample_rate_hz)
    sound_metrics = compute_estimated_sound_level_metrics(audio, sample_rate_hz)
    sound_assessment = assess_sound_environment(
        sound_metrics.estimated_laeq_dba,
        metrics.duration_s,
    )
    (
        sound_time_center_s,
        sound_unweighted_dbfs,
        sound_a_weighted_dbfs,
        sound_estimated_lzeq_dbz,
        sound_estimated_laeq_dba,
    ) = compute_estimated_sound_level_timeseries(audio, sample_rate_hz)

    write_wav_int16_mono(original_wav_filename, audio.tobytes(), sample_rate_hz)
    print(f"Audio WAV saved to: {original_wav_filename}")

    amplified_wav_filename = output_prefix.with_name(output_prefix.name + "_audio_amplified.wav")
    write_amplified_wav_int16_mono(
        amplified_wav_filename,
        audio,
        sample_rate_hz,
        metrics.amplification_gain_linear,
    )
    print(f"Amplified audio WAV saved to: {amplified_wav_filename}")

    plot_audio_waveform(audio, sample_rate_hz, output_prefix)
    plot_audio_level_dbfs(audio, sample_rate_hz, output_prefix)
    plot_audio_psd(audio, sample_rate_hz, output_prefix)
    magnitude_spectrum_filename = output_prefix.with_name(
        output_prefix.name + "_audio_magnitude_spectrum_dbfs.png"
    )
    plot_audio_magnitude_spectrum_dbfs(
        audio,
        sample_rate_hz,
        magnitude_spectrum_filename,
    )
    plot_audio_spectrogram(audio, sample_rate_hz, output_prefix)
    plot_audio_histogram(audio, metrics, output_prefix)

    sound_level_plot_filename = output_prefix.with_name(
        output_prefix.name + "_audio_estimated_sound_level.png"
    )
    plot_estimated_sound_level_timeseries(
        sound_level_plot_filename,
        sound_time_center_s,
        sound_estimated_lzeq_dbz,
        sound_estimated_laeq_dba,
    )

    sound_level_csv_filename = output_prefix.with_name(
        output_prefix.name + "_audio_sound_level_1s.csv"
    )
    write_estimated_sound_level_csv(
        sound_level_csv_filename,
        sound_time_center_s,
        sound_unweighted_dbfs,
        sound_a_weighted_dbfs,
        sound_estimated_lzeq_dbz,
        sound_estimated_laeq_dba,
    )

    metrics_filename = output_prefix.with_name(output_prefix.name + "_audio_metrics.txt")
    write_audio_metrics_report(
        metrics_filename,
        metrics,
        sound_metrics,
        sound_assessment,
    )
    environment_assessment_filename = output_prefix.with_name(
        output_prefix.name + "_audio_environment_assessment.txt"
    )
    write_sound_environment_assessment_report(
        environment_assessment_filename,
        sound_assessment,
    )
    append_audio_metrics_to_summary(
        summary_filename,
        metrics,
        sound_metrics,
        sound_assessment,
    )
    print_audio_metrics(metrics)
    print_estimated_sound_level_metrics(sound_metrics)
    print_sound_environment_assessment(sound_assessment)
    return metrics


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
    audio_feature_pages: int = 0
    light_feature_pages: int = 0
    light_raw_pages: int = 0
    light_pages: int = 0
    unknown_pages: int = 0
    sensor_records: int = 0
    light_raw_records: int = 0
    light_records: int = 0
    audio_bytes: int = 0
    audio_feature_records: int = 0
    audio_feature_payload_bytes: int = 0
    invalid_audio_feature_pages: int = 0
    invalid_audio_feature_records: int = 0
    audio_feature_payload_remainder_bytes: int = 0
    light_feature_records: int = 0
    light_feature_payload_bytes: int = 0
    invalid_light_feature_pages: int = 0
    invalid_light_feature_records: int = 0
    light_feature_payload_remainder_bytes: int = 0
    light_feature_validation_result: str = "NOT AVAILABLE"
    audio_complete_windows: int = 0
    audio_partial_windows: int = 0
    audio_feature_paired_windows: int = 0
    audio_feature_comparison_failures: int = 0
    audio_feature_comparison_result: str = "NOT AVAILABLE"
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


@dataclass
class BleInputStats:
    logical_pages: int = 0
    first_page_sequence: int | None = None
    last_page_sequence: int | None = None
    first_physical_page_index: int | None = None
    last_physical_page_index: int | None = None
    crc_mismatches: int = 0
    metadata_mismatches: int = 0


@dataclass
class VirtualNandPage:
    data: bytes
    physical_page_index: int


BLE_PAGE_CSV_COLUMNS = (
    "page_sequence",
    "physical_page_index",
    "log_generation",
    "magic",
    "page_version",
    "page_header_size",
    "page_payload_bytes",
    "logical_page_bytes",
    "page_crc32",
    "file_offset",
)


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


def _light_exposure_reference_scale() -> float:
    """Return the multiplier that maps current Clear counts to the reference exposure."""
    current_exposure = LIGHT_SENSOR_GAIN * LIGHT_SENSOR_INTEGRATION_TIME_MS
    reference_exposure = (
        LIGHT_CLASSIFICATION_REFERENCE_GAIN
        * LIGHT_CLASSIFICATION_REFERENCE_INTEGRATION_TIME_MS
    )
    if current_exposure <= 0.0 or not np.isfinite(current_exposure):
        raise ValueError("Light gain and integration time must be finite positive values.")
    return float(reference_exposure / current_exposure)


def _light_smoothing_window_samples(light_raw_df: pd.DataFrame) -> int:
    """Estimate a trailing median-filter length from the real LRAW timestamps."""
    if len(light_raw_df) < 2 or LIGHT_CLASSIFICATION_SMOOTHING_SECONDS <= 0.0:
        return 1

    elapsed_s = pd.to_numeric(
        light_raw_df["sample_elapsed_s"],
        errors="coerce",
    ).to_numpy(dtype=np.float64)
    differences = np.diff(elapsed_s)
    valid_differences = differences[
        np.isfinite(differences) & (differences > 0.0)
    ]
    if len(valid_differences) == 0:
        return 1

    median_step_s = float(np.median(valid_differences))
    if median_step_s <= 0.0 or not np.isfinite(median_step_s):
        return 1

    return max(
        1,
        int(round(LIGHT_CLASSIFICATION_SMOOTHING_SECONDS / median_step_s)),
    )


def _relative_light_category(
    reference_clear_counts: float,
    saturated: bool,
) -> tuple[int, str, str]:
    """Classify personal light exposure using empirical Clear thresholds."""

    # Saturation remains available separately through the is_saturated field,
    # but for the wearer it represents very high light exposure.
    if saturated:
        return LIGHT_CLASSIFICATION_CATEGORIES[-1]

    value = max(float(reference_clear_counts), 0.0)

    for threshold, category in zip(
        LIGHT_CLASSIFICATION_THRESHOLDS_REFERENCE_COUNTS,
        LIGHT_CLASSIFICATION_CATEGORIES,
    ):
        if value < threshold:
            return category

    return LIGHT_CLASSIFICATION_CATEGORIES[-1]

def compute_relative_light_classification(
    light_raw_df: pd.DataFrame,
) -> tuple[pd.DataFrame, LightEnvironmentAssessment | None]:
    """
    Build a per-sample personal light exposure classification from Clear counts.

    The original LRAW DataFrame is not modified. The returned score is bounded
    to 0..100 and is logarithmic for display only. Classes are empirical and
    must not be interpreted as lux.
    """
    output_columns = [
        "sample_elapsed_ms",
        "sample_elapsed_s",
        "sample_index",
        "clear_counts",
        "nir_counts",
        "clear_corrected_counts",
        "clear_reference_counts",
        "clear_smoothed_reference_counts",
        "ambient_light_score_0_100",
        "light_level_class",
        "light_level_label",
        "light_level_description",
        "is_saturated",
    ]
    if light_raw_df.empty:
        return pd.DataFrame(columns=output_columns), None

    clear_counts = pd.to_numeric(
        light_raw_df["clear_counts"],
        errors="coerce",
    ).to_numpy(dtype=np.float64)
    nir_counts = pd.to_numeric(
        light_raw_df["nir_counts"],
        errors="coerce",
    ).to_numpy(dtype=np.float64)

    if not np.all(np.isfinite(clear_counts)):
        raise ValueError("Clear channel contains non-finite values.")
    if not np.all(np.isfinite(nir_counts)):
        raise ValueError("NIR channel contains non-finite values.")

    clear_corrected = np.maximum(clear_counts - LIGHT_DARK_OFFSET_COUNTS, 0.0)
    reference_scale = _light_exposure_reference_scale()
    clear_reference = clear_corrected * reference_scale

    smoothing_window_samples = _light_smoothing_window_samples(light_raw_df)
    clear_smoothed_reference = (
        pd.Series(clear_reference)
        .rolling(
            window=smoothing_window_samples,
            min_periods=1,
            center=False,
        )
        .median()
        .to_numpy(dtype=np.float64)
    )

    score_ratio = np.clip(
        clear_smoothed_reference / LIGHT_ADC_FULL_SCALE_COUNTS,
        0.0,
        1.0,
    )
    ambient_light_score = 100.0 * np.log10(1.0 + 9.0 * score_ratio)

    saturated = clear_counts >= LIGHT_ADC_FULL_SCALE_COUNTS

    class_codes = np.empty(len(clear_counts), dtype=np.int16)
    class_labels: list[str] = []
    class_descriptions: list[str] = []
    for index, (value, is_saturated) in enumerate(
        zip(clear_smoothed_reference, saturated)
    ):
        code, label, description = _relative_light_category(
            value,
            bool(is_saturated),
        )
        class_codes[index] = code
        class_labels.append(label)
        class_descriptions.append(description)

    classification_df = pd.DataFrame(
        {
            "sample_elapsed_ms": light_raw_df["sample_elapsed_ms"].to_numpy(),
            "sample_elapsed_s": light_raw_df["sample_elapsed_s"].to_numpy(),
            "sample_index": light_raw_df["sample_index"].to_numpy(),
            "clear_counts": clear_counts.astype(np.int64),
            "nir_counts": nir_counts.astype(np.int64),
            "clear_corrected_counts": clear_corrected,
            "clear_reference_counts": clear_reference,
            "clear_smoothed_reference_counts": clear_smoothed_reference,
            "ambient_light_score_0_100": ambient_light_score,
            "light_level_class": class_codes,
            "light_level_label": class_labels,
            "light_level_description": class_descriptions,
            "is_saturated": saturated,
        },
        columns=output_columns,
    )

    sample_count = len(classification_df)
    if sample_count > 1:
        acquisition_duration_s = float(
            classification_df["sample_elapsed_s"].max()
            - classification_df["sample_elapsed_s"].min()
        )
    else:
        acquisition_duration_s = 0.0

    label_counts_series = classification_df["light_level_label"].value_counts()
    class_counts = {
        str(label): int(count)
        for label, count in label_counts_series.items()
    }
    class_percentages = {
        label: 100.0 * count / sample_count
        for label, count in class_counts.items()
    }

    dominant_code = int(
        classification_df["light_level_class"].value_counts().idxmax()
    )
    category_lookup = {
        code: (label, description)
        for code, label, description in LIGHT_CLASSIFICATION_CATEGORIES
    }
    dominant_label, dominant_description = category_lookup[dominant_code]

    saturated_sample_count = int(np.count_nonzero(saturated))
    assessment = LightEnvironmentAssessment(
        sample_count=sample_count,
        acquisition_duration_s=acquisition_duration_s,
        smoothing_window_samples=smoothing_window_samples,
        median_clear_counts=float(np.median(clear_counts)),
        median_reference_clear_counts=float(
            np.median(clear_smoothed_reference)
        ),
        median_ambient_light_score=float(np.median(ambient_light_score)),
        dominant_class_code=dominant_code,
        dominant_class_label=dominant_label,
        dominant_class_description=dominant_description,
        saturated_sample_count=saturated_sample_count,
        saturated_sample_percentage=(
            100.0 * saturated_sample_count / sample_count
        ),
        class_counts=class_counts,
        class_percentages=class_percentages,
    )
    return classification_df, assessment


def _light_assessment_report_lines(
    assessment: LightEnvironmentAssessment,
) -> list[str]:
    lines = [
        "Personal light exposure assessment",
        "Method: AS7341 Clear channel with trailing median smoothing",
        "Measurement type: empirical personal exposure class; NOT lux",
        f"Configured sensor gain: {LIGHT_SENSOR_GAIN:g}x",
        (
            "Configured integration time [ms]: "
            f"{LIGHT_SENSOR_INTEGRATION_TIME_MS:.3f}"
        ),
        (
            "Reference exposure: "
            f"{LIGHT_CLASSIFICATION_REFERENCE_GAIN:g}x, "
            f"{LIGHT_CLASSIFICATION_REFERENCE_INTEGRATION_TIME_MS:.3f} ms"
        ),
        f"Configured Clear full scale [counts]: {LIGHT_ADC_FULL_SCALE_COUNTS:.0f}",
        f"Dark offset [counts]: {LIGHT_DARK_OFFSET_COUNTS:.3f}",
        (
            "Smoothing duration [s]: "
            f"{LIGHT_CLASSIFICATION_SMOOTHING_SECONDS:.3f}"
        ),
        f"Smoothing window [samples]: {assessment.smoothing_window_samples}",
        "",
        f"Sample count: {assessment.sample_count}",
        f"Acquisition duration [s]: {assessment.acquisition_duration_s:.3f}",
        f"Median raw Clear [counts]: {assessment.median_clear_counts:.3f}",
        (
            "Median reference-normalized Clear [counts]: "
            f"{assessment.median_reference_clear_counts:.3f}"
        ),
        (
            "Median personal light score [0..100]: "
            f"{assessment.median_ambient_light_score:.3f}"
        ),
        f"Dominant exposure class: {assessment.dominant_class_label}",
        f"Dominant class description: {assessment.dominant_class_description}",
        (
            "Saturated Clear samples: "
            f"{assessment.saturated_sample_count} "
            f"({assessment.saturated_sample_percentage:.3f}%)"
        ),
        "",
        "Class occupancy",
    ]

    ordered_labels = [category[1] for category in LIGHT_CLASSIFICATION_CATEGORIES]
    for label in ordered_labels:
        count = assessment.class_counts.get(label, 0)
        percentage = assessment.class_percentages.get(label, 0.0)
        lines.append(f"{label}: {count} sample(s), {percentage:.3f}%")

    lines.extend(
        [
            "",
            "Thresholds are empirical and depend on the sensor position, optical",
            "geometry and exposure settings. They should be refined with repeated",
            "measurements in the intended use conditions.",
        ]
    )
    return lines


def write_light_environment_assessment_report(
    filename: Path,
    assessment: LightEnvironmentAssessment,
) -> None:
    filename.write_text(
        "\n".join(_light_assessment_report_lines(assessment)) + "\n",
        encoding="utf-8",
    )
    print(f"Light environment assessment saved to: {filename}")


def append_light_environment_to_summary(
    summary_filename: Path,
    assessment: LightEnvironmentAssessment,
) -> None:
    with summary_filename.open("a", encoding="utf-8") as summary_file:
        summary_file.write("\n")
        summary_file.write(
            "\n".join(_light_assessment_report_lines(assessment))
        )
        summary_file.write("\n")


def print_light_environment_assessment(
    assessment: LightEnvironmentAssessment,
) -> None:
    print("Personal light exposure assessment:")
    print(f"  Dominant class: {assessment.dominant_class_label}")
    print(
        "  Median personal light score: "
        f"{assessment.median_ambient_light_score:.3f} / 100"
    )
    print(
        "  Median reference Clear: "
        f"{assessment.median_reference_clear_counts:.3f} counts"
    )
    print(
        "  Saturated Clear samples: "
        f"{assessment.saturated_sample_count} "
        f"({assessment.saturated_sample_percentage:.3f}%)"
    )


def plot_relative_light_classification(
    classification_df: pd.DataFrame,
    filename: Path,
) -> Path | None:
    """Plot Clear counts and the empirical personal exposure class over time."""
    if classification_df.empty:
        print("No light classification data available; skipping classification plot.")
        return None

    time_s = classification_df["sample_elapsed_s"].to_numpy(dtype=np.float64)
    if len(time_s) > 1 and np.any(np.diff(time_s) < 0.0):
        x = classification_df["sample_index"].to_numpy(dtype=np.float64)
        x_label = "Sample index"
    else:
        x = time_s
        x_label = "Time [s]"

    fig, (ax_clear, ax_class) = plt.subplots(
        nrows=2,
        ncols=1,
        figsize=(14, 8),
        sharex=True,
        constrained_layout=True,
    )

    ax_clear.plot(
        x,
        classification_df["clear_reference_counts"].to_numpy(dtype=np.float64),
        linewidth=0.9,
        alpha=0.55,
        label="Reference-normalized Clear",
    )
    ax_clear.plot(
        x,
        classification_df[
            "clear_smoothed_reference_counts"
        ].to_numpy(dtype=np.float64),
        linewidth=1.5,
        label="Smoothed Clear",
    )
    saturated_mask = classification_df["is_saturated"].to_numpy(dtype=bool)
    if np.any(saturated_mask):
        ax_clear.scatter(
            x[saturated_mask],
            classification_df.loc[
                saturated_mask,
                "clear_counts",
            ].to_numpy(dtype=np.float64),
            marker="x",
            s=28,
            linewidths=1.0,
            label="Raw saturation flag",
            zorder=3,
        )

    for threshold in LIGHT_CLASSIFICATION_THRESHOLDS_REFERENCE_COUNTS:
        ax_clear.axhline(
            threshold,
            linestyle="--",
            linewidth=0.8,
            alpha=0.45,
        )
    ax_clear.axhline(
        LIGHT_ADC_FULL_SCALE_COUNTS,
        linestyle=":",
        linewidth=1.2,
        label="Configured full scale",
    )
    ax_clear.set_ylim(0.0, LIGHT_ADC_FULL_SCALE_COUNTS * 1.05)
    ax_clear.set_ylabel("Clear counts")
    ax_clear.set_title(
        "AS7341 personal light exposure classification"
    )
    ax_clear.grid(True, alpha=0.4)
    ax_clear.legend()

    ax_class.step(
        x,
        classification_df["light_level_class"].to_numpy(dtype=np.int16),
        where="post",
        linewidth=1.3,
    )
    category_rows = LIGHT_CLASSIFICATION_CATEGORIES
    category_codes = [category[0] for category in category_rows]
    category_labels = [category[1] for category in category_rows]
    ax_class.set_yticks(category_codes)
    ax_class.set_yticklabels(category_labels)
    ax_class.set_xlabel(x_label)
    ax_class.set_ylabel("Exposure class")
    ax_class.grid(True, alpha=0.4)

    fig.savefig(filename, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Relative light classification plot saved to: {filename}")
    return filename


def analyze_and_export_relative_light_level(
    light_raw_df: pd.DataFrame,
    output_prefix: Path,
    summary_filename: Path,
    diagnostics: dict,
) -> pd.DataFrame | None:
    """Export per-sample classes, a plot and a session-level assessment."""
    if light_raw_df.empty:
        print("No LRAW data available; skipping relative light classification.")
        return None

    if diagnostics.get("is_severely_suspicious"):
        print(
            "Relative light classification skipped because LRAW data is "
            "flagged as severely suspicious."
        )
        return None

    classification_df, assessment = compute_relative_light_classification(
        light_raw_df
    )
    if assessment is None:
        return None

    csv_filename = output_prefix.with_name(
        output_prefix.name + "_light_level_classification.csv"
    )
    plot_filename = output_prefix.with_name(
        output_prefix.name + "_light_level_classification.png"
    )
    report_filename = output_prefix.with_name(
        output_prefix.name + "_light_environment_assessment.txt"
    )

    classification_df.to_csv(csv_filename, index=False)
    print(f"Light classification CSV saved to: {csv_filename}")

    plot_relative_light_classification(
        classification_df,
        plot_filename,
    )
    write_light_environment_assessment_report(
        report_filename,
        assessment,
    )
    append_light_environment_to_summary(
        summary_filename,
        assessment,
    )
    print_light_environment_assessment(assessment)
    return classification_df


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


def _format_audio_payload_pattern(payload_sizes: list[int]) -> str:
    if not payload_sizes:
        return "empty"
    runs = []
    run_value = payload_sizes[0]
    run_count = 1
    for value in payload_sizes[1:]:
        if value == run_value:
            run_count += 1
        else:
            runs.append(f"{run_count}x{run_value}")
            run_value = value
            run_count = 1
    runs.append(f"{run_count}x{run_value}")
    return " + ".join(runs)


def reconstruct_audio_windows(audio_pages: list[dict]) -> tuple[list[dict], list[dict], list[str]]:
    rows = []
    complete_windows = []
    warnings = []
    current_pages = []
    current_bytes = bytearray()

    def finalize_current() -> None:
        nonlocal current_pages, current_bytes
        if not current_pages:
            return

        payload_sizes = [len(page["payload"]) for page in current_pages]
        exact_size = len(current_bytes) == AUDIO_WINDOW_BYTES
        pattern_valid = tuple(payload_sizes) == AUDIO_WINDOW_EXPECTED_PAYLOAD_PATTERN
        structurally_valid = exact_size and pattern_valid
        window_index = len(rows)
        row = {
            "audio_window_index": window_index,
            "audio_bytes": len(current_bytes),
            "audio_samples": len(current_bytes) // 2,
            "complete": exact_size,
            "structurally_valid": structurally_valid,
            "first_physical_page": current_pages[0]["physical_page_index"],
            "last_physical_page": current_pages[-1]["physical_page_index"],
            "first_page_sequence": current_pages[0]["page_sequence"],
            "last_page_sequence": current_pages[-1]["page_sequence"],
            "first_page_timestamp_ms": current_pages[0]["page_timestamp_ms"],
            "last_page_timestamp_ms": current_pages[-1]["page_timestamp_ms"],
            "payload_pattern": _format_audio_payload_pattern(payload_sizes),
        }
        rows.append(row)

        internal_window = dict(row)
        internal_window["window_bytes"] = bytes(current_bytes)
        if structurally_valid:
            complete_windows.append(internal_window)
        elif exact_size:
            warnings.append(
                f"AUD0 window {window_index} has 48000 bytes but unexpected "
                f"payload pattern: {row['payload_pattern']}"
            )
        else:
            warnings.append(
                f"Incomplete AUD0 window {window_index}: {len(current_bytes)} bytes, "
                f"pattern {row['payload_pattern']}"
            )

        current_pages = []
        current_bytes = bytearray()

    for page in audio_pages:
        payload = page["payload"]
        if len(current_bytes) + len(payload) > AUDIO_WINDOW_BYTES:
            warnings.append(
                f"AUD0 page_sequence={page['page_sequence']} would exceed "
                f"{AUDIO_WINDOW_BYTES} bytes in the current window"
            )
            finalize_current()

        current_pages.append(page)
        current_bytes.extend(payload)
        if len(current_bytes) == AUDIO_WINDOW_BYTES:
            finalize_current()

    finalize_current()
    return rows, complete_windows, warnings


def _db_error_and_match(firmware_value: float, python_value: float) -> tuple[float, bool]:
    firmware_finite = bool(np.isfinite(firmware_value))
    python_finite = bool(np.isfinite(python_value))
    if not firmware_finite or not python_finite:
        return float("nan"), firmware_finite == python_finite
    error = float(firmware_value - python_value)
    return error, abs(error) <= AUDIO_FEATURE_DB_TOLERANCE_DB


def compare_audio_feature_records(
    complete_windows: list[dict],
    audio_feature_rows: list[dict],
    sample_rate_hz: int,
) -> tuple[pd.DataFrame, dict]:
    warnings = []
    sequences = [int(row["window_sequence"]) for row in audio_feature_rows]
    sequence_monotonic = all(right > left for left, right in zip(sequences, sequences[1:]))
    sequence_gaps = [
        (left, right)
        for left, right in zip(sequences, sequences[1:])
        if right != left + 1
    ]
    if sequences and not sequence_monotonic:
        warnings.append("AFEA window_sequence is not strictly monotonic")
    if sequence_gaps:
        warnings.append(f"AFEA window_sequence gaps: {sequence_gaps}")

    paired_count = min(len(complete_windows), len(audio_feature_rows))
    rows = []
    for pair_index in range(paired_count):
        window = complete_windows[pair_index]
        firmware = audio_feature_rows[pair_index]
        samples = np.frombuffer(window["window_bytes"], dtype="<i2")
        basic = compute_audio_window_basic_metrics(samples)

        try:
            weighted = compute_embedded_iir_a_weighted_metrics(samples, sample_rate_hz)
        except ValueError as exc:
            warnings.append(f"Window {window['audio_window_index']}: {exc}")
            weighted = {
                "a_weighted_rms_counts": 0.0,
                "a_weighted_rms_dbfs": float("-inf"),
                "estimated_laeq_dba": float("-inf"),
            }

        python_environment = classify_embedded_audio_environment(
            weighted["estimated_laeq_dba"]
        )
        a_weighting_valid = bool(
            np.isfinite(weighted["a_weighted_rms_dbfs"])
            and np.isfinite(weighted["estimated_laeq_dba"])
        )
        python_flags = build_expected_audio_flags(
            complete=True,
            acquisition_valid=True,
            a_weighting_valid=a_weighting_valid,
            clipped_sample_count=basic["clipped_sample_count"],
            estimated_laeq_dba=weighted["estimated_laeq_dba"],
        )

        rms_z_error, rms_z_match = _db_error_and_match(
            firmware["rms_z_dbfs"], basic["rms_zero_mean_dbfs"]
        )
        rms_a_error, rms_a_match = _db_error_and_match(
            firmware["rms_a_dbfs"], weighted["a_weighted_rms_dbfs"]
        )
        laeq_error, laeq_match = _db_error_and_match(
            firmware["estimated_laeq_dba"], weighted["estimated_laeq_dba"]
        )
        peak_error, peak_match = _db_error_and_match(
            firmware["peak_dbfs"], basic["peak_dbfs"]
        )

        sample_count_match = int(firmware["sample_count"]) == int(samples.size)
        mean_match = (
            abs(int(firmware["mean_counts_rounded"]) - basic["mean_counts_rounded"])
            <= AUDIO_FEATURE_MEAN_TOLERANCE_COUNTS
        )
        clipped_match = (
            int(firmware["clipped_sample_count"]) == basic["clipped_sample_count"]
        )
        environment_match = int(firmware["environment_class"]) == python_environment
        flags_match = (
            int(firmware["flags"]) & AUDIO_RECONSTRUCTIBLE_FLAG_MASK
        ) == (python_flags & AUDIO_RECONSTRUCTIBLE_FLAG_MASK)
        overall_match = all(
            (
                sample_count_match,
                mean_match,
                rms_z_match,
                rms_a_match,
                laeq_match,
                peak_match,
                clipped_match,
                environment_match,
                flags_match,
            )
        )

        rows.append(
            {
                "audio_window_index": window["audio_window_index"],
                "firmware_window_sequence": firmware["window_sequence"],
                "firmware_window_start_ms": firmware["window_start_ms"],
                "sample_count_firmware": firmware["sample_count"],
                "sample_count_python": int(samples.size),
                "sample_count_match": sample_count_match,
                "mean_firmware_rounded": firmware["mean_counts_rounded"],
                "mean_python": basic["mean_counts"],
                "mean_python_rounded": basic["mean_counts_rounded"],
                "mean_match": mean_match,
                "rms_z_firmware_dbfs": firmware["rms_z_dbfs"],
                "rms_z_python_dbfs": basic["rms_zero_mean_dbfs"],
                "rms_z_error_db": rms_z_error,
                "rms_z_match": rms_z_match,
                "rms_a_firmware_dbfs": firmware["rms_a_dbfs"],
                "rms_a_python_iir_dbfs": weighted["a_weighted_rms_dbfs"],
                "rms_a_error_db": rms_a_error,
                "rms_a_match": rms_a_match,
                "laeq_firmware_dba": firmware["estimated_laeq_dba"],
                "laeq_python_dba": weighted["estimated_laeq_dba"],
                "laeq_error_db": laeq_error,
                "laeq_match": laeq_match,
                "peak_firmware_dbfs": firmware["peak_dbfs"],
                "peak_python_dbfs": basic["peak_dbfs"],
                "peak_error_db": peak_error,
                "peak_match": peak_match,
                "clipped_firmware": firmware["clipped_sample_count"],
                "clipped_python": basic["clipped_sample_count"],
                "clipped_match": clipped_match,
                "environment_firmware": firmware["environment_class"],
                "environment_python": python_environment,
                "environment_match": environment_match,
                "flags_firmware": firmware["flags"],
                "flags_python": python_flags,
                "flags_compared_mask": AUDIO_RECONSTRUCTIBLE_FLAG_MASK,
                "flags_match": flags_match,
                "overall_match": overall_match,
            }
        )

    comparison_df = pd.DataFrame(rows, columns=AUDIO_FEATURE_COMPARISON_COLUMNS)
    unpaired_windows = len(complete_windows) - paired_count
    unpaired_records = len(audio_feature_rows) - paired_count
    row_failures = int((~comparison_df["overall_match"]).sum()) if not comparison_df.empty else 0
    sequence_issue_count = int(not sequence_monotonic) + len(sequence_gaps)

    if paired_count == 0:
        final_result = "NOT AVAILABLE"
    elif row_failures or unpaired_windows or unpaired_records or sequence_issue_count:
        final_result = "FAIL"
    else:
        final_result = "PASS"

    def max_abs_error(column: str) -> float:
        if comparison_df.empty:
            return float("nan")
        values = np.abs(pd.to_numeric(comparison_df[column], errors="coerce").to_numpy(float))
        finite = values[np.isfinite(values)]
        return float(np.max(finite)) if finite.size else float("nan")

    summary = {
        "paired_windows": paired_count,
        "unpaired_audio_windows": unpaired_windows,
        "unpaired_audio_feature_records": unpaired_records,
        "window_sequence_offset": sequences[0] if sequences else None,
        "window_sequence_monotonic": sequence_monotonic,
        "window_sequence_gaps": sequence_gaps,
        "max_abs_rms_z_error_db": max_abs_error("rms_z_error_db"),
        "max_abs_rms_a_error_db": max_abs_error("rms_a_error_db"),
        "max_abs_laeq_error_db": max_abs_error("laeq_error_db"),
        "max_abs_peak_error_db": max_abs_error("peak_error_db"),
        "sample_count_mismatches": int((~comparison_df["sample_count_match"]).sum()) if not comparison_df.empty else 0,
        "mean_mismatches": int((~comparison_df["mean_match"]).sum()) if not comparison_df.empty else 0,
        "clipping_mismatches": int((~comparison_df["clipped_match"]).sum()) if not comparison_df.empty else 0,
        "class_mismatches": int((~comparison_df["environment_match"]).sum()) if not comparison_df.empty else 0,
        "flag_mismatches": int((~comparison_df["flags_match"]).sum()) if not comparison_df.empty else 0,
        "comparison_failures": row_failures + unpaired_windows + unpaired_records + sequence_issue_count,
        "final_result": final_result,
        "warnings": warnings,
    }
    return comparison_df, summary


def write_audio_feature_comparison_report(
    report_filename: Path,
    stats: ParseStats,
    audio_window_rows: list[dict],
    comparison_df: pd.DataFrame,
    comparison_summary: dict,
) -> None:
    lines = [
        "Audio feature comparison",
        f"AFEA pages: {stats.audio_feature_pages}",
        f"AFEA records: {stats.audio_feature_records}",
        f"AFEA payload bytes: {stats.audio_feature_payload_bytes}",
        f"AFEA invalid pages: {stats.invalid_audio_feature_pages}",
        f"AFEA invalid records: {stats.invalid_audio_feature_records}",
        f"AFEA remainder bytes: {stats.audio_feature_payload_remainder_bytes}",
        f"AUD0 complete windows: {stats.audio_complete_windows}",
        f"AUD0 partial windows: {stats.audio_partial_windows}",
        f"Paired windows: {comparison_summary['paired_windows']}",
        f"Unpaired AUD0 windows: {comparison_summary['unpaired_audio_windows']}",
        f"Unpaired AFEA records: {comparison_summary['unpaired_audio_feature_records']}",
        f"AFEA sequence initial offset: {comparison_summary['window_sequence_offset']}",
        f"AFEA sequence monotonic: {comparison_summary['window_sequence_monotonic']}",
        f"AFEA sequence gaps: {comparison_summary['window_sequence_gaps']}",
        f"Maximum absolute RMS Z error: {comparison_summary['max_abs_rms_z_error_db']} dB",
        f"Maximum absolute RMS A error: {comparison_summary['max_abs_rms_a_error_db']} dB",
        f"Maximum absolute LAeq error: {comparison_summary['max_abs_laeq_error_db']} dB",
        f"Maximum absolute peak error: {comparison_summary['max_abs_peak_error_db']} dB",
        f"Sample count mismatches: {comparison_summary['sample_count_mismatches']}",
        f"Mean mismatches: {comparison_summary['mean_mismatches']}",
        f"Clipping mismatches: {comparison_summary['clipping_mismatches']}",
        f"Class mismatches: {comparison_summary['class_mismatches']}",
        f"Flag mismatches: {comparison_summary['flag_mismatches']}",
        "Flag comparison mask: 0x7F (bits 0..6; reserved bit 7 excluded)",
        "",
        "AUD0 window layouts:",
    ]
    for window in audio_window_rows:
        lines.append(
            f"  Window {window['audio_window_index']}: bytes={window['audio_bytes']}, "
            f"complete={window['complete']}, valid={window['structurally_valid']}, "
            f"pattern={window['payload_pattern']}"
        )
    lines.append("")
    lines.extend(f"Warning: {warning}" for warning in comparison_summary["warnings"])
    if comparison_summary["warnings"]:
        lines.append("")

    for row in comparison_df.to_dict("records"):
        lines.extend(
            [
                f"Window {row['audio_window_index']} / sequence {row['firmware_window_sequence']}:",
                f"  RMS Z error = {row['rms_z_error_db']}",
                f"  RMS A error = {row['rms_a_error_db']}",
                f"  LAeq error = {row['laeq_error_db']}",
                f"  Peak error = {row['peak_error_db']}",
                f"  class match = {row['environment_match']}",
                f"  flags match = {row['flags_match']}",
                f"  {'PASS' if row['overall_match'] else 'FAIL'}",
            ]
        )

    lines.extend(["", f"Overall result: {comparison_summary['final_result']}"])
    report_filename.write_text("\n".join(lines) + "\n", encoding="utf-8")


def assign_light_boot_sessions(
    light_feature_df: pd.DataFrame,
) -> tuple[pd.DataFrame, int, int]:
    """Assign boot sessions from HAL tick resets without changing row order."""
    result = light_feature_df.copy()

    if result.empty:
        result["boot_session"] = pd.Series(dtype="int64")
        return result, 0, 0

    boot_sessions = []
    current_session = 0
    timestamp_resets = 0
    unexpected_regressions = 0
    previous_timestamp = None
    previous_sequence = None

    for row in result.itertuples(index=False):
        timestamp = int(row.sample_timestamp_ms)
        sequence = int(row.window_sequence)

        if previous_timestamp is not None and timestamp < previous_timestamp:
            if previous_sequence is not None and sequence > previous_sequence:
                current_session += 1
                timestamp_resets += 1
            else:
                unexpected_regressions += 1

        boot_sessions.append(current_session)
        previous_timestamp = timestamp
        previous_sequence = sequence

    result["boot_session"] = pd.Series(
        boot_sessions,
        index=result.index,
        dtype="int64",
    )
    return result, timestamp_resets, unexpected_regressions


def analyze_audio_feature_dataframe(
    audio_feature_df: pd.DataFrame,
    parser_invalid_records: int = 0,
) -> tuple[pd.DataFrame, dict]:
    """Add session metadata and compute AFEA-only descriptive metrics."""
    result = audio_feature_df.copy()
    if result.empty:
        result["boot_session"] = pd.Series(dtype="int64")
        result["session_elapsed_s"] = pd.Series(dtype="float64")
        result["observed_elapsed_s"] = pd.Series(dtype="float64")
        result["record_valid"] = pd.Series(dtype="bool")
        result["environment_class_name"] = pd.Series(dtype="object")
        result["environment_class_range"] = pd.Series(dtype="object")
        result["python_environment_class"] = pd.Series(dtype="Int64")
        result["environment_class_match"] = pd.Series(dtype="boolean")
        return result, {
            "total_records": int(parser_invalid_records),
            "decoded_records": 0,
            "valid_records": 0,
            "invalid_records": int(parser_invalid_records),
            "valid_percentage": 0.0,
            "first_window_sequence": None,
            "last_window_sequence": None,
            "first_timestamp_ms": None,
            "last_timestamp_ms": None,
            "detected_boot_sessions": 0,
            "records_per_boot_session": {},
            "session_durations_s": {},
            "total_observed_duration_s": 0.0,
            "global_timestamp_regressions": 0,
            "recognized_timestamp_resets": 0,
            "unexpected_timestamp_regressions": 0,
            "non_monotonic_timestamps_within_sessions": 0,
            "clipping_event_records": 0,
            "clipped_sample_total": 0,
            "flag_counts": {column: 0 for column, _label in AUDIO_FEATURE_FLAG_FIELDS},
            "feature_stats": {},
            "environment_class_distribution": [
                {
                    "code": code,
                    "name": AUDIO_ENVIRONMENT_CLASS_NAMES[code],
                    "range": AUDIO_ENVIRONMENT_CLASS_RANGES[code],
                    "count": 0,
                    "percentage": 0.0,
                }
                for code in range(7)
            ],
            "unknown_environment_class_records": 0,
            "environment_class_evaluated": 0,
            "environment_class_matches": 0,
            "environment_class_mismatches": 0,
            "environment_class_not_evaluated": 0,
            "warnings": [],
        }

    session_source = result[["window_sequence", "window_start_ms"]].rename(
        columns={"window_start_ms": "sample_timestamp_ms"}
    )
    (
        session_source,
        recognized_timestamp_resets,
        unexpected_timestamp_regressions,
    ) = assign_light_boot_sessions(session_source)
    result["boot_session"] = session_source["boot_session"].to_numpy(dtype=np.int64)
    result["session_elapsed_s"] = np.nan
    result["observed_elapsed_s"] = np.nan

    session_durations_s = {}
    records_per_boot_session = {}
    non_monotonic_within_sessions = 0
    for session, indexes in result.groupby("boot_session", sort=True).groups.items():
        session_timestamps = pd.to_numeric(
            result.loc[indexes, "window_start_ms"], errors="coerce"
        )
        finite_timestamps = session_timestamps[np.isfinite(session_timestamps)]
        records_per_boot_session[int(session)] = int(len(indexes))
        if finite_timestamps.empty:
            session_durations_s[int(session)] = None
            continue
        first_timestamp = float(finite_timestamps.iloc[0])
        result.loc[indexes, "session_elapsed_s"] = (
            session_timestamps - first_timestamp
        ) / 1000.0
        session_durations_s[int(session)] = float(
            (finite_timestamps.iloc[-1] - finite_timestamps.iloc[0]) / 1000.0
        )
        non_monotonic_within_sessions += int(
            (finite_timestamps.diff().dropna() < 0).sum()
        )

    positive_steps_s = []
    for _session, session_df in result.groupby("boot_session", sort=True):
        steps = pd.to_numeric(
            session_df["session_elapsed_s"], errors="coerce"
        ).diff().to_numpy(float)
        positive_steps_s.extend(steps[np.isfinite(steps) & (steps > 0)].tolist())
    session_gap_s = float(np.median(positive_steps_s)) if positive_steps_s else 1.0
    observed_offset_s = 0.0
    for _session, indexes in result.groupby("boot_session", sort=True).groups.items():
        elapsed = pd.to_numeric(result.loc[indexes, "session_elapsed_s"], errors="coerce")
        result.loc[indexes, "observed_elapsed_s"] = elapsed + observed_offset_s
        finite_elapsed = elapsed[np.isfinite(elapsed)]
        if not finite_elapsed.empty:
            observed_offset_s += float(finite_elapsed.max()) + session_gap_s

    sample_count = pd.to_numeric(result["sample_count"], errors="coerce")
    record_valid = (
        result["flag_complete"].astype(bool)
        & result["flag_acquisition_valid"].astype(bool)
        & sample_count.gt(0)
    )
    result["record_valid"] = record_valid.astype(bool)
    decoded_records = int(len(result))
    valid_records = int(record_valid.sum())
    semantic_invalid_records = decoded_records - valid_records
    invalid_records = semantic_invalid_records + int(parser_invalid_records)
    total_records = valid_records + invalid_records

    analysis_warnings = []
    environment_codes = pd.to_numeric(result["environment_class"], errors="coerce")
    environment_class_names = []
    environment_class_ranges = []
    unknown_environment_values = set()
    for raw_value, numeric_value in zip(result["environment_class"], environment_codes):
        if np.isfinite(numeric_value) and float(numeric_value).is_integer():
            code = int(numeric_value)
            if code in AUDIO_ENVIRONMENT_CLASS_NAMES:
                environment_class_names.append(AUDIO_ENVIRONMENT_CLASS_NAMES[code])
                environment_class_ranges.append(AUDIO_ENVIRONMENT_CLASS_RANGES[code])
                continue
            unknown_text = str(code)
        else:
            unknown_text = str(raw_value)
        environment_class_names.append(f"UNKNOWN_CLASS_{unknown_text}")
        environment_class_ranges.append("UNKNOWN")
        unknown_environment_values.add(unknown_text)

    result["environment_class_name"] = environment_class_names
    result["environment_class_range"] = environment_class_ranges
    for unknown_value in sorted(unknown_environment_values):
        analysis_warnings.append(
            f"Unknown AFEA environment_class={unknown_value}; "
            f"using UNKNOWN_CLASS_{unknown_value}"
        )

    estimated_laeq = pd.to_numeric(result["estimated_laeq_dba"], errors="coerce")
    python_environment_classes = [
        classify_embedded_audio_environment(value) for value in estimated_laeq
    ]
    result["python_environment_class"] = pd.array(
        python_environment_classes, dtype="Int64"
    )
    environment_class_evaluated_mask = (
        environment_codes.isin(range(7)) & np.isfinite(estimated_laeq)
    )
    environment_class_match = pd.Series(
        pd.NA, index=result.index, dtype="boolean"
    )
    environment_class_match.loc[environment_class_evaluated_mask] = (
        environment_codes.loc[environment_class_evaluated_mask].astype(int).to_numpy()
        == result.loc[
            environment_class_evaluated_mask, "python_environment_class"
        ].astype(int).to_numpy()
    )
    result["environment_class_match"] = environment_class_match
    mismatch_mask = environment_class_evaluated_mask & ~environment_class_match.fillna(False)
    for row in result.loc[mismatch_mask].itertuples(index=False):
        analysis_warnings.append(
            f"AFEA window_sequence={int(row.window_sequence)} environment mismatch: "
            f"firmware={int(row.environment_class)}, "
            f"Python={int(row.python_environment_class)}, "
            f"Estimated LAeq={float(row.estimated_laeq_dba):.3f} dBA"
        )

    environment_class_distribution = []
    for code in range(7):
        count = int((environment_codes == code).sum())
        environment_class_distribution.append(
            {
                "code": code,
                "name": AUDIO_ENVIRONMENT_CLASS_NAMES[code],
                "range": AUDIO_ENVIRONMENT_CLASS_RANGES[code],
                "count": count,
                "percentage": (100.0 * count / decoded_records) if decoded_records else 0.0,
            }
        )
    known_environment_records = sum(
        item["count"] for item in environment_class_distribution
    )

    feature_stats = {}
    for column, label, unit in AUDIO_FEATURE_METRIC_FIELDS:
        if column not in result.columns:
            continue
        values = pd.to_numeric(result.loc[record_valid, column], errors="coerce")
        finite = values[np.isfinite(values)].to_numpy(dtype=float)
        feature_stats[column] = {
            "label": label,
            "unit": unit,
            "finite_count": int(finite.size),
            "min": float(np.min(finite)) if finite.size else None,
            "max": float(np.max(finite)) if finite.size else None,
            "mean": float(np.mean(finite)) if finite.size else None,
            "median": float(np.median(finite)) if finite.size else None,
        }

    flag_counts = {
        column: int(result[column].astype(bool).sum())
        for column, _label in AUDIO_FEATURE_FLAG_FIELDS
        if column in result.columns
    }
    clipped_counts = pd.to_numeric(
        result["clipped_sample_count"], errors="coerce"
    ).fillna(0)
    clipping_events = result["flag_clipped"].astype(bool) | clipped_counts.gt(0)
    timestamp_values = pd.to_numeric(result["window_start_ms"], errors="coerce")
    total_observed_duration_s = float(sum(
        duration for duration in session_durations_s.values()
        if duration is not None and np.isfinite(duration) and duration >= 0
    ))

    summary = {
        "total_records": total_records,
        "decoded_records": decoded_records,
        "valid_records": valid_records,
        "invalid_records": invalid_records,
        "valid_percentage": (100.0 * valid_records / total_records) if total_records else 0.0,
        "first_window_sequence": int(result["window_sequence"].iloc[0]),
        "last_window_sequence": int(result["window_sequence"].iloc[-1]),
        "first_timestamp_ms": int(result["window_start_ms"].iloc[0]),
        "last_timestamp_ms": int(result["window_start_ms"].iloc[-1]),
        "detected_boot_sessions": int(result["boot_session"].nunique()),
        "records_per_boot_session": records_per_boot_session,
        "session_durations_s": session_durations_s,
        "total_observed_duration_s": total_observed_duration_s,
        "global_timestamp_regressions": int((timestamp_values.diff().dropna() < 0).sum()),
        "recognized_timestamp_resets": int(recognized_timestamp_resets),
        "unexpected_timestamp_regressions": int(unexpected_timestamp_regressions),
        "non_monotonic_timestamps_within_sessions": non_monotonic_within_sessions,
        "clipping_event_records": int(clipping_events.sum()),
        "clipped_sample_total": int(clipped_counts.sum()),
        "flag_counts": flag_counts,
        "feature_stats": feature_stats,
        "environment_class_distribution": environment_class_distribution,
        "unknown_environment_class_records": decoded_records - known_environment_records,
        "environment_class_evaluated": int(environment_class_evaluated_mask.sum()),
        "environment_class_matches": int(
            (environment_class_evaluated_mask & environment_class_match.fillna(False)).sum()
        ),
        "environment_class_mismatches": int(mismatch_mask.sum()),
        "environment_class_not_evaluated": int(
            decoded_records - environment_class_evaluated_mask.sum()
        ),
        "warnings": analysis_warnings,
    }
    return result, summary


def _format_audio_feature_summary_lines(summary: dict) -> list[str]:
    lines = [
        f"AFEA decoded records: {summary['decoded_records']}",
        f"AFEA valid records: {summary['valid_records']}",
        f"AFEA invalid records: {summary['invalid_records']}",
        f"AFEA valid percentage: {summary['valid_percentage']:.3f}%",
        f"AFEA first window_sequence: {summary['first_window_sequence']}",
        f"AFEA last window_sequence: {summary['last_window_sequence']}",
        f"AFEA first timestamp: {summary['first_timestamp_ms']} ms",
        f"AFEA last timestamp: {summary['last_timestamp_ms']} ms",
        f"AFEA boot sessions: {summary['detected_boot_sessions']}",
        f"AFEA records per boot session: {summary['records_per_boot_session']}",
        f"AFEA session durations: {summary['session_durations_s']} s",
        f"AFEA total observed duration: {summary['total_observed_duration_s']:.3f} s",
        f"AFEA global timestamp regressions: {summary['global_timestamp_regressions']}",
        f"AFEA recognized timestamp resets: {summary['recognized_timestamp_resets']}",
        "AFEA non-monotonic timestamps within sessions: "
        f"{summary['non_monotonic_timestamps_within_sessions']}",
        f"AFEA clipping event records: {summary['clipping_event_records']}",
        f"AFEA clipped sample total: {summary['clipped_sample_total']}",
        f"AFEA environment classes evaluated: {summary['environment_class_evaluated']}",
        f"AFEA environment class matches: {summary['environment_class_matches']}",
        f"AFEA environment class mismatches: {summary['environment_class_mismatches']}",
        "AFEA environment classes not evaluated: "
        f"{summary['environment_class_not_evaluated']}",
        f"AFEA unknown environment class records: {summary['unknown_environment_class_records']}",
    ]
    for column, stats in summary["feature_stats"].items():
        unit = stats["unit"]
        values = (
            f"min={_format_number(stats['min'])}, max={_format_number(stats['max'])}, "
            f"mean={_format_number(stats['mean'])}, "
            f"median={_format_number(stats['median'])} {unit}"
        )
        lines.append(
            f"AFEA {stats['label']} ({column}, {stats['finite_count']} finite): {values}"
        )
    lines.extend(["", "Audio environment class distribution"])
    for item in summary["environment_class_distribution"]:
        lines.append(
            f"  {item['code']} | {item['name']} | {item['range']} | "
            f"count={item['count']} | {item['percentage']:.3f}%"
        )
    return lines


def write_audio_feature_summary_report(report_filename: Path, summary: dict) -> None:
    lines = [
        "Audio feature summary (AFEA)",
        "RMS Z, RMS A-weighted and peak are firmware-provided dBFS values.",
        "Estimated LAeq is firmware-provided dBA; it is not labelled as calibrated dB SPL.",
        "Boot sessions use increasing window_sequence plus window_start_ms resets.",
        "session_elapsed_s resets at each boot; observed_elapsed_s concatenates sessions only for plots.",
        "",
    ]
    lines.extend(_format_audio_feature_summary_lines(summary))
    lines.extend(["", "Official audio environment class legend:"])
    for code in range(7):
        lines.append(
            f"  {code} = {AUDIO_ENVIRONMENT_CLASS_NAMES[code]} | "
            f"{AUDIO_ENVIRONMENT_CLASS_INTERVALS[code]}"
        )
    lines.extend(["", "AFEA flag counts:"])
    for column, label in AUDIO_FEATURE_FLAG_FIELDS:
        lines.append(f"  {label} ({column}): {summary['flag_counts'].get(column, 0)}")
    if summary["warnings"]:
        lines.extend(["", "Warnings:"])
        lines.extend(f"  {warning}" for warning in summary["warnings"])
    report_filename.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _plot_audio_feature_series(
    ax,
    audio_feature_df: pd.DataFrame,
    column: str,
    label: str,
    color: str,
    drawstyle: str = "default",
) -> bool:
    if column not in audio_feature_df.columns:
        ax.text(0.5, 0.5, f"{label} unavailable", ha="center", va="center", transform=ax.transAxes)
        return False
    plotted = False
    for _session, session_df in audio_feature_df.groupby("boot_session", sort=True):
        x = pd.to_numeric(session_df["observed_elapsed_s"], errors="coerce").to_numpy(float)
        y = pd.to_numeric(session_df[column], errors="coerce").to_numpy(float)
        finite = np.isfinite(x) & np.isfinite(y)
        if not finite.any():
            continue
        linestyle = "-" if int(finite.sum()) > 1 else "None"
        ax.plot(
            x[finite], y[finite], color=color, marker="o", markersize=3,
            linewidth=1.2, linestyle=linestyle, drawstyle=drawstyle,
            label=label if not plotted else "_nolegend_",
        )
        plotted = True
    if not plotted:
        ax.text(0.5, 0.5, f"No finite {label} values", ha="center", va="center", transform=ax.transAxes)
    return plotted


def _mark_audio_feature_sessions(ax, audio_feature_df: pd.DataFrame) -> None:
    first_times = audio_feature_df.groupby("boot_session", sort=True)["observed_elapsed_s"].first()
    for elapsed_s in first_times.iloc[1:]:
        ax.axvline(float(elapsed_s), color="#777777", linestyle=":", linewidth=0.9, alpha=0.7)


def plot_audio_feature_records(
    audio_feature_df: pd.DataFrame,
    output_prefix: Path,
) -> list[Path]:
    if audio_feature_df.empty:
        print("No AFEA records available; skipping audio feature plots.")
        return []

    print(f"AFEA records available: {len(audio_feature_df)}. Generating audio feature plots.")
    output_files = []

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, constrained_layout=True)
    timeline_fields = (
        ("mean_counts_rounded", "Mean microphone value [counts]", "#4C78A8", "default"),
        ("sample_count", "Samples per window [samples]", "#59A14F", "default"),
        ("environment_class", "Environment class [category]", "#F28E2B", "steps-mid"),
    )
    for ax, (column, label, color, drawstyle) in zip(axes, timeline_fields):
        _plot_audio_feature_series(ax, audio_feature_df, column, label, color, drawstyle)
        _mark_audio_feature_sessions(ax, audio_feature_df)
        ax.set_ylabel(label)
        ax.grid(True, alpha=0.3)
        if ax.lines:
            ax.legend(fontsize=8)
    environment_axis = axes[2]
    environment_axis.set_yticks(range(7))
    environment_axis.set_yticklabels(AUDIO_ENVIRONMENT_CLASS_TICK_LABELS)
    environment_axis.set_ylim(-0.5, 6.5)
    axes[-1].set_xlabel("Observed time [s] (boot sessions concatenated; dotted lines = resets)")
    axes[0].set_title("AFEA audio features over time")
    timeseries_filename = output_prefix.with_name(
        output_prefix.name + "_audio_feature_timeseries.png"
    )
    fig.savefig(timeseries_filename, dpi=200, bbox_inches="tight")
    plt.close(fig)
    output_files.append(timeseries_filename)

    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True, constrained_layout=True)
    for column, label, color in (
        ("rms_z_dbfs", "RMS Z [dBFS]", "#4C78A8"),
        ("rms_a_dbfs", "RMS A-weighted [dBFS]", "#59A14F"),
        ("peak_dbfs", "Peak [dBFS]", "#E15759"),
    ):
        _plot_audio_feature_series(axes[0], audio_feature_df, column, label, color)
    _plot_audio_feature_series(
        axes[1], audio_feature_df, "estimated_laeq_dba", "Estimated LAeq [dBA]", "#B07AA1"
    )
    for ax in axes:
        _mark_audio_feature_sessions(ax, audio_feature_df)
        ax.grid(True, alpha=0.3)
        if ax.lines:
            ax.legend(fontsize=8)
    axes[0].set_ylabel("Digital level [dBFS]")
    axes[1].set_ylabel("Estimated level [dBA]")
    axes[1].set_xlabel("Observed time [s] (boot sessions concatenated; dotted lines = resets)")
    axes[0].set_title("AFEA levels per window")
    levels_filename = output_prefix.with_name(
        output_prefix.name + "_audio_feature_levels.png"
    )
    fig.savefig(levels_filename, dpi=200, bbox_inches="tight")
    plt.close(fig)
    output_files.append(levels_filename)

    fig, ax = plt.subplots(figsize=(12, 6), constrained_layout=True)
    x = pd.to_numeric(audio_feature_df["observed_elapsed_s"], errors="coerce").to_numpy(float)
    flag_columns = [column for column, _label in AUDIO_FEATURE_FLAG_FIELDS if column in audio_feature_df]
    flag_labels = [label for column, label in AUDIO_FEATURE_FLAG_FIELDS if column in audio_feature_df]
    for lane, column in enumerate(flag_columns):
        values = audio_feature_df[column].astype(bool).to_numpy()
        colors = np.where(values, "#D62728", "#D9D9D9")
        ax.scatter(x, np.full(len(x), lane), c=colors, marker="s", s=28)
    ax.set_yticks(range(len(flag_labels)))
    ax.set_yticklabels(flag_labels)
    ax.set_xlabel("Observed time [s] (boot sessions concatenated)")
    ax.grid(True, axis="x", alpha=0.3)
    event_columns = [
        "flag_clipped", "flag_high_level", "flag_silent_or_unavailable",
        "flag_impulsive_event", "flag_reserved",
    ]
    no_quality_events = not any(
        audio_feature_df[column].astype(bool).any()
        for column in event_columns if column in audio_feature_df
    )
    quality_title = "AFEA validity, quality and event flags (red = active, grey = inactive)"
    if no_quality_events:
        quality_title += "\nNo clipping/high-level/silent/impulsive/reserved events detected"
    ax.set_title(quality_title)
    quality_filename = output_prefix.with_name(
        output_prefix.name + "_audio_feature_quality.png"
    )
    fig.savefig(quality_filename, dpi=200, bbox_inches="tight")
    plt.close(fig)
    output_files.append(quality_filename)

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), constrained_layout=True)
    for column, label, color in (
        ("rms_z_dbfs", "RMS Z [dBFS]", "#4C78A8"),
        ("rms_a_dbfs", "RMS A-weighted [dBFS]", "#59A14F"),
        ("peak_dbfs", "Peak [dBFS]", "#E15759"),
    ):
        values = pd.to_numeric(audio_feature_df[column], errors="coerce").to_numpy(float)
        finite = values[np.isfinite(values)]
        if finite.size:
            axes[0].hist(finite, bins=max(1, min(20, int(np.sqrt(finite.size)))), alpha=0.45, label=label, color=color)
    laeq = pd.to_numeric(audio_feature_df["estimated_laeq_dba"], errors="coerce").to_numpy(float)
    finite_laeq = laeq[np.isfinite(laeq)]
    if finite_laeq.size:
        axes[1].hist(finite_laeq, bins=max(1, min(20, int(np.sqrt(finite_laeq.size)))), color="#B07AA1", alpha=0.75)
    else:
        axes[1].text(0.5, 0.5, "No finite estimated LAeq values", ha="center", va="center", transform=axes[1].transAxes)
    axes[0].set_xlabel("Digital level [dBFS]")
    axes[0].set_ylabel("Windows")
    axes[1].set_xlabel("Estimated LAeq [dBA]")
    axes[1].set_ylabel("Windows")
    axes[0].set_title("AFEA digital-level distributions")
    axes[1].set_title("AFEA estimated LAeq distribution")
    for ax in axes:
        ax.grid(True, axis="y", alpha=0.3)
    if axes[0].patches:
        axes[0].legend(fontsize=8)
    distribution_filename = output_prefix.with_name(
        output_prefix.name + "_audio_feature_distribution.png"
    )
    fig.savefig(distribution_filename, dpi=200, bbox_inches="tight")
    plt.close(fig)
    output_files.append(distribution_filename)

    for output_file in output_files:
        print(f"Audio feature plot saved to: {output_file}")
    return output_files


def validate_light_feature_records(
    light_feature_rows: list[dict],
    stats: ParseStats,
) -> tuple[pd.DataFrame, dict]:
    rows = []
    warnings = []
    previous_python_class = None
    previous_boot_session = None

    sequences = [int(row["window_sequence"]) for row in light_feature_rows]
    timestamps = [int(row["sample_timestamp_ms"]) for row in light_feature_rows]
    sequence_monotonic = all(
        right > left for left, right in zip(sequences, sequences[1:])
    )
    sequence_gaps = [
        (left, right)
        for left, right in zip(sequences, sequences[1:])
        if right > left + 1
    ]
    duplicate_sequences = sorted(
        sequence for sequence in set(sequences) if sequences.count(sequence) > 1
    )
    source_df = pd.DataFrame(light_feature_rows)
    (
        source_df,
        recognized_timestamp_resets,
        unexpected_timestamp_regressions,
    ) = assign_light_boot_sessions(source_df)

    for source in source_df.to_dict("records"):
        row = dict(source)
        boot_session = int(row["boot_session"])
        if previous_boot_session is None or boot_session != previous_boot_session:
            previous_python_class = None
        previous_boot_session = boot_session

        exposure_class = int(row["exposure_class"])
        flags = int(row["flags"])
        complete = bool(row["flag_complete"])
        acquisition_valid = bool(row["flag_acquisition_valid"])
        classification_valid = bool(row["flag_classification_valid"])
        class_in_range = exposure_class in range(5)
        class_flag_consistent = (
            class_in_range if classification_valid
            else exposure_class == LIGHT_EXPOSURE_UNAVAILABLE
        )
        record_valid = (
            complete
            and acquisition_valid
            and classification_valid
            and class_in_range
        )
        expected_saturated = int(row["clear"]) >= LIGHT_SATURATION_CLEAR_COUNTS
        python_previous_class = previous_python_class

        if record_valid:
            if previous_python_class is None:
                python_class = classify_light_without_history(int(row["clear"]))
            else:
                python_class = classify_light_with_hysteresis(
                    int(row["clear"]), previous_python_class
                )
            classification_evaluated = python_class is not None
            classification_match = (
                exposure_class == python_class if classification_evaluated else None
            )
            classification_reason = (
                "" if classification_evaluated
                else "initial hysteresis state unknown"
            )
            saturation_match = (
                bool(row["flag_saturated"]) == expected_saturated
                and (not bool(row["flag_saturated"]) or exposure_class == 4)
            )
            python_flags = (
                LIGHT_FLAG_COMPLETE
                | LIGHT_FLAG_ACQUISITION_VALID
                | LIGHT_FLAG_CLASSIFICATION_VALID
                | (LIGHT_FLAG_SATURATED if expected_saturated else 0)
            )
            if classification_evaluated:
                previous_python_class = python_class
        else:
            python_class = LIGHT_EXPOSURE_UNAVAILABLE
            classification_evaluated = False
            classification_match = class_flag_consistent
            classification_reason = "record not valid"
            saturation_match = not bool(row["flag_saturated"])
            python_flags = 0

        flags_reconstructible_match = (
            flags & LIGHT_RECONSTRUCTIBLE_FLAG_MASK
        ) == (python_flags & LIGHT_RECONSTRUCTIBLE_FLAG_MASK)
        reserved_flags_valid = not bool(
            flags & (LIGHT_FLAG_RESERVED_6 | LIGHT_FLAG_RESERVED_7)
        )
        error_flags_valid = not (
            record_valid
            and (bool(row["flag_i2c_error"]) or bool(row["flag_smux_error"]))
        )
        classification_acceptable = (
            not classification_evaluated or bool(classification_match)
        )
        overall_match = all(
            (
                record_valid,
                class_flag_consistent,
                classification_acceptable,
                saturation_match,
                flags_reconstructible_match,
                reserved_flags_valid,
                error_flags_valid,
            )
        )

        row.update(
            {
                "expected_saturated": expected_saturated,
                "saturation_match": saturation_match,
                "python_previous_exposure_class": python_previous_class,
                "python_exposure_class": python_class,
                "classification_evaluated": classification_evaluated,
                "classification_match": classification_match,
                "classification_reason": classification_reason,
                "python_flags_reconstructible": python_flags,
                "flags_reconstructible_match": flags_reconstructible_match,
                "reserved_flags_valid": reserved_flags_valid,
                "record_valid": record_valid,
                "overall_match": overall_match,
            }
        )
        rows.append(row)

        sequence = int(row["window_sequence"])
        if not class_flag_consistent:
            warnings.append(
                f"Sequence {sequence}: classification flag/class combination is inconsistent"
            )
        if not reserved_flags_valid:
            warnings.append(f"Sequence {sequence}: reserved flag bits are non-zero")
        if not error_flags_valid:
            warnings.append(
                f"Sequence {sequence}: valid record carries I2C_ERROR or SMUX_ERROR"
            )
        if record_valid and not classification_evaluated:
            warnings.append(
                f"Sequence {sequence}: classification not evaluated "
                "(initial hysteresis state unknown)"
            )
        elif record_valid and not classification_match:
            warnings.append(f"Sequence {sequence}: exposure classification mismatch")
        if record_valid and not saturation_match:
            warnings.append(f"Sequence {sequence}: saturation mismatch")
        if not flags_reconstructible_match:
            warnings.append(f"Sequence {sequence}: reconstructible flag mismatch")

    light_feature_df = pd.DataFrame(rows, columns=LIGHT_FEATURE_COLUMNS)
    if not light_feature_df.empty:
        light_feature_df["python_previous_exposure_class"] = pd.array(
            light_feature_df["python_previous_exposure_class"], dtype="Int64"
        )
        light_feature_df["python_exposure_class"] = pd.array(
            light_feature_df["python_exposure_class"], dtype="Int64"
        )
        light_feature_df["classification_match"] = pd.array(
            light_feature_df["classification_match"], dtype="boolean"
        )
    global_timestamp_regressions = int(
        (
            light_feature_df["sample_timestamp_ms"].diff().dropna() < 0
        ).sum()
    ) if not light_feature_df.empty else 0
    detected_boot_sessions = (
        int(light_feature_df["boot_session"].nunique())
        if not light_feature_df.empty else 0
    )
    non_monotonic_timestamps_within_sessions = 0
    for _, session_df in light_feature_df.groupby("boot_session", sort=True):
        non_monotonic_timestamps_within_sessions += int(
            (session_df["sample_timestamp_ms"].diff().dropna() < 0).sum()
        )

    if unexpected_timestamp_regressions:
        warnings.append(
            "Unexpected LFEA timestamp regressions: "
            f"{unexpected_timestamp_regressions}"
        )
    if non_monotonic_timestamps_within_sessions:
        warnings.append(
            "Non-monotonic LFEA timestamps within boot sessions: "
            f"{non_monotonic_timestamps_within_sessions}"
        )

    valid_mask = (
        light_feature_df["record_valid"].astype(bool)
        if not light_feature_df.empty else pd.Series(dtype=bool)
    )
    semantic_invalid_records = int((~valid_mask).sum()) if not valid_mask.empty else 0
    stats.invalid_light_feature_records += semantic_invalid_records

    valid_records = int(valid_mask.sum()) if not valid_mask.empty else 0
    classification_evaluated_mask = (
        valid_mask & light_feature_df["classification_evaluated"].astype(bool)
        if not light_feature_df.empty else pd.Series(dtype=bool)
    )
    classification_match_mask = (
        light_feature_df["classification_match"].fillna(False).astype(bool)
        if not light_feature_df.empty else pd.Series(dtype=bool)
    )
    classification_evaluated = int(classification_evaluated_mask.sum())
    classification_matches = int(
        (classification_evaluated_mask & classification_match_mask).sum()
    ) if not light_feature_df.empty else 0
    classification_mismatches = int(
        (classification_evaluated_mask & ~classification_match_mask).sum()
    ) if not light_feature_df.empty else 0
    classification_not_evaluated = int(
        (valid_mask & ~light_feature_df["classification_evaluated"].astype(bool)).sum()
    ) if not light_feature_df.empty else 0
    saturation_matches = int(
        (valid_mask & light_feature_df["saturation_match"].astype(bool)).sum()
    ) if not light_feature_df.empty else 0
    saturation_mismatches = int(
        (valid_mask & ~light_feature_df["saturation_match"].astype(bool)).sum()
    ) if not light_feature_df.empty else 0
    flag_mismatches = int(
        (~light_feature_df["flags_reconstructible_match"].astype(bool)).sum()
    ) if not light_feature_df.empty else 0
    reserved_violations = int(
        (~light_feature_df["reserved_flags_valid"].astype(bool)).sum()
    ) if not light_feature_df.empty else 0

    structural_failures = (
        stats.invalid_light_feature_pages
        + stats.invalid_light_feature_records
        + stats.light_feature_payload_remainder_bytes
        + int(bool(sequences) and not sequence_monotonic)
        + len(sequence_gaps)
        + len(duplicate_sequences)
        + unexpected_timestamp_regressions
        + non_monotonic_timestamps_within_sessions
    )
    row_failures = int(
        (~light_feature_df["overall_match"].astype(bool)).sum()
    ) if not light_feature_df.empty else 0
    if light_feature_df.empty:
        final_result = "NOT AVAILABLE"
    elif structural_failures or row_failures:
        final_result = "FAIL"
    elif valid_records and classification_evaluated == 0:
        final_result = "NOT AVAILABLE"
    else:
        final_result = "PASS"

    summary = {
        "first_window_sequence": sequences[0] if sequences else None,
        "last_window_sequence": sequences[-1] if sequences else None,
        "sequence_monotonic": sequence_monotonic,
        "sequence_gaps": sequence_gaps,
        "duplicate_sequences": duplicate_sequences,
        "first_timestamp_ms": timestamps[0] if timestamps else None,
        "last_timestamp_ms": timestamps[-1] if timestamps else None,
        "detected_boot_sessions": detected_boot_sessions,
        "global_timestamp_regressions": global_timestamp_regressions,
        "recognized_timestamp_resets": recognized_timestamp_resets,
        "non_monotonic_timestamps_within_sessions":
            non_monotonic_timestamps_within_sessions,
        "unexpected_timestamp_regressions": unexpected_timestamp_regressions,
        "valid_records": valid_records,
        "invalid_records": stats.invalid_light_feature_records,
        "saturated_records": int(light_feature_df["flag_saturated"].sum())
            if not light_feature_df.empty else 0,
        "i2c_error_records": int(light_feature_df["flag_i2c_error"].sum())
            if not light_feature_df.empty else 0,
        "smux_error_records": int(light_feature_df["flag_smux_error"].sum())
            if not light_feature_df.empty else 0,
        "reserved_flag_violations": reserved_violations,
        "classification_evaluated": classification_evaluated,
        "classification_matches": classification_matches,
        "classification_mismatches": classification_mismatches,
        "classification_not_evaluated": classification_not_evaluated,
        "saturation_matches": saturation_matches,
        "saturation_mismatches": saturation_mismatches,
        "flag_mismatches": flag_mismatches,
        "final_result": final_result,
        "warnings": warnings,
    }
    return light_feature_df, summary


def align_compact_feature_records(
    audio_feature_df: pd.DataFrame,
    light_feature_df: pd.DataFrame,
) -> tuple[pd.DataFrame, dict]:
    audio_records = audio_feature_df.to_dict("records")
    light_records = light_feature_df.to_dict("records")
    audio_by_sequence = {}
    light_by_sequence = {}
    for record in audio_records:
        audio_by_sequence.setdefault(int(record["window_sequence"]), []).append(record)
    for record in light_records:
        light_by_sequence.setdefault(int(record["window_sequence"]), []).append(record)

    all_sequences = sorted(set(audio_by_sequence) | set(light_by_sequence))
    rows = []
    for sequence in all_sequences:
        audio_group = audio_by_sequence.get(sequence, [])
        light_group = light_by_sequence.get(sequence, [])
        audio = audio_group[0] if audio_group else None
        light = light_group[0] if light_group else None

        if audio is not None and light is not None:
            pair_status = "PAIRED"
        elif audio is not None:
            pair_status = "AFEA_ONLY"
        else:
            pair_status = "LFEA_ONLY"
        if len(audio_group) > 1:
            pair_status += ";DUPLICATE_AFEA"
        if len(light_group) > 1:
            pair_status += ";DUPLICATE_LFEA"

        audio_start = int(audio["window_start_ms"]) if audio is not None else None
        light_timestamp = (
            int(light["sample_timestamp_ms"]) if light is not None else None
        )
        rows.append(
            {
                "window_sequence": sequence,
                "audio_feature_present": audio is not None,
                "light_feature_present": light is not None,
                "audio_window_start_ms": audio_start,
                "light_sample_timestamp_ms": light_timestamp,
                "light_minus_audio_timestamp_ms": (
                    light_timestamp - audio_start
                    if audio_start is not None and light_timestamp is not None
                    else None
                ),
                "audio_environment_class": (
                    int(audio["environment_class"]) if audio is not None else None
                ),
                "light_exposure_class": (
                    int(light["exposure_class"]) if light is not None else None
                ),
                "audio_flags": int(audio["flags"]) if audio is not None else None,
                "light_flags": int(light["flags"]) if light is not None else None,
                "pair_status": pair_status,
            }
        )

    audio_sequences = set(audio_by_sequence)
    light_sequences = set(light_by_sequence)
    sequence_gaps = [
        (left, right)
        for left, right in zip(all_sequences, all_sequences[1:])
        if right > left + 1
    ]
    summary = {
        "paired_sequences": len(audio_sequences & light_sequences),
        "afea_without_lfea": len(audio_sequences - light_sequences),
        "lfea_without_afea": len(light_sequences - audio_sequences),
        "duplicate_afea_sequences": sorted(
            sequence for sequence, records in audio_by_sequence.items() if len(records) > 1
        ),
        "duplicate_lfea_sequences": sorted(
            sequence for sequence, records in light_by_sequence.items() if len(records) > 1
        ),
        "sequence_gaps": sequence_gaps,
    }
    return pd.DataFrame(rows, columns=COMPACT_FEATURE_ALIGNMENT_COLUMNS), summary


def write_light_feature_validation_report(
    report_filename: Path,
    stats: ParseStats,
    light_feature_df: pd.DataFrame,
    validation_summary: dict,
    alignment_summary: dict,
) -> None:
    lines = [
        "Light feature validation",
        f"LFEA pages: {stats.light_feature_pages}",
        f"LFEA records: {stats.light_feature_records}",
        f"LFEA payload bytes: {stats.light_feature_payload_bytes}",
        f"LFEA invalid pages: {stats.invalid_light_feature_pages}",
        f"LFEA invalid records: {stats.invalid_light_feature_records}",
        f"LFEA remainder bytes: {stats.light_feature_payload_remainder_bytes}",
        f"First window sequence: {validation_summary['first_window_sequence']}",
        f"Last window sequence: {validation_summary['last_window_sequence']}",
        f"Sequence monotonic: {validation_summary['sequence_monotonic']}",
        f"Sequence gaps: {validation_summary['sequence_gaps']}",
        f"Duplicate sequences: {validation_summary['duplicate_sequences']}",
        f"First timestamp: {validation_summary['first_timestamp_ms']}",
        f"Last timestamp: {validation_summary['last_timestamp_ms']}",
        f"Detected boot sessions: {validation_summary['detected_boot_sessions']}",
        f"Global timestamp regressions: {validation_summary['global_timestamp_regressions']}",
        f"Recognized timestamp resets: {validation_summary['recognized_timestamp_resets']}",
        "Non-monotonic timestamps within sessions: "
        f"{validation_summary['non_monotonic_timestamps_within_sessions']}",
        "Unexpected timestamp regressions: "
        f"{validation_summary['unexpected_timestamp_regressions']}",
        f"Valid records: {validation_summary['valid_records']}",
        f"Invalid records: {validation_summary['invalid_records']}",
        f"Saturated records: {validation_summary['saturated_records']}",
        f"I2C error records: {validation_summary['i2c_error_records']}",
        f"SMUX error records: {validation_summary['smux_error_records']}",
        f"Reserved flag violations: {validation_summary['reserved_flag_violations']}",
        f"Classification evaluated: {validation_summary['classification_evaluated']}",
        f"Classification matches: {validation_summary['classification_matches']}",
        f"Classification mismatches: {validation_summary['classification_mismatches']}",
        "Classification not evaluated: "
        f"{validation_summary['classification_not_evaluated']}",
        f"Saturation matches: {validation_summary['saturation_matches']}",
        f"Saturation mismatches: {validation_summary['saturation_mismatches']}",
        f"Flag mismatches: {validation_summary['flag_mismatches']}",
        f"AFEA/LFEA paired sequences: {alignment_summary['paired_sequences']}",
        f"AFEA without LFEA: {alignment_summary['afea_without_lfea']}",
        f"LFEA without AFEA: {alignment_summary['lfea_without_afea']}",
        f"Duplicate AFEA sequences: {alignment_summary['duplicate_afea_sequences']}",
        f"Duplicate LFEA sequences: {alignment_summary['duplicate_lfea_sequences']}",
        f"Compact sequence gaps: {alignment_summary['sequence_gaps']}",
        "",
    ]
    lines.extend(f"Warning: {warning}" for warning in validation_summary["warnings"])
    if validation_summary["warnings"]:
        lines.append("")

    for row in light_feature_df.to_dict("records"):
        classification_evaluated = bool(row["classification_evaluated"])
        python_class = (
            "N/A" if pd.isna(row["python_exposure_class"])
            else int(row["python_exposure_class"])
        )
        classification_match = (
            str(bool(row["classification_match"]))
            if classification_evaluated else "NOT EVALUATED"
        )
        record_result = (
            "NOT EVALUATED"
            if bool(row["record_valid"]) and not classification_evaluated
            else ("PASS" if row["overall_match"] else "FAIL")
        )
        lines.extend(
            [
                f"Sequence {row['window_sequence']}:",
                f"  Clear={row['clear']}",
                f"  firmware_class={row['exposure_class']}",
                f"  python_class={python_class}",
                f"  flags={row['flags_hex']}",
                f"  classification_evaluated={classification_evaluated}",
                f"  classification_match={classification_match}",
                f"  reason={row['classification_reason'] or 'N/A'}",
                f"  saturation_match={row['saturation_match']}",
                f"  {record_result}",
            ]
        )

    lines.extend(["", f"Overall result: {validation_summary['final_result']}"])
    report_filename.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _output_prefix_from_summary(summary_filename: Path) -> Path:
    suffix = "_summary.txt"
    if summary_filename.name.endswith(suffix):
        prefix_name = summary_filename.name[:-len(suffix)]
    else:
        prefix_name = summary_filename.stem
    return summary_filename.with_name(prefix_name)


def _parse_ble_csv_int(row: dict, column: str, csv_line_number: int) -> int:
    raw_value = str(row.get(column, "")).strip()
    try:
        if raw_value.lower().startswith(("0x", "+0x", "-0x")):
            return int(raw_value, 16)
        return int(raw_value, 10)
    except ValueError as exc:
        raise ValueError(
            f"Invalid BLE metadata value at CSV line {csv_line_number}: "
            f"{column}={raw_value!r}"
        ) from exc


def load_ble_virtual_pages(
    bin_filename: Path,
    csv_filename: Path,
) -> tuple[list[VirtualNandPage], BleInputStats]:
    """Load BLE logical pages and pad them with 0xFF for the legacy parser.

    Rows are parsed in ascending page_sequence order (file_offset breaks ties),
    which preserves the logger's chronological order even if CSV rows are moved.
    file_offset is always used for the actual binary read.
    """
    with csv_filename.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        missing_columns = [
            column for column in BLE_PAGE_CSV_COLUMNS
            if column not in (reader.fieldnames or [])
        ]
        if missing_columns:
            raise ValueError(
                "BLE pages CSV is missing required column(s): "
                + ", ".join(missing_columns)
            )

        metadata_rows = []
        for csv_line_number, row in enumerate(reader, start=2):
            parsed = {
                column: _parse_ble_csv_int(row, column, csv_line_number)
                for column in BLE_PAGE_CSV_COLUMNS
                if column != "magic"
            }
            parsed["magic"] = str(row.get("magic", "")).strip()
            parsed["csv_line_number"] = csv_line_number
            metadata_rows.append(parsed)

    metadata_rows.sort(key=lambda row: (row["page_sequence"], row["file_offset"]))
    stats = BleInputStats(logical_pages=len(metadata_rows))
    virtual_pages = []

    if metadata_rows:
        stats.first_page_sequence = metadata_rows[0]["page_sequence"]
        stats.last_page_sequence = metadata_rows[-1]["page_sequence"]
        stats.first_physical_page_index = metadata_rows[0]["physical_page_index"]
        stats.last_physical_page_index = metadata_rows[-1]["physical_page_index"]

    def metadata_mismatch(page_sequence: int, message: str) -> None:
        stats.metadata_mismatches += 1
        print(f"Warning: BLE metadata mismatch for page_sequence={page_sequence}: {message}")

    with bin_filename.open("rb") as bin_file:
        for row in metadata_rows:
            page_sequence = row["page_sequence"]
            logical_page_bytes = row["logical_page_bytes"]
            file_offset = row["file_offset"]

            if file_offset < 0 or logical_page_bytes < 0:
                raise ValueError(
                    f"Invalid BLE file range for page_sequence={page_sequence}: "
                    f"file_offset={file_offset}, logical_page_bytes={logical_page_bytes}"
                )
            if logical_page_bytes > PAGE_SIZE:
                raise ValueError(
                    f"BLE page_sequence={page_sequence} has logical_page_bytes="
                    f"{logical_page_bytes}, larger than virtual page size {PAGE_SIZE}"
                )

            bin_file.seek(file_offset)
            raw_logical_bytes = bin_file.read(logical_page_bytes)
            if len(raw_logical_bytes) != logical_page_bytes:
                raise ValueError(
                    f"Short BLE page read for page_sequence={page_sequence} at "
                    f"file_offset={file_offset}: expected {logical_page_bytes} byte(s), "
                    f"got {len(raw_logical_bytes)}"
                )

            computed_crc32 = binascii.crc32(raw_logical_bytes) & 0xFFFFFFFF
            expected_crc32 = row["page_crc32"] & 0xFFFFFFFF
            if computed_crc32 != expected_crc32:
                stats.crc_mismatches += 1
                print(
                    f"Warning: BLE CRC32 mismatch for page_sequence={page_sequence}: "
                    f"computed=0x{computed_crc32:08X}, expected=0x{expected_crc32:08X}"
                )

            if logical_page_bytes < PAGE_HEADER_SIZE:
                metadata_mismatch(
                    page_sequence,
                    f"logical page has only {logical_page_bytes} byte(s), fewer than "
                    f"the {PAGE_HEADER_SIZE}-byte minimum header",
                )
            else:
                (
                    magic_word,
                    actual_version,
                    actual_header_size,
                    actual_payload_bytes,
                    actual_page_sequence,
                    _page_timestamp_ms,
                ) = struct.unpack_from(PAGE_HEADER_FORMAT, raw_logical_bytes, 0)
                actual_magic = struct.pack("<I", magic_word)
                expected_magic = row["magic"].encode("ascii", errors="replace")

                if len(expected_magic) != 4 or actual_magic != expected_magic:
                    metadata_mismatch(
                        page_sequence,
                        f"header magic={actual_magic!r}, CSV magic={row['magic']!r}",
                    )
                if actual_page_sequence != page_sequence:
                    metadata_mismatch(
                        page_sequence,
                        f"header page_sequence={actual_page_sequence}, CSV={page_sequence}",
                    )
                if actual_version != row["page_version"]:
                    metadata_mismatch(
                        page_sequence,
                        f"header version={actual_version}, CSV={row['page_version']}",
                    )
                if actual_header_size != row["page_header_size"]:
                    metadata_mismatch(
                        page_sequence,
                        f"header size={actual_header_size}, CSV={row['page_header_size']}",
                    )
                if actual_payload_bytes != row["page_payload_bytes"]:
                    metadata_mismatch(
                        page_sequence,
                        f"header payload={actual_payload_bytes}, "
                        f"CSV={row['page_payload_bytes']}",
                    )
                expected_logical_size = actual_header_size + actual_payload_bytes
                if expected_logical_size != logical_page_bytes:
                    metadata_mismatch(
                        page_sequence,
                        f"header_size + payload_bytes={expected_logical_size}, "
                        f"logical_page_bytes={logical_page_bytes}",
                    )
                csv_logical_size = row["page_header_size"] + row["page_payload_bytes"]
                if csv_logical_size != logical_page_bytes:
                    metadata_mismatch(
                        page_sequence,
                        f"CSV header + payload={csv_logical_size}, "
                        f"logical_page_bytes={logical_page_bytes}",
                    )

            virtual_page = raw_logical_bytes.ljust(PAGE_SIZE, b"\xFF")
            virtual_pages.append(
                VirtualNandPage(
                    data=virtual_page,
                    physical_page_index=row["physical_page_index"],
                )
            )

    print(
        f"Loaded {len(virtual_pages)} BLE logical page(s) in ascending "
        "page_sequence order."
    )
    return virtual_pages, stats


def iter_usb_nand_pages(bin_filename: Path):
    """Yield complete physical NAND pages exactly as the historical parser did."""
    with bin_filename.open("rb") as bin_file:
        physical_page_index = 0
        while True:
            page = bin_file.read(PAGE_SIZE)
            if not page:
                break
            if len(page) != PAGE_SIZE:
                print(
                    f"Warning: incomplete final page at index {physical_page_index}; "
                    f"length={len(page)}"
                )
                break
            yield VirtualNandPage(page, physical_page_index)
            physical_page_index += 1


def parse_nand_dump(
    bin_filename: Path,
    imu_csv_filename: Path,
    light_raw_csv_filename: Path,
    light_csv_filename: Path,
    wav_filename: Path,
    summary_filename: Path,
    light_raw_diagnostics_filename: Path,
    audio_sample_rate_hz: int,
    ble_pages_csv_filename: Path | None = None,
):
    sensor_rows = []
    light_raw_rows = []
    light_rows = []
    audio_feature_rows = []
    light_feature_rows = []
    audio_pages = []
    audio_bytes = bytearray()
    light_raw_debug_records = []
    first_light_record_field_map = ""
    stats = ParseStats()

    ble_input_stats = None
    if ble_pages_csv_filename is None:
        input_pages = iter_usb_nand_pages(bin_filename)
    else:
        input_pages, ble_input_stats = load_ble_virtual_pages(
            bin_filename=bin_filename,
            csv_filename=ble_pages_csv_filename,
        )

    for input_page in input_pages:
            page = input_page.data
            physical_page_index = input_page.physical_page_index

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
            is_audio_feature_page = magic == MAGIC_AUDIO_FEATURE
            is_light_feature_page = magic == MAGIC_LIGHT_FEATURE
            if is_light_raw_page:
                stats.light_raw_pages += 1
            if is_audio_feature_page:
                stats.audio_feature_pages += 1
            if is_light_feature_page:
                stats.light_feature_pages += 1

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
                elif is_audio_feature_page:
                    stats.invalid_audio_feature_pages += 1
                elif is_light_feature_page:
                    stats.invalid_light_feature_pages += 1
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
                elif is_audio_feature_page:
                    stats.invalid_audio_feature_pages += 1
                elif is_light_feature_page:
                    stats.invalid_light_feature_pages += 1
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
                elif is_audio_feature_page:
                    stats.invalid_audio_feature_pages += 1
                elif is_light_feature_page:
                    stats.invalid_light_feature_pages += 1
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
                elif is_audio_feature_page:
                    stats.invalid_audio_feature_pages += 1
                elif is_light_feature_page:
                    stats.invalid_light_feature_pages += 1
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
                audio_pages.append(
                    {
                        "payload": bytes(payload),
                        "physical_page_index": physical_page_index,
                        "page_sequence": page_sequence,
                        "page_timestamp_ms": page_timestamp_ms,
                    }
                )

            elif magic == MAGIC_AUDIO_FEATURE:
                if version != AUDIO_FEATURE_RECORD_VERSION:
                    print(
                        f"Warning: AFEA page_sequence={page_sequence} has version={version}; "
                        f"expected {AUDIO_FEATURE_RECORD_VERSION}; page skipped"
                    )
                    stats.invalid_audio_feature_pages += 1
                    physical_page_index += 1
                    continue

                stats.audio_feature_payload_bytes += payload_bytes
                n_records = payload_bytes // AUDIO_FEATURE_RECORD_SIZE
                remainder = payload_bytes % AUDIO_FEATURE_RECORD_SIZE
                if payload_bytes == 0:
                    print(f"Warning: empty AFEA page at page_sequence={page_sequence}")
                    stats.invalid_audio_feature_pages += 1
                if remainder:
                    print(
                        f"Warning: AFEA page_sequence={page_sequence} has "
                        f"payload_bytes={payload_bytes}; ignoring {remainder} trailing byte(s)"
                    )
                    stats.invalid_audio_feature_pages += 1
                    stats.audio_feature_payload_remainder_bytes += int(remainder)

                for record_index_in_page in range(n_records):
                    record_start = record_index_in_page * AUDIO_FEATURE_RECORD_SIZE
                    record_end = record_start + AUDIO_FEATURE_RECORD_SIZE
                    record = payload[record_start:record_end]
                    try:
                        parsed = parse_audio_feature_record(record)
                    except (ValueError, struct.error) as exc:
                        print(f"Warning: invalid AFEA record skipped: {exc}")
                        stats.invalid_audio_feature_records += 1
                        continue

                    parsed["physical_page_index"] = physical_page_index
                    parsed["page_sequence"] = page_sequence
                    parsed["page_timestamp_ms"] = page_timestamp_ms
                    parsed["record_index_in_page"] = record_index_in_page
                    parsed["record_payload_offset"] = record_start
                    parsed["page_version"] = version
                    parsed["page_header_size"] = header_size
                    parsed["page_payload_bytes"] = payload_bytes
                    audio_feature_rows.append(parsed)

            elif magic == MAGIC_LIGHT_FEATURE:
                if version != LIGHT_FEATURE_RECORD_VERSION:
                    print(
                        f"Warning: LFEA page_sequence={page_sequence} has version={version}; "
                        f"expected {LIGHT_FEATURE_RECORD_VERSION}; page skipped"
                    )
                    stats.invalid_light_feature_pages += 1
                    physical_page_index += 1
                    continue

                stats.light_feature_payload_bytes += payload_bytes
                n_records = payload_bytes // LIGHT_FEATURE_RECORD_SIZE
                remainder = payload_bytes % LIGHT_FEATURE_RECORD_SIZE
                if payload_bytes == 0:
                    print(f"Warning: empty LFEA page at page_sequence={page_sequence}")
                    stats.invalid_light_feature_pages += 1
                if remainder:
                    print(
                        f"Warning: LFEA page_sequence={page_sequence} has "
                        f"payload_bytes={payload_bytes}; ignoring {remainder} trailing byte(s)"
                    )
                    stats.invalid_light_feature_pages += 1
                    stats.light_feature_payload_remainder_bytes += int(remainder)

                for record_index_in_page in range(n_records):
                    record_start = record_index_in_page * LIGHT_FEATURE_RECORD_SIZE
                    record_end = record_start + LIGHT_FEATURE_RECORD_SIZE
                    record = payload[record_start:record_end]
                    try:
                        parsed = parse_light_feature_record(record)
                    except (ValueError, struct.error) as exc:
                        print(f"Warning: invalid LFEA record skipped: {exc}")
                        stats.invalid_light_feature_records += 1
                        continue

                    parsed["physical_page_index"] = physical_page_index
                    parsed["page_sequence"] = page_sequence
                    parsed["page_timestamp_ms"] = page_timestamp_ms
                    parsed["record_index_in_page"] = record_index_in_page
                    parsed["record_payload_offset"] = record_start
                    parsed["page_version"] = version
                    parsed["page_header_size"] = header_size
                    parsed["page_payload_bytes"] = payload_bytes
                    light_feature_rows.append(parsed)

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
    stats.audio_feature_records = len(audio_feature_rows)
    stats.light_feature_records = len(light_feature_rows)

    audio_window_rows, complete_audio_windows, audio_window_warnings = (
        reconstruct_audio_windows(audio_pages)
    )
    for warning in audio_window_warnings:
        print(f"Warning: {warning}")
    stats.audio_complete_windows = len(complete_audio_windows)
    stats.audio_partial_windows = len(audio_window_rows) - len(complete_audio_windows)

    audio_feature_df, audio_feature_summary = analyze_audio_feature_dataframe(
        pd.DataFrame(audio_feature_rows, columns=AUDIO_FEATURE_COLUMNS),
        parser_invalid_records=stats.invalid_audio_feature_records,
    )
    for warning in audio_feature_summary["warnings"]:
        print(f"Warning: {warning}")
    audio_window_df = pd.DataFrame(audio_window_rows, columns=AUDIO_WINDOW_COLUMNS)
    light_feature_df, light_feature_validation = validate_light_feature_records(
        light_feature_rows=light_feature_rows,
        stats=stats,
    )
    compact_alignment_df, compact_alignment_summary = align_compact_feature_records(
        audio_feature_df=audio_feature_df,
        light_feature_df=light_feature_df,
    )
    stats.light_feature_validation_result = light_feature_validation["final_result"]
    for warning in light_feature_validation["warnings"]:
        print(f"Warning: {warning}")

    comparison_df, comparison_summary = compare_audio_feature_records(
        complete_windows=complete_audio_windows,
        audio_feature_rows=audio_feature_rows,
        sample_rate_hz=audio_sample_rate_hz,
    )
    structural_failure_count = (
        stats.audio_partial_windows
        + stats.invalid_audio_feature_pages
        + stats.invalid_audio_feature_records
    )
    if structural_failure_count:
        comparison_summary["comparison_failures"] += structural_failure_count
        comparison_summary["warnings"].append(
            "Structural audio anomalies: "
            f"AUD0={stats.audio_partial_windows}, "
            f"AFEA pages={stats.invalid_audio_feature_pages}, "
            f"AFEA records={stats.invalid_audio_feature_records}"
        )
        if comparison_summary["paired_windows"]:
            comparison_summary["final_result"] = "FAIL"
    for warning in comparison_summary["warnings"]:
        print(f"Warning: {warning}")

    stats.audio_feature_paired_windows = comparison_summary["paired_windows"]
    stats.audio_feature_comparison_failures = comparison_summary["comparison_failures"]
    stats.audio_feature_comparison_result = comparison_summary["final_result"]

    output_prefix = _output_prefix_from_summary(summary_filename)
    audio_feature_csv_filename = output_prefix.with_name(
        output_prefix.name + "_audio_feature_records.csv"
    )
    audio_feature_comparison_csv_filename = output_prefix.with_name(
        output_prefix.name + "_audio_feature_comparison.csv"
    )
    audio_feature_comparison_report_filename = output_prefix.with_name(
        output_prefix.name + "_audio_feature_comparison.txt"
    )
    audio_feature_summary_report_filename = output_prefix.with_name(
        output_prefix.name + "_audio_feature_summary.txt"
    )
    light_feature_csv_filename = output_prefix.with_name(
        output_prefix.name + "_light_feature_records.csv"
    )
    light_feature_validation_report_filename = output_prefix.with_name(
        output_prefix.name + "_light_feature_validation.txt"
    )
    compact_alignment_csv_filename = output_prefix.with_name(
        output_prefix.name + "_compact_feature_alignment.csv"
    )

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
    audio_feature_df.to_csv(audio_feature_csv_filename, index=False)
    comparison_df.to_csv(audio_feature_comparison_csv_filename, index=False)
    light_feature_df.to_csv(light_feature_csv_filename, index=False)
    compact_alignment_df.to_csv(compact_alignment_csv_filename, index=False)
    write_audio_feature_comparison_report(
        report_filename=audio_feature_comparison_report_filename,
        stats=stats,
        audio_window_rows=audio_window_rows,
        comparison_df=comparison_df,
        comparison_summary=comparison_summary,
    )
    write_audio_feature_summary_report(
        report_filename=audio_feature_summary_report_filename,
        summary=audio_feature_summary,
    )
    audio_feature_plot_files = plot_audio_feature_records(
        audio_feature_df=audio_feature_df,
        output_prefix=output_prefix,
    )
    write_light_feature_validation_report(
        report_filename=light_feature_validation_report_filename,
        stats=stats,
        light_feature_df=light_feature_df,
        validation_summary=light_feature_validation,
        alignment_summary=compact_alignment_summary,
    )

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

    input_summary = ""
    if ble_input_stats is not None:
        input_summary = (
            "Input source: BLE file\n"
            f"BLE pages bin filename: {bin_filename.name}\n"
            f"BLE pages CSV filename: {ble_pages_csv_filename.name}\n"
            f"BLE logical pages: {ble_input_stats.logical_pages}\n"
            f"BLE first page_sequence: {ble_input_stats.first_page_sequence}\n"
            f"BLE last page_sequence: {ble_input_stats.last_page_sequence}\n"
            "BLE first physical_page_index: "
            f"{ble_input_stats.first_physical_page_index}\n"
            "BLE last physical_page_index: "
            f"{ble_input_stats.last_physical_page_index}\n"
            f"BLE CRC mismatches: {ble_input_stats.crc_mismatches}\n"
            f"BLE metadata mismatches: {ble_input_stats.metadata_mismatches}\n"
            "BLE page order: page_sequence ascending (file_offset used for reads)\n\n"
        )

    summary = (
        input_summary
        + f"Total pages: {stats.total_pages}\n"
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
        "\nAudio feature records\n"
        f"AFEA pages: {stats.audio_feature_pages}\n"
        f"AFEA records: {stats.audio_feature_records}\n"
        f"AFEA payload bytes: {stats.audio_feature_payload_bytes}\n"
        f"AFEA payload remainder: {stats.audio_feature_payload_remainder_bytes}\n"
        f"AUD0 complete windows: {stats.audio_complete_windows}\n"
        f"AUD0 partial windows: {stats.audio_partial_windows}\n"
        f"AFEA comparison paired windows: {stats.audio_feature_paired_windows}\n"
        f"AFEA comparison failures: {stats.audio_feature_comparison_failures}\n"
        f"AFEA comparison final result: {stats.audio_feature_comparison_result}\n"
        f"AFEA CSV filename: {audio_feature_csv_filename.name}\n"
        f"AFEA comparison CSV filename: {audio_feature_comparison_csv_filename.name}\n"
        f"AFEA comparison report filename: {audio_feature_comparison_report_filename.name}\n"
        f"AFEA summary report filename: {audio_feature_summary_report_filename.name}\n"
        "AFEA plot filenames: "
        f"{', '.join(path.name for path in audio_feature_plot_files) or 'None'}\n"
        + "\n".join(_format_audio_feature_summary_lines(audio_feature_summary))
        + "\n"
        "\nLight feature records\n"
        f"LFEA pages: {stats.light_feature_pages}\n"
        f"LFEA records: {stats.light_feature_records}\n"
        f"LFEA payload bytes: {stats.light_feature_payload_bytes}\n"
        f"LFEA payload remainder: {stats.light_feature_payload_remainder_bytes}\n"
        f"LFEA invalid pages: {stats.invalid_light_feature_pages}\n"
        f"LFEA invalid records: {stats.invalid_light_feature_records}\n"
        f"LFEA valid records: {light_feature_validation['valid_records']}\n"
        f"LFEA saturated records: {light_feature_validation['saturated_records']}\n"
        f"LFEA detected boot sessions: {light_feature_validation['detected_boot_sessions']}\n"
        f"LFEA global timestamp regressions: {light_feature_validation['global_timestamp_regressions']}\n"
        f"LFEA recognized timestamp resets: {light_feature_validation['recognized_timestamp_resets']}\n"
        "LFEA non-monotonic timestamps within sessions: "
        f"{light_feature_validation['non_monotonic_timestamps_within_sessions']}\n"
        "LFEA unexpected timestamp regressions: "
        f"{light_feature_validation['unexpected_timestamp_regressions']}\n"
        f"LFEA classification matches: {light_feature_validation['classification_matches']}\n"
        f"LFEA classification mismatches: {light_feature_validation['classification_mismatches']}\n"
        "LFEA classification not evaluated: "
        f"{light_feature_validation['classification_not_evaluated']}\n"
        f"LFEA flag mismatches: {light_feature_validation['flag_mismatches']}\n"
        f"AFEA/LFEA paired sequences: {compact_alignment_summary['paired_sequences']}\n"
        f"AFEA without LFEA: {compact_alignment_summary['afea_without_lfea']}\n"
        f"LFEA without AFEA: {compact_alignment_summary['lfea_without_afea']}\n"
        f"LFEA validation result: {stats.light_feature_validation_result}\n"
        f"LFEA CSV filename: {light_feature_csv_filename.name}\n"
        f"LFEA validation report filename: {light_feature_validation_report_filename.name}\n"
        f"Compact alignment CSV filename: {compact_alignment_csv_filename.name}\n"
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
    audio_metrics = analyze_and_export_audio(
        audio_bytes=bytes(audio_bytes),
        sample_rate_hz=audio_sample_rate_hz,
        original_wav_filename=wav_filename,
        summary_filename=summary_filename,
    )

    print(summary)
    print(f"IMU CSV saved to: {imu_csv_filename}")
    print(f"Light raw CSV saved to: {light_raw_csv_filename}")
    print(f"Legacy light CSV saved to: {light_csv_filename}")
    print(f"Audio feature CSV saved to: {audio_feature_csv_filename}")
    print(f"Audio feature comparison CSV saved to: {audio_feature_comparison_csv_filename}")
    print(f"Audio feature comparison report saved to: {audio_feature_comparison_report_filename}")
    print(f"Audio feature summary report saved to: {audio_feature_summary_report_filename}")
    print(f"Light feature CSV saved to: {light_feature_csv_filename}")
    print(f"Light feature validation report saved to: {light_feature_validation_report_filename}")
    print(f"Compact feature alignment CSV saved to: {compact_alignment_csv_filename}")
    if audio_metrics is not None:
        print(f"Audio WAV saved to: {wav_filename}")
    print(f"Summary saved to: {summary_filename}")

    return (
        imu_df,
        light_raw_df,
        light_df,
        light_feature_df,
        bytes(audio_bytes),
        stats,
        light_raw_diagnostics,
    )


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

    fig_acc, ax_acc = plt.subplots(figsize=(12, 5))
    ax_acc.plot(t, df["acc_x_g"], label="Acc X")
    ax_acc.plot(t, df["acc_y_g"], label="Acc Y")
    ax_acc.plot(t, df["acc_z_g"], label="Acc Z")
    ax_acc.set_xlabel(x_label)
    ax_acc.set_ylabel("Acceleration [g]")
    ax_acc.set_title("Accelerometer")
    ax_acc.grid(True)
    ax_acc.legend()
    acc_png = output_prefix.with_name(output_prefix.name + "_accelerometer.png")
    fig_acc.savefig(acc_png, dpi=200, bbox_inches="tight")
    plt.close(fig_acc)

    fig_gyro, ax_gyro = plt.subplots(figsize=(12, 5))
    ax_gyro.plot(t, df["gyro_x_dps"], label="Gyro X")
    ax_gyro.plot(t, df["gyro_y_dps"], label="Gyro Y")
    ax_gyro.plot(t, df["gyro_z_dps"], label="Gyro Z")
    ax_gyro.set_xlabel(x_label)
    ax_gyro.set_ylabel("Angular rate [deg/s]")
    ax_gyro.set_title("Gyroscope")
    ax_gyro.grid(True)
    ax_gyro.legend()
    gyro_png = output_prefix.with_name(output_prefix.name + "_gyroscope.png")
    fig_gyro.savefig(gyro_png, dpi=200, bbox_inches="tight")
    plt.close(fig_gyro)


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

        fig, ax = plt.subplots(figsize=(10, 5))
        ax.bar(x_labels, values)
        ax.set_ylim(0, 1.05)
        ax.set_xlabel("Channel and central wavelength")
        ax.set_ylabel("Normalized response")
        ax.set_title(title)
        ax.grid(axis="y", alpha=0.35)
        light_png = output_prefix.with_name(output_prefix.name + suffix)
        fig.savefig(light_png, dpi=200, bbox_inches="tight")
        plt.close(fig)

        print(f"Ambient light index: {row['clear_mean_counts']} counts")
        print(f"Light level: {row['light_level_label']}")


def plot_light_feature_records(
    light_feature_df: pd.DataFrame,
    output_prefix: Path,
) -> None:
    if light_feature_df.empty:
        print("No LFEA pages found; skipping compact light feature plots.")
        return

    print(f"LFEA records available: {len(light_feature_df)}.")
    x = light_feature_df["window_sequence"].to_numpy(dtype=float)
    channel_plots = [
        ("F1 - 415 nm", "f1", "#7F3FBF", "-"),
        ("F2 - 445 nm", "f2", "#123B73", "-"),
        ("F3 - 480 nm", "f3", "#00AEEF", "-"),
        ("F4 - 515 nm", "f4", "#00D5D8", "-"),
        ("F5 - 555 nm", "f5", "#00A651", "-"),
        ("F6 - 590 nm", "f6", "#F2D600", "-"),
        ("F7 - 630 nm", "f7", "#FF9900", "-"),
        ("F8 - 680 nm", "f8", "#FF1F1F", "-"),
        ("Clear - broadband", "clear", "#666666", "--"),
        ("NIR - 910 nm", "nir", "#C00000", "-"),
    ]

    fig_channels, ax_channels = plt.subplots(figsize=(12, 6))
    for label, column, color, linestyle in channel_plots:
        ax_channels.plot(
            x,
            light_feature_df[column].to_numpy(dtype=float),
            label=label,
            color=color,
            linestyle=linestyle,
            marker="o",
            markersize=3,
            linewidth=1.1,
        )
    ax_channels.set_xlabel("Window sequence")
    ax_channels.set_ylabel("Raw counts")
    ax_channels.set_title("AS7341 compact light feature channels")
    ax_channels.grid(True, alpha=0.3)
    ax_channels.legend(ncol=2, fontsize=8)
    channels_filename = output_prefix.with_name(
        output_prefix.name + "_light_feature_channels.png"
    )
    fig_channels.savefig(channels_filename, dpi=200, bbox_inches="tight")
    plt.close(fig_channels)
    print(f"Light feature channel plot saved to: {channels_filename}")

    valid_classes = light_feature_df["exposure_class"].isin(range(5))
    fig_class, ax_class = plt.subplots(figsize=(12, 4))
    ax_class.step(
        x[valid_classes.to_numpy()],
        light_feature_df.loc[valid_classes, "exposure_class"].to_numpy(dtype=float),
        where="mid",
        color="#2E6F95",
        linewidth=1.5,
        label="Exposure class",
    )
    saturated = valid_classes & light_feature_df["flag_saturated"].astype(bool)
    if saturated.any():
        ax_class.scatter(
            light_feature_df.loc[saturated, "window_sequence"],
            light_feature_df.loc[saturated, "exposure_class"],
            color="#C00000",
            marker="o",
            s=45,
            zorder=3,
            label="Saturated",
        )
    ax_class.set_xlabel("Window sequence")
    ax_class.set_ylabel("Exposure class")
    ax_class.set_yticks(range(5))
    ax_class.set_yticklabels(["DARK", "LOW", "MODERATE", "HIGH", "VERY_HIGH"])
    ax_class.set_ylim(-0.25, 4.25)
    ax_class.set_title("AS7341 embedded exposure classification")
    ax_class.grid(True, alpha=0.3)
    ax_class.legend()
    class_filename = output_prefix.with_name(
        output_prefix.name + "_light_feature_exposure_class.png"
    )
    fig_class.savefig(class_filename, dpi=200, bbox_inches="tight")
    plt.close(fig_class)
    print(f"Light feature class plot saved to: {class_filename}")


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

    # Complete channel definition, also used by the combined plot.
    channel_plots = [
    # label, DataFrame column, color, line style
    ("F1 - 415 nm", "f1_counts", "#7F3FBF", "-"),       # violet
    ("F2 - 445 nm", "f2_counts", "#123B73", "-"),       # dark blue
    ("F3 - 480 nm", "f3_counts", "#00AEEF", "-"),       # blue/cyan
    ("F4 - 515 nm", "f4_counts", "#00D5D8", "-"),       # cyan
    ("F5 - 555 nm", "f5_counts", "#00A651", "-"),       # green
    ("F6 - 590 nm", "f6_counts", "#F2D600", "-"),       # yellow
    ("F7 - 630 nm", "f7_counts", "#FF9900", "-"),       # orange
    ("F8 - 680 nm", "f8_counts", "#FF1F1F", "-"),       # red
    ("Clear - broadband", "clear_counts", "#666666", "--"),
    ("NIR - 910 nm", "nir_counts", "#C00000", "-"),     # dark red
]

    # Subdivision requested for the separate figures.
    channel_groups = [
        {
            "title": "AS7341 raw channel counts: F1-F4",
            "suffix": "_light_raw_f1_f4.png",
            "channels": channel_plots[0:4],
        },
        {
            "title": "AS7341 raw channel counts: F5-F8",
            "suffix": "_light_raw_f5_f8.png",
            "channels": channel_plots[4:8],
        },
        {
            "title": "AS7341 raw channel counts: Clear and NIR",
            "suffix": "_light_raw_clear_nir.png",
            "channels": channel_plots[8:10],
        },
    ]

    x, x_label = get_light_plot_x_axis(light_raw_df, diagnostics)

    title_suffix = ""
    if diagnostics.get("is_severely_suspicious"):
        title_suffix = "\nWARNING: suspicious LRAW data"

    # ------------------------------------------------------------------
    # Three separate multi-panel figures
    # ------------------------------------------------------------------
    for group in channel_groups:
        group_channels = group["channels"]

        fig, axes = plt.subplots(
            nrows=len(group_channels),
            ncols=1,
            figsize=(14, 3.2 * len(group_channels)),
            sharex=True,
            constrained_layout=True,
        )

        # With two or more channels, axes is normally an ndarray.
        # np.atleast_1d also makes the code robust for future one-channel groups.
        axes = np.atleast_1d(axes)

        fig.suptitle(
            group["title"] + title_suffix,
            fontsize=14,
        )

        for axis, (label, column, color, linestyle) in zip(axes, group_channels):
            y = light_raw_df[column].to_numpy(dtype=float)

            axis.plot(
                x,
                y,
                linewidth=1.4,
                label=label,
                color=color,
                linestyle=linestyle,
            )

            axis.set_ylabel("Raw ADC counts")
            axis.grid(True, alpha=0.45)
            axis.legend(loc="upper right")

        axes[-1].set_xlabel(x_label)

        output_file = output_prefix.with_name(
            output_prefix.name + group["suffix"]
        )

        fig.savefig(
            output_file,
            dpi=200,
            bbox_inches="tight",
        )
        plt.close(fig)

        print(f"Raw light channel plot saved to: {output_file}")

    # ------------------------------------------------------------------
    # Existing combined plot with all ten channels
    # ------------------------------------------------------------------
    fig_all, ax_all = plt.subplots(
        figsize=(14, 7),
        constrained_layout=True,
    )

    for label, column, color, linestyle in channel_plots:
        ax_all.plot(
        x,
        light_raw_df[column].to_numpy(dtype=float),
        label=label,
        linewidth=1.2,
        color=color,
        linestyle=linestyle,
    )

    ax_all.set_xlabel(x_label)
    ax_all.set_ylabel("Raw ADC counts")
    ax_all.set_title(
        "AS7341 raw channel counts over time" + title_suffix
    )
    ax_all.grid(True, alpha=0.45)
    ax_all.legend(ncol=2)

    all_channels_file = output_prefix.with_name(
        output_prefix.name + "_light_raw_all_channels.png"
    )

    fig_all.savefig(
        all_channels_file,
        dpi=200,
        bbox_inches="tight",
    )
    plt.close(fig_all)

    print(f"Combined raw light plot saved to: {all_channels_file}")

    # ------------------------------------------------------------------
    # Existing separate NIR plot with maximum annotation
    # ------------------------------------------------------------------
    nir_max_idx = light_raw_df["nir_counts"].idxmax()
    nir_max_row = light_raw_df.loc[nir_max_idx]

    nir_max_value = int(nir_max_row["nir_counts"])
    nir_max_time = float(nir_max_row["sample_elapsed_s"])
    nir_max_sample = int(nir_max_row["sample_index"])

    if x_label == "Time [s]":
        max_x = nir_max_time
    else:
        max_x = nir_max_sample

    fig_nir, ax_nir = plt.subplots(
        figsize=(12, 5),
        constrained_layout=True,
    )

    ax_nir.plot(
        x,
        light_raw_df["nir_counts"].to_numpy(dtype=float),
        label="NIR - 910 nm",
        linewidth=1.4,
        color="#C00000",
        linestyle="-",
    )

    ax_nir.scatter(
        [max_x],
        [nir_max_value],
        color="#C00000",
        zorder=3,
    )

    ax_nir.annotate(
        (
            f"max={nir_max_value}\n"
            f"sample={nir_max_sample}\n"
            f"time={nir_max_time:.3f} s"
        ),
        xy=(max_x, nir_max_value),
        xytext=(10, 10),
        textcoords="offset points",
    )

    ax_nir.set_xlabel(x_label)
    ax_nir.set_ylabel("Raw ADC counts")
    ax_nir.set_title(
        "AS7341 NIR raw counts" + title_suffix
    )
    ax_nir.grid(True, alpha=0.45)
    ax_nir.legend()

    nir_file = output_prefix.with_name(
        output_prefix.name + "_light_raw_nir.png"
    )

    fig_nir.savefig(
        nir_file,
        dpi=200,
        bbox_inches="tight",
    )
    plt.close(fig_nir)

    print(f"NIR plot saved to: {nir_file}")

    print(f"Maximum NIR count: {nir_max_value}")
    print(f"NIR maximum sample index: {nir_max_sample}")
    print(f"NIR maximum time: {nir_max_time:.3f} s")

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
    single_boot_df = pd.DataFrame(
        {
            "window_sequence": [1, 2, 3],
            "sample_timestamp_ms": [10000, 20000, 30000],
        },
        index=[10, 20, 30],
    )
    assigned_df, timestamp_resets, unexpected_regressions = (
        assign_light_boot_sessions(single_boot_df)
    )
    assert assigned_df["boot_session"].tolist() == [0, 0, 0]
    assert assigned_df.index.tolist() == [10, 20, 30]
    assert timestamp_resets == 0
    assert unexpected_regressions == 0

    reboot_df = pd.DataFrame(
        {
            "window_sequence": [1, 2, 3, 4, 5],
            "sample_timestamp_ms": [10000, 20000, 30000, 5000, 15000],
        }
    )
    assigned_df, timestamp_resets, unexpected_regressions = (
        assign_light_boot_sessions(reboot_df)
    )
    assert assigned_df["boot_session"].tolist() == [0, 0, 0, 1, 1]
    assert timestamp_resets == 1
    assert unexpected_regressions == 0
    within_session_regressions = sum(
        int((session["sample_timestamp_ms"].diff().dropna() < 0).sum())
        for _, session in assigned_df.groupby("boot_session", sort=True)
    )
    assert within_session_regressions == 0

    two_reboots_df = pd.DataFrame(
        {
            "window_sequence": [1, 2, 3, 4, 5, 6],
            "sample_timestamp_ms": [10000, 20000, 1000, 11000, 500, 10500],
        }
    )
    assigned_df, timestamp_resets, unexpected_regressions = (
        assign_light_boot_sessions(two_reboots_df)
    )
    assert assigned_df["boot_session"].tolist() == [0, 0, 1, 1, 2, 2]
    assert timestamp_resets == 2
    assert unexpected_regressions == 0

    unexpected_df = pd.DataFrame(
        {
            "window_sequence": [1, 1],
            "sample_timestamp_ms": [20000, 10000],
        }
    )
    assigned_df, timestamp_resets, unexpected_regressions = (
        assign_light_boot_sessions(unexpected_df)
    )
    assert assigned_df["boot_session"].tolist() == [0, 0]
    assert timestamp_resets == 0
    assert unexpected_regressions == 1

    empty_df = pd.DataFrame(columns=["window_sequence", "sample_timestamp_ms"])
    assigned_df, timestamp_resets, unexpected_regressions = (
        assign_light_boot_sessions(empty_df)
    )
    assert assigned_df.empty
    assert "boot_session" in assigned_df.columns
    assert assigned_df["boot_session"].dtype == "int64"
    assert timestamp_resets == 0
    assert unexpected_regressions == 0

    audio_feature_record = struct.pack(
        AUDIO_FEATURE_RECORD_FORMAT,
        7,
        123456,
        AUDIO_WINDOW_SAMPLES,
        -12,
        -4321,
        -4567,
        6789,
        -100,
        2,
        4,
        0x1F,
    )
    assert len(audio_feature_record) == AUDIO_FEATURE_RECORD_SIZE
    assert struct.unpack_from("<I", audio_feature_record, 0)[0] == 7
    assert struct.unpack_from("<I", audio_feature_record, 4)[0] == 123456
    assert struct.unpack_from("<H", audio_feature_record, 8)[0] == AUDIO_WINDOW_SAMPLES
    assert struct.unpack_from("<h", audio_feature_record, 10)[0] == -12
    assert struct.unpack_from("<h", audio_feature_record, 12)[0] == -4321
    assert struct.unpack_from("<h", audio_feature_record, 14)[0] == -4567
    assert struct.unpack_from("<h", audio_feature_record, 16)[0] == 6789
    assert struct.unpack_from("<h", audio_feature_record, 18)[0] == -100
    assert struct.unpack_from("<H", audio_feature_record, 20)[0] == 2
    assert audio_feature_record[22] == 4
    assert audio_feature_record[23] == 0x1F

    parsed_audio_feature = parse_audio_feature_record(audio_feature_record)
    assert parsed_audio_feature["window_sequence"] == 7
    assert parsed_audio_feature["rms_z_dbfs"] == -43.21
    assert parsed_audio_feature["environment_label"] == "NOISY"
    assert parsed_audio_feature["flag_complete"]
    assert parsed_audio_feature["flag_high_level"]
    assert SOUND_ENVIRONMENT_THRESHOLDS_DBA == (35.0, 45.0, 55.0, 65.0, 75.0, 85.0)
    for estimated_laeq, expected_class in (
        (34.999, 0), (35.0, 1), (44.999, 1), (45.0, 2),
        (54.999, 2), (55.0, 3), (64.999, 3), (65.0, 4),
        (74.999, 4), (75.0, 5), (84.999, 5), (85.0, 6),
    ):
        assert classify_embedded_audio_environment(estimated_laeq) == expected_class
    assert AUDIO_ENVIRONMENT_CLASS_NAMES[6] == "SOUND_HIGH_EXPOSURE"
    assert AUDIO_ENVIRONMENT_CLASS_INTERVALS[1] == "35 <= Estimated LAeq < 45 dBA"

    second_audio_feature_record = struct.pack(
        AUDIO_FEATURE_RECORD_FORMAT,
        8,
        133456,
        AUDIO_WINDOW_SAMPLES,
        0,
        -4000,
        -4200,
        8000,
        -50,
        0,
        5,
        0x07,
    )
    parsed_second_audio_feature = parse_audio_feature_record(second_audio_feature_record)

    empty_afea_df, empty_afea_summary = analyze_audio_feature_dataframe(
        pd.DataFrame(columns=AUDIO_FEATURE_COLUMNS)
    )
    assert empty_afea_df.empty
    assert empty_afea_summary["valid_records"] == 0
    with tempfile.TemporaryDirectory() as temp_dir:
        assert plot_audio_feature_records(empty_afea_df, Path(temp_dir) / "zero") == []

    zero_flag_audio_feature = dict(parsed_second_audio_feature)
    zero_flag_audio_feature["flags"] = 0
    for flag_column, _flag_label in AUDIO_FEATURE_FLAG_FIELDS:
        zero_flag_audio_feature[flag_column] = False
    one_afea_df, one_afea_summary = analyze_audio_feature_dataframe(
        pd.DataFrame([zero_flag_audio_feature], columns=AUDIO_FEATURE_COLUMNS)
    )
    assert one_afea_summary["valid_records"] == 0
    assert one_afea_summary["invalid_records"] == 1
    assert one_afea_summary["detected_boot_sessions"] == 1
    assert one_afea_summary["clipping_event_records"] == 0
    assert one_afea_df["environment_class_name"].tolist() == ["SOUND_VERY_NOISY"]
    assert one_afea_df["environment_class_range"].tolist() == ["75-85 dBA"]
    assert one_afea_summary["environment_class_matches"] == 1
    assert len(one_afea_summary["environment_class_distribution"]) == 7
    with tempfile.TemporaryDirectory() as temp_dir:
        one_plot_files = plot_audio_feature_records(one_afea_df, Path(temp_dir) / "one")
        assert len(one_plot_files) == 4
        assert all(path.is_file() for path in one_plot_files)

    multi_afea_rows = []
    for sequence, timestamp_ms in ((10, 1000), (11, 2000), (12, 500)):
        row = dict(parsed_audio_feature)
        row["window_sequence"] = sequence
        row["window_start_ms"] = timestamp_ms
        multi_afea_rows.append(row)
    multi_afea_rows[1]["rms_a_dbfs"] = float("nan")
    multi_afea_rows[1]["estimated_laeq_dba"] = float("nan")
    multi_afea_df, multi_afea_summary = analyze_audio_feature_dataframe(
        pd.DataFrame(multi_afea_rows, columns=AUDIO_FEATURE_COLUMNS),
        parser_invalid_records=1,
    )
    assert multi_afea_df["boot_session"].tolist() == [0, 0, 1]
    assert multi_afea_summary["recognized_timestamp_resets"] == 1
    assert multi_afea_summary["invalid_records"] == 1
    assert multi_afea_summary["clipping_event_records"] == 3
    assert multi_afea_summary["feature_stats"]["rms_a_dbfs"]["finite_count"] == 2
    with tempfile.TemporaryDirectory() as temp_dir:
        multi_plot_files = plot_audio_feature_records(
            multi_afea_df, Path(temp_dir) / "multi"
        )
        assert len(multi_plot_files) == 4
        assert all(path.is_file() for path in multi_plot_files)

    unknown_class_row = dict(parsed_audio_feature)
    unknown_class_row["environment_class"] = 9
    unknown_class_df, unknown_class_summary = analyze_audio_feature_dataframe(
        pd.DataFrame([unknown_class_row], columns=AUDIO_FEATURE_COLUMNS)
    )
    assert unknown_class_df["environment_class_name"].tolist() == ["UNKNOWN_CLASS_9"]
    assert unknown_class_df["environment_class_range"].tolist() == ["UNKNOWN"]
    assert unknown_class_summary["unknown_environment_class_records"] == 1
    assert unknown_class_summary["environment_class_not_evaluated"] == 1
    assert unknown_class_summary["warnings"]

    mismatched_class_row = dict(parsed_audio_feature)
    mismatched_class_row["estimated_laeq_dba"] = 80.0
    mismatched_class_df, mismatched_class_summary = analyze_audio_feature_dataframe(
        pd.DataFrame([mismatched_class_row], columns=AUDIO_FEATURE_COLUMNS)
    )
    assert mismatched_class_df["python_environment_class"].tolist() == [5]
    assert mismatched_class_df["environment_class_match"].tolist() == [False]
    assert mismatched_class_summary["environment_class_mismatches"] == 1

    afea_page = _make_test_page(
        magic=MAGIC_AUDIO_FEATURE,
        payload=audio_feature_record + second_audio_feature_record,
    )
    _, _, _, _, _, stats, _ = _run_parse_test_dump(afea_page)
    assert stats.audio_feature_pages == 1
    assert stats.audio_feature_records == 2
    assert stats.audio_feature_payload_bytes == 48

    afea_remainder_page = _make_test_page(
        magic=MAGIC_AUDIO_FEATURE,
        payload=audio_feature_record + b"XYZ",
    )
    _, _, _, _, _, stats, _ = _run_parse_test_dump(afea_remainder_page)
    assert stats.audio_feature_records == 1
    assert stats.audio_feature_payload_bytes == 27
    assert stats.audio_feature_payload_remainder_bytes == 3

    light_feature_record = struct.pack(
        LIGHT_FEATURE_RECORD_FORMAT,
        1,
        2000,
        10,
        20,
        30,
        40,
        50,
        60,
        70,
        80,
        100,
        90,
        2,
        0x07,
    )
    assert struct.calcsize(LIGHT_FEATURE_RECORD_FORMAT) == 30
    assert LIGHT_FEATURE_RECORDS_PER_PAGE == 136
    assert len(light_feature_record) == LIGHT_FEATURE_RECORD_SIZE
    assert struct.unpack_from("<I", light_feature_record, 0)[0] == 1
    assert struct.unpack_from("<I", light_feature_record, 4)[0] == 2000
    for channel_index, expected in enumerate((10, 20, 30, 40, 50, 60, 70, 80, 100, 90)):
        assert struct.unpack_from("<H", light_feature_record, 8 + 2 * channel_index)[0] == expected
    assert light_feature_record[28] == 2
    assert light_feature_record[29] == 0x07

    parsed_light_feature = parse_light_feature_record(light_feature_record)
    assert parsed_light_feature["window_sequence"] == 1
    assert parsed_light_feature["sample_timestamp_ms"] == 2000
    assert parsed_light_feature["f1"] == 10
    assert parsed_light_feature["f8"] == 80
    assert parsed_light_feature["clear"] == 100
    assert parsed_light_feature["nir"] == 90
    assert parsed_light_feature["exposure_label"] == "MODERATE_EXPOSURE"
    assert parsed_light_feature["flag_complete"]
    assert parsed_light_feature["flag_acquisition_valid"]
    assert parsed_light_feature["flag_classification_valid"]
    assert not parsed_light_feature["flag_saturated"]

    saturated_light_feature_record = struct.pack(
        LIGHT_FEATURE_RECORD_FORMAT,
        2,
        12000,
        11,
        21,
        31,
        41,
        51,
        61,
        71,
        81,
        10000,
        91,
        4,
        0x0F,
    )
    parsed_saturated = parse_light_feature_record(saturated_light_feature_record)
    assert parsed_saturated["flag_saturated"]
    assert parsed_saturated["exposure_class"] == 4

    lfea_page = _make_test_page(
        magic=MAGIC_LIGHT_FEATURE,
        payload=light_feature_record + saturated_light_feature_record,
    )
    _, _, _, light_feature_df, _, stats, _ = _run_parse_test_dump(lfea_page)
    assert stats.light_feature_pages == 1
    assert stats.light_feature_records == 2
    assert stats.light_feature_payload_bytes == 60
    assert stats.light_feature_payload_remainder_bytes == 0
    assert stats.light_feature_validation_result == "PASS"
    assert light_feature_df["classification_match"].tolist() == [True, True]
    assert light_feature_df["saturation_match"].tolist() == [True, True]

    lfea_remainder_page = _make_test_page(
        magic=MAGIC_LIGHT_FEATURE,
        payload=light_feature_record + b"12345",
    )
    _, _, _, light_feature_df, _, stats, _ = _run_parse_test_dump(
        lfea_remainder_page
    )
    assert stats.light_feature_pages == 1
    assert stats.light_feature_records == 1
    assert stats.light_feature_payload_remainder_bytes == 5
    assert len(light_feature_df) == 1

    reboot_record_before = bytearray(light_feature_record)
    struct.pack_into("<I", reboot_record_before, 4, 20000)
    reboot_record_after = bytearray(light_feature_record)
    struct.pack_into("<I", reboot_record_after, 0, 2)
    struct.pack_into("<I", reboot_record_after, 4, 1000)
    reboot_validation_df, reboot_validation = validate_light_feature_records(
        [
            parse_light_feature_record(bytes(reboot_record_before)),
            parse_light_feature_record(bytes(reboot_record_after)),
        ],
        ParseStats(),
    )
    assert reboot_validation_df["boot_session"].tolist() == [0, 1]
    assert reboot_validation["detected_boot_sessions"] == 2
    assert reboot_validation["global_timestamp_regressions"] == 1
    assert reboot_validation["recognized_timestamp_resets"] == 1
    assert reboot_validation["non_monotonic_timestamps_within_sessions"] == 0
    assert reboot_validation["unexpected_timestamp_regressions"] == 0
    assert reboot_validation["final_result"] == "PASS"

    def make_hysteresis_test_record(
        sequence: int,
        timestamp_ms: int,
        clear_counts: int,
        exposure_class: int,
    ) -> dict:
        record = bytearray(light_feature_record)
        struct.pack_into("<I", record, 0, sequence)
        struct.pack_into("<I", record, 4, timestamp_ms)
        struct.pack_into("<H", record, 24, clear_counts)
        record[28] = exposure_class
        return parse_light_feature_record(bytes(record))

    hysteresis_session_df, hysteresis_session_validation = (
        validate_light_feature_records(
            [
                make_hysteresis_test_record(1, 1000, 2, 1),
                make_hysteresis_test_record(2, 2000, 3, 1),
                make_hysteresis_test_record(3, 3000, 2, 1),
                make_hysteresis_test_record(4, 500, 2, 1),
                make_hysteresis_test_record(5, 1500, 0, 0),
            ],
            ParseStats(),
        )
    )
    assert hysteresis_session_df["boot_session"].tolist() == [0, 0, 0, 1, 1]
    assert hysteresis_session_df["classification_evaluated"].tolist() == [
        False, True, True, False, True,
    ]
    assert hysteresis_session_df["python_exposure_class"].isna().tolist() == [
        True, False, False, True, False,
    ]
    assert hysteresis_session_df["python_exposure_class"].fillna(-1).tolist() == [
        -1, 1, 1, -1, 0,
    ]
    assert hysteresis_session_validation["classification_matches"] == 3
    assert hysteresis_session_validation["classification_mismatches"] == 0
    assert hysteresis_session_validation["classification_not_evaluated"] == 2
    assert hysteresis_session_validation["final_result"] == "PASS"

    _, all_unknown_validation = validate_light_feature_records(
        [make_hysteresis_test_record(1, 1000, 2, 1)],
        ParseStats(),
    )
    assert all_unknown_validation["classification_matches"] == 0
    assert all_unknown_validation["classification_mismatches"] == 0
    assert all_unknown_validation["classification_not_evaluated"] == 1
    assert all_unknown_validation["final_result"] == "NOT AVAILABLE"

    unexpected_record_before = bytearray(light_feature_record)
    struct.pack_into("<I", unexpected_record_before, 4, 20000)
    unexpected_record_after = bytearray(light_feature_record)
    struct.pack_into("<I", unexpected_record_after, 4, 10000)
    unexpected_validation_df, unexpected_validation = validate_light_feature_records(
        [
            parse_light_feature_record(bytes(unexpected_record_before)),
            parse_light_feature_record(bytes(unexpected_record_after)),
        ],
        ParseStats(),
    )
    assert unexpected_validation_df["boot_session"].tolist() == [0, 0]
    assert unexpected_validation["global_timestamp_regressions"] == 1
    assert unexpected_validation["recognized_timestamp_resets"] == 0
    assert unexpected_validation["non_monotonic_timestamps_within_sessions"] == 1
    assert unexpected_validation["unexpected_timestamp_regressions"] == 1
    assert unexpected_validation["final_result"] == "FAIL"

    for clear_counts, expected_class in (
        (2, 0),
        (3, 1),
        (49, 1),
        (50, 2),
        (6499, 2),
        (6500, 3),
        (9799, 3),
        (9800, 4),
    ):
        assert classify_light_with_hysteresis(clear_counts, None) == expected_class
    assert classify_light_with_hysteresis(9400, 4) == 4
    assert classify_light_with_hysteresis(9000, 4) == 3
    assert classify_light_with_hysteresis(46, 2) == 1
    assert classify_light_with_hysteresis(7000, 0) == 3
    assert classify_light_with_hysteresis(9999, None) == 4
    assert classify_light_with_hysteresis(10000, None) == 4
    assert not (9999 >= LIGHT_SATURATION_CLEAR_COUNTS)
    assert 10000 >= LIGHT_SATURATION_CLEAR_COUNTS
    assert not parse_light_feature_record(light_feature_record)["flag_saturated"]
    assert parse_light_feature_record(saturated_light_feature_record)["flag_saturated"]

    assert round_half_away_from_zero(1.5) == 2
    assert round_half_away_from_zero(-1.5) == -2
    assert encode_db_centi_like_firmware(float("nan")) == AUDIO_DB_CENTI_INVALID
    assert encode_db_centi_like_firmware(float("inf")) == AUDIO_DB_CENTI_INVALID

    test_time = np.arange(AUDIO_WINDOW_SAMPLES, dtype=np.float64) / 48000.0
    test_tone = np.asarray(
        10000.0 * np.sin(2.0 * np.pi * 1000.0 * test_time),
        dtype=np.int16,
    )
    tone_basic = compute_audio_window_basic_metrics(test_tone)
    tone_weighted = compute_embedded_iir_a_weighted_metrics(test_tone, 48000)
    assert abs(
        tone_weighted["a_weighted_rms_dbfs"] - tone_basic["rms_zero_mean_dbfs"]
    ) < 0.01

    tone_environment = classify_embedded_audio_environment(
        tone_weighted["estimated_laeq_dba"]
    )
    tone_flags = build_expected_audio_flags(
        complete=True,
        acquisition_valid=True,
        a_weighting_valid=True,
        clipped_sample_count=tone_basic["clipped_sample_count"],
        estimated_laeq_dba=tone_weighted["estimated_laeq_dba"],
    )
    tone_feature_record = struct.pack(
        AUDIO_FEATURE_RECORD_FORMAT,
        1,
        1000,
        AUDIO_WINDOW_SAMPLES,
        tone_basic["mean_counts_rounded"],
        encode_db_centi_like_firmware(tone_basic["rms_zero_mean_dbfs"]),
        encode_db_centi_like_firmware(tone_weighted["a_weighted_rms_dbfs"]),
        encode_db_centi_like_firmware(tone_weighted["estimated_laeq_dba"]),
        encode_db_centi_like_firmware(tone_basic["peak_dbfs"]),
        tone_basic["clipped_sample_count"],
        tone_environment,
        tone_flags,
    )
    tone_bytes = test_tone.astype("<i2", copy=False).tobytes()
    tone_dump_pages = []
    offset = 0
    for page_index, payload_size in enumerate(AUDIO_WINDOW_EXPECTED_PAYLOAD_PATTERN):
        tone_dump_pages.append(
            _make_test_page(
                magic=MAGIC_AUDIO,
                payload=tone_bytes[offset:offset + payload_size],
                page_sequence=page_index,
            )
        )
        offset += payload_size
    tone_dump_pages.append(
        _make_test_page(
            magic=MAGIC_AUDIO_FEATURE,
            payload=tone_feature_record,
            page_sequence=len(tone_dump_pages),
        )
    )
    _, _, _, _, _, stats, _ = _run_parse_test_dump(b"".join(tone_dump_pages))
    assert stats.audio_complete_windows == 1
    assert stats.audio_feature_paired_windows == 1
    assert stats.audio_feature_comparison_result == "PASS"

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
    _, light_raw_df, _, _, _, stats, _ = _run_parse_test_dump(page)
    assert stats.light_raw_records == 1
    assert int(light_raw_df["sample_elapsed_ms"].iloc[0]) == 1234

    page = _make_test_page(header_size=PAGE_HEADER_SIZE + 4, payload=record)
    _, light_raw_df, _, _, _, stats, _ = _run_parse_test_dump(page)
    assert stats.light_raw_records == 1
    assert int(light_raw_df["record_absolute_page_offset"].iloc[0]) == PAGE_HEADER_SIZE + 4

    page = _make_test_page(header_size=PAGE_HEADER_SIZE - 4, payload=record)
    _, light_raw_df, _, _, _, stats, _ = _run_parse_test_dump(page)
    assert stats.invalid_light_raw_pages == 1
    assert light_raw_df.empty

    page = _make_test_page(payload=record + b"XYZ")
    _, light_raw_df, _, _, _, stats, _ = _run_parse_test_dump(page)
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
    _, light_raw_df, _, _, _, stats, diagnostics = _run_parse_test_dump(no_lraw_page)
    assert stats.light_raw_pages == 0
    assert light_raw_df.empty
    assert list(light_raw_df.columns) == LIGHT_RAW_COLUMNS
    assert diagnostics["is_empty"]

    classification_test_df = _make_light_raw_test_df(
        [
            {"clear_counts": 0, "nir_counts": 0},
            {"clear_counts": 3, "nir_counts": 0},
            {"clear_counts": 4, "nir_counts": 0},
            {"clear_counts": 20, "nir_counts": 0},
            {"clear_counts": 49, "nir_counts": 0},
            {"clear_counts": 50, "nir_counts": 0},
            {"clear_counts": 500, "nir_counts": 0},
            {"clear_counts": 6499, "nir_counts": 0},
            {"clear_counts": 6500, "nir_counts": 0},
            {"clear_counts": 9000, "nir_counts": 0},
            {"clear_counts": 9799, "nir_counts": 0},
            {"clear_counts": 9800, "nir_counts": 0},
            {"clear_counts": 9999, "nir_counts": 0},
            {"clear_counts": 10000, "nir_counts": 0},
        ]
    )
    original_smoothing_seconds = LIGHT_CLASSIFICATION_SMOOTHING_SECONDS
    try:
        globals()["LIGHT_CLASSIFICATION_SMOOTHING_SECONDS"] = 0.0
        classification_df, assessment = compute_relative_light_classification(
            classification_test_df
        )
    finally:
        globals()["LIGHT_CLASSIFICATION_SMOOTHING_SECONDS"] = (
            original_smoothing_seconds
        )
    assert assessment is not None
    assert classification_df["light_level_label"].tolist() == [
        "DARK",
        "LOW_EXPOSURE",
        "LOW_EXPOSURE",
        "LOW_EXPOSURE",
        "LOW_EXPOSURE",
        "MODERATE_EXPOSURE",
        "MODERATE_EXPOSURE",
        "MODERATE_EXPOSURE",
        "HIGH_EXPOSURE",
        "HIGH_EXPOSURE",
        "HIGH_EXPOSURE",
        "VERY_HIGH_EXPOSURE",
        "VERY_HIGH_EXPOSURE",
        "VERY_HIGH_EXPOSURE",
    ]
    assert classification_df["is_saturated"].tolist() == [
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        False,
        True,
    ]
    assert classification_df.loc[
        classification_df["clear_counts"] == 10000,
        "light_level_label",
    ].iloc[0] == "VERY_HIGH_EXPOSURE"
    assert bool(classification_df.loc[
        classification_df["clear_counts"] == 10000,
        "is_saturated",
    ].iloc[0])
    assert assessment.saturated_sample_count == 1

    print("Internal synthetic tests passed.")


def _build_cli_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Download or analyze Smart Eyewear NAND dumps."
    )
    parser.add_argument(
        "--dump",
        type=Path,
        help="Analyze an existing NAND dump without opening the serial GUI.",
    )
    parser.add_argument(
        "--ble-pages-bin",
        type=Path,
        help="BLE logical page binary produced by ble_sync_client.py.",
    )
    parser.add_argument(
        "--ble-pages-csv",
        type=Path,
        help="BLE page metadata CSV produced by ble_sync_client.py.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help=(
            "Output directory for --dump or BLE file mode; defaults to the "
            "input binary directory."
        ),
    )
    parser.add_argument(
        "--audio-sample-rate",
        type=int,
        default=DEFAULT_AUDIO_SAMPLE_RATE,
        help=f"Audio sample rate for --dump mode (default: {DEFAULT_AUDIO_SAMPLE_RATE}).",
    )
    parser.add_argument(
        "--run-internal-tests",
        action="store_true",
        help="Run synthetic parser and DSP tests, then exit.",
    )
    return parser


def main(argv=None):
    parser = _build_cli_parser()
    args = parser.parse_args(argv)
    if args.run_internal_tests:
        run_internal_tests()
        return

    ble_mode = args.ble_pages_bin is not None or args.ble_pages_csv is not None
    if (args.ble_pages_bin is None) != (args.ble_pages_csv is None):
        parser.error("--ble-pages-bin and --ble-pages-csv must be provided together")
    if args.dump is not None and ble_mode:
        parser.error("--dump cannot be combined with BLE page input arguments")

    offline_mode = args.dump is not None or ble_mode
    ble_pages_csv_filename = None
    if ble_mode:
        bin_filename = args.ble_pages_bin.expanduser().resolve()
        ble_pages_csv_filename = args.ble_pages_csv.expanduser().resolve()
        if not bin_filename.is_file():
            print(f"Processing error: BLE pages binary not found: {bin_filename}")
            return
        if not ble_pages_csv_filename.is_file():
            print(f"Processing error: BLE pages CSV not found: {ble_pages_csv_filename}")
            return
        save_folder = (
            args.output_dir.expanduser().resolve()
            if args.output_dir is not None
            else bin_filename.parent
        )
        save_folder.mkdir(parents=True, exist_ok=True)
        output_prefix = save_folder / bin_filename.stem
        audio_sample_rate = args.audio_sample_rate
    elif args.dump is not None:
        bin_filename = args.dump.expanduser().resolve()
        if not bin_filename.is_file():
            print(f"Processing error: dump file not found: {bin_filename}")
            return
        save_folder = (
            args.output_dir.expanduser().resolve()
            if args.output_dir is not None
            else bin_filename.parent
        )
        save_folder.mkdir(parents=True, exist_ok=True)
        prefix_name = bin_filename.name
        if prefix_name.endswith("_nand_dump.bin"):
            prefix_name = prefix_name[:-len("_nand_dump.bin")]
        else:
            prefix_name = bin_filename.stem
        output_prefix = save_folder / prefix_name
        audio_sample_rate = args.audio_sample_rate
    else:
        com_port, save_folder, baud_rate, audio_sample_rate = gui_select_com_and_folder()
        if not com_port or save_folder is None:
            print("Application stopped.")
            return

        timestamp = datetime.now().strftime("SmartEyewear_%Y%m%d_%H%M%S")
        output_prefix = save_folder / timestamp
        bin_filename = output_prefix.with_name(output_prefix.name + "_nand_dump.bin")
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
        (
            imu_df,
            light_raw_df,
            light_df,
            light_feature_df,
            audio_bytes,
            stats,
            light_raw_diagnostics,
        ) = parse_nand_dump(
            bin_filename=bin_filename,
            imu_csv_filename=imu_csv_filename,
            light_raw_csv_filename=light_raw_csv_filename,
            light_csv_filename=light_csv_filename,
            wav_filename=wav_filename,
            summary_filename=summary_filename,
            light_raw_diagnostics_filename=light_raw_diagnostics_filename,
            audio_sample_rate_hz=audio_sample_rate,
            ble_pages_csv_filename=ble_pages_csv_filename,
        )

        plot_imu_data(imu_df, output_prefix)
        plot_light_raw_channels(light_raw_df, output_prefix, light_raw_diagnostics)
        plot_light_results(light_df, output_prefix)
        plot_light_feature_records(light_feature_df, output_prefix)
        analyze_and_export_relative_light_level(
            light_raw_df=light_raw_df,
            output_prefix=output_prefix,
            summary_filename=summary_filename,
            diagnostics=light_raw_diagnostics,
        )
        print(
            "LFEA embedded classification result: "
            f"{stats.light_feature_validation_result}"
        )

    except Exception as exc:
        if not offline_mode:
            messagebox.showerror("Processing error", str(exc))
        print(f"Processing error: {exc}")
        return

    if offline_mode:
        print(f"Offline processing completed. Files saved in: {save_folder}")
    else:
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

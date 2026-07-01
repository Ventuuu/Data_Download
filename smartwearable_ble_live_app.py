#!/usr/bin/env python3
"""Smart Wearable BLE client: NAND sync, real-time dashboard and simulator."""

from __future__ import annotations

import argparse
import asyncio
import binascii
import contextlib
import csv
import json
import math
import os
import queue
import random
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Any, BinaryIO, Optional, TextIO

try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # Simulation mode remains usable without BLE dependencies.
    BleakClient = None  # type: ignore[assignment]
    BleakScanner = None  # type: ignore[assignment]


DEVICE_NAME_DEFAULT = "BLE_SW"
RN4871_SERVICE_UUID = "49535343-FE7D-4AE5-8FA9-9FAFD205E455"
RN4871_MAIN_CHAR_UUID = "49535343-1E4D-4BD9-BA61-23C647249616"

SOF = b"\xA5\x5A"
PROTOCOL_VERSION = 1
MAX_FRAME_PAYLOAD_BYTES = 4096
MAX_LOGICAL_PAGE_BYTES = 4096
SYNC_STATUS_FORMAT = "<12s8I"
SYNC_STATUS_SIZE = struct.calcsize(SYNC_STATUS_FORMAT)

BLE_MSG_SYNC_START = 0x10
BLE_MSG_SYNC_STATUS = 0x11
BLE_MSG_PAGE_BEGIN = 0x20
BLE_MSG_PAGE_DATA = 0x21
BLE_MSG_PAGE_END = 0x22
BLE_MSG_ACK_THROUGH = 0x30
BLE_MSG_NACK_PAGE = 0x31
BLE_MSG_SYNC_COMPLETE = 0x40
BLE_MSG_SYNC_ABORT = 0x41

# Live protocol extension. Keep these values aligned with the firmware.
BLE_MSG_LIVE_START = 0x50
BLE_MSG_LIVE_STOP = 0x51
BLE_MSG_ACK_LIVE_START = 0x52
BLE_MSG_ACK_LIVE_STOP = 0x53
BLE_MSG_LIVE_HEARTBEAT = 0x54
BLE_MSG_LIVE_IMU = 0x60
BLE_MSG_LIVE_AFEA = 0x61
BLE_MSG_LIVE_LFEA = 0x62
BLE_MSG_ERROR = 0x7F

# Every device-to-app live payload starts with a global uint32 packet_sequence.
LIVE_PACKET_SEQUENCE_FORMAT = "<I"
LIVE_IMU_FORMAT = "<IBBBH6h"  # seq32, hh, mm, ss, sss, accel XYZ, gyro XYZ
LIVE_IMU_SIZE = struct.calcsize(LIVE_IMU_FORMAT)
LIVE_ERROR_FORMAT = "<IHI"  # seq32, error_code:u16, detail:u32
LIVE_ERROR_SIZE = struct.calcsize(LIVE_ERROR_FORMAT)
LIVE_HEARTBEAT_PERIOD_SECONDS = 2.0
AUDIO_FEATURE_RECORD_FORMAT = "<IIHhhhhhHBB"
AUDIO_FEATURE_RECORD_SIZE = struct.calcsize(AUDIO_FEATURE_RECORD_FORMAT)
LIVE_AFEA_SIZE = 4 + AUDIO_FEATURE_RECORD_SIZE
LIGHT_FEATURE_RECORD_FORMAT = "<II10HBB"
LIGHT_FEATURE_RECORD_SIZE = struct.calcsize(LIGHT_FEATURE_RECORD_FORMAT)
LIVE_LFEA_SIZE = 4 + LIGHT_FEATURE_RECORD_SIZE
AUDIO_DB_CENTI_INVALID = -32768

LIVE_EXPOSURE_LABELS = {
    0: "DARK",
    1: "LOW_EXPOSURE",
    2: "MODERATE_EXPOSURE",
    3: "HIGH_EXPOSURE",
    4: "VERY_HIGH_EXPOSURE",
    255: "UNAVAILABLE",
}
LIVE_AUDIO_ENVIRONMENT_LABELS = {
    0: "VERY_QUIET",
    1: "QUIET",
    2: "MODERATE",
    3: "LIVELY",
    4: "NOISY",
    5: "VERY_NOISY",
    6: "HIGH_EXPOSURE",
    255: "UNAVAILABLE",
}

MESSAGE_NAMES = {
    BLE_MSG_SYNC_START: "SYNC_START",
    BLE_MSG_SYNC_STATUS: "SYNC_STATUS",
    BLE_MSG_PAGE_BEGIN: "PAGE_BEGIN",
    BLE_MSG_PAGE_DATA: "PAGE_DATA",
    BLE_MSG_PAGE_END: "PAGE_END",
    BLE_MSG_ACK_THROUGH: "ACK_THROUGH",
    BLE_MSG_NACK_PAGE: "NACK_PAGE",
    BLE_MSG_SYNC_COMPLETE: "SYNC_COMPLETE",
    BLE_MSG_SYNC_ABORT: "SYNC_ABORT",
    BLE_MSG_LIVE_START: "LIVE_START",
    BLE_MSG_LIVE_STOP: "LIVE_STOP",
    BLE_MSG_ACK_LIVE_START: "ACK_LIVE_START",
    BLE_MSG_ACK_LIVE_STOP: "ACK_LIVE_STOP",
    BLE_MSG_LIVE_HEARTBEAT: "LIVE_HEARTBEAT",
    BLE_MSG_LIVE_IMU: "LIVE_IMU",
    BLE_MSG_LIVE_AFEA: "LIVE_AFEA",
    BLE_MSG_LIVE_LFEA: "LIVE_LFEA",
    BLE_MSG_ERROR: "ERROR",
}

CSV_FIELDS = [
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
]

NACK_PAGE_CRC = 1
NACK_FRAME_INCOMPLETE = 2
NACK_LOCAL_SAVE_FAILED = 3


class PageProtocolError(RuntimeError):
    """A page cannot safely be ACKed in this session."""


@dataclass(frozen=True)
class Frame:
    msg_type: int
    seq: int
    payload: bytes


@dataclass(frozen=True)
class SyncStatus:
    device_uid: bytes
    log_generation: int
    ack_valid: int
    acked_through_page_sequence: int
    oldest_available_page_sequence: int
    newest_available_page_sequence: int
    first_page_sequence_to_send: int
    sync_high_watermark_page_sequence: int
    total_unsynced_pages: int


@dataclass
class PageContext:
    log_generation: int
    page_sequence: int
    physical_page_index: int
    logical_page_bytes: int
    page_crc32: int
    magic: bytes
    page_version: int
    page_header_size: int
    page_payload_bytes: int
    data: bytearray
    next_expected_offset: int
    received_bytes: int
    generation_token: int


@dataclass(frozen=True)
class SavedPage:
    log_generation: int
    page_sequence: int
    physical_page_index: int
    logical_page_bytes: int
    page_crc32: int
    magic: bytes
    page_version: int
    page_header_size: int
    page_payload_bytes: int
    file_offset: int


@dataclass(frozen=True)
class PageTimeoutEvent:
    token: int
    page_sequence: int


class FrameParser:
    """Streaming parser: accepts concatenated and fragmented frames."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[Frame]:
        self.buffer.extend(data)
        frames: list[Frame] = []

        while True:
            sof_index = self.buffer.find(SOF)
            if sof_index < 0:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return frames
            if sof_index > 0:
                del self.buffer[:sof_index]
            if len(self.buffer) < 10:
                return frames

            version = self.buffer[2]
            msg_type = self.buffer[3]
            payload_len = self.buffer[4] | (self.buffer[5] << 8)
            seq = self.buffer[6] | (self.buffer[7] << 8)
            if payload_len > MAX_FRAME_PAYLOAD_BYTES:
                del self.buffer[:2]
                continue

            total_len = 10 + payload_len
            if len(self.buffer) < total_len:
                return frames
            frame_bytes = bytes(self.buffer[:total_len])
            del self.buffer[:total_len]

            payload = frame_bytes[8:8 + payload_len]
            received_crc = struct.unpack_from("<H", frame_bytes, 8 + payload_len)[0]
            computed_crc = crc16_ccitt_false(frame_bytes[2:8 + payload_len])
            if received_crc != computed_crc:
                print(
                    "[WARN] Dropped frame with CRC error: "
                    f"rx=0x{received_crc:04X}, calc=0x{computed_crc:04X}, "
                    f"type=0x{msg_type:02X}, seq={seq}"
                )
                continue
            if version != PROTOCOL_VERSION:
                print(
                    "[WARN] Dropped frame with unsupported version: "
                    f"{version}, type=0x{msg_type:02X}, seq={seq}"
                )
                continue
            frames.append(Frame(msg_type=msg_type, seq=seq, payload=payload))


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def crc32_iso_hdlc(data: bytes) -> int:
    return binascii.crc32(data) & 0xFFFFFFFF


def encode_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    header = struct.pack("<BBHH", PROTOCOL_VERSION, msg_type, len(payload), seq & 0xFFFF)
    return SOF + header + payload + struct.pack("<H", crc16_ccitt_false(header + payload))


def parse_sync_status(payload: bytes) -> SyncStatus:
    if len(payload) != SYNC_STATUS_SIZE:
        raise ValueError(
            f"SYNC_STATUS payload must be {SYNC_STATUS_SIZE} bytes, got {len(payload)}"
        )
    return SyncStatus(*struct.unpack(SYNC_STATUS_FORMAT, payload))


def parse_page_begin(payload: bytes, token: int = 0) -> PageContext:
    if len(payload) != 32:
        raise ValueError(f"PAGE_BEGIN payload must be 32 bytes, got {len(payload)}")
    generation, sequence, physical, logical, page_crc32 = struct.unpack_from("<IIIII", payload)
    magic = payload[20:24]
    version = payload[24]
    header_size = payload[25]
    payload_bytes = struct.unpack_from("<I", payload, 28)[0]
    if logical == 0 or logical > MAX_LOGICAL_PAGE_BYTES:
        raise ValueError(f"invalid logical page size: {logical}")
    if header_size + payload_bytes != logical:
        raise ValueError(
            f"page size fields disagree: header={header_size}, payload={payload_bytes}, logical={logical}"
        )
    return PageContext(
        generation, sequence, physical, logical, page_crc32, magic, version,
        header_size, payload_bytes, bytearray(), 0, 0, token,
    )


def parse_page_data(payload: bytes) -> tuple[int, int, int, bytes]:
    if len(payload) < 16:
        raise ValueError(f"PAGE_DATA payload too short: {len(payload)} bytes")
    generation, sequence, offset, chunk_length = struct.unpack_from("<IIIH", payload)
    if len(payload) != 16 + chunk_length:
        raise ValueError(
            f"PAGE_DATA payload length mismatch: expected {16 + chunk_length}, got {len(payload)}"
        )
    return generation, sequence, offset, payload[16:]


def parse_page_end(payload: bytes) -> tuple[int, int, int, int]:
    if len(payload) != 16:
        raise ValueError(f"PAGE_END payload must be 16 bytes, got {len(payload)}")
    return struct.unpack("<IIII", payload)


def make_ack_through(seq: int, log_generation: int, page_sequence: int) -> bytes:
    return encode_frame(BLE_MSG_ACK_THROUGH, seq, struct.pack("<II", log_generation, page_sequence))


def make_nack(seq: int, log_generation: int, page_sequence: int, reason: int = 1) -> bytes:
    return encode_frame(
        BLE_MSG_NACK_PAGE,
        seq,
        struct.pack("<IIB3x", log_generation, page_sequence, reason & 0xFF),
    )


def format_hex(data: bytes, max_bytes: int = 64) -> str:
    if len(data) <= max_bytes:
        return data.hex(" ").upper()
    return data[:max_bytes].hex(" ").upper() + f" ... ({len(data)} bytes)"


async def find_device(name: str, timeout: float):
    if BleakScanner is None:
        raise RuntimeError("Bleak is not installed. Install it with: pip install bleak")
    print(f"[SCAN] Searching for BLE device named '{name}'...")
    devices = await BleakScanner.discover(timeout=timeout)
    for device in devices:
        if device.name == name:
            print(f"[SCAN] Found {device.name} at {device.address}")
            return device
    for device in devices:
        print(f"  - {device.name!r} {device.address}")
    raise RuntimeError(f"Device '{name}' not found")


class BleSyncClient:
    def __init__(
        self,
        device_name: str = DEVICE_NAME_DEFAULT,
        output_path: Path = Path("smartwearable_ble_pages.bin"),
        pages_csv_path: Path = Path("smartwearable_ble_pages.csv"),
        max_pages: Optional[int] = None,
        scan_timeout: float = 10.0,
        write_response: bool = True,
        *,
        resume: bool = False,
        fsync_interval_pages: int = 1,
        page_timeout: float = 30.0,
        idle_timeout_seconds: float = 60.0,
        absolute_session_timeout_seconds: float = 0.0,
        debug_state: bool = False,
    ) -> None:
        self.device_name = device_name
        self.output_path = output_path
        self.pages_csv_path = pages_csv_path
        self.metadata_path = Path(f"{pages_csv_path}.resume.json")
        self.max_pages = max_pages
        self.scan_timeout = scan_timeout
        self.write_response = write_response
        self.resume = resume
        self.fsync_interval_pages = fsync_interval_pages
        self.page_timeout = page_timeout
        self.idle_timeout_seconds = idle_timeout_seconds
        self.absolute_session_timeout_seconds = absolute_session_timeout_seconds
        self.debug_state = debug_state

        self.parser = FrameParser()
        self.client: Optional[BleakClient] = None
        self.tx_seq = 1
        self.last_rx_seq: Optional[int] = None
        self.status: Optional[SyncStatus] = None
        self.current_page: Optional[PageContext] = None
        self.next_page_token = 1
        self.page_timeout_task: Optional[asyncio.Task[None]] = None
        self.session_watchdog_task: Optional[asyncio.Task[None]] = None
        self.rx_queue: asyncio.Queue[bytes | PageTimeoutEvent] = asyncio.Queue()
        self.rx_consumer_task: Optional[asyncio.Task[None]] = None
        self.accept_notifications = False
        self.consumer_count = 0

        self.pages_saved = 0
        self.persisted_pages = 0
        self.finished = asyncio.Event()
        self.stop_requested = False
        self.failure: Optional[BaseException] = None
        self.primary_cleanup_reason: Optional[str] = None
        self.disconnect_seen = False
        self.protocol_finished = False
        self.resync_identity: Optional[tuple[int, int]] = None
        self.nacked_identities: set[tuple[int, int]] = set()

        self.saved_pages: dict[tuple[bytes, int, int], SavedPage] = {}
        self.resume_metadata: Optional[dict[str, object]] = None
        self.output_file: Optional[BinaryIO] = None
        self.csv_file: Optional[TextIO] = None
        self.csv_writer: Optional[csv.DictWriter] = None
        self.monotonic_origin = time.monotonic()
        self.session_started_monotonic: Optional[float] = None
        self.last_valid_activity: Optional[float] = None
        self.last_ack_monotonic: Optional[float] = None

    def debug(self, message: str) -> None:
        if self.debug_state:
            print(message)

    def log_monotonic(self, event: str, **fields: object) -> None:
        now = time.monotonic()
        details = " ".join(f"{key}={value}" for key, value in fields.items())
        suffix = f" {details}" if details else ""
        print(
            f"[MONO] t={now:.3f} elapsed={now - self.monotonic_origin:.3f}s "
            f"event={event}{suffix}",
            flush=True,
        )

    def session_timing_fields(self, now: Optional[float] = None) -> dict[str, str]:
        now = time.monotonic() if now is None else now
        if self.session_started_monotonic is None:
            return {
                "since_sync_start_s": "not-started",
                "last_valid_activity": "none",
                "inactivity_s": "n/a",
            }
        last_activity = self.last_valid_activity or self.session_started_monotonic
        return {
            "since_sync_start_s": f"{now - self.session_started_monotonic:.3f}",
            "last_valid_activity": f"{last_activity:.3f}",
            "inactivity_s": f"{now - last_activity:.3f}",
        }

    def record_primary_failure(self, exc: BaseException, reason: str) -> bool:
        """Store only the first failure; cleanup/disconnect errors stay secondary."""
        if self.failure is not None:
            return False
        self.failure = exc
        self.primary_cleanup_reason = reason
        return True

    def record_valid_activity(self) -> None:
        """Called only by the serialized RX consumer after successful handling."""
        if self.session_started_monotonic is None:
            return
        now = time.monotonic()
        self.last_valid_activity = now
        self.debug(
            "[SESSION_ACTIVITY] "
            + " ".join(f"{key}={value}" for key, value in self.session_timing_fields(now).items())
        )

    def start_session_watchdog(self) -> None:
        if self.session_started_monotonic is not None:
            raise RuntimeError("session watchdog is already armed")
        now = time.monotonic()
        self.session_started_monotonic = now
        self.last_valid_activity = now
        absolute = (
            self.format_timeout_seconds(self.absolute_session_timeout_seconds)
            if self.absolute_session_timeout_seconds > 0
            else "disabled"
        )
        self.log_monotonic(
            "SESSION_WATCHDOG_ARMED",
            sync_start_monotonic=f"{now:.3f}",
            idle_timeout_s=self.format_timeout_seconds(self.idle_timeout_seconds),
            absolute_timeout_s=absolute,
        )
        if self.idle_timeout_seconds > 0 or self.absolute_session_timeout_seconds > 0:
            self.session_watchdog_task = asyncio.create_task(
                self.session_watchdog_worker(), name="ble-session-watchdog"
            )

    def session_timeout_error(self, now: Optional[float] = None) -> Optional[TimeoutError]:
        if self.session_started_monotonic is None:
            return None
        now = time.monotonic() if now is None else now
        last_activity = self.last_valid_activity or self.session_started_monotonic
        deadlines: list[tuple[float, str]] = []
        if self.idle_timeout_seconds > 0:
            deadlines.append((last_activity + self.idle_timeout_seconds, "idle"))
        if self.absolute_session_timeout_seconds > 0:
            deadlines.append(
                (
                    self.session_started_monotonic + self.absolute_session_timeout_seconds,
                    "absolute",
                )
            )
        if not deadlines:
            return None
        deadline, timeout_type = min(deadlines)
        if now < deadline:
            return None
        if timeout_type == "idle":
            return TimeoutError(
                "BLE session inactive for "
                f"{self.format_timeout_seconds(self.idle_timeout_seconds)} seconds"
            )
        return TimeoutError(
            "BLE session exceeded explicitly configured absolute timeout of "
            f"{self.format_timeout_seconds(self.absolute_session_timeout_seconds)} seconds"
        )

    @staticmethod
    def format_timeout_seconds(value: float) -> str:
        formatted = f"{value:.3f}".rstrip("0")
        return formatted if not formatted.endswith(".") else formatted + "0"

    async def session_watchdog_worker(self) -> None:
        try:
            while True:
                if self.protocol_finished or self.stop_requested:
                    return
                now = time.monotonic()
                timeout_error = self.session_timeout_error(now)
                if timeout_error is not None:
                    reason = str(timeout_error)
                    if self.record_primary_failure(timeout_error, reason):
                        self.log_monotonic(
                            "SESSION_WATCHDOG_TIMEOUT",
                            reason=reason,
                            **self.session_timing_fields(now),
                        )
                    self.request_stop(reason)
                    return

                assert self.session_started_monotonic is not None
                last_activity = self.last_valid_activity or self.session_started_monotonic
                deadlines = []
                if self.idle_timeout_seconds > 0:
                    deadlines.append(last_activity + self.idle_timeout_seconds)
                if self.absolute_session_timeout_seconds > 0:
                    deadlines.append(
                        self.session_started_monotonic + self.absolute_session_timeout_seconds
                    )
                await asyncio.sleep(max(0.001, min(deadlines) - now))
        except asyncio.CancelledError:
            raise

    async def cancel_session_watchdog(self) -> None:
        task = self.session_watchdog_task
        self.session_watchdog_task = None
        if task is not None and task is not asyncio.current_task():
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    async def write_frame(self, msg_type: int, payload: bytes = b"") -> None:
        if self.client is None or self.disconnect_seen:
            raise RuntimeError("BLE client is not connected")
        seq = self.tx_seq
        frame = encode_frame(msg_type, seq, payload)
        print(
            f"[TX] seq={seq} type=0x{msg_type:02X} "
            f"{MESSAGE_NAMES.get(msg_type, 'UNKNOWN')} len={len(payload)}"
        )
        self.tx_seq = (seq + 1) & 0xFFFF
        await self.client.write_gatt_char(
            RN4871_MAIN_CHAR_UUID, frame, response=self.write_response
        )

    async def send_sync_start(self) -> None:
        self.start_session_watchdog()
        await self.write_frame(BLE_MSG_SYNC_START)
        self.log_monotonic("SYNC_START_SENT")

    async def send_ack(self, log_generation: int, page_sequence: int) -> None:
        await self.write_frame(BLE_MSG_ACK_THROUGH, struct.pack("<II", log_generation, page_sequence))
        self.last_ack_monotonic = time.monotonic()
        self.log_monotonic("ACK_SENT", page=page_sequence, pages_saved=self.pages_saved)

    async def send_nack(self, log_generation: int, page_sequence: int, reason: int) -> None:
        await self.write_frame(
            BLE_MSG_NACK_PAGE,
            struct.pack("<IIB3x", log_generation, page_sequence, reason & 0xFF),
        )

    def notification_handler(self, _sender, data: bytearray) -> None:
        """Bleak callback: copy, enqueue, return. No parsing and no task creation."""
        if self.accept_notifications:
            self.rx_queue.put_nowait(bytes(data))

    def disconnected_handler(self, _client) -> None:
        current_page = None if self.current_page is None else self.current_page.page_sequence
        self.log_monotonic(
            "DISCONNECTED_CALLBACK",
            pages_saved=self.pages_saved,
            last_rx_seq=self.last_rx_seq,
            current_page=current_page,
            protocol_finished=self.protocol_finished,
            stop_requested=self.stop_requested,
            primary_error=None if self.failure is None else repr(self.failure),
            **self.session_timing_fields(),
        )
        self.disconnect_seen = True
        self.accept_notifications = False
        if not self.protocol_finished and not self.stop_requested:
            self.record_primary_failure(
                ConnectionError("BLE disconnected before session completion"),
                "BLE disconnected before session completion",
            )
        self.finished.set()

    def start_rx_pipeline(self) -> None:
        if self.rx_consumer_task is not None:
            raise RuntimeError("RX consumer is already running")
        self.accept_notifications = True
        self.rx_consumer_task = asyncio.create_task(self.single_rx_consumer(), name="ble-rx-consumer")

    async def single_rx_consumer(self) -> None:
        self.consumer_count += 1
        try:
            while True:
                item = await self.rx_queue.get()
                try:
                    if isinstance(item, PageTimeoutEvent):
                        await self.handle_page_timeout(item)
                        continue
                    self.debug(f"[RX-BYTES] {len(item)} bytes: {format_hex(item, max_bytes=40)}")
                    for frame in self.parser.feed(item):
                        handled_successfully = await self.handle_frame(frame)
                        if handled_successfully:
                            self.record_valid_activity()
                        if self.stop_requested:
                            break
                finally:
                    self.rx_queue.task_done()
                if self.stop_requested:
                    self.discard_rx_queue()
                    return
        except asyncio.CancelledError:
            raise
        except BaseException as exc:
            self.record_primary_failure(exc, f"RX consumer failed: {exc}")
            self.stop_requested = True
            self.accept_notifications = False
            self.finished.set()

    def discard_rx_queue(self) -> None:
        while True:
            try:
                self.rx_queue.get_nowait()
            except asyncio.QueueEmpty:
                return
            else:
                self.rx_queue.task_done()

    async def wait_rx_idle(self) -> None:
        await self.rx_queue.join()
        if self.failure is not None:
            raise self.failure

    async def handle_frame(self, frame: Frame) -> bool:
        name = MESSAGE_NAMES.get(frame.msg_type, "UNKNOWN")
        if self.last_rx_seq is not None:
            expected = (self.last_rx_seq + 1) & 0xFFFF
            if frame.seq != expected:
                self.debug(f"[RX-SEQUENCE] expected={expected} actual={frame.seq} (accepted)")
        self.last_rx_seq = frame.seq

        print(
            f"[RX] seq={frame.seq} type=0x{frame.msg_type:02X} {name} "
            f"payload={len(frame.payload)} bytes"
        )
        if frame.msg_type == BLE_MSG_SYNC_STATUS:
            status = parse_sync_status(frame.payload)
            self.accept_status(status)
            self.status = status
            self.print_status(status)
            return True
        elif frame.msg_type == BLE_MSG_PAGE_BEGIN:
            return await self.handle_page_begin(frame.payload)
        elif frame.msg_type == BLE_MSG_PAGE_DATA:
            return await self.handle_page_data(frame.payload)
        elif frame.msg_type == BLE_MSG_PAGE_END:
            return await self.handle_page_end(frame.payload)
        elif frame.msg_type == BLE_MSG_SYNC_COMPLETE:
            self.log_monotonic("SYNC_COMPLETE_RX", last_rx_seq=frame.seq)
            print(f"[COMPLETE] payload={frame.payload.hex(' ').upper()}")
            self.protocol_finished = True
            self.finished.set()
            return True
        elif frame.msg_type == BLE_MSG_SYNC_ABORT:
            self.log_monotonic("SYNC_ABORT_RX", last_rx_seq=frame.seq)
            print(f"[ABORT] payload={frame.payload.hex(' ').upper()}")
            self.protocol_finished = True
            self.finished.set()
            return True
        elif frame.msg_type == BLE_MSG_ERROR:
            print(f"[ERROR FRAME] payload={frame.payload.hex(' ').upper()}")
            return True
        else:
            print(f"[WARN] Unhandled message type 0x{frame.msg_type:02X}")
            return False

    async def handle_page_begin(self, payload: bytes) -> bool:
        if self.max_pages is not None and self.pages_saved >= self.max_pages:
            self.request_stop(f"Reached max pages: {self.max_pages}")
            return False
        token = self.next_page_token
        self.next_page_token += 1
        page = parse_page_begin(payload, token)
        if self.status is None:
            raise PageProtocolError("PAGE_BEGIN received before SYNC_STATUS")
        if page.log_generation != self.status.log_generation:
            raise PageProtocolError(
                f"PAGE_BEGIN generation {page.log_generation} != status {self.status.log_generation}"
            )
        if page.page_sequence == 0:
            raise PageProtocolError("PAGE_BEGIN has reserved page sequence 0")
        if (
            self.status.sync_high_watermark_page_sequence != 0
            and page.page_sequence > self.status.sync_high_watermark_page_sequence
        ):
            raise PageProtocolError(
                f"PAGE_BEGIN sequence {page.page_sequence} exceeds high watermark "
                f"{self.status.sync_high_watermark_page_sequence}"
            )
        previous = self.current_page
        if previous is not None:
            raise PageProtocolError(
                f"PAGE_BEGIN {page.page_sequence} while page {previous.page_sequence} is active"
            )

        # Assign the complete state before the first await.
        self.current_page = page
        self.resync_identity = None
        self.debug(
            f"[PAGE_BEGIN-STATE] sequence={page.page_sequence} active_page_token={token} "
            f"previous_active_page={None if previous is None else previous.page_sequence}"
        )
        print(
            f"[PAGE_BEGIN] seq={page.page_sequence} physical={page.physical_page_index} "
            f"magic={page.magic!r} logical={page.logical_page_bytes} "
            f"crc32=0x{page.page_crc32:08X}"
        )
        await self.replace_page_timeout(page)
        return True

    async def handle_page_data(self, payload: bytes) -> bool:
        generation, sequence, offset, chunk = parse_page_data(payload)
        page = self.current_page
        if page is None:
            await self.fail_mid_page(
                generation, sequence, "PAGE_DATA without PAGE_BEGIN"
            )
            return False
        if generation != page.log_generation or sequence != page.page_sequence:
            await self.fail_mid_page(generation, sequence, "PAGE_DATA identity mismatch")
            return False
        if offset != page.next_expected_offset:
            await self.fail_mid_page(
                generation,
                sequence,
                f"PAGE_DATA offset {offset} != expected {page.next_expected_offset}",
            )
            return False
        if not chunk:
            await self.fail_mid_page(generation, sequence, "PAGE_DATA has empty chunk")
            return False
        if offset + len(chunk) > page.logical_page_bytes:
            await self.fail_mid_page(generation, sequence, "PAGE_DATA exceeds logical page size")
            return False
        page.data.extend(chunk)
        page.next_expected_offset += len(chunk)
        page.received_bytes += len(chunk)
        self.debug(
            f"[PAGE_DATA-STATE] sequence={sequence} offset={offset} length={len(chunk)} "
            f"received_bytes={page.received_bytes} expected_total={page.logical_page_bytes} "
            f"active_page_token={page.generation_token}"
        )
        print(
            f"[PAGE_DATA] page={sequence} offset={offset} len={len(chunk)} "
            f"received={page.received_bytes}/{page.logical_page_bytes}"
        )
        return True

    async def handle_page_end(self, payload: bytes) -> bool:
        generation, sequence, logical_bytes, page_crc32 = parse_page_end(payload)
        page = self.current_page
        if page is None:
            await self.enter_resync(
                generation,
                sequence,
                "PAGE_END without PAGE_BEGIN",
                NACK_FRAME_INCOMPLETE,
            )
            return False
        token = page.generation_token
        if (
            generation != page.log_generation
            or sequence != page.page_sequence
            or logical_bytes != page.logical_page_bytes
            or page_crc32 != page.page_crc32
        ):
            await self.enter_resync(
                generation, sequence, "PAGE_END does not match PAGE_BEGIN", NACK_FRAME_INCOMPLETE
            )
            self.clear_active_page(token, "page-end-identity-error")
            return False
        if page.received_bytes != page.logical_page_bytes:
            await self.enter_resync(
                generation,
                sequence,
                f"incomplete page: got {page.received_bytes}, expected {page.logical_page_bytes}",
                NACK_FRAME_INCOMPLETE,
            )
            self.clear_active_page(token, "page-incomplete")
            return False
        computed_crc32 = crc32_iso_hdlc(page.data)
        if computed_crc32 != page.page_crc32:
            await self.enter_resync(
                generation,
                sequence,
                f"CRC32 mismatch: calc=0x{computed_crc32:08X}, expected=0x{page.page_crc32:08X}",
                NACK_PAGE_CRC,
            )
            self.clear_active_page(token, "page-crc-error")
            return False

        await self.cancel_page_timeout()
        duplicate = self.check_duplicate(page)
        if not duplicate:
            try:
                self.persist_page(page)
            except BaseException:
                # Firmware is in WAIT_ACK here, so this NACK is supported.
                with contextlib.suppress(Exception):
                    await self.send_nack(generation, sequence, NACK_LOCAL_SAVE_FAILED)
                raise
        else:
            print(f"[PAGE_DUPLICATE] seq={sequence} already persisted; ACK repeated")

        self.pages_saved += 1
        print(
            f"[PAGE_OK] seq={page.page_sequence} magic={page.magic!r} "
            f"bytes={page.logical_page_bytes} saved_pages={self.pages_saved}"
        )
        # Persistence (or duplicate validation) always precedes the cumulative ACK.
        await self.send_ack(page.log_generation, page.page_sequence)
        if self.pages_saved % 50 == 0:
            self.log_monotonic(
                "PAGE_MILESTONE",
                pages_saved=self.pages_saved,
                page=page.page_sequence,
                last_rx_seq=self.last_rx_seq,
            )
        self.clear_active_page(token, "acked")

        if self.max_pages is not None and self.pages_saved >= self.max_pages:
            self.request_stop(f"Reached max pages: {self.max_pages}")
        return True

    async def fail_mid_page(self, generation: int, sequence: int, reason: str) -> None:
        """NACK is invalid while firmware is still in SEND_PAGE_DATA; stop cleanly."""
        print(f"[ERROR] {reason}")
        self.resync_identity = (generation, sequence)
        token = self.current_page.generation_token if self.current_page else None
        if token is not None:
            self.clear_active_page(token, "mid-page-protocol-error")
        self.record_primary_failure(PageProtocolError(reason), reason)
        self.request_stop("Page resynchronization requires reconnect")

    async def enter_resync(
        self, generation: int, sequence: int, reason: str, nack_reason: int
    ) -> None:
        identity = (generation, sequence)
        print(f"[ERROR] {reason}")
        self.resync_identity = identity
        if identity not in self.nacked_identities:
            self.nacked_identities.add(identity)
            await self.send_nack(generation, sequence, nack_reason)
            print(f"[RESYNC] NACK sent once for generation={generation} page={sequence}")
        else:
            failure = PageProtocolError(
                f"page generation={generation} sequence={sequence} failed again after NACK"
            )
            self.record_primary_failure(failure, str(failure))
            self.request_stop("Page retry failed; reconnect required")

    def clear_active_page(self, expected_token: int, reason: str) -> bool:
        actual = None if self.current_page is None else self.current_page.generation_token
        cleared = actual == expected_token
        self.debug(
            f"[PAGE_CLEAR] reason={reason} expected_token={expected_token} actual_token={actual} "
            f"cleared={'yes' if cleared else 'no'}"
        )
        if cleared:
            self.current_page = None
        return cleared

    async def replace_page_timeout(self, page: PageContext) -> None:
        await self.cancel_page_timeout()
        if self.page_timeout > 0:
            self.page_timeout_task = asyncio.create_task(
                self.page_timeout_worker(page.generation_token, page.page_sequence),
                name=f"page-timeout-{page.page_sequence}",
            )

    async def page_timeout_worker(self, token: int, page_sequence: int) -> None:
        try:
            await asyncio.sleep(self.page_timeout)
            await self.rx_queue.put(PageTimeoutEvent(token, page_sequence))
        except asyncio.CancelledError:
            raise

    async def handle_page_timeout(self, event: PageTimeoutEvent) -> None:
        actual = None if self.current_page is None else self.current_page.generation_token
        stale = actual != event.token
        self.debug(
            f"[PAGE_TIMEOUT] token={event.token} page_sequence={event.page_sequence} "
            f"ignored_as_stale={'yes' if stale else 'no'}"
        )
        if stale:
            return
        self.clear_active_page(event.token, "page-timeout")
        failure = PageProtocolError(f"page {event.page_sequence} timed out")
        self.record_primary_failure(failure, str(failure))
        self.request_stop(f"Page timeout: {event.page_sequence}")

    async def cancel_page_timeout(self) -> None:
        task = self.page_timeout_task
        self.page_timeout_task = None
        if task is not None and task is not asyncio.current_task():
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task

    def request_stop(self, reason: str) -> None:
        if not self.stop_requested:
            print(f"[STOP] {reason}")
        if self.primary_cleanup_reason is None:
            self.primary_cleanup_reason = reason
        self.stop_requested = True
        self.accept_notifications = False
        self.finished.set()

    def open_output_files(self) -> None:
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.pages_csv_path.parent.mkdir(parents=True, exist_ok=True)
        if self.resume:
            self.load_resume_files()
            self.output_file = self.output_path.open("r+b")
            self.output_file.seek(0, os.SEEK_END)
            self.csv_file = self.pages_csv_path.open("a", newline="", encoding="utf-8")
        else:
            self.output_file = self.output_path.open("wb")
            self.csv_file = self.pages_csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        if not self.resume:
            self.csv_writer.writeheader()
            self.csv_file.flush()
            self.sync_file(self.csv_file)

    def load_resume_files(self) -> None:
        missing = [
            str(path)
            for path in (self.output_path, self.pages_csv_path, self.metadata_path)
            if not path.exists()
        ]
        if missing:
            raise RuntimeError("--resume requires existing coherent BIN, CSV and metadata: " + ", ".join(missing))
        with self.metadata_path.open("r", encoding="utf-8") as metadata_file:
            metadata = json.load(metadata_file)
        if metadata.get("format") != 1 or not isinstance(metadata.get("device_uid"), str):
            raise RuntimeError(f"invalid resume metadata: {self.metadata_path}")
        self.resume_metadata = metadata
        try:
            metadata_uid = bytes.fromhex(str(metadata["device_uid"]))
        except (KeyError, ValueError) as exc:
            raise RuntimeError(f"invalid device_uid in resume metadata: {self.metadata_path}") from exc
        if len(metadata_uid) != 12:
            raise RuntimeError(f"resume metadata device_uid must contain 12 bytes: {self.metadata_path}")

        expected_offset = 0
        with self.pages_csv_path.open("r", newline="", encoding="utf-8") as index_file, self.output_path.open("rb") as binary:
            reader = csv.DictReader(index_file)
            if reader.fieldnames != CSV_FIELDS:
                raise RuntimeError(
                    f"resume CSV columns do not match expected schema: {reader.fieldnames}"
                )
            for row_number, row in enumerate(reader, 2):
                try:
                    saved = self.saved_page_from_row(row)
                except (KeyError, ValueError) as exc:
                    raise RuntimeError(f"invalid resume CSV row {row_number}: {exc}") from exc
                key = (metadata_uid, saved.log_generation, saved.page_sequence)
                if key in self.saved_pages:
                    raise RuntimeError(f"duplicate page identity in resume CSV: {key}")
                if saved.file_offset != expected_offset:
                    raise RuntimeError(
                        f"non-contiguous file_offset at row {row_number}: "
                        f"expected {expected_offset}, got {saved.file_offset}"
                    )
                binary.seek(saved.file_offset)
                data = binary.read(saved.logical_page_bytes)
                if len(data) != saved.logical_page_bytes:
                    raise RuntimeError(f"short BIN data for page {saved.page_sequence}")
                if crc32_iso_hdlc(data) != saved.page_crc32:
                    raise RuntimeError(f"CRC32 mismatch in saved page {saved.page_sequence}")
                if data[:4] != saved.magic:
                    raise RuntimeError(f"magic mismatch in saved page {saved.page_sequence}")
                self.saved_pages[key] = saved
                expected_offset += saved.logical_page_bytes
        if self.output_path.stat().st_size != expected_offset:
            raise RuntimeError(
                f"BIN length {self.output_path.stat().st_size} != indexed length {expected_offset}"
            )
        generations = {page.log_generation for page in self.saved_pages.values()}
        if len(generations) > 1 or (generations and metadata.get("log_generation") not in generations):
            raise RuntimeError("resume files contain incompatible log generations")
        self.persisted_pages = len(self.saved_pages)

    @staticmethod
    def saved_page_from_row(row: dict[str, str]) -> SavedPage:
        crc_text = row["page_crc32"]
        return SavedPage(
            int(row["log_generation"]),
            int(row["page_sequence"]),
            int(row["physical_page_index"]),
            int(row["logical_page_bytes"]),
            int(crc_text, 0),
            row["magic"].encode("ascii"),
            int(row["page_version"]),
            int(row["page_header_size"]),
            int(row["page_payload_bytes"]),
            int(row["file_offset"]),
        )

    def accept_status(self, status: SyncStatus) -> None:
        if self.resume:
            assert self.resume_metadata is not None
            expected_uid = str(self.resume_metadata["device_uid"])
            expected_generation = int(self.resume_metadata["log_generation"])
            if status.device_uid.hex().upper() != expected_uid or status.log_generation != expected_generation:
                raise RuntimeError(
                    "resume identity mismatch: "
                    f"expected uid={expected_uid} generation={expected_generation}, "
                    f"got uid={status.device_uid.hex().upper()} generation={status.log_generation}"
                )
        else:
            metadata = {
                "format": 1,
                "device_uid": status.device_uid.hex().upper(),
                "log_generation": status.log_generation,
            }
            with self.metadata_path.open("w", encoding="utf-8") as metadata_file:
                json.dump(metadata, metadata_file, indent=2)
                metadata_file.write("\n")
                metadata_file.flush()
                os.fsync(metadata_file.fileno())
            self.resume_metadata = metadata

    def check_duplicate(self, page: PageContext) -> bool:
        if self.status is None:
            raise RuntimeError("cannot deduplicate a page before SYNC_STATUS")
        key = (self.status.device_uid, page.log_generation, page.page_sequence)
        saved = self.saved_pages.get(key)
        if saved is None:
            return False
        if (
            saved.physical_page_index != page.physical_page_index
            or saved.logical_page_bytes != page.logical_page_bytes
            or saved.page_crc32 != page.page_crc32
            or saved.magic != page.magic
            or saved.page_version != page.page_version
            or saved.page_header_size != page.page_header_size
            or saved.page_payload_bytes != page.page_payload_bytes
        ):
            raise PageProtocolError(
                f"saved identity collision for generation={page.log_generation} page={page.page_sequence}"
            )
        return True

    def persist_page(self, page: PageContext) -> None:
        if self.output_file is None or self.csv_file is None or self.csv_writer is None:
            raise RuntimeError("output files are not open")
        file_offset = self.output_file.tell()
        self.output_file.write(page.data)
        self.output_file.flush()

        row: dict[str, object] = {
            "page_sequence": page.page_sequence,
            "physical_page_index": page.physical_page_index,
            "log_generation": page.log_generation,
            "magic": page.magic.decode("ascii", errors="strict"),
            "page_version": page.page_version,
            "page_header_size": page.page_header_size,
            "page_payload_bytes": page.page_payload_bytes,
            "logical_page_bytes": page.logical_page_bytes,
            "page_crc32": f"0x{page.page_crc32:08X}",
            "file_offset": file_offset,
        }
        self.csv_writer.writerow(row)
        self.csv_file.flush()
        self.persisted_pages += 1
        if self.fsync_interval_pages > 0 and self.persisted_pages % self.fsync_interval_pages == 0:
            self.sync_file(self.output_file)
            self.sync_file(self.csv_file)

        if self.status is None:
            raise RuntimeError("cannot index a page before SYNC_STATUS")
        self.saved_pages[(self.status.device_uid, page.log_generation, page.page_sequence)] = SavedPage(
            page.log_generation,
            page.page_sequence,
            page.physical_page_index,
            page.logical_page_bytes,
            page.page_crc32,
            page.magic,
            page.page_version,
            page.page_header_size,
            page.page_payload_bytes,
            file_offset,
        )

    @staticmethod
    def sync_file(file_object: BinaryIO | TextIO) -> None:
        os.fsync(file_object.fileno())

    def flush_and_close_files(self) -> None:
        for file_object in (self.output_file, self.csv_file):
            if file_object is not None:
                with contextlib.suppress(Exception):
                    file_object.flush()
                    self.sync_file(file_object)
                with contextlib.suppress(Exception):
                    file_object.close()
        self.output_file = None
        self.csv_file = None
        self.csv_writer = None

    async def shutdown_rx_pipeline(self) -> None:
        self.accept_notifications = False
        await self.cancel_session_watchdog()
        await self.cancel_page_timeout()
        task = self.rx_consumer_task
        self.rx_consumer_task = None
        if task is not None:
            task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await task
        self.discard_rx_queue()

    async def run(self) -> None:
        primary_error: Optional[BaseException] = None
        try:
            device = await find_device(self.device_name, self.scan_timeout)
            print("[CONNECT] Connecting...")
            async with BleakClient(device, disconnected_callback=self.disconnected_handler) as client:
                self.client = client
                self.disconnect_seen = False
                self.log_monotonic("CONNECTED", address=device.address)
                print("[CONNECT] Connected")
                for service in client.services:
                    print(f"[GATT] Service {service.uuid}")
                    for char in service.characteristics:
                        print(f"  Char {char.uuid} [{','.join(char.properties)}]")

                notifications_started = False
                try:
                    self.open_output_files()
                    self.start_rx_pipeline()
                    print("[NOTIFY] Enabling notifications...")
                    await client.start_notify(RN4871_MAIN_CHAR_UUID, self.notification_handler)
                    notifications_started = True
                    await asyncio.sleep(0.5)
                    print("[SYNC] Sending SYNC_START")
                    await self.send_sync_start()
                    await self.finished.wait()
                    if self.failure is not None:
                        raise self.failure
                    if self.stop_requested:
                        # Give the board time to consume the last cumulative ACK;
                        # disconnect handling then persists its dirty ACK state.
                        await asyncio.sleep(0.2)
                except BaseException as exc:
                    self.record_primary_failure(exc, str(exc))
                    primary_error = self.failure or exc
                finally:
                    self.log_monotonic(
                        "CLIENT_CLEANUP",
                        disconnect_seen=self.disconnect_seen,
                        protocol_finished=self.protocol_finished,
                        pages_saved=self.pages_saved,
                        primary_reason=(
                            self.primary_cleanup_reason
                            or ("protocol finished" if self.protocol_finished else "normal stop")
                        ),
                        **self.session_timing_fields(),
                    )
                    self.accept_notifications = False
                    if notifications_started and not self.disconnect_seen:
                        print("[NOTIFY] Disabling notifications...")
                        with contextlib.suppress(Exception):
                            await client.stop_notify(RN4871_MAIN_CHAR_UUID)
                    await self.shutdown_rx_pipeline()
                    self.flush_and_close_files()
                self.client = None
        except BaseException as exc:
            if primary_error is None:
                self.record_primary_failure(exc, str(exc))
                primary_error = self.failure or exc
        finally:
            self.accept_notifications = False
            await self.shutdown_rx_pipeline()
            self.flush_and_close_files()

        print(f"[SAVE] Binary pages saved to: {self.output_path}")
        print(f"[SAVE] Page index CSV: {self.pages_csv_path}")
        print(f"[DONE] Pages completed this run: {self.pages_saved}")
        if primary_error is not None:
            raise primary_error

    @staticmethod
    def print_status(status: SyncStatus) -> None:
        print("[STATUS]")
        print(f"  device_uid: {status.device_uid.hex().upper()}")
        print(f"  log_generation: {status.log_generation}")
        print(f"  ack_valid: {status.ack_valid}")
        print(f"  acked_through: {status.acked_through_page_sequence}")
        print(f"  oldest_available: {status.oldest_available_page_sequence}")
        print(f"  newest_available: {status.newest_available_page_sequence}")
        print(f"  first_to_send: {status.first_page_sequence_to_send}")
        print(f"  high_watermark: {status.sync_high_watermark_page_sequence}")
        print(f"  total_unsynced: {status.total_unsynced_pages}")



# ---------------------------------------------------------------------------
# BLE live dashboard and simulator
# ---------------------------------------------------------------------------


def _decode_db_centi_live(raw_value: int) -> float:
    if raw_value == AUDIO_DB_CENTI_INVALID:
        return float("nan")
    return raw_value / 100.0


def decode_live_frame(frame: Frame) -> dict[str, Any] | None:
    """Decode one live frame.

    The live payload formats below are intentionally centralized so they can be
    updated after the firmware implementation report without touching the GUI.
    """
    if frame.msg_type in (BLE_MSG_ACK_LIVE_START, BLE_MSG_ACK_LIVE_STOP):
        if len(frame.payload) != 4:
            raise ValueError(
                f"{MESSAGE_NAMES[frame.msg_type]} payload must be 4 bytes, got {len(frame.payload)}"
            )
        packet_sequence = struct.unpack(LIVE_PACKET_SEQUENCE_FORMAT, frame.payload)[0]
        return {
            "kind": "ack",
            "ack": MESSAGE_NAMES[frame.msg_type],
            "packet_sequence": packet_sequence,
        }

    if frame.msg_type == BLE_MSG_LIVE_IMU:
        if len(frame.payload) != LIVE_IMU_SIZE:
            raise ValueError(
                f"LIVE_IMU payload must be {LIVE_IMU_SIZE} bytes, got {len(frame.payload)}"
            )
        (
            packet_sequence,
            hh,
            mm,
            ss,
            sss,
            acc_x,
            acc_y,
            acc_z,
            gyro_x,
            gyro_y,
            gyro_z,
        ) = struct.unpack(LIVE_IMU_FORMAT, frame.payload)
        timestamp_ms = (
            int(hh) * 3_600_000
            + int(mm) * 60_000
            + int(ss) * 1_000
            + int(sss)
        )
        return {
            "kind": "imu",
            "packet_sequence": packet_sequence,
            "timestamp_ms": timestamp_ms,
            "hh": hh,
            "mm": mm,
            "ss": ss,
            "sss": sss,
            "acc_x": acc_x,
            "acc_y": acc_y,
            "acc_z": acc_z,
            "gyro_x": gyro_x,
            "gyro_y": gyro_y,
            "gyro_z": gyro_z,
        }

    if frame.msg_type == BLE_MSG_LIVE_AFEA:
        if len(frame.payload) != LIVE_AFEA_SIZE:
            raise ValueError(
                f"LIVE_AFEA payload must be {LIVE_AFEA_SIZE} bytes, got {len(frame.payload)}"
            )
        packet_sequence = struct.unpack_from("<I", frame.payload, 0)[0]
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
        ) = struct.unpack_from(AUDIO_FEATURE_RECORD_FORMAT, frame.payload, 4)
        return {
            "kind": "afea",
            "packet_sequence": packet_sequence,
            "window_sequence": window_sequence,
            "window_start_ms": window_start_ms,
            "sample_count": sample_count,
            "mean_counts_rounded": mean_counts_rounded,
            "rms_z_dbfs": _decode_db_centi_live(rms_z_centi_dbfs),
            "rms_a_dbfs": _decode_db_centi_live(rms_a_centi_dbfs),
            "estimated_laeq_dba": _decode_db_centi_live(estimated_laeq_centi_dba),
            "peak_dbfs": _decode_db_centi_live(peak_centi_dbfs),
            "clipped_sample_count": clipped_sample_count,
            "environment_class": environment_class,
            "environment_label": LIVE_AUDIO_ENVIRONMENT_LABELS.get(
                environment_class, f"UNKNOWN_{environment_class}"
            ),
            "flags": flags,
        }

    if frame.msg_type == BLE_MSG_LIVE_LFEA:
        if len(frame.payload) != LIVE_LFEA_SIZE:
            raise ValueError(
                f"LIVE_LFEA payload must be {LIVE_LFEA_SIZE} bytes, got {len(frame.payload)}"
            )
        packet_sequence = struct.unpack_from("<I", frame.payload, 0)[0]
        unpacked = struct.unpack_from(LIGHT_FEATURE_RECORD_FORMAT, frame.payload, 4)
        window_sequence, sample_timestamp_ms = unpacked[:2]
        channels = unpacked[2:12]
        exposure_class, flags = unpacked[12:14]
        return {
            "kind": "lfea",
            "packet_sequence": packet_sequence,
            "window_sequence": window_sequence,
            "sample_timestamp_ms": sample_timestamp_ms,
            "f1": channels[0],
            "f2": channels[1],
            "f3": channels[2],
            "f4": channels[3],
            "f5": channels[4],
            "f6": channels[5],
            "f7": channels[6],
            "f8": channels[7],
            "clear": channels[8],
            "nir": channels[9],
            "exposure_class": exposure_class,
            "exposure_label": LIVE_EXPOSURE_LABELS.get(
                exposure_class, f"UNKNOWN_{exposure_class}"
            ),
            "flags": flags,
        }

    if frame.msg_type == BLE_MSG_ERROR:
        if len(frame.payload) != LIVE_ERROR_SIZE:
            raise ValueError(
                f"ERROR payload must be {LIVE_ERROR_SIZE} bytes, got {len(frame.payload)}"
            )
        packet_sequence, error_code, detail = struct.unpack(
            LIVE_ERROR_FORMAT, frame.payload
        )
        return {
            "kind": "error",
            "packet_sequence": packet_sequence,
            "error_code": error_code,
            "detail": detail,
            "message": f"Device error code={error_code}, detail={detail}",
        }

    return None


class LiveSimulatorWorker(threading.Thread):
    """Generate realistic IMU/AFEA/LFEA traffic without BLE hardware."""

    def __init__(self, event_queue: "queue.Queue[dict[str, Any]]") -> None:
        super().__init__(name="live-simulator", daemon=True)
        self.event_queue = event_queue
        self.command_queue: "queue.Queue[str]" = queue.Queue()
        self.stop_event = threading.Event()
        self.live_enabled = False
        self.packet_sequence = 0
        self.imu_timestamp_ms = 0
        self.window_sequence = 0
        self.next_feature_monotonic: float | None = None
        self.simulation_start_monotonic = time.monotonic()

    def send_command(self, command: str) -> None:
        self.command_queue.put(command)

    def stop(self) -> None:
        self.stop_event.set()

    def _next_packet_sequence(self) -> int:
        value = self.packet_sequence
        self.packet_sequence = (self.packet_sequence + 1) & 0xFFFFFFFF
        return value

    def _emit_ack(self, ack_name: str) -> None:
        self.event_queue.put(
            {
                "kind": "ack",
                "ack": ack_name,
                "packet_sequence": self._next_packet_sequence(),
            }
        )

    def _handle_commands(self) -> None:
        while True:
            try:
                command = self.command_queue.get_nowait()
            except queue.Empty:
                return
            if command == "start":
                if not self.live_enabled:
                    self.live_enabled = True
                    self.next_feature_monotonic = time.monotonic() + 7.0
                self._emit_ack("ACK_LIVE_START")
            elif command == "stop":
                self.live_enabled = False
                self.next_feature_monotonic = None
                self._emit_ack("ACK_LIVE_STOP")

    def _emit_imu(self) -> None:
        t = self.imu_timestamp_ms / 1000.0
        acc_x = int(3500.0 * math.sin(2.0 * math.pi * 0.7 * t))
        acc_y = int(2500.0 * math.sin(2.0 * math.pi * 0.43 * t + 0.8))
        acc_z = int(16384.0 + 1200.0 * math.sin(2.0 * math.pi * 1.1 * t))
        gyro_x = int(450.0 * math.sin(2.0 * math.pi * 0.35 * t))
        gyro_y = int(300.0 * math.sin(2.0 * math.pi * 0.51 * t + 0.4))
        gyro_z = int(520.0 * math.sin(2.0 * math.pi * 0.22 * t + 1.1))
        self.event_queue.put(
            {
                "kind": "imu",
                "packet_sequence": self._next_packet_sequence(),
                "timestamp_ms": self.imu_timestamp_ms,
                "acc_x": acc_x,
                "acc_y": acc_y,
                "acc_z": acc_z,
                "gyro_x": gyro_x,
                "gyro_y": gyro_y,
                "gyro_z": gyro_z,
            }
        )
        self.imu_timestamp_ms = (self.imu_timestamp_ms + 10) & 0xFFFFFFFF

    def _emit_features(self) -> None:
        window_start_ms = max(0, self.imu_timestamp_ms - 7000)
        ambient_phase = self.window_sequence * 0.55
        laeq = 58.0 + 8.0 * math.sin(ambient_phase) + random.uniform(-1.0, 1.0)
        rms_a = laeq - 122.4
        rms_z = rms_a + random.uniform(0.5, 2.5)
        peak = min(-1.0, rms_z + random.uniform(8.0, 16.0))
        environment_class = min(6, max(0, int((laeq - 25.0) // 10.0)))
        self.event_queue.put(
            {
                "kind": "afea",
                "packet_sequence": self._next_packet_sequence(),
                "window_sequence": self.window_sequence,
                "window_start_ms": window_start_ms,
                "sample_count": 48000,
                "mean_counts_rounded": random.randint(-8, 8),
                "rms_z_dbfs": rms_z,
                "rms_a_dbfs": rms_a,
                "estimated_laeq_dba": laeq,
                "peak_dbfs": peak,
                "clipped_sample_count": 0,
                "environment_class": environment_class,
                "environment_label": LIVE_AUDIO_ENVIRONMENT_LABELS.get(
                    environment_class, "UNKNOWN"
                ),
                "flags": 0x07,
            }
        )

        clear = int(5200 + 4200 * (0.5 + 0.5 * math.sin(ambient_phase + 0.6)))
        clear = max(0, min(10000, clear))
        exposure_class = 0 if clear < 3 else 1 if clear < 50 else 2 if clear < 6500 else 3 if clear < 9800 else 4
        spectral_shape = (0.52, 0.61, 0.74, 0.88, 0.96, 0.91, 0.78, 0.64)
        spectral = [
            max(0, min(65535, int(clear * factor + random.uniform(-80, 80))))
            for factor in spectral_shape
        ]
        nir = max(0, min(65535, int(clear * 0.72 + random.uniform(-80, 80))))
        self.event_queue.put(
            {
                "kind": "lfea",
                "packet_sequence": self._next_packet_sequence(),
                "window_sequence": self.window_sequence,
                "sample_timestamp_ms": self.imu_timestamp_ms,
                **{f"f{index + 1}": value for index, value in enumerate(spectral)},
                "clear": clear,
                "nir": nir,
                "exposure_class": exposure_class,
                "exposure_label": LIVE_EXPOSURE_LABELS[exposure_class],
                "flags": 0x07 | (0x08 if clear >= 10000 else 0),
            }
        )
        self.window_sequence = (self.window_sequence + 1) & 0xFFFFFFFF

    def run(self) -> None:
        self.event_queue.put(
            {
                "kind": "status",
                "state": "connected",
                "message": "Simulator connected",
            }
        )
        next_imu = time.monotonic()
        while not self.stop_event.is_set():
            self._handle_commands()
            now = time.monotonic()
            if self.live_enabled:
                while now >= next_imu:
                    self._emit_imu()
                    next_imu += 0.010
                if self.next_feature_monotonic is not None and now >= self.next_feature_monotonic:
                    self._emit_features()
                    self.next_feature_monotonic += 7.0
            else:
                next_imu = now + 0.010
            time.sleep(0.001)
        self.event_queue.put(
            {
                "kind": "status",
                "state": "disconnected",
                "message": "Simulator stopped",
            }
        )


class BleLiveWorker(threading.Thread):
    """Run the real Bleak connection in a background asyncio loop."""

    def __init__(
        self,
        event_queue: "queue.Queue[dict[str, Any]]",
        device_name: str,
        scan_timeout: float,
        write_response: bool,
    ) -> None:
        super().__init__(name="ble-live-worker", daemon=True)
        self.event_queue = event_queue
        self.device_name = device_name
        self.scan_timeout = scan_timeout
        self.write_response = write_response
        self.command_queue: "queue.Queue[str]" = queue.Queue()
        self.stop_event = threading.Event()
        self.parser = FrameParser()
        self.tx_seq = 1
        self.client: Any = None
        self.live_acknowledged = False
        self.next_heartbeat_monotonic: float | None = None

    def send_command(self, command: str) -> None:
        self.command_queue.put(command)

    def stop(self) -> None:
        self.stop_event.set()

    def _notification_handler(self, _sender: Any, data: bytearray) -> None:
        try:
            for frame in self.parser.feed(bytes(data)):
                event = decode_live_frame(frame)
                if event is not None:
                    if event.get("kind") == "ack":
                        if event.get("ack") == "ACK_LIVE_START":
                            self.live_acknowledged = True
                            self.next_heartbeat_monotonic = (
                                time.monotonic() + LIVE_HEARTBEAT_PERIOD_SECONDS
                            )
                        elif event.get("ack") == "ACK_LIVE_STOP":
                            self.live_acknowledged = False
                            self.next_heartbeat_monotonic = None
                    self.event_queue.put(event)
        except BaseException as exc:
            self.event_queue.put(
                {
                    "kind": "error",
                    "packet_sequence": None,
                    "message": f"Live frame decode failed: {exc}",
                }
            )

    def _disconnected_handler(self, _client: Any) -> None:
        self.live_acknowledged = False
        self.next_heartbeat_monotonic = None
        self.event_queue.put(
            {
                "kind": "status",
                "state": "disconnected",
                "message": "BLE disconnected",
            }
        )

    async def _write_command(self, msg_type: int) -> None:
        if self.client is None or not self.client.is_connected:
            raise RuntimeError("BLE is not connected")
        seq = self.tx_seq
        self.tx_seq = (self.tx_seq + 1) & 0xFFFF
        await self.client.write_gatt_char(
            RN4871_MAIN_CHAR_UUID,
            encode_frame(msg_type, seq),
            response=self.write_response,
        )

    async def _run_async(self) -> None:
        if BleakClient is None or BleakScanner is None:
            raise RuntimeError("Bleak is not installed. Install it with: pip install bleak")
        self.event_queue.put(
            {"kind": "status", "state": "scanning", "message": "Scanning for BLE device"}
        )
        device = await find_device(self.device_name, self.scan_timeout)
        self.event_queue.put(
            {"kind": "status", "state": "connecting", "message": f"Connecting to {device.address}"}
        )
        async with BleakClient(device, disconnected_callback=self._disconnected_handler) as client:
            self.client = client
            await client.start_notify(RN4871_MAIN_CHAR_UUID, self._notification_handler)
            self.event_queue.put(
                {
                    "kind": "status",
                    "state": "connected",
                    "message": f"Connected to {device.name or device.address}",
                }
            )
            while not self.stop_event.is_set() and client.is_connected:
                while True:
                    try:
                        command = self.command_queue.get_nowait()
                    except queue.Empty:
                        break
                    if command == "start":
                        await self._write_command(BLE_MSG_LIVE_START)
                    elif command == "stop":
                        await self._write_command(BLE_MSG_LIVE_STOP)

                now = time.monotonic()
                if (
                    self.live_acknowledged
                    and self.next_heartbeat_monotonic is not None
                    and now >= self.next_heartbeat_monotonic
                ):
                    await self._write_command(BLE_MSG_LIVE_HEARTBEAT)
                    self.next_heartbeat_monotonic = (
                        now + LIVE_HEARTBEAT_PERIOD_SECONDS
                    )

                await asyncio.sleep(0.02)
            if client.is_connected:
                with contextlib.suppress(Exception):
                    await client.stop_notify(RN4871_MAIN_CHAR_UUID)
        self.client = None

    def run(self) -> None:
        try:
            asyncio.run(self._run_async())
        except BaseException as exc:
            self.event_queue.put(
                {
                    "kind": "error",
                    "packet_sequence": None,
                    "message": f"BLE live worker failed: {exc}",
                }
            )
            self.event_queue.put(
                {
                    "kind": "status",
                    "state": "disconnected",
                    "message": "BLE worker stopped",
                }
            )


class LiveDashboard:
    def __init__(
        self,
        *,
        simulate: bool,
        device_name: str,
        scan_timeout: float,
        write_response: bool,
        window_seconds: float,
        refresh_ms: int,
    ) -> None:
        try:
            import tkinter as tk
            from tkinter import ttk
            from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
            from matplotlib.figure import Figure
        except ImportError as exc:
            raise RuntimeError(
                "The live dashboard requires tkinter and matplotlib. "
                "Install matplotlib and use a Python build with Tk support."
            ) from exc

        self.tk = tk
        self.ttk = ttk
        self.FigureCanvasTkAgg = FigureCanvasTkAgg
        self.root = tk.Tk()
        self.root.title("Smart Wearable BLE Live Dashboard")
        self.root.geometry("1380x900")
        self.root.minsize(1000, 700)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        self.simulate = simulate
        self.window_seconds = max(1.0, float(window_seconds))
        self.refresh_ms = max(25, int(refresh_ms))
        self.event_queue: "queue.Queue[dict[str, Any]]" = queue.Queue()
        if simulate:
            self.worker: LiveSimulatorWorker | BleLiveWorker = LiveSimulatorWorker(
                self.event_queue
            )
        else:
            self.worker = BleLiveWorker(
                self.event_queue,
                device_name=device_name,
                scan_timeout=scan_timeout,
                write_response=write_response,
            )

        self.status_var = tk.StringVar(value="Starting")
        self.live_var = tk.StringVar(value="LIVE stopped")
        self.packet_var = tk.StringVar(value="Last packet: -- | Estimated lost: 0")
        self.count_var = tk.StringVar(value="IMU 0 | AFEA 0 | LFEA 0")
        self.audio_var = tk.StringVar(value="Audio: waiting for AFEA")
        self.light_var = tk.StringVar(value="Light: waiting for LFEA")
        self.connected = False
        self.live_active = False
        self.closed = False
        self.last_packet_sequence: int | None = None
        self.estimated_lost_packets = 0
        self.imu_count = 0
        self.afea_count = 0
        self.lfea_count = 0
        self.timestamp_origin_ms: int | None = None
        max_imu_points = max(200, int(self.window_seconds * 120))
        self.imu_t = deque(maxlen=max_imu_points)
        self.acc_x = deque(maxlen=max_imu_points)
        self.acc_y = deque(maxlen=max_imu_points)
        self.acc_z = deque(maxlen=max_imu_points)
        self.gyro_x = deque(maxlen=max_imu_points)
        self.gyro_y = deque(maxlen=max_imu_points)
        self.gyro_z = deque(maxlen=max_imu_points)
        self.audio_t = deque(maxlen=120)
        self.audio_laeq = deque(maxlen=120)
        self.audio_rms_a = deque(maxlen=120)
        self.audio_peak = deque(maxlen=120)
        self.light_channels = [0.0] * 10
        self.plot_dirty = True

        controls = ttk.Frame(self.root, padding=8)
        controls.pack(fill=tk.X)
        ttk.Label(
            controls,
            text="SIMULATION" if simulate else "REAL BLE",
            font=("TkDefaultFont", 11, "bold"),
        ).pack(side=tk.LEFT, padx=(0, 12))
        self.start_button = ttk.Button(controls, text="LIVE_START", command=self.on_start)
        self.start_button.pack(side=tk.LEFT, padx=4)
        self.stop_button = ttk.Button(controls, text="LIVE_STOP", command=self.on_stop)
        self.stop_button.pack(side=tk.LEFT, padx=4)
        ttk.Button(controls, text="Clear plots", command=self.clear_plots).pack(
            side=tk.LEFT, padx=4
        )
        ttk.Button(controls, text="Close", command=self.on_close).pack(side=tk.RIGHT, padx=4)

        status_frame = ttk.Frame(self.root, padding=(8, 0, 8, 8))
        status_frame.pack(fill=tk.X)
        ttk.Label(status_frame, textvariable=self.status_var).grid(row=0, column=0, sticky="w")
        ttk.Label(status_frame, textvariable=self.live_var).grid(row=0, column=1, sticky="w", padx=20)
        ttk.Label(status_frame, textvariable=self.packet_var).grid(row=0, column=2, sticky="w", padx=20)
        ttk.Label(status_frame, textvariable=self.count_var).grid(row=0, column=3, sticky="w", padx=20)
        ttk.Label(status_frame, textvariable=self.audio_var).grid(row=1, column=0, columnspan=2, sticky="w", pady=(4, 0))
        ttk.Label(status_frame, textvariable=self.light_var).grid(row=1, column=2, columnspan=2, sticky="w", padx=20, pady=(4, 0))
        status_frame.columnconfigure(3, weight=1)

        figure = Figure(figsize=(13, 7.5), dpi=100)
        self.ax_acc = figure.add_subplot(221)
        self.ax_gyro = figure.add_subplot(222)
        self.ax_audio = figure.add_subplot(223)
        self.ax_light = figure.add_subplot(224)
        figure.tight_layout(pad=3.0)

        (self.line_acc_x,) = self.ax_acc.plot([], [], label="Acc X")
        (self.line_acc_y,) = self.ax_acc.plot([], [], label="Acc Y")
        (self.line_acc_z,) = self.ax_acc.plot([], [], label="Acc Z")
        self.ax_acc.set_title("IMU acceleration — 100 Hz")
        self.ax_acc.set_xlabel("Time [s]")
        self.ax_acc.set_ylabel("Raw counts")
        self.ax_acc.grid(True, alpha=0.3)
        self.ax_acc.legend(loc="upper right")

        (self.line_gyro_x,) = self.ax_gyro.plot([], [], label="Gyro X")
        (self.line_gyro_y,) = self.ax_gyro.plot([], [], label="Gyro Y")
        (self.line_gyro_z,) = self.ax_gyro.plot([], [], label="Gyro Z")
        self.ax_gyro.set_title("IMU gyroscope — 100 Hz")
        self.ax_gyro.set_xlabel("Time [s]")
        self.ax_gyro.set_ylabel("Raw counts")
        self.ax_gyro.grid(True, alpha=0.3)
        self.ax_gyro.legend(loc="upper right")

        (self.line_laeq,) = self.ax_audio.plot([], [], marker="o", label="Estimated LAeq [dBA]")
        (self.line_rms_a,) = self.ax_audio.plot([], [], marker="o", label="RMS A [dBFS]")
        (self.line_peak,) = self.ax_audio.plot([], [], marker="o", label="Peak [dBFS]")
        self.ax_audio.set_title("AFEA live — nominal period 7 s")
        self.ax_audio.set_xlabel("Time [s]")
        self.ax_audio.set_ylabel("Level [dB]")
        self.ax_audio.grid(True, alpha=0.3)
        self.ax_audio.legend(loc="best")

        self.light_labels = ["F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "Clear", "NIR"]
        self.light_bars = self.ax_light.bar(self.light_labels, self.light_channels)
        self.ax_light.set_title("LFEA latest spectrum — nominal period 7 s")
        self.ax_light.set_xlabel("AS7341 channel")
        self.ax_light.set_ylabel("Raw counts")
        self.ax_light.grid(True, axis="y", alpha=0.3)

        canvas = FigureCanvasTkAgg(figure, master=self.root)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=8, pady=(0, 8))
        self.canvas = canvas

    def _relative_time(self, timestamp_ms: int) -> float:
        if self.timestamp_origin_ms is None:
            self.timestamp_origin_ms = timestamp_ms
        return ((timestamp_ms - self.timestamp_origin_ms) & 0xFFFFFFFF) / 1000.0

    def _observe_packet_sequence(self, event: dict[str, Any]) -> None:
        packet_sequence = event.get("packet_sequence")
        if packet_sequence is None:
            return
        packet_sequence = int(packet_sequence) & 0xFFFFFFFF
        if self.last_packet_sequence is not None:
            expected = (self.last_packet_sequence + 1) & 0xFFFFFFFF
            delta = (packet_sequence - expected) & 0xFFFFFFFF
            if 0 < delta < 0x80000000:
                self.estimated_lost_packets += delta
        self.last_packet_sequence = packet_sequence
        self.packet_var.set(
            f"Last packet: {packet_sequence} | Estimated lost: {self.estimated_lost_packets}"
        )

    def _handle_event(self, event: dict[str, Any]) -> None:
        kind = event.get("kind")
        if kind == "status":
            state = str(event.get("state", "unknown"))
            self.connected = state == "connected"
            self.status_var.set(f"Connection: {state} — {event.get('message', '')}")
            return

        self._observe_packet_sequence(event)
        if kind == "ack":
            ack = str(event.get("ack", "ACK"))
            if ack == "ACK_LIVE_START":
                self.live_active = True
                self.live_var.set("LIVE active")
            elif ack == "ACK_LIVE_STOP":
                self.live_active = False
                self.live_var.set("LIVE stopped")
            self.status_var.set(f"Device response: {ack}")
        elif kind == "imu":
            timestamp_ms = int(event["timestamp_ms"])
            self.imu_t.append(self._relative_time(timestamp_ms))
            self.acc_x.append(float(event["acc_x"]))
            self.acc_y.append(float(event["acc_y"]))
            self.acc_z.append(float(event["acc_z"]))
            self.gyro_x.append(float(event["gyro_x"]))
            self.gyro_y.append(float(event["gyro_y"]))
            self.gyro_z.append(float(event["gyro_z"]))
            self.imu_count += 1
            self.plot_dirty = True
        elif kind == "afea":
            timestamp_ms = int(event["window_start_ms"])
            self.audio_t.append(self._relative_time(timestamp_ms))
            self.audio_laeq.append(float(event["estimated_laeq_dba"]))
            self.audio_rms_a.append(float(event["rms_a_dbfs"]))
            self.audio_peak.append(float(event["peak_dbfs"]))
            self.afea_count += 1
            self.audio_var.set(
                "Audio: seq={seq} | LAeq={laeq:.2f} dBA | class={label} | samples={samples}".format(
                    seq=event["window_sequence"],
                    laeq=event["estimated_laeq_dba"],
                    label=event["environment_label"],
                    samples=event["sample_count"],
                )
            )
            self.plot_dirty = True
        elif kind == "lfea":
            self.light_channels = [float(event[f"f{index}"]) for index in range(1, 9)] + [
                float(event["clear"]),
                float(event["nir"]),
            ]
            self.lfea_count += 1
            self.light_var.set(
                "Light: seq={seq} | Clear={clear} | NIR={nir} | class={label}".format(
                    seq=event["window_sequence"],
                    clear=event["clear"],
                    nir=event["nir"],
                    label=event["exposure_label"],
                )
            )
            self.plot_dirty = True
        elif kind == "error":
            self.status_var.set(f"Error: {event.get('message', 'unknown error')}")

        self.count_var.set(
            f"IMU {self.imu_count} | AFEA {self.afea_count} | LFEA {self.lfea_count}"
        )

    def _update_plots(self) -> None:
        if not self.plot_dirty:
            return
        t = list(self.imu_t)
        self.line_acc_x.set_data(t, list(self.acc_x))
        self.line_acc_y.set_data(t, list(self.acc_y))
        self.line_acc_z.set_data(t, list(self.acc_z))
        self.line_gyro_x.set_data(t, list(self.gyro_x))
        self.line_gyro_y.set_data(t, list(self.gyro_y))
        self.line_gyro_z.set_data(t, list(self.gyro_z))
        for axis in (self.ax_acc, self.ax_gyro):
            axis.relim()
            axis.autoscale_view(scalex=False, scaley=True)
            if t:
                right = t[-1]
                axis.set_xlim(max(0.0, right - self.window_seconds), max(self.window_seconds, right))

        audio_t = list(self.audio_t)
        self.line_laeq.set_data(audio_t, list(self.audio_laeq))
        self.line_rms_a.set_data(audio_t, list(self.audio_rms_a))
        self.line_peak.set_data(audio_t, list(self.audio_peak))
        self.ax_audio.relim()
        self.ax_audio.autoscale_view()

        for patch, value in zip(self.light_bars, self.light_channels):
            patch.set_height(value)
        light_max = max(self.light_channels, default=1.0)
        self.ax_light.set_ylim(0.0, max(1.0, light_max * 1.15))

        self.canvas.draw_idle()
        self.plot_dirty = False

    def process_events(self) -> None:
        if self.closed:
            return
        processed = 0
        while processed < 5000:
            try:
                event = self.event_queue.get_nowait()
            except queue.Empty:
                break
            self._handle_event(event)
            processed += 1
        self._update_plots()
        self.root.after(self.refresh_ms, self.process_events)

    def on_start(self) -> None:
        self.worker.send_command("start")
        self.status_var.set("LIVE_START sent; waiting for ACK")

    def on_stop(self) -> None:
        self.worker.send_command("stop")
        self.status_var.set("LIVE_STOP sent; waiting for ACK")

    def clear_plots(self) -> None:
        for series in (
            self.imu_t,
            self.acc_x,
            self.acc_y,
            self.acc_z,
            self.gyro_x,
            self.gyro_y,
            self.gyro_z,
            self.audio_t,
            self.audio_laeq,
            self.audio_rms_a,
            self.audio_peak,
        ):
            series.clear()
        self.light_channels = [0.0] * 10
        self.timestamp_origin_ms = None
        self.plot_dirty = True

    def on_close(self) -> None:
        if self.closed:
            return
        self.closed = True
        with contextlib.suppress(Exception):
            self.worker.send_command("stop")
        self.worker.stop()
        self.root.after(50, self.root.destroy)

    def run(self) -> None:
        self.worker.start()
        self.root.after(self.refresh_ms, self.process_events)
        self.root.mainloop()


def run_live_dashboard(args: argparse.Namespace) -> None:
    dashboard = LiveDashboard(
        simulate=args.mode == "simulate",
        device_name=args.name,
        scan_timeout=args.scan_timeout,
        write_response=not args.write_without_response,
        window_seconds=args.live_window_seconds,
        refresh_ms=args.live_refresh_ms,
    )
    dashboard.run()

def run_self_tests() -> None:
    assert crc16_ccitt_false(b"123456789") == 0x29B1
    assert crc32_iso_hdlc(b"123456789") == 0xCBF43926
    sync_start = encode_frame(BLE_MSG_SYNC_START, 1)
    assert sync_start.hex().upper() == "A55A011000000100DB7C"
    assert make_ack_through(2, 1, 0).hex().upper() == "A55A0130080002000100000000000000266B"
    assert make_ack_through(3, 1, 1).hex().upper() == "A55A0130080003000100000001000000D772"
    parser = FrameParser()
    assert parser.feed(sync_start[:3]) == []
    frames = parser.feed(sync_start[3:])
    assert len(frames) == 1 and frames[0].msg_type == BLE_MSG_SYNC_START and frames[0].seq == 1

    live_imu_payload = struct.pack(LIVE_IMU_FORMAT, 9, 0, 0, 1, 230, 1, 2, 3, 4, 5, 6)
    live_event = decode_live_frame(Frame(BLE_MSG_LIVE_IMU, 5, live_imu_payload))
    assert live_event is not None
    assert live_event["packet_sequence"] == 9
    assert live_event["timestamp_ms"] == 1230
    assert live_event["gyro_z"] == 6

    assert SYNC_STATUS_SIZE == struct.calcsize("<12s8I") == 44
    sync_status_payload = struct.pack(
        "<12s8I",
        bytes.fromhex("00112233445566778899AABB"),
        7,
        1,
        215,
        1,
        2500,
        216,
        2500,
        2285,
    )
    sync_status = parse_sync_status(sync_status_payload)
    assert sync_status.device_uid == bytes.fromhex("00112233445566778899AABB")
    assert sync_status.log_generation == 7
    assert sync_status.ack_valid == 1
    assert sync_status.acked_through_page_sequence == 215
    assert sync_status.oldest_available_page_sequence == 1
    assert sync_status.newest_available_page_sequence == 2500
    assert sync_status.first_page_sequence_to_send == 216
    assert sync_status.sync_high_watermark_page_sequence == 2500
    assert sync_status.total_unsynced_pages == 2285
    print("[SELFTEST] PASS")


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mode",
        choices=("sync", "live", "simulate"),
        default="sync",
        help=(
            "sync: existing NAND synchronization client; "
            "live: real BLE live dashboard; simulate: dashboard with generated data"
        ),
    )
    parser.add_argument("--name", default=DEVICE_NAME_DEFAULT, help="BLE device name")
    parser.add_argument("--output", default="smartwearable_ble_pages.bin", help="Output binary file")
    parser.add_argument("--pages-csv", default="smartwearable_ble_pages.csv", help="Output page index CSV")
    parser.add_argument("--max-pages", type=int, default=None, help="Stop after N complete pages")
    parser.add_argument("--scan-timeout", type=float, default=10.0, help="BLE scan timeout in seconds")
    parser.add_argument(
        "--write-without-response",
        action="store_true",
        help="Use Write Without Response instead of Write Request",
    )
    parser.add_argument("--resume", action="store_true", help="Validate and append to existing BIN/CSV")
    parser.add_argument(
        "--fsync-interval-pages",
        type=int,
        default=1,
        help="fsync BIN and CSV every N persisted pages (0 disables periodic fsync)",
    )
    parser.add_argument(
        "--page-timeout", type=float, default=30.0, help="Incomplete-page timeout in seconds (0 disables)"
    )
    parser.add_argument(
        "--idle-timeout-seconds",
        type=float,
        default=60.0,
        help="Stop after N seconds without a valid protocol frame (default: 60; 0 disables)",
    )
    parser.add_argument(
        "--absolute-session-timeout-seconds",
        type=float,
        default=0.0,
        help="Optional absolute session limit in seconds (default: disabled)",
    )
    parser.add_argument("--debug-state", action="store_true", help="Print page token and queue diagnostics")
    parser.add_argument(
        "--live-window-seconds",
        type=float,
        default=10.0,
        help="Visible IMU history in the live dashboard (default: 10 seconds)",
    )
    parser.add_argument(
        "--live-refresh-ms",
        type=int,
        default=100,
        help="Live dashboard refresh interval in milliseconds (default: 100)",
    )
    parser.add_argument("--self-test", action="store_true", help="Run protocol self-tests and exit")
    args = parser.parse_args(argv)
    if args.max_pages is not None and args.max_pages <= 0:
        parser.error("--max-pages must be greater than zero")
    if args.fsync_interval_pages < 0:
        parser.error("--fsync-interval-pages cannot be negative")
    if args.page_timeout < 0:
        parser.error("--page-timeout cannot be negative")
    if args.idle_timeout_seconds < 0:
        parser.error("--idle-timeout-seconds cannot be negative")
    if args.absolute_session_timeout_seconds < 0:
        parser.error("--absolute-session-timeout-seconds cannot be negative")
    if args.live_window_seconds <= 0:
        parser.error("--live-window-seconds must be greater than zero")
    if args.live_refresh_ms < 25:
        parser.error("--live-refresh-ms must be at least 25")
    return args


async def main_async(args: argparse.Namespace) -> None:
    client = BleSyncClient(
        device_name=args.name,
        output_path=Path(args.output),
        pages_csv_path=Path(args.pages_csv),
        max_pages=args.max_pages,
        scan_timeout=args.scan_timeout,
        write_response=not args.write_without_response,
        resume=args.resume,
        fsync_interval_pages=args.fsync_interval_pages,
        page_timeout=args.page_timeout,
        idle_timeout_seconds=args.idle_timeout_seconds,
        absolute_session_timeout_seconds=args.absolute_session_timeout_seconds,
        debug_state=args.debug_state,
    )
    await client.run()


def main() -> None:
    args = parse_args()
    if args.self_test:
        run_self_tests()
        return
    run_self_tests()
    try:
        if args.mode in ("live", "simulate"):
            run_live_dashboard(args)
        else:
            asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user", file=sys.stderr)


if __name__ == "__main__":
    main()

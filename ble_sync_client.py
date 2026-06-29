#!/usr/bin/env python3
"""
Smart Wearable BLE Sync Client V1

Automatic test client for STM32 + RN4871 Transparent UART BLE sync.

Requirements:
    pip install bleak

Typical usage:
    python ble_sync_client.py
    python ble_sync_client.py --name BLE_SW
    python ble_sync_client.py --output synced_pages.bin
    python ble_sync_client.py --max-pages 5

Protocol:
    Frame:
        A5 5A | version:u8 | type:u8 | length:u16LE | sequence:u16LE | payload | crc16:u16LE

    CRC16:
        CRC-16/CCITT-FALSE, poly 0x1021, init 0xFFFF, no reflection, xorout 0

    Page CRC32:
        CRC-32/ISO-HDLC, standard binascii.crc32 result.
"""

from __future__ import annotations

import argparse
import asyncio
import binascii
import csv
import struct
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

from bleak import BleakClient, BleakScanner


DEVICE_NAME_DEFAULT = "BLE_SW"

RN4871_SERVICE_UUID = "49535343-FE7D-4AE5-8FA9-9FAFD205E455"
RN4871_MAIN_CHAR_UUID = "49535343-1E4D-4BD9-BA61-23C647249616"

SOF = b"\xA5\x5A"
PROTOCOL_VERSION = 1

BLE_MSG_SYNC_START = 0x10
BLE_MSG_SYNC_STATUS = 0x11
BLE_MSG_PAGE_BEGIN = 0x20
BLE_MSG_PAGE_DATA = 0x21
BLE_MSG_PAGE_END = 0x22
BLE_MSG_ACK_THROUGH = 0x30
BLE_MSG_NACK_PAGE = 0x31
BLE_MSG_SYNC_COMPLETE = 0x40
BLE_MSG_SYNC_ABORT = 0x41
BLE_MSG_ERROR = 0x7F

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
    BLE_MSG_ERROR: "ERROR",
}


@dataclass
class Frame:
    msg_type: int
    seq: int
    payload: bytes


@dataclass
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


class FrameParser:
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

            if payload_len > 4096:
                # Bad length; discard SOF and resync.
                del self.buffer[:2]
                continue

            total_len = 2 + 1 + 1 + 2 + 2 + payload_len + 2
            if len(self.buffer) < total_len:
                return frames

            frame_bytes = bytes(self.buffer[:total_len])
            del self.buffer[:total_len]

            payload = frame_bytes[8:8 + payload_len]
            received_crc = struct.unpack_from("<H", frame_bytes, 8 + payload_len)[0]
            computed_crc = crc16_ccitt_false(frame_bytes[2:8 + payload_len])

            if received_crc != computed_crc:
                print(
                    f"[WARN] Dropped frame with CRC error: "
                    f"rx=0x{received_crc:04X}, calc=0x{computed_crc:04X}, "
                    f"type=0x{msg_type:02X}, seq={seq}"
                )
                continue

            if version != PROTOCOL_VERSION:
                print(
                    f"[WARN] Dropped frame with unsupported version: "
                    f"{version}, type=0x{msg_type:02X}, seq={seq}"
                )
                continue

            frames.append(Frame(msg_type=msg_type, seq=seq, payload=payload))


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def crc32_iso_hdlc(data: bytes) -> int:
    return binascii.crc32(data) & 0xFFFFFFFF


def encode_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    header_without_sof = struct.pack(
        "<BBHH",
        PROTOCOL_VERSION,
        msg_type,
        len(payload),
        seq & 0xFFFF,
    )
    crc = crc16_ccitt_false(header_without_sof + payload)
    return SOF + header_without_sof + payload + struct.pack("<H", crc)


def parse_sync_status(payload: bytes) -> SyncStatus:
    # uint8 uid[12]
    # uint32 log_generation
    # uint8 ack_valid
    # uint8 reserved[3]
    # uint32 acked_through
    # uint32 oldest
    # uint32 newest
    # uint32 first_to_send
    # uint32 high_watermark
    # uint32 total_unsynced
    if len(payload) < 40:
        raise ValueError(f"SYNC_STATUS payload too short: {len(payload)} bytes")

    device_uid = payload[0:12]
    (
        log_generation,
        ack_valid,
        acked_through,
        oldest,
        newest,
        first_to_send,
        high_watermark,
        total_unsynced,
    ) = struct.unpack_from("<IB3xIIIIII", payload, 12)

    return SyncStatus(
        device_uid=device_uid,
        log_generation=log_generation,
        ack_valid=ack_valid,
        acked_through_page_sequence=acked_through,
        oldest_available_page_sequence=oldest,
        newest_available_page_sequence=newest,
        first_page_sequence_to_send=first_to_send,
        sync_high_watermark_page_sequence=high_watermark,
        total_unsynced_pages=total_unsynced,
    )


def parse_page_begin(payload: bytes) -> PageContext:
    # uint32 log_generation
    # uint32 page_sequence
    # uint32 physical_page_index
    # uint32 logical_page_bytes
    # uint32 page_crc32
    # uint8  magic[4]
    # uint8  page_version
    # uint8  page_header_size
    # uint16 reserved
    # uint32 page_payload_bytes
    if len(payload) < 32:
        raise ValueError(f"PAGE_BEGIN payload too short: {len(payload)} bytes")

    (
        log_generation,
        page_sequence,
        physical_page_index,
        logical_page_bytes,
        page_crc32,
    ) = struct.unpack_from("<IIIII", payload, 0)

    magic = payload[20:24]
    page_version = payload[24]
    page_header_size = payload[25]
    page_payload_bytes = struct.unpack_from("<I", payload, 28)[0]

    return PageContext(
        log_generation=log_generation,
        page_sequence=page_sequence,
        physical_page_index=physical_page_index,
        logical_page_bytes=logical_page_bytes,
        page_crc32=page_crc32,
        magic=magic,
        page_version=page_version,
        page_header_size=page_header_size,
        page_payload_bytes=page_payload_bytes,
        data=bytearray(),
    )


def parse_page_data(payload: bytes) -> tuple[int, int, int, bytes]:
    # uint32 log_generation
    # uint32 page_sequence
    # uint32 offset
    # uint16 chunk_length
    # uint16 reserved
    # uint8 data[chunk_length]
    if len(payload) < 16:
        raise ValueError(f"PAGE_DATA payload too short: {len(payload)} bytes")

    log_generation, page_sequence, offset, chunk_length = struct.unpack_from("<IIIH", payload, 0)
    data = payload[16:16 + chunk_length]

    if len(data) != chunk_length:
        raise ValueError(
            f"PAGE_DATA chunk length mismatch: expected {chunk_length}, got {len(data)}"
        )

    return log_generation, page_sequence, offset, data


def parse_page_end(payload: bytes) -> tuple[int, int, int, int]:
    # uint32 log_generation
    # uint32 page_sequence
    # uint32 logical_page_bytes
    # uint32 page_crc32
    if len(payload) < 16:
        raise ValueError(f"PAGE_END payload too short: {len(payload)} bytes")
    return struct.unpack_from("<IIII", payload, 0)


def make_ack_through(seq: int, log_generation: int, page_sequence: int) -> bytes:
    payload = struct.pack("<II", log_generation, page_sequence)
    return encode_frame(BLE_MSG_ACK_THROUGH, seq, payload)


def make_nack(seq: int, log_generation: int, page_sequence: int, reason: int = 1) -> bytes:
    payload = struct.pack("<IIB3x", log_generation, page_sequence, reason & 0xFF)
    return encode_frame(BLE_MSG_NACK_PAGE, seq, payload)


def format_hex(data: bytes, max_bytes: int = 64) -> str:
    if len(data) <= max_bytes:
        return data.hex(" ").upper()
    return data[:max_bytes].hex(" ").upper() + f" ... ({len(data)} bytes)"


async def find_device(name: str, timeout: float):
    print(f"[SCAN] Searching for BLE device named '{name}'...")
    devices = await BleakScanner.discover(timeout=timeout)
    for d in devices:
        if d.name == name:
            print(f"[SCAN] Found {d.name} at {d.address}")
            return d
    print("[SCAN] Devices found:")
    for d in devices:
        print(f"  - {d.name!r} {d.address}")
    raise RuntimeError(f"Device '{name}' not found")


class BleSyncClient:
    def __init__(
        self,
        device_name: str,
        output_path: Path,
        pages_csv_path: Path,
        max_pages: Optional[int],
        scan_timeout: float,
        write_response: bool,
    ) -> None:
        self.device_name = device_name
        self.output_path = output_path
        self.pages_csv_path = pages_csv_path
        self.max_pages = max_pages
        self.scan_timeout = scan_timeout
        self.write_response = write_response

        self.parser = FrameParser()
        self.client: Optional[BleakClient] = None
        self.tx_seq = 1
        self.status: Optional[SyncStatus] = None
        self.current_page: Optional[PageContext] = None
        self.pages_saved = 0
        self.finished = asyncio.Event()
        self.stop_requested = False
        self.page_records: list[dict[str, object]] = []
        self.output_file = None

    async def write_frame(self, msg_type: int, payload: bytes = b"") -> None:
        if self.client is None:
            raise RuntimeError("Client not connected")

        frame = encode_frame(msg_type, self.tx_seq, payload)
        print(
            f"[TX] seq={self.tx_seq} "
            f"type=0x{msg_type:02X} {MESSAGE_NAMES.get(msg_type, 'UNKNOWN')} "
            f"len={len(payload)}"
        )
        self.tx_seq = (self.tx_seq + 1) & 0xFFFF
        if self.tx_seq == 0:
            self.tx_seq = 1

        await self.client.write_gatt_char(
            RN4871_MAIN_CHAR_UUID,
            frame,
            response=self.write_response,
        )

    async def write_raw(self, frame: bytes) -> None:
        if self.client is None:
            raise RuntimeError("Client not connected")
        await self.client.write_gatt_char(
            RN4871_MAIN_CHAR_UUID,
            frame,
            response=self.write_response,
        )

    async def send_sync_start(self) -> None:
        await self.write_frame(BLE_MSG_SYNC_START, b"")

    async def send_ack(self, log_generation: int, page_sequence: int) -> None:
        payload = struct.pack("<II", log_generation, page_sequence)
        await self.write_frame(BLE_MSG_ACK_THROUGH, payload)

    async def send_nack(self, log_generation: int, page_sequence: int, reason: int) -> None:
        payload = struct.pack("<IIB3x", log_generation, page_sequence, reason & 0xFF)
        await self.write_frame(BLE_MSG_NACK_PAGE, payload)

    def notification_handler(self, _sender, data: bytearray) -> None:
        print(f"[RX-BYTES] {len(data)} bytes: {format_hex(bytes(data), max_bytes=40)}")
        for frame in self.parser.feed(bytes(data)):
            asyncio.create_task(self.handle_frame(frame))

    async def handle_frame(self, frame: Frame) -> None:
        name = MESSAGE_NAMES.get(frame.msg_type, "UNKNOWN")

        if self.stop_requested and frame.msg_type not in (
            BLE_MSG_SYNC_COMPLETE,
            BLE_MSG_SYNC_ABORT,
            BLE_MSG_ERROR,
        ):
            print(
                f"[RX-IGNORED] seq={frame.seq} type=0x{frame.msg_type:02X} {name} "
                "because max-pages was already reached"
            )
            return

        print(
            f"[RX] seq={frame.seq} type=0x{frame.msg_type:02X} {name} "
            f"payload={len(frame.payload)} bytes"
        )

        try:
            if frame.msg_type == BLE_MSG_SYNC_STATUS:
                self.status = parse_sync_status(frame.payload)
                self.print_status(self.status)

            elif frame.msg_type == BLE_MSG_PAGE_BEGIN:
                if self.max_pages is not None and self.pages_saved >= self.max_pages:
                    self.stop_requested = True
                    self.finished.set()
                    print("[STOP] Ignoring PAGE_BEGIN because max-pages was already reached")
                    return

                page = parse_page_begin(frame.payload)
                self.current_page = page
                print(
                    f"[PAGE_BEGIN] seq={page.page_sequence} "
                    f"physical={page.physical_page_index} "
                    f"magic={page.magic!r} "
                    f"logical={page.logical_page_bytes} "
                    f"crc32=0x{page.page_crc32:08X}"
                )

            elif frame.msg_type == BLE_MSG_PAGE_DATA:
                log_generation, page_sequence, offset, chunk = parse_page_data(frame.payload)
                if self.current_page is None:
                    print("[ERROR] PAGE_DATA without PAGE_BEGIN")
                    await self.send_nack(log_generation, page_sequence, reason=2)
                    return

                page = self.current_page
                if log_generation != page.log_generation or page_sequence != page.page_sequence:
                    print("[ERROR] PAGE_DATA does not match current page")
                    await self.send_nack(log_generation, page_sequence, reason=2)
                    return

                end = offset + len(chunk)
                if end > page.logical_page_bytes:
                    print("[ERROR] PAGE_DATA exceeds expected logical page size")
                    await self.send_nack(log_generation, page_sequence, reason=2)
                    return

                if len(page.data) < page.logical_page_bytes:
                    page.data.extend(b"\x00" * (page.logical_page_bytes - len(page.data)))

                page.data[offset:end] = chunk
                print(
                    f"[PAGE_DATA] page={page_sequence} offset={offset} "
                    f"len={len(chunk)} received={self.count_received_bytes(page)}"
                )

            elif frame.msg_type == BLE_MSG_PAGE_END:
                log_generation, page_sequence, logical_bytes, page_crc32 = parse_page_end(frame.payload)
                await self.finish_page(log_generation, page_sequence, logical_bytes, page_crc32)

            elif frame.msg_type == BLE_MSG_SYNC_COMPLETE:
                print(f"[COMPLETE] payload={frame.payload.hex(' ').upper()}")
                self.finished.set()

            elif frame.msg_type == BLE_MSG_SYNC_ABORT:
                print(f"[ABORT] payload={frame.payload.hex(' ').upper()}")
                self.finished.set()

            elif frame.msg_type == BLE_MSG_ERROR:
                print(f"[ERROR FRAME] payload={frame.payload.hex(' ').upper()}")

            else:
                print(f"[WARN] Unhandled message type 0x{frame.msg_type:02X}")

        except Exception as exc:
            print(f"[ERROR] Exception while handling {name}: {exc}")

    @staticmethod
    def count_received_bytes(page: PageContext) -> int:
        # This is a rough diagnostic because zero bytes may be valid data.
        return len(page.data)

    async def finish_page(
        self,
        log_generation: int,
        page_sequence: int,
        logical_bytes: int,
        page_crc32: int,
    ) -> None:
        if self.current_page is None:
            print("[ERROR] PAGE_END without PAGE_BEGIN")
            await self.send_nack(log_generation, page_sequence, reason=2)
            return

        page = self.current_page
        if (
            log_generation != page.log_generation
            or page_sequence != page.page_sequence
            or logical_bytes != page.logical_page_bytes
            or page_crc32 != page.page_crc32
        ):
            print("[ERROR] PAGE_END does not match PAGE_BEGIN")
            await self.send_nack(log_generation, page_sequence, reason=2)
            return

        if len(page.data) != page.logical_page_bytes:
            print(
                f"[ERROR] Incomplete page data: got {len(page.data)}, "
                f"expected {page.logical_page_bytes}"
            )
            await self.send_nack(log_generation, page_sequence, reason=2)
            return

        computed_crc32 = crc32_iso_hdlc(bytes(page.data))
        if computed_crc32 != page.page_crc32:
            print(
                f"[ERROR] Page CRC32 mismatch: "
                f"calc=0x{computed_crc32:08X}, expected=0x{page.page_crc32:08X}"
            )
            await self.send_nack(log_generation, page_sequence, reason=1)
            return

        assert self.output_file is not None
        file_offset = self.output_file.tell()
        self.output_file.write(bytes(page.data))
        self.output_file.flush()

        self.page_records.append(
            {
                "page_sequence": page.page_sequence,
                "physical_page_index": page.physical_page_index,
                "log_generation": page.log_generation,
                "magic": page.magic.decode("ascii", errors="replace"),
                "page_version": page.page_version,
                "page_header_size": page.page_header_size,
                "page_payload_bytes": page.page_payload_bytes,
                "logical_page_bytes": page.logical_page_bytes,
                "page_crc32": f"0x{page.page_crc32:08X}",
                "file_offset": file_offset,
            }
        )

        self.pages_saved += 1
        print(
            f"[PAGE_OK] seq={page.page_sequence} magic={page.magic!r} "
            f"bytes={page.logical_page_bytes} saved_pages={self.pages_saved}"
        )

        await self.send_ack(page.log_generation, page.page_sequence)

        self.current_page = None

        if self.max_pages is not None and self.pages_saved >= self.max_pages:
            self.stop_requested = True
            print(f"[STOP] Reached max pages: {self.max_pages}")
            self.finished.set()

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

    def write_pages_csv(self) -> None:
        if not self.page_records:
            return

        with self.pages_csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(self.page_records[0].keys()))
            writer.writeheader()
            writer.writerows(self.page_records)

        print(f"[SAVE] Page index CSV: {self.pages_csv_path}")

    async def run(self) -> None:
        device = await find_device(self.device_name, self.scan_timeout)

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.pages_csv_path.parent.mkdir(parents=True, exist_ok=True)

        print("[CONNECT] Connecting...")
        async with BleakClient(device) as client:
            self.client = client
            print("[CONNECT] Connected")

            services = client.services
            print("[GATT] Services:")
            for service in services:
                print(f"  Service {service.uuid}")
                for char in service.characteristics:
                    props = ",".join(char.properties)
                    print(f"    Char {char.uuid} [{props}]")

            print("[NOTIFY] Enabling notifications...")
            await client.start_notify(RN4871_MAIN_CHAR_UUID, self.notification_handler)
            await asyncio.sleep(0.5)

            self.output_file = self.output_path.open("wb")
            try:
                print("[SYNC] Sending SYNC_START")
                await self.send_sync_start()

                try:
                    await asyncio.wait_for(self.finished.wait(), timeout=600.0)
                    if self.stop_requested:
                        await asyncio.sleep(0.2)
                except asyncio.TimeoutError:
                    print("[TIMEOUT] Client timeout reached")

            finally:
                print("[NOTIFY] Disabling notifications...")
                try:
                    await client.stop_notify(RN4871_MAIN_CHAR_UUID)
                except Exception:
                    pass

                if self.output_file is not None:
                    self.output_file.close()
                    self.output_file = None

        self.write_pages_csv()
        print(f"[SAVE] Binary pages saved to: {self.output_path}")
        print(f"[DONE] Pages saved: {self.pages_saved}")


def run_self_tests() -> None:
    assert crc16_ccitt_false(b"123456789") == 0x29B1
    assert crc32_iso_hdlc(b"123456789") == 0xCBF43926

    sync_start = encode_frame(BLE_MSG_SYNC_START, 1, b"")
    assert sync_start.hex().upper() == "A55A011000000100DB7C"

    ack0 = make_ack_through(2, 1, 0)
    assert ack0.hex().upper() == "A55A0130080002000100000000000000266B"

    ack1 = make_ack_through(3, 1, 1)
    assert ack1.hex().upper() == "A55A0130080003000100000001000000D772"

    parser = FrameParser()
    frames = parser.feed(sync_start[:3])
    assert frames == []
    frames = parser.feed(sync_start[3:])
    assert len(frames) == 1
    assert frames[0].msg_type == BLE_MSG_SYNC_START
    assert frames[0].seq == 1

    print("[SELFTEST] PASS")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--name", default=DEVICE_NAME_DEFAULT, help="BLE device name")
    parser.add_argument("--output", default="smartwearable_ble_pages.bin", help="Output binary file")
    parser.add_argument("--pages-csv", default="smartwearable_ble_pages.csv", help="Output page index CSV")
    parser.add_argument("--max-pages", type=int, default=None, help="Stop after N pages")
    parser.add_argument("--scan-timeout", type=float, default=10.0, help="BLE scan timeout in seconds")
    parser.add_argument(
        "--write-without-response",
        action="store_true",
        help="Use Write Without Response instead of Write Request",
    )
    parser.add_argument("--self-test", action="store_true", help="Run self tests and exit")
    return parser.parse_args()


async def main_async() -> None:
    args = parse_args()

    if args.self_test:
        run_self_tests()
        return

    run_self_tests()

    client = BleSyncClient(
        device_name=args.name,
        output_path=Path(args.output),
        pages_csv_path=Path(args.pages_csv),
        max_pages=args.max_pages,
        scan_timeout=args.scan_timeout,
        write_response=not args.write_without_response,
    )
    await client.run()


def main() -> None:
    if sys.platform.startswith("win"):
        # Bleak on Windows works with the Proactor event loop by default.
        pass

    asyncio.run(main_async())


if __name__ == "__main__":
    main()

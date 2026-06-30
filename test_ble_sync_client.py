import asyncio
import contextlib
import csv
import io
import subprocess
import struct
import sys
import tempfile
import time
import unittest
from contextlib import redirect_stdout
from pathlib import Path

from ble_sync_client import (
    BLE_MSG_ACK_THROUGH,
    BLE_MSG_ERROR,
    BLE_MSG_NACK_PAGE,
    BLE_MSG_PAGE_BEGIN,
    BLE_MSG_PAGE_DATA,
    BLE_MSG_PAGE_END,
    BleSyncClient,
    FrameParser,
    PageProtocolError,
    PageTimeoutEvent,
    SYNC_STATUS_SIZE,
    SyncStatus,
    crc32_iso_hdlc,
    encode_frame,
    parse_args,
    parse_sync_status,
)


UID = bytes.fromhex("00112233445566778899AABB")
GENERATION = 7


class FakeBleakClient:
    def __init__(self, yield_on_write: bool = True) -> None:
        self.writes: list[bytes] = []
        self.yield_on_write = yield_on_write

    async def write_gatt_char(self, _uuid, data, response=True) -> None:
        del response
        self.writes.append(bytes(data))
        if self.yield_on_write:
            await asyncio.sleep(0)


def page_bytes(sequence: int, magic: bytes = b"SENS", size: int = 40) -> bytes:
    if size < 32:
        raise ValueError("test pages need a 32-byte header")
    body = bytearray(size)
    body[:4] = magic
    struct.pack_into("<I", body, 8, sequence)
    for index in range(12, size):
        body[index] = (sequence + index) & 0xFF
    return bytes(body)


def begin_frame(frame_seq: int, page_seq: int, data: bytes, magic: bytes | None = None) -> bytes:
    magic = data[:4] if magic is None else magic
    payload = struct.pack(
        "<IIIII4sBBHI",
        GENERATION,
        page_seq,
        page_seq % 2048,
        len(data),
        crc32_iso_hdlc(data),
        magic,
        1,
        32,
        0,
        len(data) - 32,
    )
    return encode_frame(BLE_MSG_PAGE_BEGIN, frame_seq, payload)


def protocol_page_frame(frame_seq: int, page_seq: int, data: bytes, header_size: int) -> bytes:
    payload = struct.pack(
        "<IIIII4sBBHI",
        GENERATION,
        page_seq,
        page_seq % 2048,
        len(data),
        crc32_iso_hdlc(data),
        data[:4],
        data[4],
        header_size,
        0,
        len(data) - header_size,
    )
    return encode_frame(BLE_MSG_PAGE_BEGIN, frame_seq, payload)


def data_frame(frame_seq: int, page_seq: int, offset: int, chunk: bytes) -> bytes:
    payload = struct.pack("<IIIHH", GENERATION, page_seq, offset, len(chunk), 0) + chunk
    return encode_frame(BLE_MSG_PAGE_DATA, frame_seq, payload)


def end_frame(frame_seq: int, page_seq: int, data: bytes) -> bytes:
    payload = struct.pack("<IIII", GENERATION, page_seq, len(data), crc32_iso_hdlc(data))
    return encode_frame(BLE_MSG_PAGE_END, frame_seq, payload)


def complete_page_frames(first_frame_seq: int, page_seq: int, data: bytes) -> list[bytes]:
    split = min(128, len(data))
    frames = [
        begin_frame(first_frame_seq, page_seq, data),
        data_frame((first_frame_seq + 1) & 0xFFFF, page_seq, 0, data[:split]),
    ]
    next_seq = first_frame_seq + 2
    if split < len(data):
        frames.append(data_frame(next_seq & 0xFFFF, page_seq, split, data[split:]))
        next_seq += 1
    frames.append(end_frame(next_seq & 0xFFFF, page_seq, data))
    return frames


def written_types(fake: FakeBleakClient) -> list[int]:
    parser = FrameParser()
    return [frame.msg_type for raw in fake.writes for frame in parser.feed(raw)]


class BleSyncClientTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        root = Path(self.temp_dir.name)
        self.bin_path = root / "smartwearable_ble_pages.bin"
        self.csv_path = root / "smartwearable_ble_pages.csv"
        self.clients: list[BleSyncClient] = []

    async def asyncTearDown(self) -> None:
        for client in self.clients:
            await client.shutdown_rx_pipeline()
            client.flush_and_close_files()
        self.temp_dir.cleanup()

    def test_00_sync_status_real_44_byte_layout(self) -> None:
        self.assertEqual(SYNC_STATUS_SIZE, struct.calcsize("<12s8I"))
        self.assertEqual(SYNC_STATUS_SIZE, 44)
        payload = struct.pack(
            "<12s8I",
            UID,
            GENERATION,
            1,
            215,
            1,
            2500,
            216,
            2500,
            2285,
        )
        status = parse_sync_status(payload)
        self.assertEqual(
            status,
            SyncStatus(UID, GENERATION, 1, 215, 1, 2500, 216, 2500, 2285),
        )

    def make_client(self, **kwargs) -> tuple[BleSyncClient, FakeBleakClient]:
        client = BleSyncClient(
            output_path=self.bin_path,
            pages_csv_path=self.csv_path,
            page_timeout=0,
            fsync_interval_pages=0,
            **kwargs,
        )
        fake = FakeBleakClient()
        client.client = fake  # type: ignore[assignment]
        client.open_output_files()
        status = parse_sync_status(
            struct.pack("<12s8I", UID, GENERATION, 0, 0, 1, 10000, 1, 10000, 10000)
        )
        client.accept_status(status)
        client.status = status
        client.start_rx_pipeline()
        self.clients.append(client)
        return client, fake

    async def feed(self, client: BleSyncClient, *notifications: bytes) -> None:
        for notification in notifications:
            client.notification_handler(None, bytearray(notification))
        await client.wait_rx_idle()

    async def test_01_ordered_frames_save_and_ack(self) -> None:
        client, fake = self.make_client()
        data = page_bytes(1, size=200)
        frames = complete_page_frames(10, 1, data)
        await self.feed(client, *frames)
        self.assertEqual(self.bin_path.read_bytes(), data)
        self.assertEqual(written_types(fake), [BLE_MSG_ACK_THROUGH])
        self.assertIsNone(client.current_page)

    async def test_02_multiple_frames_in_one_notification(self) -> None:
        client, fake = self.make_client()
        data = page_bytes(2)
        frames = complete_page_frames(20, 2, data)
        await self.feed(client, frames[0] + frames[1], *frames[2:])
        self.assertEqual(self.bin_path.read_bytes(), data)
        self.assertEqual(written_types(fake), [BLE_MSG_ACK_THROUGH])

    async def test_03_fragmented_page_data_frame(self) -> None:
        client, fake = self.make_client()
        data = page_bytes(3)
        begin, page_data, end = complete_page_frames(30, 3, data)
        split = len(page_data) // 2
        await self.feed(client, begin, page_data[:split], page_data[split:], end)
        self.assertEqual(self.bin_path.read_bytes(), data)
        self.assertEqual(written_types(fake), [BLE_MSG_ACK_THROUGH])

    async def test_04_stale_timeout_cannot_clear_new_page(self) -> None:
        client, _fake = self.make_client()
        old_data = page_bytes(215, magic=b"LFEA")
        await self.feed(client, *complete_page_frames(100, 215, old_data))
        old_token = client.next_page_token - 1

        new_data = page_bytes(216)
        begin, page_data, _end = complete_page_frames(103, 216, new_data)
        await self.feed(client, begin)
        new_token = client.current_page.generation_token  # type: ignore[union-attr]
        await client.handle_page_timeout(PageTimeoutEvent(old_token, 215))
        await self.feed(client, page_data)
        self.assertEqual(client.current_page.generation_token, new_token)  # type: ignore[union-attr]
        self.assertEqual(client.current_page.received_bytes, len(new_data))  # type: ignore[union-attr]

    async def test_05_rapid_callbacks_use_one_ordered_consumer(self) -> None:
        client, fake = self.make_client()
        frame_seq = 500
        expected = bytearray()
        for page_seq in range(1, 51):
            data = page_bytes(page_seq)
            expected.extend(data)
            for frame in complete_page_frames(frame_seq, page_seq, data):
                client.notification_handler(None, bytearray(frame))
                frame_seq += 1
        await client.wait_rx_idle()
        self.assertEqual(client.consumer_count, 1)
        self.assertEqual(self.bin_path.read_bytes(), bytes(expected))
        self.assertEqual(written_types(fake).count(BLE_MSG_ACK_THROUGH), 50)

    async def test_06_page_data_without_begin_has_no_nack_cascade(self) -> None:
        client, fake = self.make_client()
        bad1 = data_frame(1, 99, 0, b"1234")
        bad2 = data_frame(2, 99, 4, b"5678")
        client.notification_handler(None, bytearray(bad1))
        client.notification_handler(None, bytearray(bad2))
        with self.assertRaises(PageProtocolError):
            await client.wait_rx_idle()
        # Firmware accepts NACK only after PAGE_END (WAIT_ACK), not mid-stream.
        self.assertEqual(written_types(fake).count(BLE_MSG_NACK_PAGE), 0)
        self.assertTrue(client.stop_requested)

    async def test_07_lost_ack_resume_deduplicates_and_reacks(self) -> None:
        first, _first_fake = self.make_client()
        data = page_bytes(7, magic=b"AFEA")
        await self.feed(first, *complete_page_frames(70, 7, data))
        await first.shutdown_rx_pipeline()
        first.flush_and_close_files()
        original_size = self.bin_path.stat().st_size

        resumed = BleSyncClient(
            output_path=self.bin_path,
            pages_csv_path=self.csv_path,
            resume=True,
            page_timeout=0,
            fsync_interval_pages=0,
        )
        fake = FakeBleakClient()
        resumed.client = fake  # type: ignore[assignment]
        resumed.open_output_files()
        status = parse_sync_status(
            struct.pack("<12s8I", UID, GENERATION, 1, 6, 1, 100, 7, 100, 94)
        )
        resumed.accept_status(status)
        resumed.status = status
        resumed.start_rx_pipeline()
        self.clients.append(resumed)
        await self.feed(resumed, *complete_page_frames(80, 7, data))
        self.assertEqual(self.bin_path.stat().st_size, original_size)
        with self.csv_path.open(newline="", encoding="utf-8") as csv_file:
            self.assertEqual(len(list(csv.DictReader(csv_file))), 1)
        self.assertEqual(written_types(fake), [BLE_MSG_ACK_THROUGH])

    async def test_08_stress_2500_pages(self) -> None:
        client, fake = self.make_client()
        frame_seq = 0xFFF0
        expected_offset = 0
        magic_cycle = (b"SENS", b"AFEA", b"LFEA")
        with redirect_stdout(io.StringIO()):
            for page_seq in range(1, 2501):
                data = page_bytes(page_seq, magic_cycle[(page_seq - 1) % 3])
                for frame in complete_page_frames(frame_seq, page_seq, data):
                    client.notification_handler(None, bytearray(frame))
                    frame_seq = (frame_seq + 1) & 0xFFFF
                if page_seq % 100 == 0:
                    await client.wait_rx_idle()
            await client.wait_rx_idle()

        self.assertIsNone(client.current_page)
        self.assertEqual(len(client.parser.buffer), 0)
        self.assertEqual(len(client.saved_pages), 2500)
        self.assertEqual(written_types(fake).count(BLE_MSG_ACK_THROUGH), 2500)
        with self.csv_path.open(newline="", encoding="utf-8") as csv_file:
            rows = list(csv.DictReader(csv_file))
        self.assertEqual(len(rows), 2500)
        for row in rows:
            self.assertEqual(int(row["file_offset"]), expected_offset)
            expected_offset += int(row["logical_page_bytes"])
        self.assertEqual(self.bin_path.stat().st_size, expected_offset)
        self.assertEqual(client.absolute_session_timeout_seconds, 0.0)

    async def test_09_shutdown_during_page_data_keeps_only_completed_page(self) -> None:
        client, fake = self.make_client()
        complete = page_bytes(1)
        await self.feed(client, *complete_page_frames(1, 1, complete))
        incomplete = page_bytes(2, size=80)
        await self.feed(
            client,
            begin_frame(4, 2, incomplete),
            data_frame(5, 2, 0, incomplete[:20]),
        )
        await client.shutdown_rx_pipeline()
        client.flush_and_close_files()
        self.assertEqual(self.bin_path.read_bytes(), complete)
        self.assertEqual(written_types(fake).count(BLE_MSG_ACK_THROUGH), 1)

    async def test_10_max_pages_stops_after_ack_and_ignores_following_frame(self) -> None:
        client, fake = self.make_client(max_pages=1)
        first = page_bytes(1)
        second = page_bytes(2)
        notification = b"".join(complete_page_frames(1, 1, first)) + begin_frame(4, 2, second)
        client.notification_handler(None, bytearray(notification))
        await client.wait_rx_idle()
        self.assertEqual(self.bin_path.read_bytes(), first)
        self.assertEqual(written_types(fake), [BLE_MSG_ACK_THROUGH])
        self.assertIsNone(client.current_page)
        self.assertTrue(client.stop_requested)

    async def test_11_all_together_accepts_generated_bin_and_csv(self) -> None:
        client, _fake = self.make_client()
        # A valid, empty V1 SENS page using ALL_TOGETHER.py's <IBBHII header.
        data = struct.pack("<IBBHII", int.from_bytes(b"SENS", "little"), 1, 16, 0, 1, 0)
        frames = (
            protocol_page_frame(1, 1, data, 16),
            data_frame(2, 1, 0, data),
            end_frame(3, 1, data),
        )
        await self.feed(client, *frames)
        client.flush_and_close_files()
        results = Path(self.temp_dir.name) / "BLE_RESULTS"
        completed = await asyncio.to_thread(
            subprocess.run,
            [
                sys.executable,
                str(Path(__file__).with_name("ALL_TOGETHER.py")),
                "--ble-pages-bin",
                str(self.bin_path),
                "--ble-pages-csv",
                str(self.csv_path),
                "--output-dir",
                str(results),
                "--audio-sample-rate",
                "48000",
            ],
            capture_output=True,
            text=True,
            timeout=120,
            check=False,
        )
        self.assertEqual(completed.returncode, 0, completed.stdout + completed.stderr)
        self.assertNotIn("Processing error:", completed.stdout)
        self.assertTrue(results.exists())

    async def test_12_valid_frames_keep_session_alive_beyond_600_seconds(self) -> None:
        client, _fake = self.make_client(idle_timeout_seconds=60.0)
        client.start_session_watchdog()
        now = time.monotonic()
        client.session_started_monotonic = now - 650.0
        client.last_valid_activity = now - 50.0
        for sequence in range(1, 6):
            await self.feed(client, encode_frame(BLE_MSG_ERROR, sequence, b"valid"))
        self.assertGreater(
            time.monotonic() - client.session_started_monotonic,
            600.0,
        )
        self.assertIsNone(client.session_timeout_error())
        self.assertIsNone(client.failure)

    async def test_13_idle_watchdog_sets_explicit_timeout(self) -> None:
        client, _fake = self.make_client(idle_timeout_seconds=0.03)
        client.start_session_watchdog()
        await asyncio.sleep(0.08)
        self.assertIsInstance(client.failure, TimeoutError)
        self.assertEqual(str(client.failure), "BLE session inactive for 0.03 seconds")
        self.assertTrue(client.stop_requested)

    async def test_14_invalid_bytes_and_bad_crc_do_not_renew_watchdog(self) -> None:
        client, _fake = self.make_client(idle_timeout_seconds=0.04)
        client.start_session_watchdog()
        initial_activity = client.last_valid_activity
        bad_crc = bytearray(encode_frame(BLE_MSG_ERROR, 1, b"bad-crc"))
        bad_crc[-1] ^= 0xFF
        client.notification_handler(None, bytearray())
        client.notification_handler(None, bytearray(b"random bytes"))
        client.notification_handler(None, bad_crc)
        client.notification_handler(None, bytearray(encode_frame(BLE_MSG_ERROR, 2)[:5]))
        await client.wait_rx_idle()
        self.assertEqual(client.last_valid_activity, initial_activity)
        await asyncio.sleep(0.08)
        self.assertIsInstance(client.failure, TimeoutError)

    async def test_15_disconnect_before_timeout_is_primary_connection_error(self) -> None:
        client, _fake = self.make_client(idle_timeout_seconds=1.0)
        client.start_session_watchdog()
        client.disconnected_handler(None)
        self.assertIsInstance(client.failure, ConnectionError)
        self.assertNotIsInstance(client.failure, TimeoutError)

    async def test_16_disconnect_after_timeout_preserves_timeout_error(self) -> None:
        client, _fake = self.make_client(idle_timeout_seconds=0.02)
        client.start_session_watchdog()
        await asyncio.sleep(0.06)
        primary = client.failure
        self.assertIsInstance(primary, TimeoutError)
        client.disconnected_handler(None)
        self.assertIs(client.failure, primary)

    async def test_17_zero_idle_timeout_disables_watchdog(self) -> None:
        client, _fake = self.make_client(idle_timeout_seconds=0.0)
        client.start_session_watchdog()
        self.assertIsNone(client.session_watchdog_task)
        await asyncio.sleep(0.05)
        self.assertIsNone(client.failure)
        self.assertIsNone(client.session_timeout_error())

    async def test_18_absolute_timeout_only_when_explicitly_configured(self) -> None:
        client, _fake = self.make_client(
            idle_timeout_seconds=0.0,
            absolute_session_timeout_seconds=0.03,
        )
        client.start_session_watchdog()
        await asyncio.sleep(0.08)
        self.assertIsInstance(client.failure, TimeoutError)
        self.assertIn("explicitly configured absolute timeout", str(client.failure))

    def test_19_watchdog_cli_defaults_and_disable_switches(self) -> None:
        defaults = parse_args([])
        self.assertEqual(defaults.idle_timeout_seconds, 60.0)
        self.assertEqual(defaults.absolute_session_timeout_seconds, 0.0)
        disabled = parse_args(["--idle-timeout-seconds", "0"])
        self.assertEqual(disabled.idle_timeout_seconds, 0.0)
        explicit = parse_args(["--absolute-session-timeout-seconds", "123"])
        self.assertEqual(explicit.absolute_session_timeout_seconds, 123.0)


if __name__ == "__main__":
    with contextlib.suppress(KeyboardInterrupt):
        unittest.main(verbosity=2)

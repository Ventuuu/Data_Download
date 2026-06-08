"""
Smart Eyewear NAND Logger - USB receiver, parser and visualizer.

Compatible with firmware stream:
    "LOGSTART" + uint32 total_pages + total_pages*4096 bytes + "LOGEND!!"

NAND page:
    16-byte LogPageHeader + payload

Page magic:
    b"SENS" -> IMU + AS7341 records, 40 bytes each
    b"AUD0" -> PCM int16 audio
"""

import os
import struct
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
HEADER_SIZE = 16
SENSOR_RECORD_SIZE = 40

START_MARKER = b"LOGSTART"
END_MARKER = b"LOGEND!!"

MAGIC_SENSOR = b"SENS"
MAGIC_AUDIO = b"AUD0"

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
    unknown_pages: int = 0
    sensor_records: int = 0
    audio_bytes: int = 0


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

    light = []
    for k in range(8):
        start = 17 + 2 * k
        light.append(u16_le(record[start:start + 2]))

    clear = u16_le(record[33:35])
    nir = u16_le(record[35:37])
    flicker_hz = u16_le(record[37:39])

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
        "F1": light[0], "F2": light[1], "F3": light[2], "F4": light[3],
        "F5": light[4], "F6": light[5], "F7": light[6], "F8": light[7],
        "Clear": clear,
        "NIR": nir,
        "Flicker_Hz": flicker_hz,
    }


def parse_nand_dump(bin_filename: Path, csv_filename: Path, wav_filename: Path,
                    summary_filename: Path, audio_sample_rate_hz: int):
    sensor_rows = []
    audio_bytes = bytearray()
    stats = ParseStats()

    with open(bin_filename, "rb") as f:
        physical_page_index = 0
        while True:
            page = f.read(PAGE_SIZE)
            if not page:
                break
            if len(page) != PAGE_SIZE:
                print(f"Warning: incomplete final page at index {physical_page_index}; length={len(page)}")
                break

            stats.total_pages += 1
            header = page[:HEADER_SIZE]

            try:
                magic_word, version, header_size, payload_bytes, page_sequence, page_timestamp_ms = struct.unpack(
                    "<IBBHII", header
                )
            except struct.error:
                stats.unknown_pages += 1
                physical_page_index += 1
                continue

            magic = struct.pack("<I", magic_word)
            payload = page[HEADER_SIZE:HEADER_SIZE + payload_bytes]

            if magic == MAGIC_SENSOR:
                stats.sensor_pages += 1
                n_records = payload_bytes // SENSOR_RECORD_SIZE
                if payload_bytes % SENSOR_RECORD_SIZE != 0:
                    print(f"Warning: SENS page_sequence={page_sequence} has payload_bytes={payload_bytes}")
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

            else:
                stats.unknown_pages += 1
                print(
                    f"Unknown page at physical_page={physical_page_index}: "
                    f"magic={magic!r}, sequence={page_sequence}, payload={payload_bytes}"
                )

            physical_page_index += 1

    stats.sensor_records = len(sensor_rows)
    stats.audio_bytes = len(audio_bytes)

    df = pd.DataFrame(sensor_rows)
    df.to_csv(csv_filename, index=False)

    write_wav_int16_mono(wav_filename, bytes(audio_bytes), audio_sample_rate_hz)

    summary = (
        f"Total pages: {stats.total_pages}\n"
        f"Sensor pages: {stats.sensor_pages}\n"
        f"Audio pages: {stats.audio_pages}\n"
        f"Unknown pages: {stats.unknown_pages}\n"
        f"Sensor records: {stats.sensor_records}\n"
        f"Audio bytes: {stats.audio_bytes}\n"
        f"Audio samples: {stats.audio_bytes // 2}\n"
        f"Audio sample rate assumed: {audio_sample_rate_hz} Hz\n"
        f"CSV file: {csv_filename.name}\n"
        f"WAV file: {wav_filename.name}\n"
    )
    summary_filename.write_text(summary, encoding="utf-8")

    print(summary)
    print(f"Sensor CSV saved to: {csv_filename}")
    print(f"Audio WAV saved to: {wav_filename}")
    print(f"Summary saved to: {summary_filename}")

    return df, bytes(audio_bytes), stats


def plot_sensor_data(df: pd.DataFrame, output_prefix: Path) -> None:
    if df.empty:
        print("No sensor data available; skipping sensor plots.")
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
    plt.show()

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
    plt.show()

    plt.figure(figsize=(12, 6))
    for channel in ["F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "Clear", "NIR"]:
        if channel in df:
            plt.plot(t, df[channel], label=channel)
    plt.xlabel(x_label)
    plt.ylabel("Raw counts")
    plt.title("AS7341 light channels")
    plt.grid(True)
    plt.legend(ncol=2)
    plt.tight_layout()
    light_png = output_prefix.with_name(output_prefix.name + "_light_channels.png")
    plt.savefig(light_png, dpi=200)
    plt.show()

    if "Flicker_Hz" in df.columns:
        plt.figure(figsize=(12, 3))
        plt.step(t, df["Flicker_Hz"], where="post")
        plt.xlabel(x_label)
        plt.ylabel("Flicker [Hz]")
        plt.title("Mains flicker classification")
        plt.grid(True)
        plt.tight_layout()
        flicker_png = output_prefix.with_name(output_prefix.name + "_flicker.png")
        plt.savefig(flicker_png, dpi=200)
        plt.show()


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
    plt.show()


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


def main():
    com_port, save_folder, baud_rate, audio_sample_rate = gui_select_com_and_folder()

    if not com_port or save_folder is None:
        print("Application stopped.")
        return

    timestamp = datetime.now().strftime("SmartEyewear_%Y%m%d_%H%M%S")
    output_prefix = save_folder / timestamp

    bin_filename = output_prefix.with_name(output_prefix.name + "_nand_dump.bin")
    csv_filename = output_prefix.with_name(output_prefix.name + "_imu_light.csv")
    wav_filename = output_prefix.with_name(output_prefix.name + "_audio.wav")
    summary_filename = output_prefix.with_name(output_prefix.name + "_summary.txt")

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
        df, audio_bytes, _ = parse_nand_dump(
            bin_filename=bin_filename,
            csv_filename=csv_filename,
            wav_filename=wav_filename,
            summary_filename=summary_filename,
            audio_sample_rate_hz=audio_sample_rate,
        )

        plot_sensor_data(df, output_prefix)
        plot_audio_waveform(audio_bytes, audio_sample_rate, output_prefix)

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
    main()

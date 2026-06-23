"""
Smart Eyewear NAND Logger - USB receiver, parser and visualizer.

Compatible with firmware stream:
    "LOGSTART" + uint32 total_pages + total_pages*4096 bytes + "LOGEND!!"

NAND page:
    16-byte LogPageHeader + payload

Page magic:
    b"SENS" -> IMU records, 40 bytes each; bytes 17..38 reserved
    b"AUD0" -> PCM int16 audio
    b"LITE" -> one AS7341 session result, 40-byte payload
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
MAGIC_LIGHT = b"LITE"

LIGHT_RESULT_PAYLOAD_SIZE = 40
LIGHT_RESULT_STRUCT_FORMAT = "<10H4IB3s"
NORMALIZATION_SCALE = 10000.0

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
    light_pages: int = 0
    unknown_pages: int = 0
    sensor_records: int = 0
    light_records: int = 0
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


def parse_nand_dump(bin_filename: Path, imu_csv_filename: Path, light_csv_filename: Path, wav_filename: Path,
                    summary_filename: Path, audio_sample_rate_hz: int):
    sensor_rows = []
    light_rows = []
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

            elif magic == MAGIC_LIGHT:
                stats.light_pages += 1
                if payload_bytes < LIGHT_RESULT_PAYLOAD_SIZE:
                    print(
                        f"Warning: LITE page_sequence={page_sequence} has payload_bytes={payload_bytes}; "
                        f"expected at least {LIGHT_RESULT_PAYLOAD_SIZE}"
                    )
                    physical_page_index += 1
                    continue
                if payload_bytes != LIGHT_RESULT_PAYLOAD_SIZE:
                    print(
                        f"Warning: LITE page_sequence={page_sequence} has payload_bytes={payload_bytes}; "
                        f"using first {LIGHT_RESULT_PAYLOAD_SIZE} bytes"
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
                    f"magic={magic!r}, sequence={page_sequence}, payload={payload_bytes}"
                )

            physical_page_index += 1

    stats.sensor_records = len(sensor_rows)
    stats.light_records = len(light_rows)
    stats.audio_bytes = len(audio_bytes)

    imu_df = pd.DataFrame(sensor_rows)
    light_df = pd.DataFrame(light_rows, columns=LIGHT_RESULT_COLUMNS)
    imu_df.to_csv(imu_csv_filename, index=False)
    light_df.to_csv(light_csv_filename, index=False)

    write_wav_int16_mono(wav_filename, bytes(audio_bytes), audio_sample_rate_hz)

    summary = (
        f"Total pages: {stats.total_pages}\n"
        f"Sensor pages: {stats.sensor_pages}\n"
        f"Audio pages: {stats.audio_pages}\n"
        f"Light pages: {stats.light_pages}\n"
        f"Unknown pages: {stats.unknown_pages}\n"
        f"Sensor records: {stats.sensor_records}\n"
        f"Light records: {stats.light_records}\n"
        f"Audio bytes: {stats.audio_bytes}\n"
        f"Audio samples: {stats.audio_bytes // 2}\n"
        f"Audio sample rate: {audio_sample_rate_hz} Hz\n"
        f"IMU CSV file: {imu_csv_filename.name}\n"
        f"Light CSV file: {light_csv_filename.name}\n"
        f"WAV file: {wav_filename.name}\n"
    )
    if not light_df.empty:
        last_light = light_df.iloc[-1]
        summary += (
            f"Last light class: {last_light['light_level_label']}\n"
            f"Last Clear mean counts: {last_light['clear_mean_counts']}\n"
            f"Last sample count: {last_light['sample_count']}\n"
            f"Last acquisition duration: {last_light['acquisition_duration_ms']} ms\n"
        )
    summary_filename.write_text(summary, encoding="utf-8")

    print(summary)
    print(f"IMU CSV saved to: {imu_csv_filename}")
    print(f"Light CSV saved to: {light_csv_filename}")
    print(f"Audio WAV saved to: {wav_filename}")
    print(f"Summary saved to: {summary_filename}")

    return imu_df, light_df, bytes(audio_bytes), stats


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


def plot_light_sensor_channels(light_df: pd.DataFrame, output_prefix: Path) -> None:
    if light_df.empty:
        print("No LITE pages found; skipping light sensor channel plot.")
        return

    channel_plots = [
        ("F1", "normalized_f1_raw", "C3"),
        ("F2", "normalized_f2_raw", "C4"),
        ("F3", "normalized_f3_raw", "C5"),
        ("F4", "normalized_f4_raw", "C6"),
        ("F5", "normalized_f5_raw", "C7"),
        ("F6", "normalized_f6_raw", "C8"),
        ("F7", "normalized_f7_raw", "C9"),
        ("F8", "normalized_f8_raw", "C1"),
        ("NIR", "normalized_nir_raw", "C2"),
        ("Clear", "clear_mean_counts", "C0"),
    ]

    x = np.arange(len(light_df), dtype=int)
    fig, axes = plt.subplots(len(channel_plots) + 1, 1, figsize=(15, 22), sharex=True)
    fig.suptitle("Light sensor channels", fontsize=14)

    for axis, (label, column, color) in zip(axes[:-1], channel_plots):
        axis.plot(x, light_df[column].to_numpy(), color=color, marker="o", linewidth=1.4)
        axis.set_title(f"Light filter {label}" if label != "Clear" else "Clear channel")
        axis.set_ylabel("counts")
        axis.grid(True)

    level_axis = axes[-1]
    level_axis.step(
        x,
        light_df["light_level_class"].to_numpy(),
        where="post",
        color="C10",
        linewidth=1.4,
    )
    level_axis.set_yticks(sorted(LIGHT_LEVEL_LABELS.keys()))
    level_axis.set_yticklabels([LIGHT_LEVEL_LABELS[key] for key in sorted(LIGHT_LEVEL_LABELS.keys())])
    level_axis.set_title("Light level class")
    level_axis.set_xlabel("Light record index")
    level_axis.grid(True)

    plt.tight_layout()
    light_channels_png = output_prefix.with_name(output_prefix.name + "_light_sensor_channels.png")
    plt.savefig(light_channels_png, dpi=200)


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


def main():
    com_port, save_folder, baud_rate, audio_sample_rate = gui_select_com_and_folder()

    if not com_port or save_folder is None:
        print("Application stopped.")
        return

    timestamp = datetime.now().strftime("SmartEyewear_%Y%m%d_%H%M%S")
    output_prefix = save_folder / timestamp

    bin_filename = output_prefix.with_name(output_prefix.name + "_nand_dump.bin")
    imu_csv_filename = output_prefix.with_name(output_prefix.name + "_imu.csv")
    light_csv_filename = output_prefix.with_name(output_prefix.name + "_light_results.csv")
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
        imu_df, light_df, audio_bytes, _ = parse_nand_dump(
            bin_filename=bin_filename,
            imu_csv_filename=imu_csv_filename,
            light_csv_filename=light_csv_filename,
            wav_filename=wav_filename,
            summary_filename=summary_filename,
            audio_sample_rate_hz=audio_sample_rate,
        )

        plot_imu_data(imu_df, output_prefix)
        plot_light_results(light_df, output_prefix)
        plot_light_sensor_channels(light_df, output_prefix)
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
    main()

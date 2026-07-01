import numpy as np
#import matplotlib.pyplot as plt
import serial
import os
import pandas as pd
from datetime import datetime
from tkinter import Tk, filedialog, ttk, messagebox, StringVar, Label, Button
from serial.tools import list_ports

BYTES_PER_SAMPLE = 40  # must match firmware's BYTES_PER_SAMPLE / STRIDE_BYTES_PER_SAMPLE

# ATTENTION: ???????????
# This script now reads the 7 colours (F1..F8 + clear + NIR + mains) and plots them in separate subplots, along with the IMU data. 
# The mains frequency is categorized and plotted as a categorical track.
# Don't forget to re-upload the Imu-logger to the mcu before running this script. (latest change 18/05/2026)
# Because now the script reads 40 bytes per sample instead of 32, 
# the script will read junk data if the firmware is not updated first.

def convert_16bit_signed(lo, hi):
    combined = (hi.astype(np.uint16) << 8) | lo.astype(np.uint16)
    return combined.view(np.int16)


def conv_imu(arr):
    x = convert_16bit_signed(arr[:, 0], arr[:, 1])
    y = convert_16bit_signed(arr[:, 2], arr[:, 3])
    z = convert_16bit_signed(arr[:, 4], arr[:, 5])
    return x, y, z


def conv_gyro(arr):
    x = convert_16bit_signed(arr[:, 0], arr[:, 1])
    y = convert_16bit_signed(arr[:, 2], arr[:, 3])
    z = convert_16bit_signed(arr[:, 4], arr[:, 5])
    return x, y, z


def receive_and_save_data(ser, bin_filename, packet_size=4096, max_packets=2048 * 64):
    """Riceve pacchetti via seriale e li salva in binario."""
    with open(bin_filename, 'wb') as f:
        for packet_count in range(max_packets):
            data = ser.read(packet_size)
            if not data:
                continue
            if data == b'T':  # terminatore
                print("Received Terminator Character.")
                break
            f.write(data)
            print(f"📥 Received packet {packet_count + 1}")
    ser.close()
    print("📴 Serial COM Port Closed.")


def process_bin_file(bin_filename, csv_filename=None):
    # lists for timestamp and IMU
    hh_list, mm_list, ss_list, sss_list = [], [], [], []
    acc_x_list, acc_y_list, acc_z_list = [], [], []
    gyro_x_list, gyro_y_list, gyro_z_list = [], [], []

    # lists for light data
    f1_list, f2_list, f3_list, f4_list = [], [], [], []
    f5_list, f6_list, f7_list, f8_list = [], [], [], []
    clear_list, nir_list = [], []
    mains_hz_list = []
    mains_category_list = []
    step_count_list =[]

    with open(bin_filename, "rb") as f:
        while True:
            pagina = f.read(4096)
            if len(pagina) < 4096:
                break

            valid_bytes = pagina[:4080]

            for i in range(0, len(valid_bytes), BYTES_PER_SAMPLE):
                subpkt = valid_bytes[i:i + BYTES_PER_SAMPLE]
                if len(subpkt) < BYTES_PER_SAMPLE:
                    continue

                # timestamp
                hh = subpkt[0]
                mm = subpkt[1]
                ss = subpkt[2]
                sss = subpkt[3] | (subpkt[4] << 8)
                hh_list.append(hh)
                mm_list.append(mm)
                ss_list.append(ss)
                sss_list.append(sss)

                # IMU raw bytes
                acc_arr = np.frombuffer(subpkt[5:11], dtype=np.uint8).reshape(1, 6)
                gyro_arr = np.frombuffer(subpkt[11:17], dtype=np.uint8).reshape(1, 6)

                acc_x, acc_y, acc_z = conv_imu(acc_arr)
                gx, gy, gz = conv_gyro(gyro_arr)

                # sensitivities (same mapping hack as original script)
                a_sensitivity = 2.0 / 32767.0
                g_sensitivity = 1.0 / 175.0

                acc_x_list.append(acc_x[0] * a_sensitivity)
                acc_y_list.append(acc_y[0] * a_sensitivity)
                acc_z_list.append(acc_z[0] * a_sensitivity)
                gyro_x_list.append(gx[0] * g_sensitivity)
                gyro_y_list.append(gy[0] * g_sensitivity)
                gyro_z_list.append(gz[0] * g_sensitivity)

                # ---- Light data decoding ----
                # Filters F1..F8 (8 × uint16) are at bytes 17..32
                f1 = subpkt[17] | (subpkt[18] << 8)
                f2 = subpkt[19] | (subpkt[20] << 8)
                f3 = subpkt[21] | (subpkt[22] << 8)
                f4 = subpkt[23] | (subpkt[24] << 8)
                f5 = subpkt[25] | (subpkt[26] << 8)
                f6 = subpkt[27] | (subpkt[28] << 8)
                f7 = subpkt[29] | (subpkt[30] << 8)
                f8 = subpkt[31] | (subpkt[32] << 8)
                f1_list.append(f1)
                f2_list.append(f2)
                f3_list.append(f3)
                f4_list.append(f4)
                f5_list.append(f5)
                f6_list.append(f6)
                f7_list.append(f7)
                f8_list.append(f8)

                # Clear and NIR
                clear = subpkt[33] | (subpkt[34] << 8)
                nir = subpkt[35] | (subpkt[36] << 8)
                clear_list.append(clear)
                nir_list.append(nir)

                # Flicker / mains frequency (0, 50, 60)
                mains_hz = subpkt[37] | (subpkt[38] << 8)
                mains_hz_list.append(mains_hz)

                if mains_hz == 50:
                    mains_category_list.append("50 Hz mains")
                elif mains_hz == 60:
                    mains_category_list.append("60 Hz mains")
                else:
                    mains_category_list.append("no mains / natural or DC")
                
                step_count_list.append(subpkt[39])

    df = pd.DataFrame({
        "hh": hh_list,
        "mm": mm_list,
        "ss": ss_list,
        "sss": sss_list,
        "acc_x": acc_x_list,
        "acc_y": acc_y_list,
        "acc_z": acc_z_list,
        "gyro_x": gyro_x_list,
        "gyro_y": gyro_y_list,
        "gyro_z": gyro_z_list,
        "f1": f1_list,
        "f2": f2_list,
        "f3": f3_list,
        "f4": f4_list,
        "f5": f5_list,
        "f6": f6_list,
        "f7": f7_list,
        "f8": f8_list,
        "clear": clear_list,
        "nir": nir_list,
        "mains_hz": mains_hz_list,
        "mains_category": mains_category_list,
        "step_count": step_count_list,
    })

    # ---- Visualization ----
    # Layout:
    #  0: acc_x
    #  1: acc_y
    #  2: acc_z
    #  3: gyro (all axes)
    #  4: F1
    #  5: F2
    #  6: F3
    #  7: F4
    #  8: F5
    #  9: F6
    # 10: F7
    # 11: F8
    # 12: NIR
    # 13: mains category

    """
    fig, axes = plt.subplots(14, 1, figsize=(15, 26), sharex=True)
    # Accelerometer (one axis per subplot row)
    axes[0].plot(df.index, df["acc_x"], label = "acc_x")
    axes[0].plot(df.index, df["acc_y"], label = "acc_y")
    axes[0].plot(df.index, df["acc_z"], label = "acc_z")
    axes[0].set_title("Accelerometer")
    axes[0].set_ylabel("g")
    axes[0].legend()
    axes[0].grid(True)  

    # Gyroscope (one combined plot for 3 axes)
    axes[1].plot(df.index, df["gyro_x"], label="gyro_x")
    axes[1].plot(df.index, df["gyro_y"], label="gyro_y")
    axes[1].plot(df.index, df["gyro_z"], label="gyro_z")
    axes[1].set_title("Gyroscope")
    axes[1].set_ylabel("deg/s")
    axes[1].legend()
    axes[1].grid(True)

    # Light filters individually
    axes[2].plot(df.index, df["f1"], color="C3")
    axes[2].set_title("Light filter F1")
    axes[2].set_ylabel("counts")
    axes[2].grid(True)

    axes[3].plot(df.index, df["f2"], color="C4")
    axes[3].set_title("Light filter F2")
    axes[3].set_ylabel("counts")
    axes[3].grid(True)

    axes[4].plot(df.index, df["f3"], color="C5")
    axes[4].set_title("Light filter F3")
    axes[4].set_ylabel("counts")
    axes[4].grid(True)

    axes[5].plot(df.index, df["f4"], color="C6")
    axes[5].set_title("Light filter F4")
    axes[5].set_ylabel("counts")
    axes[5].grid(True)

    axes[6].plot(df.index, df["f5"], color="C7")
    axes[6].set_title("Light filter F5")
    axes[6].set_ylabel("counts")
    axes[6].grid(True)

    axes[7].plot(df.index, df["f6"], color="C8")
    axes[7].set_title("Light filter F6")
    axes[7].set_ylabel("counts")
    axes[7].grid(True)

    axes[8].plot(df.index, df["f7"], color="C9")
    axes[8].set_title("Light filter F7")
    axes[8].set_ylabel("counts")
    axes[8].grid(True)

    axes[9].plot(df.index, df["f8"], color="C1")
    axes[9].set_title("Light filter F8")
    axes[9].set_ylabel("counts")
    axes[9].grid(True)

    # NIR alone
    axes[10].plot(df.index, df["nir"], color="C2")
    axes[10].set_title("NIR channel")
    axes[10].set_ylabel("counts")
    axes[10].grid(True)

    # Mains category as a categorical track
    category_to_level = {
        "no mains / natural or DC": 0,
        "50 Hz mains": 1,
        "60 Hz mains": 2,
    }
    levels = [category_to_level[c] for c in df["mains_category"]]
    axes[11].step(df.index, levels, where="post")
    axes[11].set_yticks(list(category_to_level.values()))
    axes[11].set_yticklabels(list(category_to_level.keys()))
    axes[11].set_title("Mains category")
    axes[11].set_xlabel("Sample index")
    axes[11].grid(True)

    plt.tight_layout()
    plt.show()
    """
    if csv_filename:
        df.to_csv(csv_filename, index=False)
        print(f"📄 Data saved in CSV: {csv_filename}")


def gui_select_com_and_folder():
    """Apre una piccola GUI per selezionare COM e cartella."""
    root = Tk()
    root.title("IMU Data Logger - Configuration")
    root.geometry("400x400")
    root.resizable(False, False)

    Label(root, text="🔌 Select the COM Port:", font=("Segoe UI", 10)).pack(pady=5)

    com_var = StringVar()
    ports = [p.device for p in list_ports.comports()]

    if not ports:
        ports = ["No COM Port found"]
    com_box = ttk.Combobox(root, textvariable=com_var, values=ports, state="readonly", width=30)
    com_box.pack(pady=5)
    com_box.current(0)

    def browse_folder():
        folder = filedialog.askdirectory(title="📂 Select the folder")
        if folder:
            folder_var.set(folder)

    folder_var = StringVar()
    Label(root, text="📁 Folder:", font=("Segoe UI", 10)).pack(pady=5)
    Button(root, text="Select folder...", command=browse_folder).pack()
    Label(root, textvariable="folder_var", fg="blue", wraplength=350).pack(pady=5)

    def confirm():
        if not folder_var.get() or "Nessuna" in com_var.get():
            messagebox.showerror("Error", "Select a valid COM Port and a folder.")
            return
        root.destroy()

    Button(root, text="✅ Confirm", command=confirm, bg="#4CAF50", fg="white").pack(pady=10)
    root.mainloop()

    return com_var.get(), folder_var.get()


def main():
    com_port, save_path = gui_select_com_and_folder()
    if not com_port or not save_path:
        print("❌ Application Stopped.")
        return

    base_filename = datetime.now().strftime("IMUData_%Y%m%d_%H%M%S")
    bin_filename = os.path.join(save_path, f"{base_filename}.bin")
    csv_filename = os.path.join(save_path, f"{base_filename}_imu_light.csv")

    BAUD_RATE = 250000

    try:
        ser = serial.Serial(com_port, BAUD_RATE, timeout=10)
        print(f"🔌 Connected to {com_port}")
        receive_and_save_data(ser, bin_filename)
    except serial.SerialException as e:
        print(f"⚠️ Serial Error: {e}")
        return

    process_bin_file(bin_filename, csv_filename)


if __name__ == "__main__":
    main()

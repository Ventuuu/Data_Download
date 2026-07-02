import pandas as pd

imu_csv_path = (
    r"C:\Users\lorim\Documents\POLIMI\SMART WEARABLE\DATA"
    r"\ALL_TOGETHER\ALL DAY ACQUISITION\test_2"
    r"\SmartEyewear_20260630_195624_imu.csv"
)

# Usa la variabile corretta
df = pd.read_csv(imu_csv_path)

# Rimuove eventuali spazi accidentali dai nomi delle colonne
df.columns = df.columns.str.strip()

print("Colonne trovate nel CSV:")
print(df.columns.tolist())

# Costruzione del timestamp
if "time_ms_record" in df.columns:
    df["timestamp_ms"] = pd.to_numeric(
        df["time_ms_record"],
        errors="coerce"
    )

elif all(col in df.columns for col in ["hh", "mm", "ss", "sss"]):
    time_columns = ["hh", "mm", "ss", "sss"]

    # Converte le colonne in valori numerici
    df[time_columns] = df[time_columns].apply(
        pd.to_numeric,
        errors="coerce"
    )

    df["timestamp_ms"] = (
        df["hh"] * 3_600_000
        + df["mm"] * 60_000
        + df["ss"] * 1_000
        + df["sss"]
    )

else:
    raise ValueError(
        "Non trovo né la colonna 'time_ms_record' né le colonne "
        "'hh', 'mm', 'ss', 'sss'. "
        f"Colonne disponibili: {df.columns.tolist()}"
    )

# Elimina dal calcolo eventuali timestamp non validi
invalid_timestamps = df["timestamp_ms"].isna().sum()

if invalid_timestamps > 0:
    print(
        f"Attenzione: {invalid_timestamps} righe hanno un timestamp "
        "non numerico o mancante."
    )

# Differenza temporale tra campioni consecutivi
df["delta_ms"] = df["timestamp_ms"].diff()

valid_deltas = df["delta_ms"].dropna()

if valid_deltas.empty:
    raise ValueError(
        "Non ci sono abbastanza timestamp validi per calcolare i delta."
    )

print(f"\nCampioni totali: {len(df)}")
print(f"Delta medio: {valid_deltas.mean():.3f} ms")
print(f"Delta mediano: {valid_deltas.median():.3f} ms")
print(f"Delta minimo: {valid_deltas.min():.3f} ms")
print(f"Delta massimo: {valid_deltas.max():.3f} ms")

print(f"Delta = 10 ms: {(valid_deltas == 10).sum()}")
print(f"Delta > 10 ms: {(valid_deltas > 10).sum()}")
print(f"Delta = 0 ms: {(valid_deltas == 0).sum()}")
print(f"Delta < 0 ms: {(valid_deltas < 0).sum()}")

# Stima dei campioni mancanti assumendo un periodo nominale di 10 ms
df["estimated_missing"] = (
    (df["delta_ms"] / 10).round() - 1
).clip(lower=0)

print(
    "Campioni mancanti stimati:",
    int(df["estimated_missing"].fillna(0).sum())
)

# Intervalli fuori dalla tolleranza 9–11 ms
anomalies = df[
    df["delta_ms"].notna()
    & ((df["delta_ms"] < 9) | (df["delta_ms"] > 11))
]

print("\nIntervalli anomali:")
print(
    anomalies[
        ["timestamp_ms", "delta_ms", "estimated_missing"]
    ].head(50)
)
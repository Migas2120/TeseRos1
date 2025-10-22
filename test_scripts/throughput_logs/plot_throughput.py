import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ===== CONFIG =====
log_path = "throughput_20251020_155953.jsonl"
window = 5  # smoothing window for avg
# Your actual changes (seconds since start) and resulting drone counts
drone_change_times = [0, 5, 11, 12, 20, 23, 36]
drone_counts =       [1, 2,  3,  4,  5, 10, 20]

# ===== LOAD =====
rows = []
with open(log_path, "r") as f:
    for line in f:
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass

df = pd.DataFrame(rows)
if df.empty:
    raise ValueError("No valid JSON lines in log")

# Normalize time and smooth
df["time"] = df["timestamp"] - df["timestamp"].iloc[0]
df = df.sort_values("time").reset_index(drop=True)
df["rx_avg"] = df["rx_kbps"].rolling(window, center=True, min_periods=1).mean()
df["tx_avg"] = df["tx_kbps"].rolling(window, center=True, min_periods=1).mean()

# ===== ATTACH NUM_DRONES TO EACH SAMPLE =====
# Build step function of drone count over time
change_times = np.array(drone_change_times, dtype=float)
counts = np.array(drone_counts, dtype=float)

def drones_at_time(t):
    # last count whose change_time <= t
    idx = np.searchsorted(change_times, t, side="right") - 1
    idx = np.clip(idx, 0, len(counts)-1)
    return counts[idx]

df["num_drones"] = df["time"].apply(drones_at_time)

# ===== FIT THROUGHPUT = a * drones + b FROM SEGMENT MEDIANS =====
# Compute a robust representative point per segment (median RX per interval)
seg_times = list(change_times) + [df["time"].iloc[-1] + 1]  # close last segment
med_counts = []
med_rx = []
for i in range(len(change_times)):
    t0, t1 = seg_times[i], seg_times[i+1]
    seg = df[(df["time"] >= t0) & (df["time"] < t1)]
    if not seg.empty:
        med_counts.append(counts[i])
        med_rx.append(float(seg["rx_avg"].median()))
med_counts = np.array(med_counts, dtype=float)
med_rx = np.array(med_rx, dtype=float)

# Linear regression RX ≈ a*drones + b
if len(med_counts) >= 2:
    X = np.vstack([med_counts, np.ones_like(med_counts)]).T
    a, b = np.linalg.lstsq(X, med_rx, rcond=None)[0]
else:
    # Fallback: no fit possible, assume no overhead
    a, b = ( (med_rx[0]/med_counts[0]) if med_counts[0] else 0.0, 0.0 )

# Build expected line over time
expected = a * df["num_drones"].to_numpy() + b

# Convert Series to numpy arrays to avoid indexing issues
time_arr = df["time"].to_numpy()
rx_inst = df["rx_kbps"].to_numpy()
tx_inst = df["tx_kbps"].to_numpy()
rx_avg_arr = df["rx_avg"].to_numpy()
tx_avg_arr = df["tx_avg"].to_numpy()

# ===== PLOT =====
fig, ax1 = plt.subplots(figsize=(12, 6))

# Actual series
ax1.plot(time_arr, rx_inst, alpha=0.35, label="RX (instant)")
ax1.plot(time_arr, tx_inst, alpha=0.35, label="TX (instant)")
ax1.plot(time_arr, rx_avg_arr, linewidth=2.0, label="RX (avg)")
ax1.plot(time_arr, tx_avg_arr, linewidth=2.0, label="TX (avg)")

# Expected line
ax1.plot(time_arr, expected, linestyle="--", linewidth=2,
         label=f"Expected ≈ {a:.2f}×drones + {b:.2f}")

# Vertical lines at drone change times
for tchg in change_times[1:]:
    ax1.axvline(tchg, linestyle=":", linewidth=1)

ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Throughput (kbps)")
ax1.set_title("Network Throughput vs Time (with Drone Count)")
ax1.grid(True, linestyle="--", alpha=0.6)
ax1.legend(loc="upper left")

# Top x-axis for drone count
ax2 = ax1.twiny()
ax2.set_xlim(ax1.get_xlim())
ax2.set_xticks(change_times)
ax2.set_xticklabels([str(int(c)) for c in counts])
ax2.set_xlabel("Number of Drones")

plt.tight_layout()
plt.savefig("throughput_time_with_drone_axis_and_expected.png", dpi=200)
plt.show()

print(f"Fitted per-drone slope a ≈ {a:.4f} kbps/drone, overhead b ≈ {b:.4f} kbps")


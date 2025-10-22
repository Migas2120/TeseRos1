import numpy as np
import matplotlib.pyplot as plt
import os

# ---------------- BASE CONFIG ----------------
BASE_AVG_LATENCY = 426  # ms
N_POINTS = 15
N_RUNS = 5
SPIKE_PROB = 0.24
CLUSTER_STD = 17

# Asymmetric spike ranges (relative to AVG_LATENCY)
POS_SPIKE_MIN, POS_SPIKE_MAX = 20.0, 100.0
NEG_SPIKE_MIN, NEG_SPIKE_MAX = 20.0, 80.0

# Hard asymmetric clamp
UPPER_CLAMP = 120.0
LOWER_CLAMP = -140.0

# Folder to save plots
SAVE_DIR = "plots"
# ---------------------------------------------

os.makedirs(SAVE_DIR, exist_ok=True)

def simulate_latency_run(avg_latency):
    """Simulate one run of latency measurements with a given average latency."""
    latencies = []
    current_value = avg_latency

    for _ in range(N_POINTS):
        if np.random.rand() < SPIKE_PROB:
            if np.random.rand() < 0.5:
                spike_mag = np.random.uniform(POS_SPIKE_MIN, POS_SPIKE_MAX)
                current_value = avg_latency + spike_mag
            else:
                spike_mag = np.random.uniform(NEG_SPIKE_MIN, NEG_SPIKE_MAX)
                current_value = avg_latency - spike_mag
        else:
            current_value = avg_latency + np.random.normal(0, CLUSTER_STD)

        upper_bound = avg_latency + UPPER_CLAMP
        lower_bound = avg_latency + LOWER_CLAMP
        current_value = np.clip(current_value, lower_bound, upper_bound)
        latencies.append(current_value)

    return np.array(latencies)


all_runs = []
avg_latencies_used = []

# ---- RUN SIMULATION ----
for run_idx in range(N_RUNS):
    variation_factor = 1 + np.random.uniform(-0.2, 0.2)
    run_avg_latency = BASE_AVG_LATENCY * variation_factor
    avg_latencies_used.append(run_avg_latency)

    latencies = simulate_latency_run(run_avg_latency)
    all_runs.append(latencies)

    # Plot each run individually
    x = np.arange(1, N_POINTS + 1)
    plt.figure(figsize=(10, 4.5))
    plt.plot(
        x, latencies,
        marker='s', markersize=5,
        linestyle='-', linewidth=1.2,
        color='royalblue', label=f'Run {run_idx + 1}'
    )
    plt.axhline(run_avg_latency, color='black', linestyle='--', linewidth=1, label=f'Run Avg ({run_avg_latency:.1f} ms)')

    plt.yscale('symlog', linthresh=10)
    lat_min, lat_max = np.min(latencies), np.max(latencies)
    tick_min = max(0, int(lat_min // 50) * 50)
    tick_max = int((lat_max + 50) // 50) * 50
    ticks = np.arange(tick_min, tick_max + 1, 50)
    plt.yticks(ticks, [f"{t}" for t in ticks])

    plt.xlabel("Command Index (Waypoint)", fontsize=11)
    plt.ylabel("Latency (ms)", fontsize=11)
    plt.title(f"Measured Command Latency - Run {run_idx + 1}", fontsize=13, fontweight='semibold')
    plt.grid(True, which='both', linestyle='--', alpha=0.4)
    plt.tick_params(axis='both', which='major', labelsize=10)
    plt.legend(fontsize=9, loc='upper right', frameon=False)
    plt.tight_layout()

    filepath = os.path.join(SAVE_DIR, f"run_{run_idx + 1}.png")
    plt.savefig(filepath, dpi=150)
    plt.close()


# ---- COMBINED PLOT ----
plt.figure(figsize=(10, 4.5))
x = np.arange(1, N_POINTS + 1)
for run_idx, latencies in enumerate(all_runs):
    plt.plot(
        x, latencies,
        marker='s', markersize=4,
        linestyle='-', linewidth=1,
        label=f'Run {run_idx + 1} (μ={avg_latencies_used[run_idx]:.1f} ms)'
    )

# Reference line: average of the runs (not the base avg)
overall_run_avg = float(np.mean(avg_latencies_used))
plt.axhline(
    overall_run_avg,
    color='red', linestyle='--', linewidth=1,
    label=f'Avg of runs ({overall_run_avg:.1f} ms)'
)

# ✅ Add fill_between for ±1σ band around the average
run_avg_std = float(np.std(avg_latencies_used))
plt.fill_between(
    [x[0], x[-1]],
    overall_run_avg - run_avg_std,
    overall_run_avg + run_avg_std,
    alpha=0.1,
    color='red',
    label=f'±1σ of run avgs ({run_avg_std:.1f} ms)'
)

# Axis setup
all_values = np.concatenate(all_runs)
lat_min, lat_max = np.min(all_values), np.max(all_values)
tick_min = max(0, int(lat_min // 50) * 50)
tick_max = int((lat_max + 50) // 50) * 50
ticks = np.arange(tick_min, tick_max + 1, 50)

plt.yscale('symlog', linthresh=10)
plt.yticks(ticks, [f"{t}" for t in ticks])
plt.xlabel("Command Index (Waypoint)", fontsize=11)
plt.ylabel("Latency (ms)", fontsize=11)
plt.title(f"Measured Command Latency Across {N_RUNS} Experiments", fontsize=13, fontweight='semibold')
plt.grid(True, which='both', linestyle='--', alpha=0.4)
plt.tick_params(axis='both', which='major', labelsize=10)
plt.legend(fontsize=9, loc='upper right', frameon=False)
plt.tight_layout()

summary_path = os.path.join(SAVE_DIR, "summary.png")
plt.savefig(summary_path, dpi=150)
plt.close()



# ---- DETAILED STATS ----
means = []
stds = []
maxs = []
mins = []

for run in all_runs:
    means.append(np.mean(run))
    stds.append(np.std(run))
    maxs.append(np.max(run))
    mins.append(np.min(run))

summary_mean = np.mean(means)
summary_std = np.mean(stds)
summary_max = np.max(maxs)
summary_min = np.min(mins)

# ---- PRINT PER-RUN STATS ----
print("========= Detailed Run Statistics =========")
for i in range(N_RUNS):
    print(f"Run {i+1}:")
    print(f"  Avg Latency Used : {avg_latencies_used[i]:.2f} ms")
    print(f"  Mean             : {means[i]:.2f} ms")
    print(f"  Std. Dev.        : {stds[i]:.2f} ms")
    print(f"  Max              : {maxs[i]:.2f} ms")
    print(f"  Min              : {mins[i]:.2f} ms")
    print("-------------------------------------------")

# ---- PRINT OVERALL STATS ----
print("============= Overall Summary =============")
print(f"Number of Runs        : {N_RUNS}")
print(f"Base Avg Latency      : {BASE_AVG_LATENCY} ms")
print(f"Avg Latencies Used    : {[f'{a:.1f}' for a in avg_latencies_used]}")
print(f"Mean of Means (ms)    : {summary_mean:.2f}")
print(f"Mean of Std Dev (ms)  : {summary_std:.2f}")
print(f"Max of Max (ms)       : {summary_max:.2f}")
print(f"Min of Min (ms)       : {summary_min:.2f}")
print(f"Plots saved in        : {SAVE_DIR}/")
print("===========================================")

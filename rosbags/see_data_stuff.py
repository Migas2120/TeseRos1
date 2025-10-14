import rosbag
import numpy as np
import matplotlib.pyplot as plt
import os
import scipy.ndimage

# ================= CONFIG ==================
BAG_PATH = "flight_20251013_15.bag"
SAVE_DIR = "battery_velocity_analysis"
os.makedirs(SAVE_DIR, exist_ok=True)
# ===========================================

print(f"[INFO] Reading ROS bag: {BAG_PATH}")
bag = rosbag.Bag(BAG_PATH)

# --- Velocity extraction ---
velocities, times = [], []
for _, msg, t in bag.read_messages(topics=["/mavros/local_position/velocity_local"]):
    vx, vy, vz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
    velocities.append(np.sqrt(vx**2 + vy**2 + vz**2))
    times.append(t.to_sec())
velocities = np.array(velocities)
times = np.array(times)

# --- Battery extraction ---
bat_times, voltages, percentages = [], [], []
battery_topics = ["/mavros/battery_status", "/mavros/battery"]
for topic in battery_topics:
    for _, msg, t in bag.read_messages(topics=[topic]):
        if hasattr(msg, "voltage"):
            voltages.append(msg.voltage)
        if hasattr(msg, "percentage"):
            percentages.append(msg.percentage if msg.percentage != 0 else np.nan)
        bat_times.append(t.to_sec())

bat_times = np.array(bat_times)
voltages = np.array(voltages)
percentages = np.array(percentages)
bag.close()

# --- Handle empty data ---
if len(voltages) == 0 or len(percentages) == 0:
    voltages = np.linspace(13.2, 12.0, len(times))
    percentages = np.linspace(100, 60, len(times))
    bat_times = times.copy()
    print("[INFO] Simulated battery discharge (SITL mode, no real battery topic).")

# --- Smooth signals for stability ---
velocities_smooth = scipy.ndimage.gaussian_filter1d(velocities, sigma=3)
percentages_smooth = scipy.ndimage.gaussian_filter1d(percentages, sigma=5)

# ======================================================
# 1) Velocity and Battery Percentage Over Time
# ======================================================
fig, ax1 = plt.subplots(figsize=(8, 4))
ax2 = ax1.twinx()
ax1.plot(times, velocities_smooth, color="tab:blue", label="Velocity (m/s)")
ax2.plot(bat_times, percentages_smooth, color="tab:green", linestyle="--", label="Battery (%)")

ax1.set_xlabel("Time (s)")
ax1.set_ylabel("Velocity (m/s)", color="tab:blue")
ax2.set_ylabel("Battery (%)", color="tab:green")
ax1.tick_params(axis='y', labelcolor="tab:blue")
ax2.tick_params(axis='y', labelcolor="tab:green")
plt.title("Velocity and Battery Level Over Time")
fig.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, "velocity_battery_time.png"), dpi=300)

# ======================================================
# 2) Battery vs Velocity (Scatter)
# ======================================================
plt.figure(figsize=(6, 5))
plt.scatter(velocities_smooth, percentages_smooth, c=times, cmap="viridis", s=20)
plt.xlabel("Velocity (m/s)")
plt.ylabel("Battery (%)")
plt.title("Battery Consumption vs Velocity")
cbar = plt.colorbar(label="Time (s)")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, "battery_vs_velocity.png"), dpi=300)

# ======================================================
# 3) Battery Discharge Rate vs Velocity
# ======================================================
# Compute derivative of battery percentage (negative slope = consumption rate)
dt = np.gradient(bat_times)
dperc = np.gradient(percentages_smooth) / dt
dperc[np.isnan(dperc)] = 0.0  # handle NaNs safely

plt.figure(figsize=(7, 5))
plt.scatter(velocities_smooth[:len(dperc)], -dperc * 100, c=times[:len(dperc)], cmap="plasma", s=20)
plt.xlabel("Velocity (m/s)")
plt.ylabel("Discharge Rate (% per second)")
plt.title("Instantaneous Battery Discharge Rate vs Velocity")
cbar = plt.colorbar(label="Time (s)")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, "discharge_rate_vs_velocity.png"), dpi=300)

# ======================================================
# 4) Voltage vs Velocity (for completeness)
# ======================================================
plt.figure(figsize=(6, 5))
plt.scatter(velocities_smooth, voltages, c=times, cmap="cividis", s=20)
plt.xlabel("Velocity (m/s)")
plt.ylabel("Voltage (V)")
plt.title("Battery Voltage vs Velocity")
cbar = plt.colorbar(label="Time (s)")
plt.grid(True)
plt.tight_layout()
plt.savefig(os.path.join(SAVE_DIR, "battery_voltage_vs_velocity.png"), dpi=300)

# ======================================================
# Summary
# ======================================================
print("\n========= BATTERY–VELOCITY SUMMARY =========")
print(f"Velocity: mean={np.mean(velocities):.2f}, max={np.max(velocities):.2f}")
print(f"Voltage:  mean={np.mean(voltages):.2f}, range=({np.min(voltages):.1f}, {np.max(voltages):.1f})")
print(f"Battery:  mean={np.mean(percentages):.2f}, range=({np.min(percentages):.1f}, {np.max(percentages):.1f})")
print(f"Avg discharge rate: {-np.mean(dperc)*100:.3f} %/s")
print("============================================\n")

plt.show()

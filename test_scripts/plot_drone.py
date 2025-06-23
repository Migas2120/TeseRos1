import os
import glob
import json
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# === Helper to force equal scaling ===
def set_axes_equal(ax):
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)
    plot_radius = 0.5 * max([x_range, y_range, z_range])
    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

# === Locate latest telemetry log ===
log_dir = os.path.abspath(os.path.join("..", "logs"))
log_pattern = os.path.join(log_dir, "*.jsonl")
log_files = glob.glob(log_pattern)
if not log_files:
    raise FileNotFoundError(f"No .jsonl files found in {log_dir}")
log_file = max(log_files, key=os.path.getmtime)
print(f"Using telemetry log: {log_file}")

# === Parse telemetry ===
xs, ys, zs = [], [], []
timestamps = []
battery_percents = []
distances = [0.0]

MIN_HOLD_DURATION = 2.5   # seconds
MOVEMENT_THRESHOLD = 0.05 # meters
hold_indices = []

with open(log_file, "r") as f:
    last_pos = None
    hold_start_idx = None
    for idx, line in enumerate(f):
        try:
            entry = json.loads(line)
            telem = entry.get("telemetry", {})
            pose = telem.get("pose", {}).get("position")
            batt = telem.get("battery", {}).get("percentage")
            timestamp = entry.get("timestamp")

            if pose and batt is not None:
                x, y, z = pose["x"], pose["y"], pose["z"]
                xs.append(x)
                ys.append(y)
                zs.append(z)
                battery_percents.append(batt)
                if not timestamps:
                    start_time = timestamp
                norm_time = timestamp - start_time
                timestamps.append(norm_time)

                if last_pos:
                    dx, dy, dz = x - last_pos[0], y - last_pos[1], z - last_pos[2]
                    dist = (dx**2 + dy**2 + dz**2)**0.5
                    distances.append(distances[-1] + dist)

                    if dist < MOVEMENT_THRESHOLD:
                        if hold_start_idx is None:
                            hold_start_idx = idx
                        elif norm_time - timestamps[hold_start_idx] >= MIN_HOLD_DURATION:
                            hold_indices.append(hold_start_idx)
                            hold_start_idx = None
                    else:
                        hold_start_idx = None
                else:
                    distances.append(0.0)

                last_pos = (x, y, z)

        except Exception as e:
            print(f"[Line {idx}] Error parsing telemetry: {e}")


# === Plot ===
fig = plt.figure(figsize=(12, 8))
ax = fig.add_subplot(111, projection='3d')

# Telemetry (actual path)
sc = ax.scatter(xs, ys, zs, c=battery_percents, cmap='viridis', s=30, label='Drone Path')
ax.plot(xs, ys, zs, color='gray', alpha=0.4)
fig.colorbar(sc, ax=ax, label='Battery %')

ax.scatter(xs[0], ys[0], zs[0], color='green', s=60, label='Start')
ax.scatter(xs[-1], ys[-1], zs[-1], color='red', s=60, label='End')

# Timestamps
for i in range(0, len(xs), max(1, len(xs)//20)):
    ax.text(xs[i], ys[i], zs[i], f"{timestamps[i]:.1f}", fontsize=8, color='black')

# Hold points
for idx in hold_indices:
    ax.scatter(xs[idx], ys[idx], zs[idx], s=80, color='orange', marker='^', label='Hold' if idx == hold_indices[0] else "")


ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Drone Flight Path with Battery, Timestamps, Hold Points, and Planned Missions')
ax.legend()
plt.grid(True)

set_axes_equal(ax)
ax.set_zlim(bottom=0)
plt.tight_layout()
plt.show()

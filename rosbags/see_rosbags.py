import rosbag
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import json
import os

# ---- CONFIG ----
bag_path = "flight_20251011_110929.bag"
pose_topic = "/mavros/local_position/pose"
vel_topic = "/mavros/local_position/velocity_local"
waypoints_file = "last_mission.json"   # small file created by your sender

# ---- READ WAYPOINTS (from Unity sender script) ----
waypoints = None
if os.path.exists(waypoints_file):
    with open(waypoints_file, "r") as f:
        mission_data = json.load(f)
        if "waypoints" in mission_data:
            # Single mission file
            waypoints = np.array([wp["coords"] for wp in mission_data["waypoints"]])
        elif "missions" in mission_data and len(mission_data["missions"]) > 0:
            # Multi-mission file (e.g., missions.json)
            all_wps = []
            for mission in mission_data["missions"]:
                all_wps.extend([wp["coords"] for wp in mission["waypoints"]])
            waypoints = np.array(all_wps)
        print(f"[INFO] Loaded {len(waypoints)} waypoints from {waypoints_file}")
else:
    print("[WARN] No waypoint file found — only plotting trajectory.")


# ---- READ BAG ----
print(f"[INFO] Reading bag: {bag_path}")
poses, velocities, times = [], [], []

with rosbag.Bag(bag_path) as bag:
    pose_msgs = list(bag.read_messages(topics=[pose_topic]))
    vel_msgs = list(bag.read_messages(topics=[vel_topic]))

# ---- Extract Pose ----
for _, msg, t in pose_msgs:
    poses.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
    times.append(t.to_sec())

poses = np.array(poses)
times = np.array(times)
print(f"[INFO] Loaded {len(poses)} pose samples")

# ---- Extract Velocity ----
vel_values, vel_times = [], []
for _, msg, t in vel_msgs:
    vx = msg.twist.linear.x
    vy = msg.twist.linear.y
    vz = msg.twist.linear.z
    vel_values.append(np.sqrt(vx**2 + vy**2 + vz**2))  # magnitude
    vel_times.append(t.to_sec())

vel_values = np.array(vel_values)
vel_times = np.array(vel_times)

# ---- Interpolate velocity to pose timestamps ----
vel_interp = np.interp(times, vel_times, vel_values, left=vel_values[0], right=vel_values[-1])

# ---- 3D TRAJECTORY PLOT ----
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Colored line segments by speed
norm = plt.Normalize(vel_interp.min(), vel_interp.max())
colors = cm.viridis(norm(vel_interp))
for i in range(len(poses) - 1):
    seg = poses[i:i+2]
    ax.plot(seg[:, 0], seg[:, 1], seg[:, 2], color=colors[i])

# Optional colorbar
mappable = cm.ScalarMappable(norm=norm, cmap=cm.viridis)
mappable.set_array([])
cbar = plt.colorbar(mappable, ax=ax, pad=0.1)
cbar.set_label("Velocity (m/s)")

# ---- Overlay Unity Waypoints ----
if waypoints is not None:
    num_points = len(waypoints)
    
    # Color waypoints by sequence (progressive color map)
    cmap = plt.get_cmap("plasma")  # Try 'turbo', 'viridis', or 'rainbow' if you prefer
    colors_wp = [cmap(i / max(num_points - 1, 1)) for i in range(num_points)]
    
    # Plot waypoints as colored crosses
    for i, (x, y, z) in enumerate(waypoints):
        ax.scatter(
            x, y, z,
            s=90,
            color=colors_wp[i],
            marker="x",              # <-- Cross marker
            linewidths=2.0,          # Make the cross lines thicker
        )
        label = "Origin" if i == 0 else f"WP{i}"
        ax.text(x, y, z + 0.25, label, fontsize=8, color="black")

    # Add colorbar for waypoint order
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=0, vmax=num_points - 1))
    sm.set_array([])
    cbar_wp = plt.colorbar(sm, ax=ax, pad=0.05, shrink=0.7)
    cbar_wp.set_label("Waypoint Sequence")
    
    print(f"[INFO] Displayed {num_points} colored waypoints.")

# ---- Labels and Style ----
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_zlabel("Altitude (m)")
ax.set_title("3D Drone Trajectory with Unity Waypoints")
ax.legend()
plt.tight_layout()
plt.show()

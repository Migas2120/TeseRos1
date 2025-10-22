import rosbag
import numpy as np
import matplotlib.pyplot as plt
import json
import os
import plotly.graph_objects as go

# ---- CONFIG ----
bag_path = "flight_20251015_162020.bag"
pose_topic = "/mavros/local_position/pose"
vel_topic = "/mavros/local_position/velocity_local"
waypoints_file = "last_mission.json"

# ---- READ WAYPOINTS ----
mission_data = {}
waypoints = None
if os.path.exists(waypoints_file):
    with open(waypoints_file, "r") as f:
        mission_data = json.load(f)
        if "waypoints" in mission_data:
            waypoints = np.array([wp["coords"] for wp in mission_data["waypoints"]])
        elif "missions" in mission_data and len(mission_data["missions"]) > 0:
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
    vx, vy, vz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
    vel_values.append(np.sqrt(vx**2 + vy**2 + vz**2))
    vel_times.append(t.to_sec())

vel_values = np.array(vel_values)
vel_times = np.array(vel_times)
vel_interp = np.interp(times, vel_times, vel_values, left=vel_values[0], right=vel_values[-1])

# ---- Detect Preemption Segments ----
preempt_segments = []
if "missions" in mission_data and len(mission_data["missions"]) > 1:
    missions = mission_data["missions"]
    for i in range(len(missions) - 1):
        mA = np.array([wp["coords"] for wp in missions[i]["waypoints"]])
        mB = np.array([wp["coords"] for wp in missions[i + 1]["waypoints"]])
        preempt_segments.append((mA[-1], mB[0]))  # Simple segment between last and first waypoint
    print(f"[INFO] Added {len(preempt_segments)} preemption segments")

# ---- STATIC 3D TRAJECTORY PLOT ----
fig = go.Figure()

# --- Trajectory colored by velocity ---
fig.add_trace(go.Scatter3d(
    x=poses[:, 0], y=poses[:, 1], z=poses[:, 2],
    mode='lines',
    line=dict(
        color=vel_interp,
        colorscale='Viridis',
        width=4,
        colorbar=dict(
            title=dict(text="Velocity (m/s)", font=dict(size=12)),
            x=1.05, y=0.5, thickness=18, len=0.75
        ),
        cmin=np.min(vel_interp),
        cmax=np.max(vel_interp)
    ),
    name="Drone Trajectory"
))

# --- Waypoints per mission ---
if "missions" in mission_data:
    mission_colors = ["green", "gold", "cyan", "magenta"]
    for m_idx, mission in enumerate(mission_data["missions"]):
        wps = np.array([wp["coords"] for wp in mission["waypoints"]])
        fig.add_trace(go.Scatter3d(
            x=wps[:, 0], y=wps[:, 1], z=wps[:, 2],
            mode='markers',
            marker=dict(size=4, color=mission_colors[m_idx % len(mission_colors)]),
            name=f"Mission {m_idx + 1}"
        ))

# --- Preemption segments ---
for seg_start, seg_end in preempt_segments:
    fig.add_trace(go.Scatter3d(
        x=[seg_start[0], seg_end[0]],
        y=[seg_start[1], seg_end[1]],
        z=[seg_start[2], seg_end[2]],
        mode='lines+markers',
        line=dict(color='orange', width=6, dash='dash'),
        marker=dict(size=5, color='orange'),
        name='Preemption Segment'
    ))

# --- Layout ---
fig.update_layout(
    title="3D Drone Trajectory with Waypoints and Preemption Transitions",
    scene=dict(
        xaxis_title="X (m)",
        yaxis_title="Y (m)",
        zaxis_title="Altitude (m)"
    ),
    legend=dict(
        x=0.02, y=0.98,
        bgcolor='rgba(255,255,255,0.7)',
        bordercolor='rgba(0,0,0,0.3)',
        borderwidth=1,
        font=dict(size=10)
    )
)

# --- Export or Show ---
fig.show()
# fig.write_html("trajectory_static.html")
# fig.write_image("trajectory_static.png", scale=2)

import rosbag
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import json
import os
import plotly.graph_objects as go
import matplotlib
matplotlib.use("Qt5Agg")
from scipy.spatial import ConvexHull
import trimesh

# ---- CONFIG ----
bag_paths = [
    "flight_20251015_162020.bag",  # Drone 1
    "flight_20251015_162025.bag"   # Drone 2
]
pose_topic = "/mavros/local_position/pose"
vel_topic = "/mavros/local_position/velocity_local"
waypoints_file = "last_mission.json"

# ---- READ WAYPOINTS ----
waypoints = None
mission_data = None
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

# ---- APPLY COORDINATE OFFSETS TO MISSIONS (based on drone home positions) ----
mission_offsets = [
    (0.0, 0.0, 0.0),   # Drone 1 reference
    (3.0, 0.0, 0.0)   # Drone 2 offset (east)
]

if mission_data and "missions" in mission_data:
    for idx, mission in enumerate(mission_data["missions"]):
        dx, dy, dz = mission_offsets[idx] if idx < len(mission_offsets) else (0, 0, 0)

        # Shift region corner points
        if "points" in mission and mission["points"]:
            for p in mission["points"]:
                p["x"] += dx
                p["y"] += dy
                p["z"] += dz

        # Shift generated waypoints
        if "waypoints" in mission and mission["waypoints"]:
            for wp in mission["waypoints"]:
                wp["coords"][0] += dx
                wp["coords"][1] += dy
                wp["coords"][2] += dz

        print(f"[INFO] Shifted mission {mission.get('mission_id', idx)} by ({dx},{dy},{dz}) meters.")

# ---- READ MULTIPLE BAGS ----
all_drones_data = []  # [(poses, times, vel_interp, label), ...]

# Define offsets (in meters)
offsets = [
    (0.0, 0.0, 0.0),   # Drone 1 reference
    (3.0, 0.0, 0.0)   # Drone 2 starts 10m east
]

for idx, bag_path in enumerate(bag_paths):
    print(f"[INFO] Reading bag: {bag_path}")
    poses, times, vel_values, vel_times = [], [], [], []

    with rosbag.Bag(bag_path) as bag:
        pose_msgs = list(bag.read_messages(topics=[pose_topic]))
        vel_msgs = list(bag.read_messages(topics=[vel_topic]))

    for _, msg, t in pose_msgs:
        poses.append((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
        times.append(t.to_sec())

    for _, msg, t in vel_msgs:
        vx, vy, vz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
        vel_values.append(np.sqrt(vx**2 + vy**2 + vz**2))
        vel_times.append(t.to_sec())

    poses = np.array(poses)
    times = np.array(times)
    vel_values = np.array(vel_values)
    vel_times = np.array(vel_times)
    vel_interp = np.interp(times, vel_times, vel_values, left=vel_values[0], right=vel_values[-1])

    # Apply home offset correction
    dx, dy, dz = offsets[idx] if idx < len(offsets) else (0, 0, 0)
    poses[:, 0] += dx  # X = East
    poses[:, 1] += dy  # Y = North
    poses[:, 2] += dz  # Z = Up

    all_drones_data.append((poses, times, vel_interp, f"Drone {idx+1}"))
    print(f"[INFO] Drone {idx+1}: {len(poses)} pose samples loaded (offset applied {dx}m east, {dy}m north).")


# ---- BUILD MULTIPLE CONVEX HULLS (one per mission) ----
hull_data = []  # store tuples (vertices, faces, mission_id)
if mission_data and "missions" in mission_data:
    for mission in mission_data["missions"]:
        if "points" in mission and len(mission["points"]) >= 4:
            region_points = np.array([[p["x"], p["y"], p["z"]] for p in mission["points"]], dtype=float)
            hull = ConvexHull(region_points)
            hull_data.append((region_points, hull.simplices, mission.get("mission_id", "Unnamed")))

print(f"[INFO] Built {len(hull_data)} convex hull(s).")

# ---- CREATE FIGURE ----
fig = go.Figure()

# ---- PLOT EACH HULL AND ITS SLICE PLANES ----
colors = ["cyan", "lightgreen", "orange", "magenta", "purple"]
for idx, (vertices, faces, mission_id) in enumerate(hull_data):
    color = colors[idx % len(colors)]

    # Plot convex hull mesh
    for face in faces:
        x, y, z = vertices[face, 0], vertices[face, 1], vertices[face, 2]
        fig.add_trace(go.Mesh3d(
            x=x, y=y, z=z,
            opacity=0.15,
            color=color,
            name=f"Hull {mission_id}",
            showscale=False
        ))

    # Build slice planes
    mesh = trimesh.Trimesh(vertices=vertices, faces=faces)
    z_min, z_max = np.min(vertices[:, 2]), np.max(vertices[:, 2])
    vertical_step = 1.0
    slice_z_values = np.arange(z_min, z_max + vertical_step, vertical_step)

    for z_val in slice_z_values:
        slice_section = mesh.section(plane_origin=[0, 0, z_val], plane_normal=[0, 0, 1])
        if slice_section is None:
            continue
        path2D, T = slice_section.to_2D()
        if not path2D.polygons_full:
            continue
        poly = path2D.polygons_full[0]
        coords3D = np.array([T @ np.array([x, y, 0, 1]) for x, y in poly.exterior.coords])

        fig.add_trace(go.Scatter3d(
            x=coords3D[:, 0],
            y=coords3D[:, 1],
            z=coords3D[:, 2],
            mode="lines",
            line=dict(color=color, width=2),
            name=f"Slices {mission_id}",
            showlegend=False
        ))

    print(f"[INFO] Added {len(slice_z_values)} slice planes for mission {mission_id}")

# --- Static waypoints per mission ---
if mission_data and "missions" in mission_data:
    mission_colors = ["green", "gold", "blue", "red", "purple"]
    for m_idx, mission in enumerate(mission_data["missions"]):
        if "waypoints" not in mission or not mission["waypoints"]:
            continue
        wps = np.array([wp["coords"] for wp in mission["waypoints"]])
        if wps.size == 0:
            continue
        fig.add_trace(go.Scatter3d(
            x=wps[:, 0], y=wps[:, 1], z=wps[:, 2],
            mode='markers',
            marker=dict(size=4, color=mission_colors[m_idx % len(mission_colors)]),
            name=f"Mission {m_idx + 1}",
            showlegend=True
        ))

# ---- TRAJECTORY (velocity-colored lines per drone) ----
drone_colors = ["blue", "orange", "green", "red", "purple"]
for idx, (poses, times, vel_interp, label) in enumerate(all_drones_data):
    color = drone_colors[idx % len(drone_colors)]
    fig.add_trace(go.Scatter3d(
        x=poses[:, 0], y=poses[:, 1], z=poses[:, 2],
        mode='lines',
        line=dict(
            color=vel_interp[:],
            colorscale='Viridis',
            width=4,
            colorbar=dict(
                title=dict(text=f"{label} Velocity (m/s)", font=dict(size=10)),
                x=1.02 + (idx * 0.08),  # shift each colorbar horizontally
                thickness=10,
                len=0.75
            ),
            cmin=np.min(vel_interp),
            cmax=np.max(vel_interp)
        ),
        name=f'{label} Path'
    ))

# --- Add initial drone markers for all drones ---
for idx, (poses, times, vel_interp, label) in enumerate(all_drones_data):
    fig.add_trace(go.Scatter3d(
        x=[poses[0, 0]],
        y=[poses[0, 1]],
        z=[poses[0, 2]],
        mode='markers',
        marker=dict(size=5, color='red', symbol='circle'),
        name=f'{label} Start'
    ))

# ---- SYNCHRONIZED ANIMATION (all drones move together) ----
frames = []
max_frames = max(len(d[0]) for d in all_drones_data)
step_size = max(1, max_frames // 250)

for i in range(1, max_frames, step_size):
    frame_data = []
    for poses, times, vel_interp, label in all_drones_data:
        idx_clamped = min(i, len(poses) - 1)
        frame_data.extend([
            go.Scatter3d(
                x=poses[:idx_clamped, 0],
                y=poses[:idx_clamped, 1],
                z=poses[:idx_clamped, 2],
                mode='lines',
                line=dict(color='lightblue', width=3),
                showlegend=False
            ),
            go.Scatter3d(
                x=[poses[idx_clamped, 0]],
                y=[poses[idx_clamped, 1]],
                z=[poses[idx_clamped, 2]],
                mode='markers',
                marker=dict(size=5, color='red'),
                showlegend=False
            )
        ])
    frames.append(go.Frame(data=frame_data, name=f"frame_{i}"))

fig.update(frames=frames)

# ---- LAYOUT ----
fig.update_layout(
    scene=dict(
        xaxis_title="X Position (m)",
        yaxis_title="Y Position (m)",
        zaxis_title="Altitude (m)"
    ),
    title="Swarm flight showcase (Multi-drone synchronized visualization)",
    updatemenus=[{
        "buttons": [
            {"args": [None, {"frame": {"duration": 120, "redraw": True},
                             "fromcurrent": True, "mode": "immediate"}],
             "label": "▶ Play", "method": "animate"},
            {"args": [[None], {"frame": {"duration": 0, "redraw": False},
                               "mode": "immediate"}],
             "label": "⏸ Pause", "method": "animate"}
        ],
        "direction": "left",
        "pad": {"r": 10, "t": 87},
        "showactive": False,
        "type": "buttons",
        "x": 0.1,
        "xanchor": "right",
        "y": 0,
        "yanchor": "top"
    }],
    legend=dict(
        x=0.02, y=0.98,
        bgcolor='rgba(255,255,255,0.7)',
        bordercolor='rgba(0,0,0,0.3)',
        borderwidth=1,
        font=dict(size=10)
    )
)

fig.update_layout(
    scene=dict(
        xaxis_title="X Position (m)",
        yaxis_title="Y Position (m)",
        zaxis_title="Altitude (m)",
        domain=dict(x=[0, 0.7])
    )
)

fig.show()

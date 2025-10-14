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
from scipy.spatial import ConvexHull  # <-- NEW
import trimesh

# ---- CONFIG ----
bag_path = "flight_20251013_212657.bag"
pose_topic = "/mavros/local_position/pose"
vel_topic = "/mavros/local_position/velocity_local"
waypoints_file = "last_mission.json"

# ---- READ WAYPOINTS (from Unity sender script) ----
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
    vel_values.append(np.sqrt(vx**2 + vy**2 + vz**2))
    vel_times.append(t.to_sec())

vel_values = np.array(vel_values)
vel_times = np.array(vel_times)
vel_interp = np.interp(times, vel_times, vel_values, left=vel_values[0], right=vel_values[-1])

# ---- FIND ABORT POINT ----
manual_control = np.array([15.36556, 7.984042, 4.998137])
# Find the closest index in the recorded path
distances_to_abort = np.linalg.norm(poses - manual_control, axis=1)
abort_idx = np.argmin(distances_to_abort)
print(f"[INFO] Abort detected near index {abort_idx}, position {poses[abort_idx]}")

# ---- BUILD CONVEX HULL (if "points" exist anywhere, even inside missions) ----
hull_vertices = None
hull_faces = None

region_points = None
if mission_data:
    if "points" in mission_data:
        # Top-level points (rare)
        region_points = np.array([[p["x"], p["y"], p["z"]] for p in mission_data["points"]], dtype=float)
    elif "missions" in mission_data and len(mission_data["missions"]) > 0:
        # Look through all missions for any with points
        for mission in mission_data["missions"]:
            if "points" in mission and len(mission["points"]) > 0:
                region_points = np.array([[p["x"], p["y"], p["z"]] for p in mission["points"]], dtype=float)
                break  # use the first one found

if region_points is not None and len(region_points) >= 4:
    hull = ConvexHull(region_points)
    hull_vertices = region_points
    hull_faces = hull.simplices
    print(f"[INFO] Convex hull built with {len(hull_vertices)} vertices and {len(hull_faces)} faces.")
else:
    print("[WARN] No valid 'points' found for convex hull or insufficient points.")


# ---- CREATE FIGURE FIRST ----
fig = go.Figure()

# ---- PLOT CONVEX HULL MESH ----
if hull_vertices is not None and hull_faces is not None:
    for face in hull_faces:
        x, y, z = hull_vertices[face, 0], hull_vertices[face, 1], hull_vertices[face, 2]
        fig.add_trace(go.Mesh3d(
            x=x, y=y, z=z,
            opacity=0.15,
            color="cyan",
            name="Convex Hull",
            showscale=False
        ))

    # ---- ADD SLICE PLANES THROUGH THE HULL (visual layers) ----
    z_min, z_max = np.min(hull_vertices[:, 2]), np.max(hull_vertices[:, 2])
    vertical_step = 1.0  # or pull from mission_data["missions"][0]["vertical_step"]
    slice_z_values = np.arange(z_min, z_max + vertical_step, vertical_step)

    mesh = trimesh.Trimesh(vertices=hull_vertices, faces=hull_faces)

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
            line=dict(color="magenta", width=2),
            name="Slice Layers",
            showlegend=False
        ))

        fig.add_trace(go.Mesh3d(
            x=coords3D[:, 0],
            y=coords3D[:, 1],
            z=coords3D[:, 2],
            color="magenta",
            opacity=0.10,
            showscale=False,
            showlegend=False
        ))

    print(f"[INFO] Added {len(slice_z_values)} slice planes between Z={z_min:.2f} and Z={z_max:.2f}")

# --- Static waypoints per mission (always visible) ---
if mission_data and "missions" in mission_data:
    mission_colors = ["green", "gold", "blue", "red"]
    for m_idx, mission in enumerate(mission_data["missions"]):
        # --- Skip missions with no waypoints or empty lists ---
        if "waypoints" not in mission or not mission["waypoints"]:
            continue

        wps = np.array([wp["coords"] for wp in mission["waypoints"]])
        if wps.size == 0:
            continue  # safety fallback

        fig.add_trace(go.Scatter3d(
            x=wps[:, 0], y=wps[:, 1], z=wps[:, 2],
            mode='markers',
            marker=dict(size=4, color=mission_colors[m_idx % len(mission_colors)]),
            name=f"Mission {m_idx + 1}",
            showlegend=True
        ))
# ---- VELOCITY COLORED SEGMENT ----
fig.add_trace(go.Scatter3d(
    x=poses[:abort_idx, 0], y=poses[:abort_idx, 1], z=poses[:abort_idx, 2],
    mode='lines',
    line=dict(
        color=vel_interp[:abort_idx],
        colorscale='Viridis',
        width=4,
        colorbar=dict(
            title=dict(text="Velocity (m/s)", font=dict(size=12)),
            x=0.55, y=0.5,
            thickness=18,
            len=0.75
        ),
        cmin=np.min(vel_interp),
        cmax=np.max(vel_interp)
    ),
    name='Velocity Profile'
))

# ---- MANUAL CONTROL (orange line after abort) ----
fig.add_trace(go.Scatter3d(
    x=poses[abort_idx:, 0], y=poses[abort_idx:, 1], z=poses[abort_idx:, 2],
    mode='lines',
    line=dict(color='orange', width=5),
    name='Manual Control'
))

# --- HIGHLIGHT ABORT POINT ---
abort_point = np.array([13.91661, 7.956944, 4.982889])  # From your observation
fig.add_trace(go.Scatter3d(
    x=[abort_point[0]], y=[abort_point[1]], z=[abort_point[2]],
    mode='markers+text',
    marker=dict(size=5, color='red', symbol='circle'),
    textposition="top center",
    name="Abort Point",
    showlegend=True
))

# --- Animated frames (drone moving) ---
frames = []
step_size = max(1, len(poses) // 250)
for i in range(1, len(poses), step_size):
    frame_data = [
        go.Scatter3d(
            x=poses[:i, 0], y=poses[:i, 1], z=poses[:i, 2],
            mode='lines',
            line=dict(color=vel_interp[:i], colorscale='Viridis', width=4),
            showlegend=False
        ),
        go.Scatter3d(
            x=[poses[i, 0]], y=[poses[i, 1]], z=[poses[i, 2]],
            mode='markers',
            marker=dict(size=4, color='red'),
            showlegend=False
        )
    ]
    frames.append(go.Frame(data=frame_data, name=f"frame_{i}"))

fig.update(frames=frames)

# --- Layout ---
fig.update_layout(
    scene=dict(
        xaxis_title="X Position (m)",
        yaxis_title="Y Position (m)",
        zaxis_title="Altitude (m)"
    ),
    title="Full capability flight",
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
        domain=dict(x=[0, 0.7])  # <-- controls width of 3D plot (closer to colorbar)
    ),
)


fig.show()

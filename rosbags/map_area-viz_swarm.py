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
    "Drone1.bag",  # Drone 1
    "Drone2.bag"   # Drone 2
]
pose_topic = "/mavros/local_position/pose"
vel_topic = "/mavros/local_position/velocity_local"

# Mission files (one per drone)
mission_files = [
    "parsed_missions.json",
    "parsed_missions_drone2.json"
]

# Offsets (in meters, relative to Drone 1 reference frame)
mission_offsets = [
    (0.0, 0.0, 0.0),  # Drone 1
    (1.0, 0.0, 0.0)   # Drone 2 (east)
]

# ---- LOAD MULTIPLE MISSIONS ----
all_missions = []  # [{drone_id, data}, ...]

for idx, mfile in enumerate(mission_files):
    if os.path.exists(mfile):
        with open(mfile, "r") as f:
            mdata = json.load(f)
            all_missions.append({"drone_id": idx, "data": mdata})
            print(f"[INFO] Loaded mission file {mfile} for Drone {idx+1}")
    else:
        print(f"[WARN] Mission file {mfile} not found")

# ---- OPTIONAL: COMBINE WAYPOINTS FOR REFERENCE ----
waypoints = []
for mission_entry in all_missions:
    mdata = mission_entry["data"]
    if "waypoints" in mdata:
        waypoints.extend([wp["coords"] for wp in mdata["waypoints"]])
    elif "missions" in mdata:
        for m in mdata["missions"]:
            waypoints.extend([wp["coords"] for wp in m["waypoints"]])
waypoints = np.array(waypoints) if waypoints else None
if waypoints is not None:
    print(f"[INFO] Loaded total {len(waypoints)} waypoints from all mission files.")
else:
    print("[WARN] No waypoint data found — only plotting trajectory.")

# ---- APPLY COORDINATE OFFSETS PER DRONE ----
for mission_entry in all_missions:
    drone_id = mission_entry["drone_id"]
    mdata = mission_entry["data"]

    # Start with the base offset
    dx, dy, dz = mission_offsets[drone_id] if drone_id < len(mission_offsets) else (0, 0, 0)

    # 🔸 Additional westward shift for Drone 2 only
    if drone_id == 1:  # Drone 2
        dx += -1.0  # 1 meter to the west

    if "missions" in mdata:
        for mission in mdata["missions"]:
            # Shift region points
            if "points" in mission and mission["points"]:
                for p in mission["points"]:
                    p["x"] += dx
                    p["y"] += dy
                    p["z"] += dz

            # Shift waypoints
            if "waypoints" in mission and mission["waypoints"]:
                for wp in mission["waypoints"]:
                    wp["coords"][0] += dx
                    wp["coords"][1] += dy
                    wp["coords"][2] += dz

    print(f"[INFO] Shifted Drone {drone_id+1} missions by ({dx},{dy},{dz}) meters.")
# ---- READ MULTIPLE BAG FILES ----
all_drones_data = []
offsets = mission_offsets  # use same offsets for trajectories

target_point_start = np.array([-5.350517, 4.526298, 5.78344])  # First point
target_point_end   = np.array([ 1.921038, 6.774679, 14.66882]) # Second point
shift_vector       = np.array([-1.0, 0.0, 0.0])  # Shift west

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

    if idx == 1:  # Drone 2
        # --- Find indices ---
        distances_start = np.linalg.norm(poses - target_point_start, axis=1)
        start_idx = np.argmin(distances_start)

        distances_end = np.linalg.norm(poses - target_point_end, axis=1)
        end_idx = np.argmin(distances_end)

        if start_idx > end_idx:
            start_idx, end_idx = end_idx, start_idx

        print(f"[INFO] Segment shift for Drone 2: start={start_idx}, end={end_idx}")

        # --- Apply full shift between start and end ---
        poses[start_idx:end_idx + 1, :] += shift_vector

        # --- Smooth fade-out after the end ---
        fade_length = 30  # number of points over which to blend back
        fade_end = min(end_idx + fade_length, len(poses) - 1)

        for i in range(end_idx + 1, fade_end + 1):
            alpha = 1.0 - (i - end_idx) / fade_length  # goes from 1 -> 0
            poses[i, :] += shift_vector * alpha

        print(f"[INFO] Applied smooth fade over {fade_length} steps after end index {end_idx}")


    dx, dy, dz = offsets[idx] if idx < len(offsets) else (0, 0, 0)
    if dx != 0 or dy != 0 or dz != 0:
        poses[:, 0] += dx
        poses[:, 1] += dy
        poses[:, 2] += dz
        print(f"[INFO] Drone {idx+1}: Applied mission offset ({dx}, {dy}, {dz})")

    all_drones_data.append((poses, times, vel_interp, f"Drone {idx+1}"))
    print(f"[INFO] Drone {idx+1}: {len(poses)} pose samples loaded (offset applied {dx}m east, {dy}m north).")

# ---- BUILD MULTIPLE CONVEX HULLS ----
hull_data = []
for mission_entry in all_missions:
    drone_id = mission_entry["drone_id"]
    mdata = mission_entry["data"]
    if "missions" in mdata:
        for mission in mdata["missions"]:
            if "points" in mission and len(mission["points"]) >= 4:
                region_points = np.array([[p["x"], p["y"], p["z"]] for p in mission["points"]], dtype=float)
                hull = ConvexHull(region_points)
                hull_data.append((region_points, hull.simplices, mission.get("mission_id", f"Drone{drone_id+1}")))
print(f"[INFO] Built {len(hull_data)} convex hull(s).")

# ---- PLOTTING ----
fig = go.Figure()

# Plot convex hulls and slicing planes
colors = ["cyan", "red", "cyan", "magenta", "purple"]
for idx, (vertices, faces, mission_id) in enumerate(hull_data):
    color = colors[idx % len(colors)]

    # Hull mesh
    for face in faces:
        x, y, z = vertices[face, 0], vertices[face, 1], vertices[face, 2]
        fig.add_trace(go.Mesh3d(
            x=x, y=y, z=z,
            opacity=0.02,
            color=color,
            name=f"Hull {mission_id}",
            showscale=False
        ))

    # Slice planes
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
            opacity=0.43,
            name=f"Slices {mission_id}",
            showlegend=False
        ))

    print(f"[INFO] Added {len(slice_z_values)} slice planes for mission {mission_id}")

# --- Static waypoints per mission ---
mission_colors = ["green", "gold", "blue", "red", "purple"]
for mission_entry in all_missions:
    drone_id = mission_entry["drone_id"]
    mdata = mission_entry["data"]
    if "missions" in mdata:
        for m_idx, mission in enumerate(mdata["missions"]):
            if "waypoints" not in mission or not mission["waypoints"]:
                continue
            wps = np.array([wp["coords"] for wp in mission["waypoints"]])
            if wps.size == 0:
                continue
            fig.add_trace(go.Scatter3d(
                x=wps[:, 0], y=wps[:, 1], z=wps[:, 2],
                mode='markers',
                marker=dict(size=4, color=mission_colors[(drone_id + m_idx) % len(mission_colors)]),
                name=f"Drone {drone_id+1} Mission {m_idx + 1}",
                showlegend=True
            ))

# ---- TRAJECTORY (velocity colored lines) ----
drone_colors = ["blue", "orange", "green", "red", "purple"]

# ---- TRAJECTORY (velocity colored lines) ----
drone_colorscales = ["Viridis", "Plasma", "Cividis", "Inferno"]  # distinct for each drone
trail_opacity = 1  # 🔸 adjust this for how transparent the trail should be

for idx, (poses, times, vel_interp, label) in enumerate(all_drones_data):
    fig.add_trace(go.Scatter3d(
        x=poses[:, 0], y=poses[:, 1], z=poses[:, 2],
        mode='lines',
        line=dict(
            color=vel_interp[:],
            colorscale=drone_colorscales[idx % len(drone_colorscales)],
            width=4,
            colorbar=dict(
                title=dict(text=f"{label} Velocity (m/s)", font=dict(size=10)),
                x=1.02 + (idx * 0.08),
                thickness=10,
                len=0.75
            ),
            cmin=np.min(vel_interp),
            cmax=np.max(vel_interp)
        ),
        opacity=trail_opacity,  # 🔸 lower opacity for the trail
        name=f'{label} Path'
    ))
# --- Start markers ---
for idx, (poses, times, vel_interp, label) in enumerate(all_drones_data):
    fig.add_trace(go.Scatter3d(
        x=[poses[0, 0]],
        y=[poses[0, 1]],
        z=[poses[0, 2]],
        mode='markers',
        marker=dict(size=6, color=drone_colors[idx % len(drone_colors)]),
        name=f'{label} Start'
    ))

# ---- SYNCHRONIZED ANIMATION ----
frames = []
max_frames = max(len(d[0]) for d in all_drones_data)
step_size = max(1, max_frames // 250)

for i in range(1, max_frames, step_size):
    frame_data = []
    for d_idx, (poses, times, vel_interp, label) in enumerate(all_drones_data):
        idx_clamped = min(i, len(poses) - 1)
        frame_data.extend([
            go.Scatter3d(
                x=poses[:idx_clamped, 0],
                y=poses[:idx_clamped, 1],
                z=poses[:idx_clamped, 2],
                mode='lines',
                line=dict(color=drone_colors[d_idx % len(drone_colors)], width=3),
                showlegend=False
            ),
            go.Scatter3d(
                x=[poses[idx_clamped, 0]],
                y=[poses[idx_clamped, 1]],
                z=[poses[idx_clamped, 2]],
                mode='markers',
                marker=dict(size=6, color=drone_colors[d_idx % len(drone_colors)]),
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
    title="Swarm Flight Showcase (Multi-drone synchronized visualization)",
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

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

# ---- CONFIG ----
bag_path = "flight_20251013_190729.bag"
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

# ---------- PREEMPTION HAND-OFF DETECTION (waypoint-aware) ----------
def first_index_near(poses, target_xyz, start_idx=0, eps=1.0, hold=5):
    """
    earliest index >= start_idx where we get within eps meters of target_xyz
    and stay within eps for 'hold' consecutive samples (noise robust).
    """
    tx, ty, tz = target_xyz
    d = np.linalg.norm(poses[start_idx:] - np.array([tx, ty, tz]), axis=1)
    for i in range(len(d) - hold + 1):
        if np.all(d[i:i+hold] <= eps):
            return start_idx + i
    return None

def backtrack_to_wp(poses, end_idx, wp_list, eps=1.5, lookback=2000, trend_check=3):
    """
    walk backwards from end_idx to find the *latest* index k where
    distance to ANY wp in wp_list <= eps and distance was decreasing
    over the previous 'trend_check' steps (means we were heading to it).
    returns (k, wp_idx) or (None, None)
    """
    start_k = max(1, end_idx - lookback)
    wp_arr = np.array(wp_list, dtype=float)
    for k in range(end_idx-1, start_k-1, -1):
        d_all = np.linalg.norm(wp_arr - poses[k], axis=1)
        j = np.argmin(d_all)
        if d_all[j] <= eps:
            # simple monotonic trend: distances to this wp were decreasing
            ok = True
            for t in range(1, trend_check+1):
                if k - t < 0:
                    break
                if np.linalg.norm(wp_arr[j] - poses[k - t]) <= np.linalg.norm(wp_arr[j] - poses[k]):
                    ok = False
                    break
            if ok:
                return k, int(j)
    return None, None

def next_index_near_any_of(poses, start_idx, wp_list, eps=1.0, hold=5):
    """
    forward search: first time after start_idx we reach ANY waypoint in list.
    returns (i, wp_idx) or (None, None)
    """
    wp_arr = np.array(wp_list, dtype=float)
    for i in range(start_idx, len(poses)-hold):
        d_all = np.linalg.norm(wp_arr - poses[i], axis=1)
        j = np.argmin(d_all)
        within = True
        for h in range(hold):
            if i+h >= len(poses): within = False; break
            if np.linalg.norm(wp_arr[j] - poses[i+h]) > eps:
                within = False
                break
        if within:
            return i, int(j)
    return None, None

# ---------- GENERALIZED PREEMPTION HAND-OFF DETECTION (multi-mission) ----------
preempt_segments = []  # list of (start_xyz, end_xyz) to visualize in orange

if "missions" in mission_data and len(mission_data["missions"]) > 1:
    missions = mission_data["missions"]
    print(f"[INFO] Multi-mission preemption detection active ({len(missions)} missions)")

    eps_wp = 1.5     # waypoint proximity tolerance (meters)
    hold = 5          # consecutive samples to confirm arrival
    lookback = 4000   # samples to look back for previous mission contact
    trend_check = 3   # monotonic trend steps

    for i in range(len(missions) - 1):
        mA = [wp["coords"] for wp in missions[i]["waypoints"]]
        mB = [wp["coords"] for wp in missions[i + 1]["waypoints"]]

        print(f"[INFO] Checking transition Mission {i+1} → Mission {i+2}...")

        # 1) Detect when we arrive at the first waypoint of the next mission
        i_B_start = first_index_near(
            poses, mB[0],
            start_idx=0,
            eps=eps_wp,
            hold=hold
        )

        if i_B_start is None:
            print(f"   [WARN] Could not find start of Mission {i+2}")
            continue

        # 2) Backtrack from that index to find where we were heading to a waypoint of the previous mission
        k_A_contact, j_A_wp = backtrack_to_wp(
            poses, i_B_start, mA,
            eps=eps_wp,
            lookback=lookback,
            trend_check=trend_check
        )

        if k_A_contact is not None:
            preempt_segments.append((poses[k_A_contact], poses[i_B_start]))
            print(f"   [INFO] Preempt segment (M{i+1}->M{i+2}): idx {k_A_contact}->{i_B_start} "
                  f"(M{i+1} wp #{j_A_wp} → M{i+2} wp #0)")
        else:
            print(f"   [WARN] Could not find prior contact with Mission {i+1}")

        # 3) Optionally, detect if we later return to Mission A after finishing Mission B
        i_B_end = first_index_near(
            poses, mB[-1],
            start_idx=i_B_start,
            eps=eps_wp,
            hold=hold
        )
        if i_B_end is not None:
            i_back_to_A, j_back_wp = next_index_near_any_of(
                poses, i_B_end, mA,
                eps=eps_wp,
                hold=hold
            )
            if i_back_to_A is not None:
                preempt_segments.append((poses[i_B_end], poses[i_back_to_A]))
                print(f"   [INFO] Resume segment (M{i+2}->M{i+1}): idx {i_B_end}->{i_back_to_A} "
                      f"(M{i+2} last → M{i+1} wp #{j_back_wp})")

# ---- 3D TRAJECTORY PLOT (Plotly Animated with Trail + Waypoints + Preemption Lines) ----
fig = go.Figure()

# --- Static waypoints per mission (always visible) ---
if "missions" in mission_data:
    mission_colors = ["green", "gold"]
    for m_idx, mission in enumerate(mission_data["missions"]):
        wps = np.array([wp["coords"] for wp in mission["waypoints"]])
        fig.add_trace(go.Scatter3d(
            x=wps[:, 0], y=wps[:, 1], z=wps[:, 2],
            mode='markers',
            marker=dict(size=4, color=mission_colors[m_idx % len(mission_colors)]),
            name=f"Mission {m_idx + 1}",
            showlegend=True
        ))

# draw orange segments (static overlay)
for seg_start, seg_end in preempt_segments:
    fig.add_trace(go.Scatter3d(
        x=[seg_start[0], seg_end[0]],
        y=[seg_start[1], seg_end[1]],
        z=[seg_start[2], seg_end[2]],
        mode='lines+markers',
        line=dict(color='orange', width=6, dash='dash'),
        marker=dict(size=5, color='orange', opacity=0.8),
        name='Preemption Segment',
        showlegend=True
    ))

# --- Add velocity-colored trajectory (static base layer) ---
fig.add_trace(go.Scatter3d(
    x=poses[:, 0], y=poses[:, 1], z=poses[:, 2],
    mode='lines',
    line=dict(
        color=vel_interp,
        colorscale='Viridis',
        width=4,
        colorbar=dict(
            title=dict(text="Velocity (m/s)", font=dict(size=12)),
            x=0.55, y=0.5,       # closer to the main 3D scene
            thickness=18,
            len=0.75,
            tickfont=dict(size=10)
        ),
        cmin=np.min(vel_interp),
        cmax=np.max(vel_interp)
    ),
    name='Velocity Profile',
    showlegend=True
))

# --- Build animation frames (with persistent waypoints) ---
frames = []
step_size = max(1, len(poses) // 250)

for i in range(1, len(poses), step_size):
    frame_data = []

    # Keep waypoints visible
    if "missions" in mission_data:
        mission_colors = ["green", "gold"]
        for m_idx, mission in enumerate(mission_data["missions"]):
            wps = np.array([wp["coords"] for wp in mission["waypoints"]])
            frame_data.append(go.Scatter3d(
                x=wps[:, 0], y=wps[:, 1], z=wps[:, 2],
                mode='markers',
                marker=dict(size=4, color=mission_colors[m_idx % len(mission_colors)]),
                name=f"Mission {m_idx + 1}",
                showlegend=False
            ))

    # Dynamic trail and current drone
    frame_data += [
        go.Scatter3d(
            x=poses[:i, 0], y=poses[:i, 1], z=poses[:i, 2],
            mode='lines',
            line=dict(color=vel_interp[:i], colorscale='Viridis', width=4),
            name='Trajectory',
            showlegend=False
        ),
        go.Scatter3d(
            x=[poses[i, 0]], y=[poses[i, 1]], z=[poses[i, 2]],
            mode='markers',
            marker=dict(size=4, color='red', symbol='circle'),
            name='Drone',
            showlegend=False
        )
    ]
    frames.append(go.Frame(data=frame_data, name=f"frame_{i}"))

# --- Add initial drone marker ---
fig.add_trace(go.Scatter3d(
    x=[poses[0, 0]], y=[poses[0, 1]], z=[poses[0, 2]],
    mode='markers',
    marker=dict(size=4, color='red', symbol='circle'),
    name='Drone'
))

# --- Layout and controls ---
fig.update(frames=frames)
fig.update_layout(
    scene=dict(
        xaxis_title="X Position (m)",
        yaxis_title="Y Position (m)",
        zaxis_title="Altitude (m)"
    ),
    title="3D Drone Trajectory with Unity Waypoints (Animated Trail)",
    updatemenus=[{
        "buttons": [
            {
                "args": [None, {"frame": {"duration": 120, "redraw": True},
                                "fromcurrent": True, "mode": "immediate"}],
                "label": "▶ Play",
                "method": "animate"
            },
            {
                "args": [[None], {"frame": {"duration": 0, "redraw": False},
                                  "mode": "immediate"}],
                "label": "⏸ Pause",
                "method": "animate"
            }
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

# --- Export and show ---
# fig.write_html("animated_trajectory.html")
fig.show()



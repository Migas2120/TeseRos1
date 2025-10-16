import numpy as np
import plotly.graph_objects as go

# === PARAMETERS ===
theta_deg = 265       # rotation between Unity frame and ENU (y-theta)
theta = np.deg2rad(theta_deg)

# === DEFINE CORE ELEMENTS ===
Uh = np.array([0, 0, 0])             # Unity Home
Drone_ENU = np.array([2, 0, 2])      # Drone position in ENU frame (world position)
Up = Drone_ENU + np.array([0, 0.1, 0])  # Unity marker 0.5 m above real drone

# Example waypoints in Unity's unaligned frame
waypoints_unity = np.array([
    [0.5, 0.1, 2.0],
    [2.0, 0.1, 1.5],
    [1.0, 0.1, 3.0]
])

# === ROTATE UNITY FRAME BY THETA TO ALIGN TO ENU ===
R = np.array([
    [np.cos(theta), 0, -np.sin(theta)],
    [0, 1, 0],
    [np.sin(theta), 0,  np.cos(theta)]
])

waypoints_enu = (waypoints_unity - Up) @ R.T + Drone_ENU

# === PLOTLY SCENE ===
fig = go.Figure()

# Unity origin
fig.add_trace(go.Scatter3d(
    x=[Uh[0]], y=[Uh[1]], z=[Uh[2]],
    mode='markers+text',
    text=["Unity Home"],
    textposition="bottom center",
    marker=dict(size=7, color='orange', symbol='x'),
    name="Unity Origin"
))

# Drone (ENU frame)
fig.add_trace(go.Scatter3d(
    x=[Drone_ENU[0]], y=[Drone_ENU[1]], z=[Drone_ENU[2]],
    mode='markers+text',
    text=["Drone (ENU)"],
    textposition="top center",
    marker=dict(size=9, color='red'),
    name="Drone (ENU)"
))

# Drone marker (Unity frame)
fig.add_trace(go.Scatter3d(
    x=[Up[0]], y=[Up[1]], z=[Up[2]],
    mode='markers+text',
    text=["Drone (Unity frame)"],
    textposition="top center",
    marker=dict(size=8, color='purple'),
    name="Drone (Unity)"
))

# Original Unity waypoints
fig.add_trace(go.Scatter3d(
    x=waypoints_unity[:, 0], y=waypoints_unity[:, 1], z=waypoints_unity[:, 2],
    mode='markers+text',
    text=[f"W{i}" for i in range(len(waypoints_unity))],
    textposition="top center",
    marker=dict(size=6, color='blue'),
    name="Unity Waypoints"
))

# Rotated (ENU-aligned) waypoints
fig.add_trace(go.Scatter3d(
    x=waypoints_enu[:, 0], y=waypoints_enu[:, 1], z=waypoints_enu[:, 2],
    mode='markers+text',
    text=[f"W{i}'" for i in range(len(waypoints_enu))],
    textposition="top center",
    marker=dict(size=6, color='green'),
    name="ENU Waypoints"
))

# Arrows showing rotation
for i in range(len(waypoints_unity)):
    fig.add_trace(go.Scatter3d(
        x=[Up[0], waypoints_unity[i, 0]],
        y=[Up[1], waypoints_unity[i, 1]],
        z=[Up[2], waypoints_unity[i, 2]],
        mode='lines',
        line=dict(color='blue', width=2),
        showlegend=False
    ))
    fig.add_trace(go.Scatter3d(
        x=[Drone_ENU[0], waypoints_enu[i, 0]],
        y=[Drone_ENU[1], waypoints_enu[i, 1]],
        z=[Drone_ENU[2], waypoints_enu[i, 2]],
        mode='lines',
        line=dict(color='green', width=2, dash='dot'),
        showlegend=False
    ))

# === FRAME AXES ===
def draw_axes(origin, R, name_prefix, color):
    length = 1.5
    axes = np.array([
        [length, 0, 0],
        [0, length, 0],
        [0, 0, length]
    ])
    labels = ['X', 'Y', 'Z']
    for i in range(3):
        end = origin + axes[i] @ R.T
        fig.add_trace(go.Scatter3d(
            x=[origin[0], end[0]],
            y=[origin[1], end[1]],
            z=[origin[2], end[2]],
            mode='lines+text',
            text=[None, f"{name_prefix}{labels[i]}"],
            textposition="top center",
            line=dict(color=color, width=4),
            showlegend=False
        ))

# Unity frame (before calibration)
draw_axes(Uh, np.eye(3), "Unity-", "blue")

# ENU frame (after calibration)
draw_axes(Drone_ENU, R, "ENU-", "green")

# === ARC SHOWING THETA ===
arc_radius = 1.0
arc_points = np.linspace(0, theta, 50)
arc_x = np.cos(arc_points) * arc_radius
arc_z = np.sin(arc_points) * arc_radius
fig.add_trace(go.Scatter3d(
    x=arc_x, y=np.zeros_like(arc_x), z=arc_z,
    mode='lines+text',
    text=[None, None, f"θ={theta_deg}°"],
    textposition="top center",
    line=dict(color='orange', width=3, dash='dot'),
    name="Rotation θ"
))

# === SCENE SETTINGS ===
fig.update_layout(
    title=f"Manual Calibration Visualization — θ = {theta_deg}° (Unity → ENU)",
    scene=dict(
        xaxis_title="East (X)",
        yaxis_title="Up (Y)",
        zaxis_title="North (Z)",
        aspectmode="cube"
    ),
    showlegend=True
)

fig.show()

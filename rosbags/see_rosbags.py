import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# --- Quaternion to Euler ---
def quat_to_euler(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.sign(sinp) * (np.pi/2) if abs(sinp) >= 1 else np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

# --- Load pose (altitude + XY) ---
pose_df = pd.read_csv("pose.csv")
pose_df["time_s"] = (pose_df["%time"].values - pose_df["%time"].values[0]) * 1e-9
altitude = pose_df["field.pose.position.z"].values
pose_time = pose_df["time_s"].values
x_pos = pose_df["field.pose.position.x"].values
y_pos = pose_df["field.pose.position.y"].values

# --- Load IMU (attitude) ---
imu_df = pd.read_csv("imu.csv")
imu_df["time_s"] = (imu_df["%time"].values - imu_df["%time"].values[0]) * 1e-9
rolls, pitches, yaws = [], [], []
for _, row in imu_df.iterrows():
    roll, pitch, yaw = quat_to_euler(
        row["field.orientation.x"],
        row["field.orientation.y"],
        row["field.orientation.z"],
        row["field.orientation.w"]
    )
    rolls.append(np.degrees(roll))
    pitches.append(np.degrees(pitch))
    yaws.append(np.degrees(yaw))
imu_time = imu_df["time_s"].values

# --- Load velocity ---
vel_df = pd.read_csv("velocity.csv")
vel_df["time_s"] = (vel_df["%time"].values - vel_df["%time"].values[0]) * 1e-9
vx = vel_df["field.twist.linear.x"].values
vy = vel_df["field.twist.linear.y"].values
vz = vel_df["field.twist.linear.z"].values
vel_time = vel_df["time_s"].values

# --- Load battery ---
batt_df = pd.read_csv("battery.csv")
batt_df["time_s"] = (batt_df["%time"].values - batt_df["%time"].values[0]) * 1e-9
voltage = batt_df["field.voltage"].values
batt_time = batt_df["time_s"].values

# --- Create Multi-Panel Flight Dashboard ---
fig, axs = plt.subplots(4, 1, figsize=(12, 12), sharex=True)

# Altitude
axs[0].plot(pose_time, altitude, label="Altitude (m)", color="blue")
axs[0].set_ylabel("Altitude (m)")
axs[0].set_title("Drone Flight Analysis")
axs[0].grid(True)
axs[0].legend()

# Attitude
axs[1].plot(imu_time, rolls, label="Roll (°)")
axs[1].plot(imu_time, pitches, label="Pitch (°)")
axs[1].plot(imu_time, yaws, label="Yaw (°)")
axs[1].set_ylabel("Angle (°)")
axs[1].grid(True)
axs[1].legend()

# Velocity
axs[2].plot(vel_time, vx, label="Vx")
axs[2].plot(vel_time, vy, label="Vy")
axs[2].plot(vel_time, vz, label="Vz")
axs[2].set_ylabel("Velocity (m/s)")
axs[2].grid(True)
axs[2].legend()

# Battery
axs[3].plot(batt_time, voltage, label="Voltage (V)", color="green")
axs[3].set_xlabel("Time (s)")
axs[3].set_ylabel("Voltage (V)")
axs[3].grid(True)
axs[3].legend()

plt.tight_layout()
plt.show()

# --- XY Trajectory Plot ---
plt.figure(figsize=(8, 8))
plt.plot(x_pos, y_pos, label="Trajectory")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Drone XY Ground Trajectory")
plt.axis("equal")  # Keep scale equal for X/Y
plt.grid(True)
plt.legend()
plt.show()

from mpl_toolkits.mplot3d import Axes3D  # needed for 3D projection

# --- 3D Trajectory Plot ---
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection="3d")

ax.plot(x_pos, y_pos, altitude, label="3D Trajectory", color="blue")

ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.set_zlabel("Altitude (m)")
ax.set_title("Drone 3D Flight Trajectory")
ax.legend()
plt.show()


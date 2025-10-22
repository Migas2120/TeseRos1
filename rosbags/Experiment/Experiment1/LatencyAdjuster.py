import json
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.distance import cdist
import os

# -------------------- CONFIG --------------------
BAG_PATH = "Experiment1.bag"
LATENCY_FILE = "latency_20251018_115727.jsonl"
MISSION_FILE = "parsed_missions.json"
POSE_TOPIC = "/mavros/local_position/pose"
VEL_TOPIC = "/mavros/local_position/velocity_local"
EPSILON = 0.15  # meters threshold to consider waypoint reached
SAVE_DIR = "latency_analysis_pos"
os.makedirs(SAVE_DIR, exist_ok=True)
# ------------------------------------------------


def load_latency_data(filepath):
    """Load JSON lines latency file."""
    entries = []
    with open(filepath, "r") as f:
        for line in f:
            try:
                entries.append(json.loads(line.strip()))
            except Exception:
                continue
    return entries


def load_waypoints(filepath):
    """Parse mission file for waypoints (supports both 'coords' and x/y/z)."""
    with open(filepath, "r") as f:
        data = json.load(f)

    waypoints = []
    if "missions" in data:
        for mission in data["missions"]:
            # Use 'waypoints' if present, otherwise fallback to 'points'
            for wp in mission.get("waypoints", mission.get("points", [])):
                if "coords" in wp:
                    waypoints.append(np.array(wp["coords"]))
                elif all(k in wp for k in ("x", "y", "z")):
                    waypoints.append(np.array([wp["x"], wp["y"], wp["z"]]))
    elif "waypoints" in data:
        for wp in data["waypoints"]:
            if "coords" in wp:
                waypoints.append(np.array(wp["coords"]))
            elif all(k in wp for k in ("x", "y", "z")):
                waypoints.append(np.array([wp["x"], wp["y"], wp["z"]]))

    return np.array(waypoints)

def read_ros_bag(bag_path):
    """Extract poses and velocities from the ROS bag."""
    poses, times, vel, veltime = [], [], [], []
    bag = rosbag.Bag(bag_path)

    for _, msg, t in bag.read_messages(topics=[POSE_TOPIC]):
        poses.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        times.append(t.to_sec())

    for _, msg, t in bag.read_messages(topics=[VEL_TOPIC]):
        vx, vy, vz = msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z
        vel.append(np.sqrt(vx**2 + vy**2 + vz**2))
        veltime.append(t.to_sec())

    bag.close()
    poses = np.array(poses)
    times = np.array(times)
    vel = np.array(vel)
    veltime = np.array(veltime)

    # interpolate velocity to pose timestamps
    vel_interp = np.interp(times, veltime, vel, left=vel[0], right=vel[-1])
    return poses, times, vel_interp


def find_time_near_waypoint(poses, times, wp, eps=0.15):
    """Find the first timestamp where the drone is within epsilon of a waypoint."""
    dists = np.linalg.norm(poses - wp, axis=1)
    idx = np.where(dists <= eps)[0]
    if len(idx) > 0:
        return times[idx[0]], idx[0]
    return None, None


# ---------------- MAIN PROCESS ----------------

print("[INFO] Loading data...")
latency_data = load_latency_data(LATENCY_FILE)
waypoints = load_waypoints(MISSION_FILE)
poses, times, vel_interp = read_ros_bag(BAG_PATH)
print(f"[INFO] Loaded {len(latency_data)} latency entries and {len(waypoints)} waypoints")

# Filter pos / pos_raw
pos_data = [e for e in latency_data if e["command"] == "pos"]
pos_raw_data = [e for e in latency_data if e["command"] == "pos_raw"]

# sort by timestamp
pos_data.sort(key=lambda x: x["timestamp"])
pos_raw_data.sort(key=lambda x: x["timestamp"])

# ---------------- ANALYSIS ----------------

results = []
for i, wp in enumerate(waypoints[:len(pos_raw_data)]):
    t_reach, idx_reach = find_time_near_waypoint(poses, times, wp, eps=EPSILON)
    if t_reach is None:
        continue

    # backtrack small displacement
    back_idx = max(0, idx_reach - 10)
    dist = np.linalg.norm(poses[idx_reach] - poses[back_idx])
    v_local = np.mean(vel_interp[back_idx:idx_reach + 1]) if dist > 0 else 0

    measured_latency = (dist / max(v_local, 1e-3)) * 1000  # ms
    logged_latency = pos_raw_data[i]["latency_ms"]

    results.append({
        "waypoint": i + 1,
        "logged_latency_ms": logged_latency,
        "measured_latency_ms": measured_latency,
        "velocity_m_s": v_local,
        "distance_m": dist
    })

# ---------------- VISUALIZATION ----------------

if len(results) > 0:
    logged = [r["logged_latency_ms"] for r in results]
    measured = [r["measured_latency_ms"] for r in results]
    velocity = [r["velocity_m_s"] for r in results]
    wp_idx = [r["waypoint"] for r in results]

    # 1. Logged vs Measured Latency
    plt.figure(figsize=(8, 5))
    plt.plot(wp_idx, logged, label="Logged Latency (pos_raw)", marker="o")
    plt.plot(wp_idx, measured, label="Measured Latency (Pose)", marker="s")
    plt.xlabel("Waypoint Index")
    plt.ylabel("Latency (ms)")
    plt.title("Logged vs Measured Command Latency per Waypoint")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(SAVE_DIR, "latency_comparison.png"), dpi=300)

    # 2. Latency vs Velocity
    plt.figure(figsize=(7, 5))
    plt.scatter(velocity, logged, c="tab:blue", label="Logged")
    plt.scatter(velocity, measured, c="tab:orange", label="Measured")
    plt.xlabel("Average Local Velocity (m/s)")
    plt.ylabel("Latency (ms)")
    plt.title("Latency vs Velocity Relationship")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(SAVE_DIR, "latency_vs_velocity.png"), dpi=300)

    # Summary statistics
    mean_logged = np.mean(logged)
    mean_measured = np.mean(measured)
    print(f"\n[RESULTS SUMMARY]")
    print(f"  Avg Logged Latency:   {mean_logged:.2f} ms")
    print(f"  Avg Measured Latency: {mean_measured:.2f} ms")
    print(f"  Difference:           {mean_measured - mean_logged:.2f} ms\n")

else:
    print("[WARN] No valid results found for waypoint correlation.")

plt.show()

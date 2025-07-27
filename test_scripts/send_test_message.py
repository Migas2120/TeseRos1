import socket
import json
import time
import math

def send(sock, payload, delay=2):
    print(f"\n[Test] Sending:\n{json.dumps(payload, indent=2)}")
    sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
    time.sleep(delay)

def polygon_cube(cx, cy, size, z_min=5.0, z_max=15.0):
    """
    Returns the 8 corner points of a cube centered at (cx, cy),
    with horizontal edge length `size` and vertical extent from z_min to z_max.
    """
    half = size / 2
    xs = [cx - half, cx + half]
    ys = [cy - half, cy + half]
    zs = [z_min, z_max]
    corners = []
    for x in xs:
        for y in ys:
            for z in zs:
                corners.append({"x": x, "y": y, "z": z})
    return corners

def create_map_area_payload(
    mission_id,
    center,
    size,
    priority=5,
    preempt=False,
    base_altitude=5.0,
    height=10.0,
    vertical_step=1.0,
    grid_spacing=2.0,
):
    cx, cy = center
    points = polygon_cube(cx, cy, size, z_min=base_altitude, z_max=base_altitude + height)
    return {
        "type":           "map_area",
        "mission_id":     mission_id,
        "points":         points,
        "vertical_step":  vertical_step,
        "grid_spacing":   grid_spacing,
        "priority":       priority,
        "preempt":        preempt
    }

def run_test_sequence(sock):
    # Just send one map_area mission with a cube region
    payload = create_map_area_payload(
        mission_id="test_map_area_cube",
        center=(0.0, 0.0),
        size=50.0,
        priority=5,
        preempt=False,
        base_altitude=5.0,
        height=20.0,        # cube height
        vertical_step=1.0,  # small vertical steps
        grid_spacing=2.0    # dense grid → lots of waypoints
    )
    send(sock, payload, delay=1)

    print("\n[Test] Payload sent. Now waiting for DroneInstance to process…\n")
    # Give it plenty of time to slice & start flying (and drain battery)
    time.sleep(120)  # two minutes; adjust as needed

def main():
    host, port = "127.0.0.1", 65432
    try:
        print(f"[Test] Connecting to {host}:{port}...")
        with socket.create_connection((host, port), timeout=5) as sock:
            print("[Test] Connected.")
            run_test_sequence(sock)
    except Exception as e:
        print(f"[Test] Error: {e}")

if __name__ == "__main__":
    main()

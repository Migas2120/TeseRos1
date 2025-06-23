import socket
import json
import time
import math


def send(sock, payload, delay=2):
    print(f"\n[Test] Sending:\n{json.dumps(payload, indent=2)}")
    sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))  # newline for TCP framing
    time.sleep(delay)


def polygon_square(cx, cy, size):
    half = size / 2
    return [
        [cx - half, cy - half],
        [cx + half, cy - half],
        [cx + half, cy + half],
        [cx - half, cy + half]
    ]


def create_map_area_payload(mission_id, center, size, priority, preempt=False):
    return {
        "type": "map_area",
        "mission_id": mission_id,
        "points": polygon_square(*center, size),
        "altitude": 8.0,
        "grid_spacing": 4.0,
        "priority": priority,
        "preempt": preempt
    }


def create_cylinder_inspection_payload(mission_id, center, radius, height, loops, priority, preempt=False):
    cx, cy = center
    waypoints = []

    steps = loops * 10
    for i in range(steps):
        angle = 2 * math.pi * i / 10
        z = (i / steps) * height + 5.0
        x = cx + radius * math.cos(angle)
        y = cy + radius * math.sin(angle)
        waypoints.append([x, y, z])

    return {
        "type": "add_mission",
        "mission_id": mission_id,
        "waypoints": waypoints,
        "mode": "once",
        "priority": priority,
        "preempt": preempt
    }


def run_test_sequence(sock):
    send(sock, {"type": "command", "command": "arm", "id": 0})
    send(sock, {"type": "command", "command": "mode", "mode": "GUIDED", "id": 0})
    send(sock, {"type": "command", "command": "takeoff", "altitude": 5.0, "id": 0}, delay=8)

    # Mission A: bottom-left corner
    send(sock, create_map_area_payload(
        mission_id="zone_a_mapping",
        center=(10, 10),
        size=20,
        priority=7,
        preempt=False
    ))
    time.sleep(15)

    # Preempting Mission B: tower inspection
    send(sock, create_cylinder_inspection_payload(
        mission_id="tower_inspection",
        center=(50, 50),
        radius=6,
        height=25,
        loops=2,
        priority=1,
        preempt=True
    ))
    time.sleep(30)

    # Mission C: top-left corner
    send(sock, create_map_area_payload(
        mission_id="zone_c_mapping",
        center=(10, 90),
        size=20,
        priority=6,
        preempt=False
    ))
    time.sleep(20)

    # Mission D: bottom-right corner
    send(sock, create_map_area_payload(
        mission_id="zone_d_mapping",
        center=(90, 10),
        size=20,
        priority=5,
        preempt=True
    ))
    time.sleep(20)

    send(sock, {
        "type": "command",
        "command": "land",
        "id": 0,
        "altitude": 5.0,
        "latitude": 0.0,
        "longitude": 0.0
    })


def main():
    host = "127.0.0.1"
    port = 65432

    try:
        print(f"[Test] Connecting to {host}:{port}...")
        with socket.create_connection((host, port), timeout=5) as sock:
            print("[Test] Connected.")
            run_test_sequence(sock)
            print("\n[Test] Full multi-mission test completed.\n")
    except Exception as e:
        print(f"[Test] Error: {e}")


if __name__ == "__main__":
    main()

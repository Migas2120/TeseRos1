import socket
import json
import time
import math


def send(sock, command, delay=2):
    print(f"[Test] Sending:\n{json.dumps(command, indent=2)}")
    sock.sendall(json.dumps(command).encode('utf-8'))
    time.sleep(delay)


def create_circle_mission(mission_id="basic_test_mission", altitude=10.0):
    cx, cy = 25, 25
    r = 15
    waypoints = [
        [cx + r * math.cos(a), cy + r * math.sin(a), altitude]
        for a in [i * math.pi / 4 for i in range(8)]
    ]
    return {
        "type": "add_mission",
        "mission_id": mission_id,
        "waypoints": waypoints,
        "mode": "once"
    }


def main():
    host = "127.0.0.1"
    port = 65432

    try:
        print(f"[Test] Connecting to {host}:{port}...")
        with socket.create_connection((host, port), timeout=5) as sock:
            print("[Test] Connected.")

            # Step 1: Arm
            send(sock, {"type": "command", "command": "arm", "id": 0})

            # Step 2: Set mode GUIDED
            send(sock, {"type": "command", "command": "mode", "mode": "GUIDED", "id": 0})

            # Step 3: Takeoff
            send(sock, {"type": "command", "command": "takeoff", "altitude": 5.0, "id": 0}, delay=10)

            # Step 4: Add a mission
            mission = create_circle_mission()
            send(sock, mission)

            # Step 5: Skip current waypoint
            send(sock, {"type": "command", "command": "skip_wp"}, delay=1)

            # Step 6: Add a new waypoint at index 2
            send(sock, {
                "type": "edit_mission",
                "action": "add",
                "index": 2,
                "waypoint": [10.0, 10.0, 10.0]
            })

            # Step 7: Move waypoint at index 3
            send(sock, {
                "type": "edit_mission",
                "action": "move",
                "index": 3,
                "waypoint": [15.0, 15.0, 10.0]
            })

            # Step 8: Remove waypoint at index 4
            send(sock, {
                "type": "edit_mission",
                "action": "remove",
                "index": 4
            })

            # Step 9: Abort mission
            send(sock, {"type": "command", "command": "abort", "id": 0})

            # Step 10: Resume mission execution
            send(sock, {"type": "resume_missions"})

            # Step 11: Land
            send(sock, {
                "type": "command",
                "command": "land",
                "id": 0,
                "altitude": 5.0,
                "latitude": 0.0,
                "longitude": 0.0
            })

            print("[Test] Basic functionality test completed.")

    except Exception as e:
        print(f"[Test] Error: {e}")


if __name__ == "__main__":
    main()

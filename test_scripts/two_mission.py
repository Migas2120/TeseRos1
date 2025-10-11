import socket, json, time, math

HOST, PORT = "127.0.0.1", 65432

def send(sock, payload):
    msg = json.dumps(payload) + "\n"
    sock.sendall(msg.encode("utf-8"))
    print(f"[Client] Sent mission {payload['mission_id']} (preempt={payload.get('preempt', False)})")

def generate_path(center, spacing=5.0, altitude=5.0, count=5, angle_deg=0.0):
    waypoints = []
    angle = math.radians(angle_deg)
    dx = math.cos(angle) * spacing
    dy = math.sin(angle) * spacing

    for i in range(count):
        x = center[0] + i * dx
        y = center[1] + i * dy
        z = altitude
        waypoints.append({
            "coords": [x, y, z],
            "hold_duration": 0
        })
    return waypoints

def create_path_payload(mission_id, center, spacing=5.0, altitude=5.0, count=5,
                        angle_deg=0.0, preempt=False, priority=5):
    return {
        "type": "add_mission",
        "mission_id": mission_id,
        "waypoints": generate_path(center, spacing, altitude, count, angle_deg),
        "priority": priority,
        "mode": "once",
        "patrol_loops": 2,
        "preempt": preempt
    }

def main():
    try:
        with socket.create_connection((HOST, PORT), timeout=5) as s:
            print("[Client] Connected")

            # First mission: straight line path
            mission1 = create_path_payload(
                mission_id="path_mission_1",
                center=(0.0, 0.0),
                spacing=4.0,
                altitude=4.0,
                count=5
            )
            send(s, mission1)

            # Wait to let mission 1 start executing
            time.sleep(30)

            # Second mission: diagonal path, preempts first
            mission2 = create_path_payload(
                mission_id="path_mission_2_preempt",
                center=(5.0, 5.0),
                spacing=4.0,
                altitude=4.0,
                count=20,
                angle_deg=45.0,
                preempt=True
            )
            send(s, mission2)

            # Wait to allow server to process missions
            time.sleep(200)

            # Read server responses
            buf = b""
            while True:
                chunk = s.recv(4096)
                if not chunk:
                    break
                buf += chunk
                *lines, buf = buf.split(b"\n")
                for ln in lines:
                    if ln.strip():
                        print(f"[Server] {ln.decode('utf-8')}")

    except Exception as e:
        print(f"[Client] Error: {e}")

if __name__ == "__main__":
    main()

# single_drone_send.py
import socket, json, time

HOST, PORT = "127.0.0.1", 65432

def send(sock, payload):
    msg = json.dumps(payload) + "\n"
    sock.sendall(msg.encode("utf-8"))
    print(f"[Client] Sent mission {payload['mission_id']} (preempt={payload['preempt']})")

def cube_unity_points(cx, cy, base_alt, size):
    """
    Returns 8 corners of a cube (Unity Vector3 serialization).
    """
    half = size / 2.0
    pts = []
    for dx in [-half, half]:
        for dy in [-half, half]:
            for dz in [0, size]:
                pts.append({
                    "x": cx + dx,
                    "y": cy + dy,
                    "z": base_alt + dz
                })
    return pts

def create_cube_payload(mission_id, center, base_alt, size,
                        priority=5, preempt=False):
    cx, cy = center
    points = cube_unity_points(cx, cy, base_alt, size)
    return {
        "type": "map_area",
        "mission_id": mission_id,
        "points": points,
        "priority": priority,
        "vertical_step": 4.0,
        "grid_spacing": 2.0,
        "preempt": preempt,
    }

def main():
    try:
        with socket.create_connection((HOST, PORT), timeout=5) as s:
            print("[Client] Connected")

            # First mission
            payload1 = create_cube_payload(
                mission_id="cube_demo",
                center=(0.0, 0.0),
                base_alt=2.0,
                size=10.0
            )
            send(s, payload1)

            # Small pause so the server processes the first
            time.sleep(40)

            # Second mission: same cube, shifted center, preempt enabled
            payload2 = create_cube_payload(
                mission_id="cube_demo_preempt",
                center=(8.0, 8.0),   # shifted away from origin
                base_alt=2.0,
                size=10.0,
                preempt=True
            )
            send(s, payload2)

            time.sleep(200)

            # Read server responses if any
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

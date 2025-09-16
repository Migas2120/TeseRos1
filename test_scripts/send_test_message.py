# multi_drone_send.py
import socket, json, time, threading, argparse, math

HOST, PORT = "127.0.0.1", 65432

def send(sock, payload):
    msg = json.dumps(payload) + "\n"
    sock.sendall(msg.encode("utf-8"))

def polygon_cylinder_unity_points(cx, cy, radius, z_min=5.0, z_max=15.0, segments=32):
    """
    Approximates a vertical cylinder by sampling 'segments' points
    on the bottom (z_min) and top (z_max) circles.

    Unity-style Vector3 serialization expected by your server:
      Vector3(x, z_alt, y_horiz)  ->  {x:x, y:z_alt, z:y_horiz}
    """
    pts = []
    for k in range(segments):
        th = 2.0 * math.pi * k / segments
        x = cx + radius * math.cos(th)
        y_horiz = cy + radius * math.sin(th)
        # bottom ring
        pts.append({"x": x, "y": z_min, "z": y_horiz})
        # top ring
        pts.append({"x": x, "y": z_max, "z": y_horiz})
    return pts

def create_map_area_payload(mission_id, center, radius, base_alt, height, vstep, gspace,
                            priority=5, preempt=False, segments=32):
    cx, cy = center
    points = polygon_cylinder_unity_points(
        cx, cy, radius, z_min=base_alt, z_max=base_alt + height, segments=segments
    )
    return {
        "type": "map_area",
        "mission_id": mission_id,
        "points": points,
        "vertical_step": vstep,
        "grid_spacing": gspace,
        "priority": priority,
        "preempt": preempt,
    }

def drone_thread(name, payload, host, port, speed=None):
    try:
        with socket.create_connection((host, port), timeout=5) as s:
            hello = s.recv(4096).decode("utf-8").strip()
            print(f"[{name}] {hello}")
            if speed is not None:
                send(s, {"type":"set_params","speed":float(speed)})
            send(s, payload)

            # read server responses (acks, world_state, collision_alerts)
            end = time.time() + 500  # run output for ~60s, adjust as needed
            buf = b""
            while time.time() < end:
                chunk = s.recv(4096)
                if not chunk:
                    break
                buf += chunk
                *lines, buf = buf.split(b"\n")
                for ln in lines:
                    if ln.strip():
                        print(f"[{name}][Server] {ln.decode('utf-8')}")
    except Exception as e:
        print(f"[{name}] Error: {e}")

def main():
    ap = argparse.ArgumentParser()
    # geometry / mission
    ap.add_argument("--sep", type=float, default=10.0, help="center separation on Y axis (m)")
    ap.add_argument("--radius", type=float, default=10.0, help="cylinder radius (m)")
    ap.add_argument("--base", type=float, default=0.0, help="base altitude (m)")
    ap.add_argument("--height", type=float, default=12.0, help="vertical extent (m)")
    ap.add_argument("--vstep", type=float, default=2.0, help="vertical step (m)")
    ap.add_argument("--gspace", type=float, default=3.0, help="grid spacing (m)")
    ap.add_argument("--segments", type=int, default=32, help="circle polygon segments")
    # speeds
    ap.add_argument("--speed1", type=float, default=2.0)
    ap.add_argument("--speed2", type=float, default=2.5)
    # connection
    ap.add_argument("--host", type=str, default=HOST)
    ap.add_argument("--port", type=int, default=PORT)
    args = ap.parse_args()

    # two cylinders, offset on Y so you can create overlap by using small --sep
    p1 = create_map_area_payload(
        mission_id="cyl_d1", center=(0.0, -args.sep/2), radius=args.radius,
        base_alt=args.base, height=args.height, vstep=args.vstep, gspace=args.gspace,
        segments=args.segments
    )
    p2 = create_map_area_payload(
        mission_id="cyl_d2", center=(0.0,  args.sep/2), radius=args.radius,
        base_alt=args.base, height=args.height, vstep=args.vstep, gspace=args.gspace,
        segments=args.segments
    )

    t1 = threading.Thread(target=drone_thread, args=("D1", p1, args.host, args.port, args.speed1))
    t2 = threading.Thread(target=drone_thread, args=("D2", p2, args.host, args.port, args.speed2))
    t1.start(); t2.start()
    t1.join(); t2.join()

if __name__ == "__main__":
    main()

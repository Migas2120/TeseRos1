import socket
import json
import time
import math

HOST, PORT = "127.0.0.1", 65432


def send(sock, payload):
    msg = json.dumps(payload) + "\n"
    sock.sendall(msg.encode("utf-8"))
    print(f"[Client] Sent: {payload['type']} ({payload.get('mission_id', payload.get('command', ''))})")


# ---------------------------------------------------------------------
#   REGION GENERATORS
# ---------------------------------------------------------------------

def generate_cube(center=(10, 10, 10), size=10.0):
    """Generates 8 corner vertices of a cube centered at (cx, cy, cz)."""
    cx, cy, cz = center
    half = size / 2.0
    cube_points = []
    for dx in [-half, half]:
        for dy in [-half, half]:
            for dz in [-half, half]:
                cube_points.append({"x": cx + dx, "y": cy + dy, "z": cz + dz})
    return cube_points


def generate_recon_path(center=(-10, -10, 5), size=10.0):
    """Simple square pattern for recon testing."""
    cx, cy, cz = center
    half = size / 2.0
    return [
        [cx - half, cy - half, cz],
        [cx + half, cy - half, cz + 1],
        [cx + half, cy + half, cz + 1],
        [cx - half, cy + half, cz]
    ]


# ---------------------------------------------------------------------
#   MAIN MENU
# ---------------------------------------------------------------------

def show_menu():
    print("\n===== INTEGRATED TEST CLIENT =====")
    print("1. Send 3D Map Area (Cube Region)")
    print("2. Add Recon Mission (Preempt)")
    print("3. Edit Mission (add/move/remove)")
    print("4. Change Priority")
    print("5. Skip Waypoint")
    print("6. Resume Mission")
    print("7. Abort Mission")
    print("8. Return to Base")
    print("9. Toggle Manual Mode (enter/exit)")
    print("10. Send Manual Stick Command")
    print("11. List Missions")
    print("0. Exit")
    print("==================================")
    return input("Choose an action: ")


# ---------------------------------------------------------------------
#   MAIN LOOP
# ---------------------------------------------------------------------

def main():
    missions = []  # track sent missions: [{"id": "...", "type": "...", "priority": ...}]

    try:
        with socket.create_connection((HOST, PORT), timeout=5) as s:
            print(f"[Client] Connected to MiddleMan at {HOST}:{PORT}\n")

            manual_mode = False

            while True:
                choice = show_menu()

                # === 1. MAP AREA MISSION ===
                if choice == "1":
                    region = generate_cube(center=(10, 10, 10), size=10)
                    mission_id = f"map_area_cube_{len(missions) + 1}"

                    # Ask for preemption option
                    preempt_input = input("Enable preemption? (y/N): ").strip().lower()
                    preempt_flag = preempt_input == "y"

                    payload = {
                        "type": "map_area",
                        "mission_id": mission_id,
                        "points": region,
                        "grid_spacing": 3.0,
                        "vertical_step": 2.0,
                        "priority": 80,
                        "preempt": preempt_flag
                    }

                    send(s, payload)
                    missions.append({"id": mission_id, "type": "map_area", "priority": 80, "preempt": preempt_flag})


                # === 2. RECON MISSION ===
                elif choice == "2":
                    mission_id = f"recon_task_{len(missions) + 1}"
                    waypoints = generate_recon_path(center=(-10, -10, 5), size=10)

                    # Ask for preemption option
                    preempt_input = input("Enable preemption? (y/N): ").strip().lower()
                    preempt_flag = preempt_input == "y"

                    payload = {
                        "type": "add_mission",
                        "mission_id": mission_id,
                        "waypoints": waypoints,
                        "mode": "once",
                        "priority": 95,
                        "preempt": preempt_flag
                    }

                    send(s, payload)
                    missions.append({"id": mission_id, "type": "recon", "priority": 95, "preempt": preempt_flag})

                # === 3. EDIT MISSION ===
                elif choice == "3":
                    print("\n[Edit Mission]")
                    if not missions:
                        print("No missions stored yet.")
                        continue

                    # Show available missions
                    for i, m in enumerate(missions):
                        print(f"{i + 1}. {m['id']} ({m['type']}, priority={m['priority']})")

                    sel = input("Select mission by number (or press Enter for active): ").strip()
                    mission_id = None
                    if sel:
                        try:
                            idx = int(sel) - 1
                            if 0 <= idx < len(missions):
                                mission_id = missions[idx]["id"]
                        except ValueError:
                            print("Invalid number. Using active mission.")

                    action = input("Action (add/move/remove): ").strip()
                    index = int(input("Index: "))
                    wp = None
                    if action in ["add", "move"]:
                        x = float(input("x: "))
                        y = float(input("y: "))
                        z = float(input("z: "))
                        wp = [x, y, z]

                    payload = {
                        "type": "edit_mission",
                        "mission_id": mission_id,
                        "action": action,
                        "index": index
                    }
                    if wp:
                        payload["waypoint"] = wp
                    send(s, payload)

                # === 4. CHANGE PRIORITY ===
                elif choice == "4":
                    if not missions:
                        print("No missions stored yet.")
                        continue
                    for i, m in enumerate(missions):
                        print(f"{i + 1}. {m['id']} ({m['type']}, priority={m['priority']})")

                    idx = int(input("Select mission number: ")) - 1
                    if 0 <= idx < len(missions):
                        new_pri = int(input("New priority: "))
                        missions[idx]["priority"] = new_pri
                        payload = {
                            "type": "change_priority",
                            "mission_id": missions[idx]["id"],
                            "priority": new_pri
                        }
                        send(s, payload)
                    else:
                        print("Invalid selection.")

                elif choice == "5":
                    send(s, {"type": "command", "command": "skip_wp"})

                elif choice == "6":
                    send(s, {"type": "resume_missions"})

                elif choice == "7":
                    send(s, {"type": "command", "command": "abort"})

                elif choice == "8":
                    send(s, {"type": "command", "command": "RTL"})

                elif choice == "9":
                    manual_mode = not manual_mode
                    command = "start_manual" if manual_mode else "stop_manual"
                    payload = {"type": "command", "command": command}
                    send(s, payload)
                    print(f"{'Entered' if manual_mode else 'Exited'} manual mode")

                elif choice == "10":
                    if not manual_mode:
                        print("Enable manual mode first (option 9).")
                        continue

                    print("\n[Manual Control] Enter joystick inputs:")
                    axes = {
                        "ly": float(input("forward/back: ")),
                        "lx": float(input("left/right: ")),
                        "ry": float(input("up/down: ")),
                        "rx": float(input("yaw: "))
                    }
                    payload = {"type": "manual_control", "axes": axes}
                    send(s, payload)

                # === 11. LIST MISSIONS ===
                elif choice == "11":
                    if not missions:
                        print("No missions stored yet.")
                    else:
                        print("\n--- Current Missions ---")
                        for i, m in enumerate(missions):
                            print(f"{i + 1}. {m['id']} ({m['type']}, priority={m['priority']})")

                elif choice == "0":
                    print("Exiting.")
                    break

                else:
                    print("Invalid choice.")

                # --- Non-blocking receive for logs ---
                time.sleep(0.3)
                s.settimeout(0.1)
                try:
                    data = s.recv(4096)
                    if data:
                        for ln in data.split(b"\n"):
                            if ln.strip():
                                print(f"[Server] {ln.decode('utf-8')}")
                except socket.timeout:
                    pass

    except Exception as e:
        print(f"[Client] Error: {e}")


if __name__ == "__main__":
    main()

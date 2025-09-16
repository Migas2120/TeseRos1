# plot_multi.py
import socket, json, time, argparse
from datetime import datetime
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

HOST, PORT = "127.0.0.1", 65432
N_SAMPLES = 300000

def send(sock, obj):
    sock.sendall((json.dumps(obj) + "\n").encode("utf-8"))

def read_lines(sock, timeout=5.0):
    sock.settimeout(timeout)
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf += chunk
            *lines, buf = buf.split(b"\n")
            for ln in lines:
                if ln.strip():
                    yield ln.decode("utf-8")
        except Exception:
            break

def fetch_and_plot(drone_ids):
    colors = {}
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    with socket.create_connection((HOST, PORT), timeout=5) as s:
        hello = s.recv(4096).decode("utf-8").strip()
        print("[Server]", hello)

        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        # Dump CSVs for all requested drones
        for did in drone_ids:
            send(s, {"type":"dump_history_csv","drone_id": did, "path": f"run_{ts}_drone_{did}.csv"})
            for line in read_lines(s, timeout=2.0):
                msg = json.loads(line)
                if msg.get("type") == "ack" and msg.get("req") == "dump_history_csv":
                    print(f"[CSV] {msg.get('path')} rows={msg.get('rows')} (drone {msg.get('drone_id')})")
                    break
                if msg.get("type") == "error":
                    print("[Server ERROR]", msg.get("error"))
                    break

        # Fetch and plot
        for did in drone_ids:
            send(s, {"type":"get_history","drone_id": did, "n": N_SAMPLES})
            xs, ys, zs = [], [], []
            for line in read_lines(s, timeout=3.0):
                msg = json.loads(line)
                if msg.get("type") == "history" and msg.get("drone_id") == did:
                    samples = msg.get("samples", [])
                    print(f"[History] drone {did}: {len(samples)} samples")
                    for p in samples:
                        xs.append(p["x"]); ys.append(p["y"]); zs.append(p["z"])
                    break
                if msg.get("type") == "error":
                    print("[Server ERROR]", msg.get("error"))
                    break

            if xs:
                ax.plot(xs, ys, zs, label=f"Drone {did}")

    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title("Multi-Drone Paths")
    ax.legend()
    plt.show()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--drones", type=int, nargs="+", default=[1,2], help="drone IDs to plot")
    args = ap.parse_args()
    fetch_and_plot(args.drones)

if __name__ == "__main__":
    main()

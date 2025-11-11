

import socket
import json
import time
import threading
import datetime
import logging
import os

# ======================================================
# CONFIG
# ======================================================
CALLBACK_HOST = "0.0.0.0"
CALLBACK_PORT = 12345  # local telemetry server (we receive here)
REMOTE_HOST   = "127.0.0.1"
REMOTE_PORT   = 65432  # drone-side middleman (we send missions to here)

# ======================================================
# LOGGER SETUP
# ======================================================
logger = logging.getLogger("TCPBidirectional")
logger.setLevel(logging.DEBUG)
ch = logging.StreamHandler()
ch.setFormatter(logging.Formatter("%(asctime)s | %(levelname)s | %(message)s"))
logger.addHandler(ch)

# ======================================================
# THROUGHPUT MONITOR
# ======================================================
class ThroughputMonitor:
    def __init__(self, logger, log_file=None):
        self.logger = logger
        self.rx_bytes = 0
        self.tx_bytes = 0
        self.last_time = time.time()
        self.running = True
        self._lock = threading.Lock()
        self.log_file = log_file

    def add_rx(self, n):
        with self._lock:
            self.rx_bytes += n

    def add_tx(self, n):
        with self._lock:
            self.tx_bytes += n

    def start(self):
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            time.sleep(1.0)
            with self._lock:
                now = time.time()
                elapsed = now - self.last_time
                if elapsed <= 0:
                    continue

                rx_kbps = (self.rx_bytes * 8 / 1000) / elapsed
                tx_kbps = (self.tx_bytes * 8 / 1000) / elapsed

                self.logger.info(f"[Throughput] RX={rx_kbps:.2f} kbps | TX={tx_kbps:.2f} kbps")

                if self.log_file:
                    try:
                        with open(self.log_file, "a") as f:
                            entry = {
                                "timestamp": time.time(),
                                "rx_kbps": rx_kbps,
                                "tx_kbps": tx_kbps
                            }
                            f.write(json.dumps(entry) + "\n")
                    except Exception as e:
                        self.logger.debug(f"[Throughput] Failed to write log: {e}")

                self.rx_bytes = 0
                self.tx_bytes = 0
                self.last_time = now

    def stop(self):
        self.running = False



# ======================================================
# MISSIONS
# ======================================================
mission_add = {
    "type": "add_mission",
    "mission_id": "Mission_3070",
    "waypoints": [
        {"coords": [0.9739, 9.8443, 0.5266]},
        {"coords": [-5.3206, 13.3277, 4.9848]},
        {"coords": [5.0721, 12.0682, 8.2090]},
        {"coords": [-5.4314, 8.5361, 7.9325]},
        {"coords": [-0.5979, 18.6385, 6.6714]},
        {"coords": [-7.7980, 7.8753, 4.1556]}
    ],
    "mode": "once",
    "patrol_loops": 1,
    "priority": 1,
    "preempt": False
}

# --- 40-point mission (map_area) ---
mission_40 = {
    "type": "map_area",
    "mission_id": "Mission_7939",
    "points": [
        {"x": -0.6824213862419128, "y": 3.2474277019500732, "z": 1.415939211845398},
        {"x": -0.8669881820678711, "y": 3.677643060684204, "z": 1.415939211845398},
        {"x": -1.1754653453826904, "y": 4.0297675132751465, "z": 1.415939211845398},
        {"x": -1.5776573419570923, "y": 4.269333362579346, "z": 1.415939211845398},
        {"x": -2.0341944694519043, "y": 4.372889995574951, "z": 1.415939211845398},
        {"x": -2.500387668609619, "y": 4.330300331115723, "z": 1.415939211845398},
        {"x": -2.93060302734375, "y": 4.145733833312988, "z": 1.415939211845398},
        {"x": -3.2827277183532715, "y": 3.83725643157959, "z": 1.415939211845398},
        {"x": -3.5222935676574707, "y": 3.4350643157958984, "z": 1.415939211845398},
        {"x": -3.625849723815918, "y": 2.978527545928955, "z": 1.415939211845398},
        {"x": -3.5832605361938477, "y": 2.512333869934082, "z": 1.415939211845398},
        {"x": -3.398693323135376, "y": 2.0821187496185303, "z": 1.415939211845398},
        {"x": -3.0902159214019775, "y": 1.7299940586090088, "z": 1.415939211845398},
        {"x": -2.6880240440368652, "y": 1.4904284477233887, "z": 1.415939211845398},
        {"x": -2.2314865589141846, "y": 1.3868720531463623, "z": 1.415939211845398},
        {"x": -1.765294075012207, "y": 1.4294615983963013, "z": 1.415939211845398},
        {"x": -1.3350787162780762, "y": 1.6140284538269043, "z": 1.415939211845398},
        {"x": -0.9829539060592651, "y": 1.9225058555603027, "z": 1.415939211845398},
        {"x": -0.7433883547782898, "y": 2.3246970176696777, "z": 1.415939211845398},
        {"x": -0.6398317813873291, "y": 2.7812345027923584, "z": 1.415939211845398},
        {"x": -0.6824213862419128, "y": 3.2474277019500732, "z": 6.403487205505371},
        {"x": -0.8669881820678711, "y": 3.677643060684204, "z": 6.403487205505371},
        {"x": -1.1754653453826904, "y": 4.0297675132751465, "z": 6.403487205505371},
        {"x": -1.5776573419570923, "y": 4.269333362579346, "z": 6.403487205505371},
        {"x": -2.0341944694519043, "y": 4.372889995574951, "z": 6.403487205505371},
        {"x": -2.500387668609619, "y": 4.330300331115723, "z": 6.403487205505371},
        {"x": -2.93060302734375, "y": 4.145733833312988, "z": 6.403487205505371},
        {"x": -3.2827277183532715, "y": 3.83725643157959, "z": 6.403487205505371},
        {"x": -3.5222935676574707, "y": 3.4350643157958984, "z": 6.403487205505371},
        {"x": -3.625849723815918, "y": 2.978527545928955, "z": 6.403487205505371},
        {"x": -3.5832605361938477, "y": 2.512333869934082, "z": 6.403487205505371},
        {"x": -3.398693323135376, "y": 2.0821187496185303, "z": 6.403487205505371},
        {"x": -3.0902159214019775, "y": 1.7299940586090088, "z": 6.403487205505371},
        {"x": -2.6880240440368652, "y": 1.4904284477233887, "z": 6.403487205505371},
        {"x": -2.2314865589141846, "y": 1.3868720531463623, "z": 6.403487205505371},
        {"x": -1.765294075012207, "y": 1.4294615983963013, "z": 6.403487205505371},
        {"x": -1.3350787162780762, "y": 1.6140284538269043, "z": 6.403487205505371},
        {"x": -0.9829539060592651, "y": 1.9225058555603027, "z": 6.403487205505371},
        {"x": -0.7433883547782898, "y": 2.3246970176696777, "z": 6.403487205505371},
        {"x": -0.6398317813873291, "y": 2.7812345027923584, "z": 6.403487205505371}
    ],
    "grid_spacing": 0.5058296918869019,
    "vertical_step": 1.6625159978866577,
    "priority": 86,
    "preempt": True
}


# ======================================================
# TCP SERVER (to receive telemetry back)
# ======================================================
def handle_client(conn, addr, monitor: ThroughputMonitor):
    """Handle telemetry from a single drone connection."""
    logger.info(f"[SERVER] Telemetry connected from {addr}")
    try:
        while True:
            data = conn.recv(4096)
            if not data:
                logger.info(f"[SERVER] Connection closed by {addr}")
                break
            monitor.add_rx(len(data))
            try:
                msg = json.loads(data.decode("utf-8").strip())
                logger.debug(f"[SERVER] Telemetry from {addr}: {msg}")
            except Exception:
                # don't spam logs if invalid json
                pass
    except Exception as e:
        logger.error(f"[SERVER] Error with {addr}: {e}")
    finally:
        conn.close()
        logger.info(f"[SERVER] Closed connection with {addr}")


def telemetry_server(monitor: ThroughputMonitor):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((CALLBACK_HOST, CALLBACK_PORT))
    # allow multiple pending connections
    server.listen(10)
    logger.info(f"[SERVER] Listening for telemetry on {CALLBACK_HOST}:{CALLBACK_PORT} ...")

    try:
        while True:
            conn, addr = server.accept()
            threading.Thread(
                target=handle_client, args=(conn, addr, monitor), daemon=True
            ).start()
    except Exception as e:
        logger.error(f"[SERVER] Fatal error: {e}")
    finally:
        server.close()
        logger.info("[SERVER] Closed telemetry server.")


# ======================================================
# TCP CLIENT (to send missions)
# ======================================================
def send(sock, data, monitor: ThroughputMonitor):
    payload = json.dumps(data) + "\n"
    sock.sendall(payload.encode("utf-8"))
    monitor.add_tx(len(payload.encode("utf-8")))

    n = len(data.get("waypoints", data.get("points", [])))
    logger.info(f"[CLIENT] Sent {data['mission_id']} ({data['type']}, {n} entries)")


# ======================================================
# MAIN
# ======================================================
def main():

    # Setup monitor + logs
    log_dir = os.path.join(os.getcwd(), "throughput_logs")
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, f"throughput_{datetime.datetime.now():%Y%m%d_%H%M%S}.jsonl")    
    
    monitor = ThroughputMonitor(logger, log_file=log_path)
    monitor.start()

    # Start telemetry server
    threading.Thread(target=telemetry_server, args=(monitor,), daemon=True).start()

    input("Press Enter to connect to remote and send missions...")

    logger.info(f"[CLIENT] Connecting to remote {REMOTE_HOST}:{REMOTE_PORT} ...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((REMOTE_HOST, REMOTE_PORT))
    logger.info("[CLIENT] Connected successfully.")

    

    try:
        # Send both missions
        send(sock, mission_add, monitor)
        time.sleep(8)
        send(sock, mission_40, monitor)

        logger.info("[CLIENT] Both missions sent. Waiting for telemetry on callback port...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logger.info("\n[CLIENT] Closing connection...")
    finally:
        monitor.stop()
        sock.close()


if __name__ == "__main__":
    main()

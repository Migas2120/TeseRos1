import socket
import json
import time
import random
import threading

HOST = "127.0.0.1"
PORT = 12345

# Default number of packets per second
PACKETS_PER_SECOND = 1  

# Interval at which to print throughput (seconds)
THROUGHPUT_INTERVAL = 1.0

# Stats
sent_packets = 0
lock = threading.Lock()

# Control variable
rate_lock = threading.Lock()

def send_packets():
    global sent_packets, PACKETS_PER_SECOND
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f"[+] Connected to {HOST}:{PORT}")

        while True:
            # Generate random telemetry values
            domain_id = random.randint(1000, 9999)
            mission_id = random.randint(1, 50)
            battery_percentage = round(random.uniform(20, 100), 2)
            battery_voltage = round(random.uniform(10, 16), 2)
            x, y, z = [round(random.uniform(-100, 100), 3) for _ in range(3)]
            mode = random.choice(["MANUAL", "AUTO", "RTL", "LOITER"])
            armed = random.choice([True, False])

            packet = {
                "type": "telemetry_update",
                "id": domain_id,
                "timestamp": time.time(),
                "active_mission": mission_id,
                "battery": {
                    "percentage": battery_percentage,
                    "voltage": battery_voltage,
                },
                "position": {
                    "x": x,
                    "y": y,
                    "z": z,
                },
                "status": {
                    "mode": mode,
                    "armed": armed,
                }
            }

            message = json.dumps(packet).encode('utf-8')
            s.sendall(message + b"\n")

            with lock:
                sent_packets += 1

            with rate_lock:
                current_rate = PACKETS_PER_SECOND
            time.sleep(1 / current_rate)

def measure_throughput():
    global sent_packets
    prev_count = 0
    while True:
        time.sleep(THROUGHPUT_INTERVAL)
        with lock:
            current = sent_packets
        throughput = (current - prev_count) / THROUGHPUT_INTERVAL
        print(f"[THROUGHPUT] {throughput:.2f} packets/sec")
        prev_count = current

def user_input_thread():
    global PACKETS_PER_SECOND
    while True:
        try:
            new_rate = float(input("Enter new packets per second: "))
            if new_rate <= 0:
                print("⚠️ Rate must be greater than 0")
                continue
            with rate_lock:
                PACKETS_PER_SECOND = new_rate
            print(f"[✔] Packet rate updated to {new_rate} packets/sec")
        except ValueError:
            print("❌ Invalid input. Please enter a number.")

if __name__ == "__main__":
    t1 = threading.Thread(target=send_packets, daemon=True)
    t2 = threading.Thread(target=measure_throughput, daemon=True)
    t3 = threading.Thread(target=user_input_thread, daemon=True)
    t1.start()
    t2.start()
    t3.start()
    t1.join()
    t2.join()
    t3.join()

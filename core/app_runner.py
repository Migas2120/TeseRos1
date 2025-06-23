# app_runner.py

import time
import threading
import json
import sys
import os

# Add ROS package root to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from tcp.server import TCPServer
from drone_instance.drone_instance import DroneInstance


class AppRunner:
    """
    Lightweight ROS 1 runner for a single-drone companion computer.
    Accepts TCP input from Unity and forwards it to the drone's control interface.
    """

    def __init__(self, logger):
        self.logger = logger
        self.tcp_server = None
        self.tcp_server_thread = None

        self.drone = DroneInstance(domain_id=0, logger=self.logger)
        self._log("info", "AppRunner initialized with 1 DroneInstance (ID 0)")

    def start(self):
        self._log("info", "Starting AppRunner...")

        self.tcp_server = TCPServer(
            ros_node=self,
            logger=self.logger,
            host='0.0.0.0',
            port=65432
        )
        self.tcp_server_thread = threading.Thread(target=self.tcp_server.start, daemon=True)
        self.tcp_server_thread.start()

        self._log("info", "TCP Server is running. Waiting for Unity commands...")

    def tick(self):
        """
        Called every second from main loop. Delegates to drone tick.
        """
        self.drone.tick()

    def publish_from_unity(self, message_json):
        try:
            data = json.loads(message_json)
            self._log("debug", f"Received Unity message: {data}")
            self.drone.publish_from_unity(data)
        except Exception as e:
            self._log("error", f"Error processing Unity message: {e}")

    def shutdown(self):
        self._log("info", "Shutting down TCP and drone...")
        if self.tcp_server:
            self.tcp_server.stop()
        self.drone.shutdown()

    def _log(self, level: str, msg: str):
        tag = "[AppRunner]"
        if self.logger:
            log_fn = getattr(self.logger, level, self.logger.info)
            log_fn(f"{tag} {msg}")
        else:
            print(f"{tag} {msg}")

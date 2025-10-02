import time
import json
import threading

import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))        # .../ros1_server/scripts
package_root = os.path.abspath(os.path.join(current_dir, '..')) # .../ros1_server
sys.path.insert(0, package_root)

from telemetry.telemetry_handlers.pose_handler import PoseHandler
from telemetry.telemetry_handlers.battery_handler import BatteryHandler
from telemetry.telemetry_handlers.gps_handler import GPSHandler
from telemetry.telemetry_handlers.status_handler import StatusHandler

class TelemetryManager:
    def __init__(self, logger=None, telemetry_log_file=None, log_interval=1.0):
        self.logger = logger 
        self.telemetry_log_file = telemetry_log_file
        self.log_interval = log_interval  # Seconds between logs
        self._stop_flag = threading.Event()

        self.handlers = [
            PoseHandler(logger=logger),
            BatteryHandler(logger=logger),
            GPSHandler(logger=logger),
            StatusHandler(logger=logger)
        ]


        if telemetry_log_file:
            log_dir = os.path.dirname(telemetry_log_file)
            if log_dir and not os.path.exists(log_dir):
                os.makedirs(log_dir)
            self.logger.info(f"[TelemetryManager] Logging to {telemetry_log_file}")
        
        self.thread = threading.Thread(target=self._run_logger, daemon=True)
        self.thread.start()
        self.logger.info(f"[TelemetryManager] Background telemetry logging started.")

    def _run_logger(self):
        while not self._stop_flag.is_set():
            self.log_once()
            time.sleep(self.log_interval)
            
    def get_all(self):
        data = {}
        for handler in self.handlers:
            serialized = handler.get_serialized()
            if serialized:
                data.update(serialized)
        return data

    def log_once(self):
        """
        Serializes and appends current telemetry snapshot to JSONL file.
        """
        if not self.telemetry_log_file:
            return

        data = self.get_all()
        if not data:
            return

        snapshot = {
            "timestamp": time.time(),
            "telemetry": data
        }

        try:
            with open(self.telemetry_log_file, "a") as f:
                f.write(json.dumps(snapshot) + "\n")
        except Exception as e:
            self.logger.error(f"[TelemetryManager] Failed to write telemetry log: {e}")
    
    def get_all_latest_status(self):
        return self.get_all()

    def get_latest_pose(self):
        for handler in self.handlers:
            if hasattr(handler, "get_pose"):
                return handler.get_pose()
        return None
    
    def get_pose_offset(self):
        for handler in self.handlers:
            if hasattr(handler, "get_offset"):
                return handler.get_offset()
        return None

    def shutdown(self):
        self._stop_flag.set()
        self.thread.join()
        self.logger.info("[TelemetryManager] Logging thread stopped.")
